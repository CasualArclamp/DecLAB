// made by CasualArclamp
//! The FEC stage of a DVB-S2 receiver: demodulated PLFRAMEs in, BBFRAMEs
//! out (from DecSAT's engine).
//!
//! pilots out → scale by the measured gain → max-log LLRs, de-interleaved →
//! LDPC (layered min-sum) → BCH → BB descrambling → BBHEADER (CRC-8).

use std::collections::HashMap;

use decsat_core::Iq;
use decsat_fec::demap::{Mapper, quantize};
use decsat_fec::vlsnr::VlsnrCode;
use decsat_fec::{Bch, BchError, DecodeOutcome, FecParams, LdpcCode, LdpcDecoder};
use decsat_frame::pi2bpsk::pi2_soft;
use decsat_frame::vlsnr::{self as vl, Slot};
use decsat_frame::{BbHeader, BbHeaderError, PlsInfo, bb_scramble};

use crate::demod::{PILOT_AFTER, PILOT_PERIOD, PlFrame};

/// LLR quantization: steps per LLR unit (the decoder is happy from 2 to 8).
const LLR_SCALE: f32 = 4.0;
/// LDPC iteration budget per frame.
const MAX_ITERATIONS: usize = 50;

/// One decoded BBFRAME.
#[derive(Debug, Clone)]
pub struct BbFrame {
    pub pls: PlsInfo,
    /// The descrambled BBFRAME: header, data field, padding (K_bch bits).
    pub bytes: Vec<u8>,
    pub header: Result<BbHeader, BbHeaderError>,
    pub ldpc: DecodeOutcome,
    /// Bits BCH corrected, or why it could not.
    pub bch: Result<usize, BchError>,
    pub es_n0_db: f32,
}

impl BbFrame {
    /// BCH accepted the codeword and the BBHEADER's CRC-8 checks.
    pub fn ok(&self) -> bool {
        self.bch.is_ok() && self.header.is_ok()
    }
}

struct Code {
    params: FecParams,
    mapper: Mapper,
    ldpc: LdpcDecoder,
    bch: Bch,
}

/// A VL-SNR code's decoders.
struct VlCode {
    code: &'static VlsnrCode,
    ldpc: LdpcDecoder,
    bch: Bch,
    /// QPSK demapping for the 2/9 code.
    qpsk: Option<Mapper>,
}

/// Decodes PLFRAMEs; holds a decoder per code met so far.
#[derive(Default)]
pub struct FecDecoder {
    /// Give up frames whose decoding stalls (the thread is behind: see
    /// `LdpcDecoder::decode_hurried`).
    pub hurry: bool,
    codes: HashMap<(bool, u8), Option<Code>>,
    vl_codes: HashMap<u8, Option<VlCode>>,
    sent: Vec<f32>,
    data: Vec<Iq>,
    llr: Vec<f32>,
    quantized: Vec<i8>,
    info: Vec<u8>,
}

impl FecDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Decode one frame; `None` for a dummy frame, a frame with no code to
    /// decode (S2 short 9/10, S2X VL-SNR and reserved codes).
    pub fn decode(&mut self, f: &PlFrame) -> Option<BbFrame> {
        if f.pls.dummy_frame {
            return None;
        }
        if let (Some(set), Some((k, _))) = (f.pls.vlsnr, f.vlsnr) {
            return self.decode_vlsnr(f, set, k);
        }
        let FecDecoder {
            hurry,
            codes,
            data,
            llr,
            quantized,
            info,
            ..
        } = self;
        let code = codes
            .entry((f.pls.short_fecframe, f.pls.modcod))
            .or_insert_with(|| build_code(f.pls))
            .as_mut()?;
        let p = code.params;

        // Data symbols only, on the constellation's unit-power scale.
        let inv = 1.0 / f.gain.max(1e-6);
        data.clear();
        data.extend(
            f.payload
                .iter()
                .enumerate()
                .filter(|(i, _)| !f.pls.has_pilots || i % PILOT_PERIOD < PILOT_AFTER)
                .map(|(_, &y)| y * inv),
        );
        let n = p.n_ldpc;
        if data.len() != code.mapper.symbols() {
            return None;
        }
        llr.resize(n, 0.0);
        quantized.resize(n, 0);
        info.resize(p.n_bch / 8, 0);

        code.mapper.demap_llr(data, f.noise_var * inv * inv, llr);
        quantize(llr, LLR_SCALE, quantized);
        let ldpc = if *hurry {
            code.ldpc.decode_hurried(quantized, info, MAX_ITERATIONS)
        } else {
            code.ldpc.decode(quantized, info, MAX_ITERATIONS)
        };

        let mut bytes = info.clone();
        let bch = code.bch.decode(&mut bytes);
        bytes.truncate(p.bbframe_bytes());
        bb_scramble(&mut bytes);
        let header = BbHeader::parse(&bytes);
        Some(BbFrame {
            pls: f.pls,
            bytes,
            header,
            ldpc,
            bch,
            es_n0_db: f.es_n0_db(),
        })
    }
}

impl FecDecoder {
    /// A VL-SNR frame (EN 302 307-2 §5.5.2.6): data symbols from the
    /// layout, LLRs (pi/2-BPSK, spreading combined; or QPSK), the shortened
    /// and punctured bits put back, then LDPC and BCH as usual.
    fn decode_vlsnr(&mut self, f: &PlFrame, set: u8, k: u8) -> Option<BbFrame> {
        let vc = self
            .vl_codes
            .entry(k)
            .or_insert_with(|| {
                let code = VlsnrCode::for_header(k)?;
                Some(VlCode {
                    code,
                    ldpc: LdpcDecoder::new(LdpcCode::new(code.table)),
                    bch: Bch::new(code.frame, 12, code.n_bch),
                    qpsk: code.qpsk.then(|| {
                        Mapper::s2(
                            decsat_fec::Constellation::qpsk(),
                            decsat_core::CodeRate::new(2, 9),
                            code.sent_bits(),
                        )
                    }),
                })
            })
            .as_mut()?;
        let code = vc.code;
        let inv = 1.0 / f.gain.max(1e-6);
        let noise = (f.noise_var * inv * inv).max(1e-6);
        let layout = vl::layout(set);
        self.data.clear();
        self.data.extend(
            f.payload
                .iter()
                .zip(layout)
                .filter(|(_, s)| **s == Slot::Data)
                .map(|(&y, _)| y * inv),
        );
        if self.data.len() != code.symbols() {
            return None;
        }
        self.sent.clear();
        match &vc.qpsk {
            Some(m) => {
                self.sent.resize(code.sent_bits(), 0.0);
                m.demap_llr(&self.data, noise, &mut self.sent);
            }
            None => {
                // Unit-amplitude 2-PAM: LLR = 4·y/N0; spread bits add.
                let reps = if code.spread { 2 } else { 1 };
                for pair in self.data.chunks(reps).enumerate() {
                    let (j, ys) = pair;
                    let llr: f32 = ys
                        .iter()
                        .enumerate()
                        .map(|(r, &y)| 4.0 * pi2_soft(j * reps + r, y) / noise)
                        .sum();
                    self.sent.push(llr);
                }
            }
        }
        let n = code.n_ldpc();
        self.llr.resize(n, 0.0);
        code.expand_llr(&self.sent, &mut self.llr);
        self.quantized.resize(n, 0);
        quantize(&self.llr, LLR_SCALE, &mut self.quantized);
        self.info.resize(code.k_ldpc() / 8, 0);
        let ldpc = vc
            .ldpc
            .decode(&self.quantized, &mut self.info, MAX_ITERATIONS);

        let at = code.xs / 8;
        let mut bytes = self.info[at..at + code.n_bch / 8].to_vec();
        let bch = vc.bch.decode(&mut bytes);
        bytes.truncate(code.bbframe_bytes());
        bb_scramble(&mut bytes);
        // A ragged K (the medium codes): the last byte's spare bits are
        // BCH parity, not BBFRAME.
        if !code.k_bch.is_multiple_of(8)
            && let Some(last) = bytes.last_mut()
        {
            *last &= 0xFF << (8 - code.k_bch % 8);
        }
        let header = BbHeader::parse(&bytes);
        Some(BbFrame {
            pls: f.pls,
            bytes,
            header,
            ldpc,
            bch,
            es_n0_db: f.es_n0_db(),
        })
    }
}

fn build_code(pls: PlsInfo) -> Option<Code> {
    let mc = pls.modcod()?;
    let params = FecParams::new(mc.frame, mc.rate)?;
    Some(Code {
        params,
        mapper: Mapper::for_modcod(&mc)?,
        ldpc: LdpcDecoder::new(LdpcCode::new(params.ldpc_table())),
        bch: Bch::new(mc.frame, params.t, params.n_bch),
    })
}

