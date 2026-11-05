// made by CasualArclamp
//! LoRa, and the two meshes that use it most: Meshtastic and MeshCore.
//!
//! - [`codec`]: LoRa's coding chain (gray, interleaver, Hamming, header,
//!   whitening, CRC), ported from gr-lora_sdr (GPL-3.0-or-later).
//! - [`receiver`]: CSS demodulation, DecLAB's own: one channel split into
//!   62.5/125/250/500 kHz branches, every spreading factor 7–12 watched in
//!   each, so the presets are found by themselves.
//! - [`meshtastic`]: the radio header, AES-CTR with the channel's key (the
//!   default public key built in), and the protobuf payloads (text,
//!   position, node info, telemetry …), field numbers from
//!   meshtastic/protobufs (GPL-3.0).
//! - [`meshcore`]: the packet header, route and path, adverts, and group
//!   text on the Public channel (MeshCore, MIT).
//! - [`sim`]: a LoRa modulator and packets of both kinds, for tests and
//!   the simulator.

pub mod codec;
pub mod crypto;
pub mod meshcore;
pub mod meshtastic;
mod mt_names;
pub mod proto;
pub mod receiver;
pub mod sim;

pub use receiver::{Receiver, RxFrame};

/// The channel keys a receiver reads the meshes with.
#[derive(Clone)]
pub struct Keys {
    pub meshtastic: Vec<meshtastic::Channel>,
    pub meshcore: Vec<meshcore::Channel>,
}

impl Default for Keys {
    /// Every Meshtastic preset's default channel, and MeshCore's Public.
    fn default() -> Self {
        Keys {
            meshtastic: meshtastic::default_channels(),
            meshcore: vec![meshcore::Channel::public()],
        }
    }
}

/// What a frame's payload is, by its sync word.
#[derive(Debug, Clone)]
pub enum Payload {
    Meshtastic(meshtastic::Packet),
    MeshCore(meshcore::Packet),
    /// Another sync word (LoRaWAN's 0x34 …), or a payload that did not parse.
    Other,
}

pub fn decode_payload(sync: Option<u8>, bytes: &[u8], keys: &Keys) -> Payload {
    match sync {
        Some(meshtastic::SYNC_WORD) => {
            meshtastic::decode(bytes, &keys.meshtastic).map_or(Payload::Other, Payload::Meshtastic)
        }
        Some(meshcore::SYNC_WORD) => {
            meshcore::decode(bytes, &keys.meshcore).map_or(Payload::Other, Payload::MeshCore)
        }
        _ => Payload::Other,
    }
}
