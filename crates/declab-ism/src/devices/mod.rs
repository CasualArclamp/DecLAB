// made by CasualArclamp
//! The device decoders, one module per rtl_433 source file (each credits
//! its source). Each module has `DEVICES` (its `r_device`s) and `SAMPLES`
//! (known transmissions for the tests and the simulator).
//!
//! To add a device file: write the module, add it to `mod` and [`MODULES`].

#[cfg(test)]
use crate::device::Ctx;
use crate::device::Device;
use crate::sim::Sample;

pub mod acurite;
pub mod ambient_weather;
pub mod ambientweather_wh31e;
pub mod bresser_5in1;
pub mod bresser_6in1;
pub mod bresser_7in1;
pub mod current_cost;
pub mod efergy_e2_classic;
pub mod ert_idm;
pub mod ert_scm;
pub mod fineoffset;
pub mod fineoffset_wh1080;
pub mod generic_remote;
pub mod honeywell;
pub mod interlogix;
pub mod lacrosse_tx141x;
pub mod lacrosse_tx35;
pub mod nexus;
pub mod oregon_scientific;
pub mod prologue;
pub mod rubicson;
pub mod schraeder;
pub mod thermopro_tx2;
pub mod tpms_citroen;
pub mod tpms_ford;
pub mod tpms_hyundai_vdo;
pub mod tpms_kia;
pub mod tpms_renault;
pub mod tpms_toyota;

/// Every module's devices and samples, in rtl_433's protocol order
/// (roughly: older protocols first); priority sorts them at run time.
pub static MODULES: &[(&str, &[Device], &[Sample])] = &[
    ("nexus", nexus::DEVICES, nexus::SAMPLES),
    (
        "oregon_scientific",
        oregon_scientific::DEVICES,
        oregon_scientific::SAMPLES,
    ),
    (
        "lacrosse_tx141x",
        lacrosse_tx141x::DEVICES,
        lacrosse_tx141x::SAMPLES,
    ),
    (
        "lacrosse_tx35",
        lacrosse_tx35::DEVICES,
        lacrosse_tx35::SAMPLES,
    ),
    (
        "ambient_weather",
        ambient_weather::DEVICES,
        ambient_weather::SAMPLES,
    ),
    (
        "thermopro_tx2",
        thermopro_tx2::DEVICES,
        thermopro_tx2::SAMPLES,
    ),
    ("prologue", prologue::DEVICES, prologue::SAMPLES),
    ("rubicson", rubicson::DEVICES, rubicson::SAMPLES),
    ("acurite", acurite::DEVICES, acurite::SAMPLES),
    ("fineoffset", fineoffset::DEVICES, fineoffset::SAMPLES),
    (
        "fineoffset_wh1080",
        fineoffset_wh1080::DEVICES,
        fineoffset_wh1080::SAMPLES,
    ),
    (
        "ambientweather_wh31e",
        ambientweather_wh31e::DEVICES,
        ambientweather_wh31e::SAMPLES,
    ),
    ("bresser_5in1", bresser_5in1::DEVICES, bresser_5in1::SAMPLES),
    ("bresser_6in1", bresser_6in1::DEVICES, bresser_6in1::SAMPLES),
    ("bresser_7in1", bresser_7in1::DEVICES, bresser_7in1::SAMPLES),
    (
        "generic_remote",
        generic_remote::DEVICES,
        generic_remote::SAMPLES,
    ),
    ("honeywell", honeywell::DEVICES, honeywell::SAMPLES),
    ("interlogix", interlogix::DEVICES, interlogix::SAMPLES),
    ("schraeder", schraeder::DEVICES, schraeder::SAMPLES),
    ("tpms_citroen", tpms_citroen::DEVICES, tpms_citroen::SAMPLES),
    ("tpms_renault", tpms_renault::DEVICES, tpms_renault::SAMPLES),
    ("tpms_toyota", tpms_toyota::DEVICES, tpms_toyota::SAMPLES),
    ("tpms_ford", tpms_ford::DEVICES, tpms_ford::SAMPLES),
    (
        "tpms_hyundai_vdo",
        tpms_hyundai_vdo::DEVICES,
        tpms_hyundai_vdo::SAMPLES,
    ),
    ("tpms_kia", tpms_kia::DEVICES, tpms_kia::SAMPLES),
    ("current_cost", current_cost::DEVICES, current_cost::SAMPLES),
    (
        "efergy_e2_classic",
        efergy_e2_classic::DEVICES,
        efergy_e2_classic::SAMPLES,
    ),
    ("ert_scm", ert_scm::DEVICES, ert_scm::SAMPLES),
    ("ert_idm", ert_idm::DEVICES, ert_idm::SAMPLES),
];

/// Every built-in device.
pub fn all() -> impl Iterator<Item = &'static Device> {
    MODULES.iter().flat_map(|m| m.1.iter())
}

/// Every sample with the device that sends it.
pub fn samples() -> impl Iterator<Item = (&'static Device, &'static Sample)> {
    MODULES
        .iter()
        .flat_map(|m| m.2.iter().map(move |s| (&m.1[s.dev as usize], s)))
}

/// Check that each sample decodes, through its device alone, to its model
/// and expected fields (the modules' tests call this).
#[cfg(test)]
pub(crate) fn check_samples(devs: &[Device], samples: &[Sample]) {
    for s in samples {
        let dev = &devs[s.dev as usize];
        let mut bits = crate::bits::BitBuffer::parse(s.code);
        let mut ctx = Ctx::default();
        let n = (dev.decode)(&mut ctx, &mut bits);
        assert!(n > 0, "{}: {} gave {n} for {}", dev.name, s.model, s.code);
        let m = ctx
            .out
            .iter()
            .find(|m| m.model == s.model)
            .unwrap_or_else(|| panic!("{}: no {} in {:?}", dev.name, s.model, ctx.out));
        for (k, v) in s.expect {
            let got = m
                .get(k)
                .unwrap_or_else(|| panic!("{}: no field {k} in {m:?}", s.model))
                .to_string();
            assert_eq!(&got, v, "{}: field {k}", s.model);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_sample_decodes_through_its_slicer() {
        // The rows back into pulses and through the slicer: what the
        // receiver will do, less the radio.
        let mut fails = Vec::new();
        for (dev, s) in samples() {
            let lv = crate::sim::levels(dev, s);
            let rate = 500_000.0f32;
            let mut p = crate::slicer::PulseData {
                sample_rate: rate,
                ..Default::default()
            };
            crate::sim::levels_to_pulses(&lv, rate, &mut p);
            let mut bits = crate::bits::BitBuffer::new();
            let mut ctx = Ctx::default();
            crate::slicer::slice(&p, dev, &mut bits, &mut ctx);
            let m = ctx.out.iter().find(|m| m.model == s.model);
            if m.is_none() {
                fails.push(format!(
                    "{} ({}): got {:?}",
                    s.model,
                    dev.modulation.name(),
                    ctx.out.iter().map(|m| &m.model).collect::<Vec<_>>()
                ));
            }
        }
        assert!(
            fails.is_empty(),
            "not decoded after slicing:
{}",
            fails.join(
                "
"
            )
        );
    }
}
