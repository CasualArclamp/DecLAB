// made by CasualArclamp
//! Every decoder DecLAB knows about.
//!
//! To add one: put it in a file of its own in this folder (copy `level.rs`),
//! add `mod yourfile;` below and its factory to [`ALL`]. That is the whole
//! registration.

use declab_api::DecoderFactory;

mod level;

/// The mode picker's list, in the order it shows them within a family.
pub static ALL: &[&dyn DecoderFactory] = &[
    &level::Factory,
];

/// Look a decoder up by its [`DecoderInfo::id`](declab_api::DecoderInfo::id).
pub fn find(id: &str) -> Option<&'static dyn DecoderFactory> {
    ALL.iter().copied().find(|f| f.info().id == id)
}
