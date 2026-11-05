// made by CasualArclamp
//! ISM-band devices, as rtl_433 decodes them: weather stations,
//! temperature/humidity sensors, remotes and door/window sensors, tyre-
//! pressure sensors, energy monitors and utility meters, at 315, 345, 433,
//! 868 and 915 MHz.
//!
//! The device side is ported from rtl_433 (<https://github.com/merbanan/rtl_433>,
//! GPL-2.0-or-later; DecLAB is GPL-3.0-or-later): the bit buffer and bit
//! helpers ([`bits`]), the pulse slicers ([`slicer`]), the `r_device`
//! shape ([`device`]), the flex decoder ([`flex`]) and each device file
//! ([`devices`]), each crediting its source.
//!
//! The radio side is DecLAB's own ([`burst`], [`receiver`]): rtl_433 runs
//! one envelope and one FM discriminator over its whole band; here a short
//! FFT watches the whole channel, each burst is cut out where it is, at
//! its own bandwidth, so two sensors at different frequencies at once both
//! decode, and a narrow sensor is not buried in the band's noise.
//!
//! - [`sim`]: rows back into pulses and IQ for tests and the simulator.

pub mod bits;
pub mod burst;
pub mod device;
pub mod devices;
pub mod flex;
pub mod pulse;
pub mod receiver;
pub mod sim;
pub mod slicer;

pub use device::{Device, Group, Message, Modulation, Value};
pub use receiver::{Decoded, Receiver, ReceiverConfig};
