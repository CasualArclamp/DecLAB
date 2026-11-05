// made by CasualArclamp
//! Just enough protobuf (the wire format, protobuf.dev "Encoding") to read
//! Meshtastic's messages and, for the simulator, write them: a field is a
//! varint key (number << 3 | wire type) then a varint, 8 bytes, a length
//! and bytes, or 4 bytes.

/// One field's value as it came off the wire.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Wire<'a> {
    Varint(u64),
    Fixed64(u64),
    Bytes(&'a [u8]),
    Fixed32(u32),
}

impl<'a> Wire<'a> {
    pub fn u64(&self) -> Option<u64> {
        match *self {
            Wire::Varint(v) | Wire::Fixed64(v) => Some(v),
            Wire::Fixed32(v) => Some(v as u64),
            Wire::Bytes(_) => None,
        }
    }

    pub fn u32(&self) -> Option<u32> {
        self.u64().map(|v| v as u32)
    }

    /// `int32` (varint, two's complement) or `sfixed32`.
    pub fn i32(&self) -> Option<i32> {
        self.u64().map(|v| v as i32)
    }

    /// `sint32` (zigzag).
    pub fn sint32(&self) -> Option<i32> {
        self.u64().map(|v| ((v >> 1) as i32) ^ -((v & 1) as i32))
    }

    pub fn f32(&self) -> Option<f32> {
        match *self {
            Wire::Fixed32(v) => Some(f32::from_bits(v)),
            _ => None,
        }
    }

    pub fn bytes(&self) -> Option<&'a [u8]> {
        match *self {
            Wire::Bytes(b) => Some(b),
            _ => None,
        }
    }

    pub fn str(&self) -> Option<String> {
        self.bytes()
            .map(|b| String::from_utf8_lossy(b).into_owned())
    }
}

fn varint(b: &[u8], i: &mut usize) -> Option<u64> {
    let mut v = 0u64;
    for shift in (0..64).step_by(7) {
        let x = *b.get(*i)?;
        *i += 1;
        v |= ((x & 0x7f) as u64) << shift;
        if x & 0x80 == 0 {
            return Some(v);
        }
    }
    None
}

/// Every field of a message, in order; None if the bytes are not a valid
/// protobuf message (a wrong key gives garbage that rarely parses whole).
pub fn parse(b: &[u8]) -> Option<Vec<(u32, Wire<'_>)>> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let key = varint(b, &mut i)?;
        let num = (key >> 3) as u32;
        if num == 0 || num > 536_870_911 {
            return None;
        }
        let w = match key & 7 {
            0 => Wire::Varint(varint(b, &mut i)?),
            1 => {
                let v = u64::from_le_bytes(b.get(i..i + 8)?.try_into().ok()?);
                i += 8;
                Wire::Fixed64(v)
            }
            2 => {
                let n = varint(b, &mut i)? as usize;
                let s = b.get(i..i.checked_add(n)?)?;
                i += n;
                Wire::Bytes(s)
            }
            5 => {
                let v = u32::from_le_bytes(b.get(i..i + 4)?.try_into().ok()?);
                i += 4;
                Wire::Fixed32(v)
            }
            _ => return None,
        };
        out.push((num, w));
    }
    Some(out)
}

/// The first field with this number.
pub fn get<'a>(fields: &[(u32, Wire<'a>)], num: u32) -> Option<Wire<'a>> {
    fields.iter().find(|(n, _)| *n == num).map(|(_, w)| *w)
}

/// A message being written (the simulator's packets).
#[derive(Default)]
pub struct Writer(pub Vec<u8>);

impl Writer {
    fn put_varint(&mut self, mut v: u64) {
        while v >= 0x80 {
            self.0.push(v as u8 | 0x80);
            v >>= 7;
        }
        self.0.push(v as u8);
    }

    pub fn varint(&mut self, num: u32, v: u64) -> &mut Self {
        self.put_varint((num as u64) << 3);
        self.put_varint(v);
        self
    }

    pub fn sint32(&mut self, num: u32, v: i32) -> &mut Self {
        self.varint(num, ((v << 1) ^ (v >> 31)) as u32 as u64)
    }

    pub fn fixed32(&mut self, num: u32, v: u32) -> &mut Self {
        self.put_varint((num as u64) << 3 | 5);
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }

    pub fn float(&mut self, num: u32, v: f32) -> &mut Self {
        self.fixed32(num, v.to_bits())
    }

    pub fn bytes(&mut self, num: u32, b: &[u8]) -> &mut Self {
        self.put_varint((num as u64) << 3 | 2);
        self.put_varint(b.len() as u64);
        self.0.extend_from_slice(b);
        self
    }

    pub fn done(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let m = Writer::default()
            .varint(1, 300)
            .sint32(9, -12)
            .fixed32(4, 0xdeadbeef)
            .float(2, 3.5)
            .bytes(3, b"hi")
            .done();
        let f = parse(&m).unwrap();
        assert_eq!(get(&f, 1).unwrap().u64(), Some(300));
        assert_eq!(get(&f, 9).unwrap().sint32(), Some(-12));
        assert_eq!(get(&f, 4).unwrap().u32(), Some(0xdeadbeef));
        assert_eq!(get(&f, 2).unwrap().f32(), Some(3.5));
        assert_eq!(get(&f, 3).unwrap().str().as_deref(), Some("hi"));
        assert!(parse(&[0x0a, 0x05, 1]).is_none());
    }
}
