//! SHA-256, for checking a downloaded star map against the hash on record.
//!
//! Written out here, sixty lines from FIPS 180-4, rather than taken from a crate: it is used once,
//! on one file, to tell a whole download from a damaged one, and a crate would add half a dozen
//! entries to a lock file this program has kept to two. Nothing here is a secret or a signature;
//! the hash guards against a truncated or corrupted file, and against NASA having replaced the
//! file with another.

use std::io::Read;

/// The first 32 bits of the fractional parts of the cube roots of the first 64 primes.
const K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

/// The first 32 bits of the fractional parts of the square roots of the first 8 primes.
const INITIAL: [u32; 8] = [
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
];

/// One 64-byte block folded into `state`.
fn compress(state: &mut [u32; 8], block: &[u8; 64]) {
    let mut w = [0u32; 64];
    for (word, bytes) in w.iter_mut().zip(block.as_chunks::<4>().0) {
        *word = u32::from_be_bytes(*bytes);
    }
    for i in 16..64 {
        let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
        let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16]
            .wrapping_add(s0)
            .wrapping_add(w[i - 7])
            .wrapping_add(s1);
    }
    let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = *state;
    for i in 0..64 {
        let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
        let choose = (e & f) ^ (!e & g);
        let t1 = h
            .wrapping_add(s1)
            .wrapping_add(choose)
            .wrapping_add(K[i])
            .wrapping_add(w[i]);
        let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
        let majority = (a & b) ^ (a & c) ^ (b & c);
        let t2 = s0.wrapping_add(majority);
        (h, g, f, e, d, c, b, a) = (g, f, e, d.wrapping_add(t1), c, b, a, t1.wrapping_add(t2));
    }
    for (sum, add) in state.iter_mut().zip([a, b, c, d, e, f, g, h]) {
        *sum = sum.wrapping_add(add);
    }
}

/// The SHA-256 of everything `from` yields, as 64 lowercase hexadecimal digits: the form
/// `sha256sum` writes.
pub fn hex_of(from: &mut dyn Read) -> std::io::Result<String> {
    let mut state = INITIAL;
    let mut length: u64 = 0;
    // Whole blocks are folded in as they are read; `held` is the tail short of a block.
    let mut held: Vec<u8> = Vec::with_capacity(64);
    let mut buffer = vec![0u8; 1 << 16];
    loop {
        let read = match from.read(&mut buffer) {
            Ok(0) => break,
            Ok(read) => read,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e),
        };
        length += read as u64;
        let mut fresh = &buffer[..read];
        if !held.is_empty() {
            let take = (64 - held.len()).min(fresh.len());
            held.extend_from_slice(&fresh[..take]);
            fresh = &fresh[take..];
            let Ok(block) = <&[u8; 64]>::try_from(held.as_slice()) else {
                continue;
            };
            compress(&mut state, block);
            held.clear();
        }
        let (blocks, tail) = fresh.as_chunks::<64>();
        held.extend_from_slice(tail);
        for block in blocks {
            compress(&mut state, block);
        }
    }
    // The padding: a one bit, zeros to eight bytes short of a block, and the length in bits.
    held.push(0x80);
    while held.len() % 64 != 56 {
        held.push(0);
    }
    held.extend_from_slice(&(length.wrapping_mul(8)).to_be_bytes());
    for block in held.as_chunks::<64>().0 {
        compress(&mut state, block);
    }
    Ok(state.iter().map(|word| format!("{word:08x}")).collect())
}
