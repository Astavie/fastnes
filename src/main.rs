#![feature(test)]
#![feature(adt_const_params)]
#![feature(generic_const_exprs)]
#![feature(const_mut_refs)]

extern crate test;

use std::fs::read;

mod cpu;

#[cfg(test)]
mod tests {
    use super::*;
    use test::Bencher;

    #[test]
    fn nestest_complete() {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let rom = &mut file[16..16400];
        rom[0xFFFC & 0x3FFF] = 0x00;
        rom[0xFFFD & 0x3FFF] = 0xC0;
        let rom: [u8; 16384] = rom.try_into().unwrap();

        let instr = cpu::instructions();
        cpu::nestest(rom, instr);
    }

    #[bench]
    fn bench_nestest(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let rom = &mut file[16..16400];
        rom[0xFFFC & 0x3FFF] = 0x00;
        rom[0xFFFD & 0x3FFF] = 0xC0;
        let rom: [u8; 16384] = rom.try_into().unwrap();

        let instr = cpu::instructions();
        b.iter(|| cpu::nestest(rom, instr.clone()));
    }
}

fn main() {
    let mut file = read("test/nestest/nestest.nes").unwrap();
    let rom = &mut file[16..16400];
    rom[0xFFFC & 0x3FFF] = 0x00;
    rom[0xFFFD & 0x3FFF] = 0xC0;
    let rom: [u8; 16384] = rom.try_into().unwrap();

    let instr = cpu::instructions();
    cpu::nestest(rom, instr);
}
