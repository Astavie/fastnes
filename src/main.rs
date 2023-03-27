#![allow(incomplete_features)]
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
    use repeated::repeated;
    use test::Bencher;

    #[test]
    fn nestest() {
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

    // repeated!(for op in [0;255] {

    //     #[bench]
    //     fn bench_instr_%%op%%(b: &mut Bencher) {
    //         let mut file = read("test/nestest/nestest.nes").unwrap();
    //         let rom = &mut file[16..16400];
    //         rom[0xFFFC & 0x3FFF] = 0x00;
    //         rom[0xFFFD & 0x3FFF] = 0xC0;
    //         let rom: [u8; 16384] = rom.try_into().unwrap();

    //         let instrs = cpu::instructions();
    //         let instr = instrs[%%op%%];

    //         let mut cpu = cpu::MOS6502::new(rom, instrs, cpu::DummyPPU());
    //         cpu.reset();
    //         cpu.instruction();

    //         b.iter(|| instr(&mut cpu));
    //     }

    // });
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
