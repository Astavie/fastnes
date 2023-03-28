#![allow(incomplete_features)]
#![feature(test)]
#![feature(adt_const_params)]
#![feature(generic_const_exprs)]
#![feature(const_mut_refs)]

extern crate test;

mod cpu;

#[cfg(test)]
mod tests {
    use test::Bencher;

    use super::*;
    use std::fs::read;

    #[bench]
    fn nestest(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let bytes = &mut file[16..16400];
        bytes[0xFFFC & 0x3FFF] = 0x00;
        bytes[0xFFFD & 0x3FFF] = 0xC0;

        let mut rom: [u8; 0x8000] = [0; 0x8000];
        for i in 0..0x8000 {
            rom[i] = bytes[i & 0x3FFF]
        }

        let mut cpu = cpu::CPU::<cpu::DummyPPU>::new(rom);

        b.iter(|| {
            cpu.reset();

            const END_CYCLE: usize = 26548;
            const END_ADDR: u16 = 0xC6A2;

            while cpu.ppu_cycle / 3 < END_CYCLE {
                cpu.instruction();
            }

            assert_eq!(cpu.ppu_cycle / 3, END_CYCLE);
            assert_eq!(cpu.PC, END_ADDR);
        });
    }

    fn test_file(file: &str) {
        let mut file = read(file).unwrap();
        let rom = &mut file[16..32784];
        let rom: [u8; 0x8000] = rom.try_into().unwrap();

        let mut cpu = cpu::CPU::<cpu::FastPPU>::new(rom);

        cpu.reset();

        let mut started = false;

        loop {
            cpu.instruction();

            if !started && cpu.ram_work[0] == 0x80 {
                started = true;
            }

            if started && cpu.ram_work[0] != 0x80 {
                break;
            }

            if cpu.ppu_cycle > 30_000_000 {
                println!("forcefully halted");
                break;
            }
        }

        let mut read = 4;
        while cpu.ram_work[read] != 0 && read < 0x2000 {
            print!("{}", char::from(cpu.ram_work[read]));
            read += 1;
        }

        assert_eq!(cpu.ram_work[0], 0);
    }

    #[test]
    fn vbl_basics() {
        test_file("test/ppu_vbl_nmi/01-vbl_basics.nes");
    }

    #[test]
    fn vbl_set_time() {
        test_file("test/ppu_vbl_nmi/02-vbl_set_time.nes");
    }

    #[test]
    fn vbl_clear_time() {
        test_file("test/ppu_vbl_nmi/03-vbl_clear_time.nes");
    }

    #[test]
    fn nmi_control() {
        test_file("test/ppu_vbl_nmi/04-nmi_control.nes");
    }

    #[test]
    fn nmi_timing() {
        test_file("test/ppu_vbl_nmi/05-nmi_timing.nes");
    }

    #[test]
    fn nmi_suppression() {
        test_file("test/ppu_vbl_nmi/06-suppression.nes");
    }

    #[test]
    fn nmi_on_timing() {
        test_file("test/ppu_vbl_nmi/07-nmi_on_timing.nes");
    }

    #[test]
    fn nmi_off_timing() {
        test_file("test/ppu_vbl_nmi/08-nmi_off_timing.nes");
    }

    #[test]
    fn even_odd_frames() {
        test_file("test/ppu_vbl_nmi/09-even_odd_frames.nes");
    }

    #[test]
    fn even_odd_timing() {
        test_file("test/ppu_vbl_nmi/10-even_odd_timing.nes");
    }
}

fn main() {}
