#![feature(test)]

extern crate test;

pub mod cart;
pub mod cpu;
pub mod input;
pub mod nes;
pub mod ppu;

#[cfg(test)]
mod tests {
    use test::Bencher;

    use crate::cart::Cartridge;

    use super::*;
    use std::{
        assert_eq, format,
        fs::{read, read_to_string},
    };

    #[bench]
    fn dk1000(b: &mut Bencher) {
        let Ok(rom) = read("rom/dk.nes") else { return };

        let mut dk = nes::NES::new(
            cart::NROM::from_ines(rom),
            input::Controllers::disconnected(),
            ppu::FastPPU::new(),
        );

        // run dk for 1000 frames
        b.iter(|| {
            dk.reset();

            const FRAME: usize = 29780;
            const FRAMES: usize = 1000;
            const TARGET: usize = FRAME * FRAMES + FRAMES / 2;

            while dk.cycle_number() < TARGET {
                dk.instruction();
            }
        })
    }

    #[bench]
    fn smb1000(b: &mut Bencher) {
        let Ok(rom) = read("rom/smb.nes") else { return };

        let mut dk = nes::NES::new(
            cart::NROM::from_ines(rom),
            input::Controllers::disconnected(),
            ppu::FastPPU::new(),
        );

        // run dk for 1000 frames
        b.iter(|| {
            dk.reset();

            const FRAME: usize = 29780;
            const FRAMES: usize = 1000;
            const TARGET: usize = FRAME * FRAMES + FRAMES / 2;

            while dk.cycle_number() < TARGET {
                dk.instruction();
            }
        })
    }

    #[test]
    fn nestest_debug() {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let bytes = &mut file[16..16400];
        bytes[0xFFFC & 0x3FFF] = 0x00;
        bytes[0xFFFD & 0x3FFF] = 0xC0;

        let log = read_to_string("test/nestest/nestest.log").unwrap();
        let mut addrs = log.lines().map(|line| &line[0..4]);

        let cartridge = cart::NROM::new_128(
            cart::Mirroring::Horizontal,
            [0; 0x2000],
            bytes.try_into().unwrap(),
        );
        let mut nes = nes::NES::new(cartridge, input::Controllers::disconnected(), ppu::DummyPPU);

        const END_CYCLE: usize = 26548;
        const END_ADDR: u16 = 0xC6A2;

        while nes.cycle_number() < END_CYCLE {
            nes.instruction();

            assert_eq!(format!("{:04X}", nes.cpu.PC), addrs.next().unwrap());

            let op = nes.read(nes.cpu.PC);
            println!(
                "{:04X} {:02X} A:{:02X} X:{:02X} Y:{:02X} SP:{:02X} cyc:{}",
                nes.cpu.PC,
                op,
                nes.cpu.A,
                nes.cpu.X,
                nes.cpu.Y,
                nes.cpu.SP,
                nes.cycle_number()
            );
        }

        assert_eq!(nes.cycle_number(), END_CYCLE);
        assert_eq!(nes.cpu.PC, END_ADDR);
    }

    #[bench]
    fn nestest_dummy(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let bytes = &mut file[16..16400];
        bytes[0xFFFC & 0x3FFF] = 0x00;
        bytes[0xFFFD & 0x3FFF] = 0xC0;

        let cartridge = cart::NROM::new_128(
            cart::Mirroring::Horizontal,
            [0; 0x2000],
            bytes.try_into().unwrap(),
        );
        let mut nes = nes::NES::new(cartridge, input::Controllers::disconnected(), ppu::DummyPPU);

        b.iter(|| {
            nes.reset();

            const END_CYCLE: usize = 26548;
            const END_ADDR: u16 = 0xC6A2;

            while nes.cycle_number() < END_CYCLE {
                nes.instruction();
            }

            assert_eq!(nes.cycle_number(), END_CYCLE);
            assert_eq!(nes.cpu.PC, END_ADDR);
        });
    }

    #[bench]
    fn nestest_dummy_enum(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let bytes = &mut file[16..16400];
        bytes[0xFFFC & 0x3FFF] = 0x00;
        bytes[0xFFFD & 0x3FFF] = 0xC0;

        let cartridge = cart::CartridgeEnum::NROM(cart::NROM::new_128(
            cart::Mirroring::Horizontal,
            [0; 0x2000],
            bytes.try_into().unwrap(),
        ));
        let mut nes = nes::NES::new(cartridge, input::Controllers::disconnected(), ppu::DummyPPU);

        b.iter(|| {
            nes.reset();

            const END_CYCLE: usize = 26548;
            const END_ADDR: u16 = 0xC6A2;

            while nes.cycle_number() < END_CYCLE {
                nes.instruction();
            }

            assert_eq!(nes.cycle_number(), END_CYCLE);
            assert_eq!(nes.cpu.PC, END_ADDR);
        });
    }

    #[bench]
    fn nestest_fast(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let bytes = &mut file[16..16400];
        bytes[0xFFFC & 0x3FFF] = 0x00;
        bytes[0xFFFD & 0x3FFF] = 0xC0;

        let cartridge = cart::NROM::new_128(
            cart::Mirroring::Horizontal,
            [0; 0x2000],
            bytes.try_into().unwrap(),
        );
        let mut nes =
            nes::NES::new(cartridge, input::Controllers::disconnected(), ppu::FastPPU::new());

        b.iter(|| {
            nes.reset();

            const END_CYCLE: usize = 26548;
            const END_ADDR: u16 = 0xC6A2;

            while nes.cycle_number() < END_CYCLE {
                nes.instruction();
            }

            assert_eq!(nes.cycle_number(), END_CYCLE);
            assert_eq!(nes.cpu.PC, END_ADDR);
        });
    }

    #[bench]
    fn nestest_fast_enum(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let bytes = &mut file[16..16400];
        bytes[0xFFFC & 0x3FFF] = 0x00;
        bytes[0xFFFD & 0x3FFF] = 0xC0;

        let cartridge = cart::CartridgeEnum::NROM(cart::NROM::new_128(
            cart::Mirroring::Horizontal,
            [0; 0x2000],
            bytes.try_into().unwrap(),
        ));
        let mut nes =
            nes::NES::new(cartridge, input::Controllers::disconnected(), ppu::FastPPU::new());

        b.iter(|| {
            nes.reset();

            const END_CYCLE: usize = 26548;
            const END_ADDR: u16 = 0xC6A2;

            while nes.cycle_number() < END_CYCLE {
                nes.instruction();
            }

            assert_eq!(nes.cycle_number(), END_CYCLE);
            assert_eq!(nes.cpu.PC, END_ADDR);
        });
    }

    fn test_file(file: &str) {
        let mut cpu =
            nes::NES::read_ines(file, input::Controllers::disconnected(), ppu::FastPPU::new());

        let mut started = false;

        loop {
            cpu.instruction();

            if !started && cpu.read(0x6000) == 0x80 {
                started = true;
            }

            if started && cpu.read(0x6000) != 0x80 {
                break;
            }

            if cpu.cycle_number() > 10_000_000 {
                println!("forcefully halted");
                assert!(false);
            }
        }

        let mut read = 0x6004;
        while cpu.read(read) != 0 && read < 0x8000 {
            print!("{}", char::from(cpu.read(read)));
            read += 1;
        }

        assert_eq!(cpu.read(0x6000), 0);
    }

    #[test]
    fn vbl_basics() { test_file("test/ppu_vbl_nmi/01-vbl_basics.nes"); }

    #[test]
    fn vbl_set_time() { test_file("test/ppu_vbl_nmi/02-vbl_set_time.nes"); }

    #[test]
    fn vbl_clear_time() { test_file("test/ppu_vbl_nmi/03-vbl_clear_time.nes"); }

    #[test]
    fn nmi_control() { test_file("test/ppu_vbl_nmi/04-nmi_control.nes"); }

    #[test]
    fn nmi_timing() { test_file("test/ppu_vbl_nmi/05-nmi_timing.nes"); }

    #[test]
    fn nmi_suppression() { test_file("test/ppu_vbl_nmi/06-suppression.nes"); }

    #[test]
    fn nmi_on_timing() { test_file("test/ppu_vbl_nmi/07-nmi_on_timing.nes"); }

    #[test]
    fn nmi_off_timing() { test_file("test/ppu_vbl_nmi/08-nmi_off_timing.nes"); }

    #[test]
    fn even_odd_frames() { test_file("test/ppu_vbl_nmi/09-even_odd_frames.nes"); }

    #[test]
    fn even_odd_timing() { test_file("test/ppu_vbl_nmi/10-even_odd_timing.nes"); }

    #[test]
    fn sprite_hit_basics() { test_file("test/ppu_sprite_hit/01-basics.nes"); }

    #[test]
    fn sprite_hit_alignment() { test_file("test/ppu_sprite_hit/02-alignment.nes"); }

    #[test]
    fn sprite_hit_corners() { test_file("test/ppu_sprite_hit/03-corners.nes"); }

    #[test]
    fn sprite_hit_flip() { test_file("test/ppu_sprite_hit/04-flip.nes"); }

    #[test]
    fn sprite_hit_left_clip() { test_file("test/ppu_sprite_hit/05-left_clip.nes"); }

    #[test]
    fn sprite_hit_right_edge() { test_file("test/ppu_sprite_hit/06-right_edge.nes"); }

    #[test]
    fn sprite_hit_screen_bottom() { test_file("test/ppu_sprite_hit/07-screen_bottom.nes"); }

    #[test]
    fn sprite_hit_double_height() { test_file("test/ppu_sprite_hit/08-double_height.nes"); }

    #[test]
    fn sprite_hit_timing() { test_file("test/ppu_sprite_hit/09-timing.nes"); }

    #[test]
    fn sprite_hit_timing_order() { test_file("test/ppu_sprite_hit/10-timing_order.nes"); }
}
