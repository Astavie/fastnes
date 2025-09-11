pub mod cart;
pub mod cpu;
pub mod input;
pub mod nes;
pub mod ppu;

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        assert_eq, format,
        fs::{read, read_to_string},
        sync::{atomic::AtomicU8, Arc},
    };

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

    #[test]
    fn accuracy_coin() {
        let left = Arc::new(AtomicU8::new(0b00001000)); // pressing Start
        let right = Arc::new(AtomicU8::new(0));
        let mut nes = nes::NES::read_ines(
            "test/AccuracyCoin.nes",
            input::Controllers::standard_2p(&left, &right),
            ppu::FastPPU::new(),
        );

        let addr_running: u16 = 0x35;

        // Perform tests
        while nes.read_internal(addr_running) != 1 {
            nes.instruction();
        }
        while nes.read_internal(addr_running) != 0 {
            nes.instruction();
        }

        // Check results
        let addr_suites_start: u16 = 0x8200;
        let addr_suites_end: u16 = nes.read_cart_rom_word(addr_suites_start);

        for addr_suite_entry in (addr_suites_start..addr_suites_end).step_by(2) {
            let addr_suite = nes.read_cart_rom_word(addr_suite_entry);

            // suites are made up of:
            // name bytes, $FF, list of tests, $FF
            let (suite_name, mut addr_test) = nes.read_cart_rom_string(addr_suite, 0xFF);
            println!("{}", suite_name);

            while nes.read_cart_rom(addr_test) != 0xFF {
                // suite tests are made up of:
                // name bytes, $FF, result address, rom address

                let (test_name, addr_addr_result) = nes.read_cart_rom_string(addr_test, 0xFF);
                let addr_result = nes.read_cart_rom_word(addr_addr_result);
                let result = nes.read_internal(addr_result) >> 2;

                print!("  {} ... ", test_name);
                assert_eq!(result, 0);
                println!("ok");

                addr_test = addr_addr_result.wrapping_add(4);
            }
        }
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
