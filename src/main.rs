#![allow(incomplete_features)]
#![feature(test)]
#![feature(adt_const_params)]
#![feature(generic_const_exprs)]
#![feature(const_mut_refs)]
#![feature(is_some_and)]

use std::fs::read;
use std::time::Duration;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::rect::{Point, Rect};
use sdl2::render::TextureQuery;

extern crate test;

mod cartridge;
mod controller;
mod cpu;
mod instruction;
mod ppu;

#[cfg(test)]
mod tests {
    use test::Bencher;

    use super::*;
    use std::fs::read;

    #[bench]
    fn nestest_dummy(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let bytes = &mut file[16..16400];
        bytes[0xFFFC & 0x3FFF] = 0x00;
        bytes[0xFFFD & 0x3FFF] = 0xC0;

        let cartridge = cartridge::NROM::new_128(
            cartridge::Mirroring::Horizontal,
            [0; 0x2000],
            bytes.try_into().unwrap(),
        );
        let mut cpu = cpu::CPU::new(
            cartridge,
            controller::Controllers::disconnected(),
            ppu::DummyPPU,
        );

        b.iter(|| {
            cpu.reset();

            const END_CYCLE: usize = 26548;
            const END_ADDR: u16 = 0xC6A2;

            while cpu.cycle() < END_CYCLE {
                cpu.instruction();
            }

            assert_eq!(cpu.cycle(), END_CYCLE);
            assert_eq!(cpu.PC, END_ADDR);
        });
    }

    #[bench]
    fn nestest_fast(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let bytes = &mut file[16..16400];
        bytes[0xFFFC & 0x3FFF] = 0x00;
        bytes[0xFFFD & 0x3FFF] = 0xC0;

        let cartridge = cartridge::NROM::new_128(
            cartridge::Mirroring::Horizontal,
            [0; 0x2000],
            bytes.try_into().unwrap(),
        );
        let mut cpu = cpu::CPU::new(
            cartridge,
            controller::Controllers::disconnected(),
            ppu::FastPPU::new(),
        );

        b.iter(|| {
            cpu.reset();

            const END_CYCLE: usize = 26548;
            const END_ADDR: u16 = 0xC6A2;

            while cpu.cycle() < END_CYCLE {
                cpu.instruction();
            }

            assert_eq!(cpu.cycle(), END_CYCLE);
            assert_eq!(cpu.PC, END_ADDR);
        });
    }

    fn test_file(file: &str) {
        let file = read(file).unwrap();

        let cartridge = cartridge::NROM::new_256(
            cartridge::Mirroring::Horizontal,
            [0; 0x2000],
            file[16..32784].try_into().unwrap(),
        );
        let mut cpu = cpu::CPU::new(
            cartridge,
            controller::Controllers::disconnected(),
            ppu::FastPPU::new(),
        );

        cpu.reset();

        let mut started = false;

        loop {
            cpu.instruction();

            if !started && cpu.read(0x6000) == 0x80 {
                started = true;
            }

            if started && cpu.read(0x6000) != 0x80 {
                break;
            }

            if cpu.cycle() > 10_000_000 {
                println!("forcefully halted");
                break;
            }
        }

        let mut read = 0x6004;
        while cpu.read(read) != 0 && read < 0x8000 {
            print!("{}", char::from(cpu.read(read)));
            read += 1;
        }

        assert_eq!(cpu.read(read), 0);
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

fn open_file(
    path: &str,
    controllers: controller::Controllers,
    ppu: impl ppu::PPU + 'static,
) -> cpu::CPU {
    let file = read(path).unwrap();

    let chr_start = 16 + 0x4000 * usize::from(file[4]);
    let chr_end = chr_start + 0x2000;

    let chr = match file[5] {
        0 => [0; 0x2000],
        1 => file[chr_start..chr_end].try_into().unwrap(),
        _ => panic!(),
    };

    let mirroring = if file[6] & 1 == 0 {
        cartridge::Mirroring::Horizontal
    } else {
        cartridge::Mirroring::Vertical
    };

    let cartridge = match file[4] {
        1 => cartridge::NROM::new_128(mirroring, chr, file[16..chr_start].try_into().unwrap()),
        2 => cartridge::NROM::new_256(mirroring, chr, file[16..chr_start].try_into().unwrap()),
        _ => panic!(),
    };

    cpu::CPU::new(cartridge, controllers, ppu)
}

fn main() {
    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    let window = video_subsystem
        .window("fastnes", 256, 240)
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();

    canvas.set_draw_color(Color::RGB(0, 255, 255));
    canvas.clear();
    canvas.present();

    let (controllers, input) = controller::Controllers::standard();
    let mut cpu = open_file("rom/dk.nes", controllers, ppu::FastPPU::new());
    cpu.reset();

    let mut cycle_target = 0;
    let mut event_pump = sdl_context.event_pump().unwrap();

    let ttf_context = sdl2::ttf::init().unwrap();
    let font = ttf_context.load_font("test/pixeloid.ttf", 14).unwrap();

    let texture_creator = canvas.texture_creator();

    'running: loop {
        canvas.clear();

        let frame = cpu.frame();

        for screen_y in 0..240 {
            for screen_x in 0..256 {
                let color = frame[screen_x + screen_y * 256];
                canvas.set_draw_color(Color::RGB(color.r, color.g, color.b));
                canvas
                    .draw_point(Point::new(
                        screen_x.try_into().unwrap(),
                        screen_y.try_into().unwrap(),
                    ))
                    .unwrap();
            }
        }

        cycle_target += 29780;
        while cpu.cycle() < cycle_target {
            cpu.instruction();
        }

        let op = cpu.read(cpu.PC);
        let surface = font
            .render(format!("PC: ${:04X} OP:{:02X}", cpu.PC, op).as_str())
            .blended(Color::RGBA(255, 0, 0, 255))
            .unwrap();
        let texture = texture_creator
            .create_texture_from_surface(&surface)
            .unwrap();

        let TextureQuery { width, height, .. } = texture.query();

        canvas
            .copy(&texture, None, Some(Rect::new(0, 0, width, height)))
            .unwrap();

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } => break 'running,
                Event::KeyDown {
                    keycode: Some(Keycode::Z),
                    ..
                } => {
                    input.set(input.get() | 0b00000001);
                }
                Event::KeyDown {
                    keycode: Some(Keycode::X),
                    ..
                } => {
                    input.set(input.get() | 0b00000010);
                }
                Event::KeyDown {
                    keycode: Some(Keycode::RShift),
                    ..
                } => {
                    input.set(input.get() | 0b00000100);
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Return),
                    ..
                } => {
                    input.set(input.get() | 0b00001000);
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Up),
                    ..
                } => {
                    input.set(input.get() | 0b00010000);
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Down),
                    ..
                } => {
                    input.set(input.get() | 0b00100000);
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Left),
                    ..
                } => {
                    input.set(input.get() | 0b01000000);
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Right),
                    ..
                } => {
                    input.set(input.get() | 0b10000000);
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Z),
                    ..
                } => {
                    input.set(input.get() & 0b11111110);
                }
                Event::KeyUp {
                    keycode: Some(Keycode::X),
                    ..
                } => {
                    input.set(input.get() & 0b11111101);
                }
                Event::KeyUp {
                    keycode: Some(Keycode::RShift),
                    ..
                } => {
                    input.set(input.get() & 0b11111011);
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Return),
                    ..
                } => {
                    input.set(input.get() & 0b11110111);
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Up),
                    ..
                } => {
                    input.set(input.get() & 0b11101111);
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Down),
                    ..
                } => {
                    input.set(input.get() & 0b11011111);
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Left),
                    ..
                } => {
                    input.set(input.get() & 0b10111111);
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Right),
                    ..
                } => {
                    input.set(input.get() & 0b01111111);
                }
                _ => (),
            }
        }

        canvas.present();
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }
}
