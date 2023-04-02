#![allow(incomplete_features)]
#![feature(test)]
#![feature(adt_const_params)]
#![feature(generic_const_exprs)]
#![feature(const_mut_refs)]
#![feature(is_some_and)]

use rand::Rng;

use sdl2::event::Event;
use sdl2::pixels::Color;
use sdl2::rect::Point;

use std::fs::read;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread;
use std::time::Duration;

extern crate rand;
extern crate test;

mod cartridge;
mod controller;
mod cpu;
mod emupool;
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
        let mut cpu = open_file(
            file,
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

    #[test]
    fn sprite_hit_basics() {
        test_file("test/ppu_sprite_hit/01-basics.nes");
    }

    #[test]
    fn sprite_hit_alignment() {
        test_file("test/ppu_sprite_hit/02-alignment.nes");
    }

    #[test]
    fn sprite_hit_corners() {
        test_file("test/ppu_sprite_hit/03-corners.nes");
    }

    #[test]
    fn sprite_hit_flip() {
        test_file("test/ppu_sprite_hit/04-flip.nes");
    }

    #[test]
    fn sprite_hit_left_clip() {
        test_file("test/ppu_sprite_hit/05-left_clip.nes");
    }

    #[test]
    fn sprite_hit_right_edge() {
        test_file("test/ppu_sprite_hit/06-right_edge.nes");
    }

    #[test]
    fn sprite_hit_screen_bottom() {
        test_file("test/ppu_sprite_hit/07-screen_bottom.nes");
    }

    #[test]
    fn sprite_hit_double_height() {
        test_file("test/ppu_sprite_hit/08-double_height.nes");
    }

    #[test]
    fn sprite_hit_timing() {
        test_file("test/ppu_sprite_hit/09-timing.nes");
    }

    #[test]
    fn sprite_hit_timing_order() {
        test_file("test/ppu_sprite_hit/10-timing_order.nes");
    }
}

pub fn open_file(
    path: &str,
    controllers: controller::Controllers,
    ppu: impl ppu::PPU + 'static + Send,
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

    let mut cpu = cpu::CPU::new(cartridge, controllers, ppu);
    cpu.reset();
    cpu
}

fn main() {
    let (tx_frame, rx_frame) = mpsc::channel();
    let (tx_inputs, rx_inputs) = mpsc::channel();

    let frame_request = Arc::new(AtomicBool::new(false));

    let tx_framed_cloned = tx_frame.clone();
    let frame_request_cloned = Arc::clone(&frame_request);
    thread::spawn(move || -> ! {
        let mut input_lists: Vec<Vec<u8>> = Vec::new();
        let marios = 16;

        let single_mario_run = 10;

        for _ in 0..marios {
            input_lists.push(vec![
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0b00001000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ]);
        }

        let mut generation = 0;
        let mut last_best = 0;

        let pool = emupool::EmuPool::new(8, "rom/smb.nes", single_mario_run * 2);
        let mut last_revert = 1;
        let mut reverts = 1;

        loop {
            let mut results = Vec::new();
            for _ in 0..marios {
                results.push((false, 0));
            }

            // train marios
            for run in 0..single_mario_run {
                let (tx, rx) = mpsc::channel();

                for mario in 0..marios {
                    if run > 0 {
                        // add random inputs
                        if mario == 0 {
                            // mario 0 always repeats
                            let last = *input_lists[mario].last().unwrap();
                            input_lists[mario].push(last);
                        } else if mario <= marios / 2 {
                            // random movement
                            let input: u8 = rand::thread_rng().gen();
                            input_lists[mario].push(input & 0b11110011);
                        } else {
                            // flip one bit from last input
                            let last = *input_lists[mario].last().unwrap();
                            let input: u8 = rand::thread_rng().gen_range(0..8);
                            input_lists[mario].push(last ^ ((1 << input) & 0b11110011));
                        }
                    }

                    let inputs = input_lists[mario].clone();
                    let tx = tx.clone();

                    let frame_request = Arc::clone(&frame_request_cloned);
                    let tx_frame = tx_framed_cloned.clone();
                    pool.run(move |cpu, input| {
                        if frame_request
                            .compare_exchange_weak(
                                true,
                                false,
                                Ordering::Relaxed,
                                Ordering::Relaxed,
                            )
                            .is_ok()
                        {
                            tx_frame.send(Box::new(cpu.frame())).unwrap();
                        }

                        if cpu.frame_no() < inputs.len() {
                            input.store(inputs[cpu.frame_no()], Ordering::Relaxed);
                            true
                        } else {
                            let mut mario_position: u32 = u32::from(cpu.read(0x075f)) << 24 // world number
                    | u32::from(cpu.read(0x0760)) << 16 // area number
                    | u32::from(cpu.read(0x6d)) << 8 // player page
                    | u32::from(cpu.read(0x86)); // player x

                            let mut score: u32 = 0;
                            for i in 0..=5 {
                                let digit = i;
                                score *= 10;
                                score += u32::from(cpu.read(0x07dd + digit));
                            }

                            let mode = cpu.read(0x0770);

                            let mario_y =
                                u16::from(cpu.read(0xb5)) << 8 | u16::from(cpu.read(0xce));
                            if mario_y > 456
                                || cpu.read(0x0e) == 6
                                || cpu.read(0x0e) == 11
                                || mode == 0
                                || mode == 3
                            {
                                mario_position = 0;
                                score = 0;
                            }

                            let fitness: u64 = u64::from(score) << 32 | u64::from(mario_position);
                            let cutscene = cpu.read(0x0e) <= 7 || mode == 2;
                            tx.send((mario, fitness, cutscene)).unwrap();
                            false
                        }
                    });
                }

                for _ in 0..marios {
                    let (mario, fitness, cutscene) = rx.recv().unwrap();
                    if run == 0 {
                        results[mario] = (cutscene, fitness);
                    } else {
                        let last_result = results[mario].1;
                        if fitness >= last_result || cutscene {
                            // this is better
                            results[mario] = (cutscene, fitness);
                        } else {
                            // this is worse
                            input_lists[mario].pop();
                        }
                    }
                }
            }

            // get best mario
            {
                let positions: Vec<_> = results
                    .iter()
                    .map(|x| (x.1 as u16, (x.1 >> 16) as u8, (x.1 >> 24) as u8))
                    .collect();
                println!("gen {}: {:?}", generation, positions);
            }

            let successful = results
                .iter()
                .enumerate()
                .max_by_key(|(_, x)| *x)
                .unwrap()
                .0;
            let best = input_lists[successful].clone();

            // play back most successful version
            tx_inputs.send(best.clone()).unwrap();

            // check for any improvement
            if results.iter().all(|x| x.1 <= last_best && !x.0) {
                last_best = 0;

                if pool.frame_no() == last_revert {
                    reverts += 1;
                    if reverts > 20 {
                        reverts = 20;
                    }
                } else {
                    reverts = 1;
                    last_revert = pool.frame_no();
                }

                // reset last 2 inputs
                for inputs in input_lists.iter_mut() {
                    for _ in 0..single_mario_run * reverts {
                        inputs.pop();
                    }
                }

                for _ in 0..reverts {
                    pool.revert();
                }
            } else {
                last_best = results[successful].1;

                // mario hivemind
                for mario in 0..marios {
                    input_lists[mario] = best.clone();
                }

                pool.run_save(move |cpu, input| {
                    if cpu.frame_no() < best.len() - 1 {
                        input.store(best[cpu.frame_no()], Ordering::Relaxed);
                        true
                    } else {
                        false
                    }
                });
            }

            generation += 1;
        }
    });

    thread::spawn(move || {
        let (controllers, input) = controller::Controllers::standard();
        let mut cpu = open_file("rom/smb.nes", controllers, ppu::FastPPU::new());

        let mut last: Vec<u8> = vec![];

        loop {
            let mut best = rx_inputs.recv().unwrap();
            while let Ok(inputs) = rx_inputs.try_recv() {
                best = inputs;
            }

            cpu.reset();

            let mut skip = true;
            for frame in 0..best.len() {
                if frame >= last.len() || last[frame] != best[frame] {
                    skip = false;
                }

                input.store(best[frame], Ordering::Relaxed);

                while cpu.frame_no() == frame {
                    cpu.instruction();
                }

                if !skip {
                    tx_frame.send(Box::new(cpu.frame())).unwrap();
                    ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
                }
            }

            last = best;
        }
    });

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

    let mut event_pump = sdl_context.event_pump().unwrap();

    'running: loop {
        canvas.clear();

        let frame = match rx_frame.try_recv() {
            Ok(frame) => frame,
            Err(_) => {
                frame_request.store(true, Ordering::Relaxed);
                rx_frame.recv().unwrap()
            }
        };

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

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } => break 'running,
                _ => (),
            }
        }

        canvas.present();
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }
}
