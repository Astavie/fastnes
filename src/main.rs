#![allow(incomplete_features)]
#![feature(test)]
#![feature(adt_const_params)]
#![feature(generic_const_exprs)]
#![feature(const_mut_refs)]
#![feature(is_some_and)]

use rand::Rng;

use sdl2::event::Event;
use sdl2::pixels::Color;
use sdl2::rect::Rect;

use std::fs::read;
use std::sync::atomic::{AtomicUsize, Ordering};
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

unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    ::core::slice::from_raw_parts((p as *const T) as *const u8, ::core::mem::size_of::<T>())
}

fn main() {
    let (tx_frame, rx_frame) = mpsc::channel();
    let (tx_inputs, rx_inputs) = mpsc::channel();

    let (tx_playback, rx_playback) = mpsc::channel();

    let frame_request = Arc::new(AtomicUsize::new(0));
    let marios = 16;
    let (tx_loop, rx_loop) = mpsc::channel();
    let (tx_frame_loop, rx_frame_loop) = mpsc::channel();
    mario_trainer(
        &tx_frame,
        &frame_request,
        marios,
        &tx_inputs,
        tx_loop,
        Some(|cpu| u32::from(cpu.read(0x075f)) >= 4),
    );

    thread::spawn(move || {
        let (controllers, input) = controller::Controllers::standard();
        let mut cpu = open_file("rom/smb.nes", controllers, ppu::FastPPU::new());

        let mut last: Vec<u8> = vec![];

        loop {
            let mut best: Vec<u8> = rx_inputs.recv().unwrap();
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
                    tx_playback.send(Box::new(cpu.frame())).unwrap();
                    ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
                }
            }

            last = best;
        }
    });

    thread::spawn(move || {
        let (controllers, input) = controller::Controllers::standard();
        let mut cpu = open_file("rom/smb.nes", controllers, ppu::FastPPU::new());

        let best: Vec<u8> = rx_loop.recv().unwrap();

        loop {
            cpu.reset();

            for frame in 0..best.len() {
                input.store(best[frame], Ordering::Relaxed);

                while cpu.frame_no() == frame {
                    cpu.instruction();
                }

                tx_frame_loop.send(Box::new(cpu.frame())).unwrap();
                ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
            }
        }
    });

    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    let window = video_subsystem
        .window("fastnes", 256 * 5, 240 * 4)
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();

    canvas.set_draw_color(Color::RGB(0, 255, 255));
    canvas.clear();
    canvas.present();

    let texture_creator = canvas.texture_creator();

    let mut event_pump = sdl_context.event_pump().unwrap();
    let mut frame = 0;

    'running: loop {
        // canvas.clear();

        // get training mario frames
        let mut frames = Vec::new();
        for _ in 0..marios + 2 {
            frames.push(None);
        }

        frame += 1;
        frame_request.store(frame, Ordering::Relaxed);

        while let Ok((mario, frame)) = rx_frame.try_recv() {
            frames[mario + 1] = Some(frame);
        }

        while let Ok(frame) = rx_playback.try_recv() {
            frames[0] = Some(frame);
        }

        while let Ok(frame) = rx_frame_loop.try_recv() {
            frames[marios + 1] = Some(frame);
        }

        for (idx, frame) in frames.iter().enumerate() {
            if let Some(frame) = frame {
                let mut texture = texture_creator
                    .create_texture_streaming(Some(sdl2::pixels::PixelFormatEnum::RGB888), 256, 240)
                    .unwrap();
                texture
                    .with_lock(None, |buf, _pitch| unsafe {
                        buf.copy_from_slice(any_as_u8_slice(&(**frame)))
                    })
                    .unwrap();

                let mut start_x = 256 * idx;
                let mut start_y = 0;
                while start_x >= 256 * 5 {
                    start_x -= 256 * 4;
                    start_y += 240;
                }

                if idx == marios + 1 {
                    start_x = 0;
                    start_y = 240 * 3;
                }

                canvas
                    .copy(&texture, None, Rect::new(start_x as i32, start_y, 256, 240))
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

struct MarioPool {
    emus: emupool::EmuPool,
    input_lists: Vec<Vec<u8>>,
    last_best: u64,
    reverts: usize,
    stucks: usize,
}

fn mario_trainer(
    tx_frame: &mpsc::Sender<(usize, Box<[ppu::Color; 61440]>)>,
    frame_request: &Arc<AtomicUsize>,
    marios: usize,
    tx_inputs: &mpsc::Sender<Vec<u8>>,
    tx_loop: mpsc::Sender<Vec<u8>>,
    ender: Option<fn(&mut cpu::CPU) -> bool>,
) {
    let tx_inputs = tx_inputs.clone();
    let tx_frame = tx_frame.clone();
    let frame_request_cloned = Arc::clone(frame_request);
    thread::spawn(move || -> ! {
        let single_mario_run = 20;
        let revert_time = 20;
        let revert_after_stucks = 3;
        let max_reverts = 5;

        let mut pool = MarioPool {
            emus: emupool::EmuPool::new(12, "rom/smb.nes", revert_time),
            input_lists: vec![],
            last_best: 0,
            reverts: 1,
            stucks: 0,
        };

        for _ in 0..marios {
            pool.input_lists.push(vec![
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0b00001000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ]);
        }

        let mut frame = 0;
        let mut ended = false;

        loop {
            let request = frame_request_cloned.load(Ordering::Relaxed);
            let draw = request > frame;
            frame = request;

            let mut results = Vec::new();
            for _ in 0..marios {
                results.push((false, 0, false, false));
            }

            // train marios
            let (tx, rx) = mpsc::channel();

            for mario in 0..marios {
                for _ in 0..single_mario_run {
                    // add random inputs
                    if mario == 0 {
                        // mario 0 always repeats
                        let last = *pool.input_lists[mario].last().unwrap();
                        pool.input_lists[mario].push(last);
                    } else if mario < marios / 2 {
                        // random movement
                        let input: u8 = rand::thread_rng().gen();
                        pool.input_lists[mario].push(input & 0b11110011);
                    } else {
                        // flip one bit from last input
                        let last = *pool.input_lists[mario].last().unwrap();
                        let input: u8 = rand::thread_rng().gen_range(0..8);
                        pool.input_lists[mario].push(last ^ ((1 << input) & 0b11110011));
                    }
                }

                let inputs = pool.input_lists[mario].clone();
                let tx = tx.clone();
                let tx_frame = tx_frame.clone();
                pool.emus.run(move |cpu, input| {
                    if cpu.frame_no() < inputs.len() {
                        input.store(inputs[cpu.frame_no()], Ordering::Relaxed);
                        true
                    } else {
                        if draw {
                            tx_frame.send((mario, Box::new(cpu.frame()))).unwrap();
                        }

                        let screen_left = u16::from(cpu.read(0x071a)) << 8 // screen page
                    | u16::from(cpu.read(0x071c)); // screen x

                        let mut score: u32 = 0;
                        for i in 0..=5 {
                            let digit = i;
                            score *= 10;
                            score += u32::from(cpu.read(0x07dd + digit));
                        }

                        let mario_position: u32 = u32::from(cpu.read(0x075f)) << 24
                            | u32::from(cpu.read(0x0760)) << 16
                            | u32::from(screen_left);

                        let engine = cpu.read(0x0e);
                        let task = cpu.read(0x0772);
                        let mode = cpu.read(0x0770);

                        let mario_y = u16::from(cpu.read(0xb5)) << 8 | u16::from(cpu.read(0xce));
                        let cutscene =
                            engine <= 5 || engine == 7 || mode == 2 || (mode == 1 && task != 3);
                        let dying = (mario_y > 456
                            || engine == 6
                            || engine == 11
                            || mode == 0
                            || mode == 3)
                            && !cutscene;

                        let time = u16::from(cpu.read(0x07f8)) * 100
                            + u16::from(cpu.read(0x07f9)) * 10
                            + u16::from(cpu.read(0x07fa));

                        let out_of_time = time < 30 && !cutscene;

                        let fitness: u64 = u64::from(score) << 32 | u64::from(mario_position >> 2);

                        tx.send((
                            mario,
                            fitness,
                            cutscene,
                            dying,
                            out_of_time,
                            ender.map(|f| f(cpu)).unwrap_or(false),
                        ))
                        .unwrap();

                        false
                    }
                });
            }

            for _ in 0..marios {
                let (mario, fitness, cutscene, dying, out_of_time, end) = rx.recv().unwrap();
                if end && !ended {
                    ended = true;
                    tx_loop.send(pool.input_lists[mario].clone()).unwrap();
                }
                results[mario] = (cutscene, fitness, dying, out_of_time);
            }

            // get best mario
            let mut bests: Vec<_> = results.iter().cloned().enumerate().collect();
            bests.sort_by_key(|(_, x)| if x.2 { (false, 0) } else { (x.0, x.1) });

            let successful = bests.last().unwrap().0;
            let best = pool.input_lists[successful].clone();

            // check for any improvement
            if results.iter().all(|x| x.3) {
                pool.stucks += 400;
            } else if results
                .iter()
                .all(|x| (x.1 <= pool.last_best || x.2) && !x.0)
            {
                pool.stucks += 1;
            } else {
                pool.stucks = 0;
            }

            if pool.stucks >= revert_after_stucks || results.iter().all(|x| x.2 && !x.0) {
                pool.reverts += 1;

                // reset last inputs
                let amount = pool.reverts + pool.stucks;
                for inputs in pool.input_lists.iter_mut() {
                    for _ in 0..revert_time * amount {
                        inputs.pop();
                    }
                }

                for _ in 0..pool.reverts + pool.stucks {
                    pool.emus.revert();
                }

                if pool.reverts >= max_reverts {
                    pool.last_best = 0;
                    pool.reverts = 1;
                }

                pool.stucks = 0;
            } else {
                pool.last_best = results[successful].1;
                let best_inputs = pool.input_lists[bests.last().unwrap().0].clone();

                // mario hivemind
                for mario in 0..marios {
                    pool.input_lists[mario] = best_inputs.clone();
                }

                tx_inputs.send(best_inputs).unwrap();

                pool.emus.run_save(move |cpu, input| {
                    if cpu.frame_no() < best.len() - 1 {
                        input.store(best[cpu.frame_no()], Ordering::Relaxed);
                        true
                    } else {
                        false
                    }
                });
            }
        }
    });
}
