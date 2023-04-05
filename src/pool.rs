use threadpool::ThreadPool;

use crate::{input::Controllers, nes::NES, ppu::FastPPU};
use std::sync::{atomic::AtomicU8, Arc, Mutex};

pub struct EmuPool {
    pool: ThreadPool,
    states: Arc<Mutex<Vec<NES>>>,
    save_every: usize,
}

impl EmuPool {
    pub fn new(workers: usize, path: &str, save_every: usize) -> Self {
        let pool = ThreadPool::new(workers);
        Self {
            // emulators: Arc::new(emulators),
            pool,
            save_every,
            states: Arc::new(Mutex::new(vec![NES::read_ines(
                path,
                Controllers::disconnected(),
                FastPPU::new(),
            )])),
        }
    }
    pub fn run(&self, mut f: impl FnMut(&mut NES, &Arc<AtomicU8>) -> bool + Send + 'static) {
        let states = Arc::clone(&self.states);
        self.pool.execute(move || {
            let (controllers, input) = Controllers::standard();

            let mut cpu = states.lock().unwrap().last().unwrap().clone();
            cpu.set_controllers(controllers);

            for frame in cpu.frame_no().. {
                if !f(&mut cpu, &input) {
                    break;
                }
                while cpu.frame_no() == frame {
                    cpu.instruction();
                }
            }
        });
    }
    pub fn revert(&self) {
        let mut vec = self.states.lock().unwrap();
        if vec.len() > 1 {
            vec.pop();
        }
    }
    pub fn run_save(&self, mut f: impl FnMut(&mut NES, &Arc<AtomicU8>) -> bool + Send + 'static) {
        let (controllers, input) = Controllers::standard();

        let mut vec = self.states.lock().unwrap();
        let mut cpu = vec.pop().unwrap();
        cpu.set_controllers(controllers);

        let start = cpu.frame_no();
        for frame in start.. {
            if !f(&mut cpu, &input) {
                break;
            }
            if frame >= start && frame % self.save_every == 0 {
                // save intermediate state
                vec.push(cpu.clone());
            }
            while cpu.frame_no() == frame {
                cpu.instruction();
            }
        }

        cpu.set_controllers(Controllers::disconnected());
    }
    pub fn frame_no(&self) -> usize {
        self.states.lock().unwrap().last_mut().unwrap().frame_no()
    }
}
