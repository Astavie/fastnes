#![feature(test)]
#![feature(adt_const_params)]
#![feature(generic_const_exprs)]
#![feature(const_mut_refs)]

extern crate test;

use std::{cell::RefCell, fmt::Debug, fs::read, rc::Rc};

mod cpu;
mod cpu2;
mod cpu3;
mod cpu4;
mod cpu5;
mod cpu6;
mod cpu7;

mod mem;

#[derive(Debug, Default)]
pub struct Pin<T> {
    conn: Rc<RefCell<Connection<T>>>,
}

impl Pin<Signal> {
    pub fn assert(&self) {
        self.conn.borrow_mut().hold += 1;
        self.set(Signal::Low);
    }
    pub fn release(&self) {
        let mut conn = self.conn.borrow_mut();
        conn.hold -= 1;
        if conn.hold == 0 {
            drop(conn);
            self.set(Signal::High);
        }
    }
}

#[derive(Default)]
pub struct Connection<T> {
    val: T,
    hold: u8,
    listeners: Vec<Box<dyn Fn(T)>>,
}

impl<T: Debug> Debug for Connection<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.val.fmt(f)
    }
}

impl<T: Copy + Eq> Pin<T> {
    pub fn new(val: T) -> Self {
        Pin {
            conn: Rc::new(RefCell::from(Connection {
                val,
                hold: 0,
                listeners: vec![],
            })),
        }
    }

    pub fn get(&self) -> T {
        self.conn.borrow().val
    }

    pub fn connect(&mut self, pin: &Pin<T>) {
        self.conn = Rc::clone(&pin.conn)
    }

    pub fn set(&self, val: T) {
        if self.get() == val {
            return;
        }
        self.conn.borrow_mut().val = val;

        for listener in &self.conn.borrow().listeners {
            listener(val);
        }
    }

    pub fn subscribe<P: 'static>(&self, rc: &Rc<RefCell<P>>, f: impl Fn(&mut P, T) + 'static) {
        let weak = Rc::downgrade(rc);
        self.conn.borrow_mut().listeners.push(Box::new(move |val| {
            // failure to borrow mut is usually because of some cycle
            if let Ok(mut p) = weak.upgrade().unwrap().try_borrow_mut() {
                f(&mut p, val);
            }
        }));
    }
}

pub trait Chip {
    fn subscribe(&self, rc: &Rc<RefCell<Self>>);
    fn start(self) -> Rc<RefCell<Self>>
    where
        Self: Sized,
    {
        let rc = Rc::new(RefCell::new(self));
        rc.borrow().subscribe(&rc);
        rc
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Signal {
    #[default]
    High,
    Low,
}

#[cfg(test)]
mod tests {
    use std::fs::read_to_string;

    use super::*;
    use test::Bencher;

    #[test]
    fn nestest_complete() {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let rom = &mut file[16..16400];
        rom[0xFFFC & 0x3FFF] = 0x00;
        rom[0xFFFD & 0x3FFF] = 0xC0;
        let rom: [u8; 16384] = rom.try_into().unwrap();

        let instr = cpu7::instructions();

        cpu7::nestest::<true>(rom, instr);
    }

    #[bench]
    fn bench_nestest1(b: &mut Bencher) {
        b.iter(|| cpu::nestest::<false>());
    }

    #[bench]
    fn bench_nestest2(b: &mut Bencher) {
        b.iter(|| cpu2::nestest::<false>());
    }

    #[bench]
    fn bench_nestest3(b: &mut Bencher) {
        let instr = Rc::new(cpu3::cycles());
        b.iter(|| cpu3::nestest::<false>(instr.clone()));
    }

    #[bench]
    fn bench_nestest3half(b: &mut Bencher) {
        let instr = Rc::new(cpu3::cycles());
        b.iter(|| {
            let mut mem = mem::NESTest::new("test/nestest/nestest.nes");
            let mut cpu = cpu3::MOS6502::new(instr.clone());

            cpu.reset();

            for _ in 0..26554 {
                cpu.cycle_start();
                if cpu.pin_RW.get() == Signal::High {
                    cpu.pin_D.set(mem.get(cpu.pin_A.get()));
                }
                cpu.cycle_end();
                if cpu.pin_RW.get() == Signal::Low {
                    mem.set(cpu.pin_A.get(), cpu.pin_D.get());
                }
            }
        });
    }

    #[bench]
    fn bench_nestest4(b: &mut Bencher) {
        let instr = Rc::new(cpu4::cycles());

        let mut file = read("test/nestest/nestest.nes").unwrap();
        let rom = &mut file[16..16400];
        rom[0xFFFC & 0x3FFF] = 0x00;
        rom[0xFFFD & 0x3FFF] = 0xC0;
        let rom: [u8; 16384] = rom.try_into().unwrap();

        b.iter(|| cpu4::nestest::<false>(rom, instr.clone()));
    }

    #[bench]
    fn bench_nestest5(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let rom = &mut file[16..16400];
        rom[0xFFFC & 0x3FFF] = 0x00;
        rom[0xFFFD & 0x3FFF] = 0xC0;
        let rom: [u8; 16384] = rom.try_into().unwrap();

        let instr = cpu5::instructions();

        b.iter(|| cpu5::nestest::<false>(rom, instr));
    }

    #[bench]
    fn bench_nestest6(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let rom = &mut file[16..16400];
        rom[0xFFFC & 0x3FFF] = 0x00;
        rom[0xFFFD & 0x3FFF] = 0xC0;
        let rom: [u8; 16384] = rom.try_into().unwrap();

        let instr = cpu6::instructions();

        b.iter(|| cpu6::nestest::<false>(rom, instr));
    }

    #[bench]
    fn bench_nestest7(b: &mut Bencher) {
        let mut file = read("test/nestest/nestest.nes").unwrap();
        let rom = &mut file[16..16400];
        rom[0xFFFC & 0x3FFF] = 0x00;
        rom[0xFFFD & 0x3FFF] = 0xC0;
        let rom: [u8; 16384] = rom.try_into().unwrap();

        let instr = cpu7::instructions();

        b.iter(|| cpu7::nestest::<false>(rom, instr.clone()));
    }
}

fn main() {
    let mut file = read("test/nestest/nestest.nes").unwrap();
    let rom = &mut file[16..16400];
    rom[0xFFFC & 0x3FFF] = 0x00;
    rom[0xFFFD & 0x3FFF] = 0xC0;
    let rom: [u8; 16384] = rom.try_into().unwrap();

    let instr = cpu7::instructions();

    cpu7::nestest::<false>(rom, instr);
}
