use std::{cell::RefCell, fs::read, path::Path, rc::Rc};

use crate::{Chip, Pin, Signal};

#[derive(Debug)]
#[allow(non_snake_case)]
pub struct ROM32K {
    // pins
    pub pin_A: Pin<u16>,
    pub pin_D: Pin<u8>,
    pub pin_CE: Pin<Signal>,

    // memory
    mem: [u8; 32768],
}

impl ROM32K {
    fn notify(&mut self) {
        if self.pin_CE.get() == Signal::Low {
            let addr = self.pin_A.get() & 0x7FFF;
            let data = self.mem[usize::from(addr)];
            self.pin_D.set(data);
        }
    }
}

impl Chip for ROM32K {
    fn subscribe(&self, rc: &Rc<RefCell<Self>>) {
        self.pin_A.subscribe(rc, |s, _| s.notify());
        self.pin_CE.subscribe(rc, |s, _| s.notify());
    }
}

impl ROM32K {
    pub fn new() -> Self {
        ROM32K {
            pin_A: Pin::default(),
            pin_D: Pin::default(),
            pin_CE: Pin::default(),
            mem: [0x00; 32768],
        }
    }
}

#[derive(Debug)]
#[allow(non_snake_case)]
pub struct NESTest {
    // pins
    pub pin_A: Pin<u16>,
    pub pin_D: Pin<u8>,
    pub pin_RW: Pin<Signal>,

    // memory
    ram: [u8; 2048],
    rom: [u8; 16384],
}

impl NESTest {
    pub fn set(&mut self, addr: u16, data: u8) {
        let addr = usize::from(addr);
        let val = if addr < 0x0800 {
            &mut self.ram[addr]
        } else if addr >= 0x8000 {
            &mut self.rom[addr & 0x3FFF]
        } else {
            return;
        };
        *val = data;
    }
    pub fn get(&self, addr: u16) -> u8 {
        let addr = usize::from(addr);
        let val = if addr < 0x0800 {
            &self.ram[addr]
        } else if addr >= 0x8000 {
            &self.rom[addr & 0x3FFF]
        } else {
            return 0;
        };
        *val
    }
    fn notify(&mut self) {
        let addr = usize::from(self.pin_A.get());
        let val = if addr < 0x0800 {
            &mut self.ram[addr]
        } else if addr >= 0x8000 {
            &mut self.rom[addr & 0x3FFF]
        } else {
            return;
        };

        match self.pin_RW.get() {
            Signal::High => self.pin_D.set(*val),
            Signal::Low => *val = self.pin_D.get(),
        }
    }
}

impl Chip for NESTest {
    fn subscribe(&self, rc: &Rc<RefCell<Self>>) {
        self.pin_A.subscribe(rc, |s, _| s.notify());
        self.pin_D.subscribe(rc, |s, _| s.notify());
        self.pin_RW.subscribe(rc, |s, _| s.notify());
    }
}

impl NESTest {
    pub fn from_rom(rom: [u8; 16384]) -> Self {
        NESTest {
            pin_A: Pin::default(),
            pin_D: Pin::default(),
            pin_RW: Pin::default(),

            ram: [0; 2048],
            rom,
        }
    }
    pub fn new<P: AsRef<Path>>(nes: P) -> Self {
        let mut file = read(nes).unwrap();
        let rom = &mut file[16..16400];
        rom[0xFFFC & 0x3FFF] = 0x00;
        rom[0xFFFD & 0x3FFF] = 0xC0;
        Self::from_rom(rom.try_into().unwrap())
    }
}
