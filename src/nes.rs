use std::fs::read;

use crate::{
    cart::{Cartridge, CartridgeEnum, Mirroring, NROM},
    cpu::CPU,
    input::Controllers,
    ppu::{DrawOptions, Frame, PPU},
};

pub struct NES<C: Cartridge, P: PPU> {
    // memory
    ram_internal: [u8; 0x0800],
    open: u8,
    ppu_cycle: usize,

    // components
    pub cart: C,
    pub ppu: P,
    pub controllers: Controllers,
    pub cpu: CPU,
}

impl<P: PPU> NES<CartridgeEnum, P> {
    pub fn read_ines(path: &str, controllers: Controllers, ppu: P) -> Self {
        let file = read(path).unwrap();

        let chr_start = 16 + 0x4000 * usize::from(file[4]);
        let chr_end = chr_start + 0x2000;

        let chr = match file[5] {
            0 => [0; 0x2000],
            1 => file[chr_start..chr_end].try_into().unwrap(),
            _ => panic!(),
        };

        let mirroring = if file[6] & 1 == 0 {
            Mirroring::Horizontal
        } else {
            Mirroring::Vertical
        };

        let cartridge = match file[4] {
            1 => CartridgeEnum::NROM(NROM::new_128(
                mirroring,
                chr,
                file[16..chr_start].try_into().unwrap(),
            )),
            2 => CartridgeEnum::NROM(NROM::new_256(
                mirroring,
                chr,
                file[16..chr_start].try_into().unwrap(),
            )),
            _ => panic!(),
        };

        Self::new(cartridge, controllers, ppu)
    }
}

impl<C: Cartridge, P: PPU> NES<C, P> {
    // FIXME: get rid of ppu_cycle so this can be placed in cpu.rs
    pub(crate) fn cycle(&mut self) { self.ppu_cycle += 3; }
    pub(crate) fn poll_interrupts(&mut self) {
        // TODO: irq
        if self.ppu.nmi(self.ppu_cycle) {
            self.cpu.nmi();
        }
    }

    pub fn read_internal(&self, addr: u16) -> u8 { self.ram_internal[usize::from(addr & 0x07FF)] }
    pub fn write_internal(&mut self, addr: u16, data: u8) {
        self.ram_internal[usize::from(addr & 0x07FF)] = data;
    }
    pub fn read(&mut self, addr: u16) -> u8 {
        self.open = match addr & 0xE000 {
            0x0000 => self.read_internal(addr),
            0x2000 => self.ppu.read(self.ppu_cycle, addr, &self.cart),
            0x4000 => self.read_apu(addr),
            0x6000 => self.cart.read_prg_ram(addr).unwrap_or(self.open),
            _ => self.cart.read_prg_rom(addr).unwrap_or(self.open),
        };
        self.open
    }
    pub fn write(&mut self, addr: u16, data: u8) {
        match addr & 0xE000 {
            0x0000 => self.write_internal(addr, data),
            0x2000 => self.ppu.write(self.ppu_cycle, addr, data, &mut self.cart),
            0x4000 => self.write_apu(addr, data),
            0x6000 => self.cart.write_prg_ram(addr, data),
            _ => self.cart.write_prg_rom(addr, data),
        }
    }

    // `read_addr` and `write_addr` need to be very short apparently for the fastest runtime
    // so we are splitting off rare code paths into seperate functions that must not be inlined
    #[inline(never)]
    fn read_apu(&mut self, addr: u16) -> u8 {
        match addr {
            0x4016 => self.controllers.read_left(self.open),
            0x4017 => self.controllers.read_right(self.open),
            _ => {
                // TODO: APU
                // todo!("read ${:04X}", addr),
                self.open
            }
        }
    }

    // `read_addr` and `write_addr` need to be very short apparently for the fastest runtime
    // so we are splitting off rare code paths into seperate functions that must not be inlined
    #[inline(never)]
    fn write_apu(&mut self, addr: u16, data: u8) {
        match addr {
            // OAMDMA
            0x4014 => {
                // FIXME:
                // this does not accurately emulate writing to OAMDMA with Read-Modify-Write instructions
                // these instructions write twice, and OAMDMA can only start on a read cycle
                // so the first write to OAMDMA should be ignored

                // halted cycle
                self.cycle();
                self.read(self.cpu.PC);

                // align cycle
                if self.cycle_number() & 1 == 0 {
                    self.cycle();
                    self.read(self.cpu.PC);
                }

                // DMA
                let high = u16::from(data) << 8;
                for low in 0..=255 {
                    // get cycle
                    self.cycle();
                    let data = self.read(high | low);

                    // put cycle
                    self.cycle();
                    self.write(0x2004, data);
                }
            }
            0x4016 => self.controllers.write(data),
            _ => {
                // TODO: APU
                // todo!("write ${:04X} = {:08b}", addr, data)
            }
        }
    }
}

impl<C: Cartridge, P: PPU> NES<C, P> {
    // PUBLIC INTERFACE
    pub fn next_frame(&mut self) {
        let now = self.frame_number();
        while self.frame_number() == now {
            self.instruction();
        }
    }
    pub fn reset(&mut self) {
        self.ppu_cycle = 0;
        self.cpu.reset();
        self.ppu.reset();
    }
    pub fn new(cart: C, controllers: Controllers, ppu: P) -> Self {
        NES {
            ppu_cycle: 0,
            ppu,
            controllers,
            cart,
            ram_internal: [0; 0x0800],
            open: 0,
            cpu: CPU::new(),
        }
    }
    pub fn cycle_number(&self) -> usize { self.ppu_cycle / 3 }
    pub fn draw_frame(&self, options: DrawOptions) -> Frame {
        self.ppu.draw_frame(&self.cart, options)
    }
    pub fn frame_number(&mut self) -> usize { self.ppu.frame_number(self.ppu_cycle) }
}
