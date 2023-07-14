use std::fs::read;

use crate::{
    cart::{Cartridge, CartridgeEnum, Mirroring, NROM},
    cpu::{Bus, CPU},
    input::Controllers,
    ppu::{Color, DrawOptions, PPU},
};

pub struct NES<C: Cartridge, P: PPU> {
    nes: NESNoCPU<C, P>,
    cpu: CPU,
}

struct NESNoCPU<C: Cartridge, P: PPU> {
    // memory
    ram_internal: [u8; 0x0800],
    open: u8,

    // FIXME
    ppu_cycle: usize,

    // components
    cart: C,
    ppu: P,
    controllers: Controllers,
}

impl<C: Cartridge, P: PPU> Bus for NESNoCPU<C, P> {
    fn read_cycle(&mut self, addr: u16) -> u8 {
        self.ppu_cycle += 3;
        self.read(addr)
    }
    fn write_cycle(&mut self, addr: u16, data: u8) {
        self.ppu_cycle += 3;
        self.write(addr, data)
    }
    fn poll_interrupts(&mut self) -> (bool, bool) {
        (false, self.ppu.nmi(self.ppu_cycle))
    }
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

impl<C: Cartridge, P: PPU> NESNoCPU<C, P> {
    fn read_internal(&self, addr: u16) -> u8 {
        self.ram_internal[usize::from(addr & 0x07FF)]
    }
    fn write_internal(&mut self, addr: u16, data: u8) {
        self.ram_internal[usize::from(addr & 0x07FF)] = data;
    }
    fn read(&mut self, addr: u16) -> u8 {
        self.open = match addr & 0xE000 {
            0x0000 => self.read_internal(addr),
            0x2000 => self.ppu.read(self.ppu_cycle, addr, &self.cart),
            0x4000 => self.read_apu(addr),
            0x6000 => self.cart.read_prg_ram(addr).unwrap_or(self.open),
            _ => self.cart.read_prg_rom(addr).unwrap_or(self.open),
        };
        self.open
    }
    fn write(&mut self, addr: u16, data: u8) {
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
                // FIXME: should read from PC
                self.read_cycle(0);

                // align cycle
                if (self.ppu_cycle / 3) & 1 == 0 {
                    // FIXME: should read from PC
                    self.read_cycle(0);
                }

                // DMA
                let high = u16::from(data) << 8;
                for low in 0..=255 {
                    // get cycle
                    let data = self.read_cycle(high | low);

                    // put cycle
                    self.write_cycle(0x2004, data);
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
    pub fn instruction(&mut self) {
        self.cpu.instruction(&mut self.nes)
    }
    pub fn next_frame(&mut self) {
        let now = self.frame_number();
        while self.frame_number() == now {
            self.instruction();
        }
    }
    pub fn reset(&mut self) {
        self.nes.ppu_cycle = 0;
        self.cpu.reset();
        self.nes.ppu.reset();
    }
    pub fn new(cart: C, controllers: Controllers, ppu: P) -> Self {
        let mut nes = NES {
            nes: NESNoCPU {
                ppu_cycle: 0,
                ppu,
                controllers,
                cart,
                ram_internal: [0; 0x0800],
                open: 0,
            },
            cpu: CPU::new(),
        };
        nes.reset();
        nes
    }
    pub fn cycle_number(&self) -> usize {
        self.nes.ppu_cycle / 3
    }
    pub fn draw_frame(&self, options: DrawOptions) -> [Color; 61440] {
        self.nes.ppu.frame(&self.nes.cart, options)
    }
    pub fn frame_number(&mut self) -> usize {
        self.nes.ppu.frame_number(self.nes.ppu_cycle)
    }
    pub fn set_controllers(&mut self, c: Controllers) {
        self.nes.controllers = c;
    }
    pub fn read_internal(&self, addr: u16) -> u8 {
        self.nes.read_internal(addr)
    }
    pub fn write_internal(&mut self, addr: u16, data: u8) {
        self.nes.write_internal(addr, data)
    }
    pub fn read(&mut self, addr: u16) -> u8 {
        self.nes.read(addr)
    }
    pub fn write(&mut self, addr: u16, data: u8) {
        self.nes.write(addr, data)
    }
}
