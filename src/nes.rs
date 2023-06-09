use std::fs::read;

use crate::{
    cart::{Cartridge, Mirroring, NROM},
    cpu::INSTRUCTIONS,
    input::Controllers,
    ppu::{Color, DrawOptions, PPU},
};

pub(crate) type DynCartridge = Option<Box<dyn Cartridge + Send>>;

#[allow(non_snake_case)]
pub struct NES {
    // memory
    rom_cache: [u8; 0x8000],
    ram_internal: [u8; 0x0800],
    open: u8,

    // registers
    pub(crate) PC: u16,
    pub(crate) SP: u8,
    pub(crate) P: u8,
    pub(crate) A: u8,
    pub(crate) X: u8,
    pub(crate) Y: u8,

    // interrupts
    pub(crate) irq_sample: bool,
    pub(crate) nmi_sample: bool,
    pub(crate) res_sample: bool,
    pub(crate) hardware_interrupt: bool,

    // cartridge
    cart: DynCartridge,

    // ppu
    ppu_cycle: usize,
    ppu: Box<dyn PPU + Send>,

    // controllers
    controllers: Controllers,
}

impl Clone for NES {
    fn clone(&self) -> Self {
        Self {
            rom_cache: self.rom_cache,
            ram_internal: self.ram_internal,
            open: self.open,
            PC: self.PC,
            SP: self.SP,
            P: self.P,
            A: self.A,
            X: self.X,
            Y: self.Y,
            irq_sample: self.irq_sample,
            nmi_sample: self.nmi_sample,
            res_sample: self.res_sample,
            hardware_interrupt: self.hardware_interrupt,
            cart: self.cart.as_ref().map(|c| (*c).clone()),
            ppu_cycle: self.ppu_cycle,
            ppu: self.ppu.clone(),
            controllers: Controllers::disconnected(),
        }
    }
}

impl NES {
    // PUBLIC INTERFACE
    pub fn instruction(&mut self) {
        // get instruction to perform
        let op = if self.nmi_sample || self.irq_sample || self.res_sample {
            self.hardware_interrupt = true;
            self.read_pc();
            0
        } else {
            self.read_pc()
        };

        let instr = INSTRUCTIONS[usize::from(op)];
        instr(self);
    }
    pub fn next_frame(&mut self) {
        let now = self.frame_no();
        while self.frame_no() == now {
            self.instruction();
        }
    }
    pub fn reset(&mut self) {
        self.res_sample = true;
        self.ppu_cycle = 0;
        self.SP = 0;
        self.ppu.reset();
    }
    pub fn new(
        cart: impl Cartridge + 'static + Send,
        controllers: Controllers,
        ppu: impl PPU + 'static + Send,
    ) -> Self {
        NES {
            PC: 0,
            SP: 0,
            P: 0,
            A: 0,
            X: 0,
            Y: 0,
            ppu_cycle: 0,
            ram_internal: [0; 0x0800],
            ppu: Box::new(ppu),
            irq_sample: false,
            nmi_sample: false,
            res_sample: false,
            hardware_interrupt: false,
            controllers,
            open: 0,
            rom_cache: cart.cache_prg_rom(),
            cart: Some(Box::new(cart)),
        }
    }
    pub fn cycle(&self) -> usize {
        self.ppu_cycle / 3
    }
    pub fn frame(&self, options: DrawOptions) -> [Color; 61440] {
        self.ppu.frame(&self.cart, options)
    }
    pub fn frame_no(&mut self) -> usize {
        self.ppu.frame_no(self.ppu_cycle)
    }
    pub fn set_controllers(&mut self, c: Controllers) {
        self.controllers = c;
    }
    pub fn read_ines(path: &str, controllers: Controllers, ppu: impl PPU + 'static + Send) -> Self {
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
            1 => NROM::new_128(mirroring, chr, file[16..chr_start].try_into().unwrap()),
            2 => NROM::new_256(mirroring, chr, file[16..chr_start].try_into().unwrap()),
            _ => panic!(),
        };

        let mut cpu = Self::new(cartridge, controllers, ppu);
        cpu.reset();
        cpu
    }

    // HELPER FUNCTIONS

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
                self.read_addr(self.PC);

                // align cycle
                if self.cycle() & 1 == 0 {
                    self.read_addr(self.PC);
                }

                // DMA
                let high = u16::from(data) << 8;
                for low in 0..=255 {
                    // get cycle
                    let data = self.read_addr(high | low);

                    // put cycle
                    self.write_addr(0x2004, data);
                }
            }
            0x4016 => self.controllers.write(data),
            _ => {
                // TODO: APU
                // todo!("write ${:04X} = {:08b}", addr, data)
            }
        }
    }

    pub fn read(&mut self, addr: u16) -> u8 {
        self.open = match addr & 0xE000 {
            0x0000 => self.ram_internal[usize::from(addr & 0x07FF)],
            0x2000 => self.ppu.read(self.ppu_cycle, addr, &self.cart),
            0x4000 => self.read_apu(addr),
            0x6000 => self
                .cart
                .as_mut()
                .map(|c| c.read_prg_ram(addr))
                .flatten()
                .unwrap_or(self.open),
            _ => self
                .cart
                .as_ref()
                .map(|_| self.rom_cache[usize::from(addr & 0x7FFF)])
                .unwrap_or(self.open),
        };
        self.open
    }
    pub(crate) fn poll(&mut self) {
        // sample irq and nmi (nmi stays on while irq gets reset every cycle)
        // self.irq_sample = self.irq > 0;
        self.nmi_sample = self.nmi_sample || self.ppu.nmi(self.ppu_cycle);
    }
    pub(crate) fn flags(&mut self, val: u8) -> u8 {
        self.P &= 0b01111101;

        // negative flag
        self.P |= val & 0b10000000;

        // zero flag
        if val == 0 {
            self.P |= 0b00000010;
        }

        val
    }
    pub(crate) fn compare(&mut self, data: u8, reg: u8) {
        let (val, carry) = reg.overflowing_sub(data);
        self.P &= 0b11111110;
        if !carry {
            self.P |= 0b00000001;
        }
        self.flags(val);
    }

    // READ/WRITE ACTIONS
    pub(crate) fn write_addr(&mut self, addr: u16, data: u8) {
        if !self.res_sample {
            self.ppu_cycle += 3;
            match addr & 0xE000 {
                0x0000 => self.ram_internal[usize::from(addr & 0x07FF)] = data,
                0x2000 => self.ppu.write(self.ppu_cycle, addr, data, &mut self.cart),
                0x4000 => self.write_apu(addr, data),
                0x6000 => {
                    if let Some(cart) = self.cart.as_mut() {
                        cart.write_prg_ram(addr, data);
                    }
                }
                _ => {
                    if let Some(cart) = self.cart.as_mut() {
                        cart.write_prg_rom(addr, data, &mut self.rom_cache);
                    }
                }
            }
        } else {
            self.read_addr(addr);
        }
    }
    pub(crate) fn read_zeropage(&mut self, addr: u8) -> u8 {
        self.ppu_cycle += 3;
        self.ram_internal[usize::from(addr)]
    }
    pub(crate) fn write_zeropage(&mut self, addr: u8, data: u8) {
        self.ppu_cycle += 3;
        self.ram_internal[usize::from(addr)] = data;
    }
    pub(crate) fn read_addr(&mut self, addr: u16) -> u8 {
        self.ppu_cycle += 3;
        self.read(addr)
    }
    pub(crate) fn read_ptr_lo(&mut self, ptr: u8) -> u8 {
        self.read_addr(u16::from(ptr))
    }
    pub(crate) fn read_ptr_hi(&mut self, ptr: u8) -> u8 {
        self.read_addr(u16::from(ptr.wrapping_add(1)))
    }
    pub(crate) fn read_pc(&mut self) -> u8 {
        let d = self.read_addr(self.PC);
        if !self.hardware_interrupt {
            self.PC += 1;
        }
        d
    }
    pub(crate) fn poke_pc(&mut self) {
        self.read_addr(self.PC);
    }
    pub(crate) fn push(&mut self, data: u8) {
        self.ppu_cycle += 3;
        if !self.res_sample {
            self.ram_internal[usize::from(0x0100 | u16::from(self.SP))] = data;
        }
        self.SP = self.SP.wrapping_sub(1);
    }
    pub(crate) fn push_lo(&mut self) {
        self.push(self.PC as u8);
    }
    pub(crate) fn push_hi(&mut self) {
        self.push((self.PC >> 8) as u8);
    }
    pub(crate) fn pop(&mut self) -> u8 {
        let data = self.peek();
        self.SP = self.SP.wrapping_add(1);
        data
    }
    pub(crate) fn peek(&mut self) -> u8 {
        self.ppu_cycle += 3;
        self.ram_internal[usize::from(0x0100 | u16::from(self.SP))]
    }
}
