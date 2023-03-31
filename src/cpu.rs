use crate::{
    cartridge::Cartridge,
    controller::Controllers,
    instruction::{instructions, Instructions},
    ppu::{Color, PPU},
};

pub(crate) type DynCartridge = Option<Box<dyn Cartridge>>;

#[allow(non_snake_case)]
pub struct CPU {
    // instruction list
    instructions: Instructions,

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
    ppu: Box<dyn PPU>,

    // controllers
    controllers: Controllers,
}

impl CPU {
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

        let instr = self.instructions[usize::from(op)];
        instr(self);
    }
    pub fn reset(&mut self) {
        self.res_sample = true;
        self.ppu_cycle = 0;
        self.SP = 0;
        self.ppu.reset();
    }
    pub fn new(
        cart: impl Cartridge + 'static,
        controllers: Controllers,
        ppu: impl PPU + 'static,
    ) -> Self {
        CPU {
            instructions: instructions(),
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
    pub fn frame(&self) -> [Color; 61440] {
        self.ppu.frame(&self.cart)
    }

    // HELPER FUNCTIONS

    // `read_addr` and `write_addr` need to be very short apparently for the fastest runtime
    // so we are splitting off rare code paths into seperate functions that must not be inlined
    #[inline(never)]
    fn read_apu(&mut self, addr: u16) -> u8 {
        match addr {
            0x4016 => self.controllers.read_left(),
            0x4017 => self.controllers.read_right(),
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
            0x4016 => self.controllers.write(data),
            _ => {
                // TODO: APU
                // todo!("write ${:04X} = {:08b}", addr, data)
            }
        }
    }

    pub(crate) fn read(&mut self, addr: u16) -> u8 {
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
