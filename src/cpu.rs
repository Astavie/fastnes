use enumset::__internal::EnumSetTypeRepr;

use crate::{cart::Cartridge, nes::NES, ppu::PPU};

#[allow(non_snake_case)]
#[derive(Clone)]
pub struct CPU {
    // buses
    pub addr: u16,

    // registers
    pub PC: u16,
    pub SP: u8,
    pub P: u8,
    pub A: u8,
    pub X: u8,
    pub Y: u8,

    // interrupts
    irq_sample: bool,
    nmi_sample: bool,
    res_sample: bool,
    hardware_interrupt: bool,
}

// READ/WRITE
macro_rules! mode_abs {
    ($name_r: ident, $name_w: ident, $name_rw: ident, $addr: ident (w $(, $index: ident)?)) => {
        mode_full!($name_r, $name_w, $name_rw, $addr(w $(, $index)?), cycle_read, cycle_write);
    };
    ($name_r: ident, $name_w: ident, $name_rw: ident, $addr: ident ($($index: ident)?)) => {
        mode_full!($name_r, $name_w, $name_rw, $addr($($index)?), cycle_read, cycle_write);
    };
}

macro_rules! mode_zpg {
    ($name_r: ident, $name_w: ident, $name_rw: ident, $addr: ident (w $(, $index: ident)?)) => {
        mode_full!($name_r, $name_w, $name_rw, $addr(w $(, $index)?), cycle_read, cycle_write);
    };
    ($name_r: ident, $name_w: ident, $name_rw: ident, $addr: ident ($($index: ident)?)) => {
        mode_full!($name_r, $name_w, $name_rw, $addr($($index)?), cycle_read, cycle_write);
    };
}

macro_rules! mode_full {
    ($name_r: ident, $name_w: ident, $name_rw: ident, $addr: ident (w $(, $index: ident)?), $read: ident, $write: ident) => {
        fn $name_r(&mut self, f: fn(&mut CPU, u8)) {
            self.cpu.addr = u16::from(self.$addr(false $(, self.cpu.$index)?));
            self.poll_interrupts();
            let data = self.$read();
            f(&mut self.cpu, data);
        }
        fn $name_w(&mut self, f: fn(&mut CPU) -> u8) {
            let addr = u16::from(self.$addr(true $(, self.cpu.$index)?));
            self.poll_interrupts();
            let data = f(&mut self.cpu);
            self.cpu.addr = addr;
            self.$write(data);
        }
        fn $name_rw(&mut self, f: fn(&mut CPU, u8) -> u8) {
            self.cpu.addr = u16::from(self.$addr(true $(, self.cpu.$index)?));
            let data = self.$read();
            self.$write(data);
            self.poll_interrupts();
            let data = f(&mut self.cpu, data);
            self.$write(data);
        }
    };
    ($name_r: ident, $name_w: ident, $name_rw: ident, $addr: ident ($($index: ident)?), $read: ident, $write: ident) => {
        fn $name_r(&mut self, f: fn(&mut CPU, u8)) {
            self.cpu.addr = u16::from(self.$addr($(self.cpu.$index)?));
            self.poll_interrupts();
            let data = self.$read();
            f(&mut self.cpu, data);
        }
        fn $name_w(&mut self, f: fn(&mut CPU) -> u8) {
            let addr = u16::from(self.$addr($(self.cpu.$index)?));
            self.poll_interrupts();
            let data = f(&mut self.cpu);
            self.cpu.addr = addr;
            self.$write(data);
        }
        fn $name_rw(&mut self, f: fn(&mut CPU, u8) -> u8) {
            self.cpu.addr = u16::from(self.$addr($(self.cpu.$index)?));
            let data = self.$read();
            self.$write(data);
            self.poll_interrupts();
            let data = f(&mut self.cpu, data);
            self.$write(data);
        }
    };
}

impl<C: Cartridge, P: PPU> NES<C, P> {
    pub fn instruction(&mut self) {
        // get instruction to perform
        let op = if self.cpu.nmi_sample || self.cpu.irq_sample || self.cpu.res_sample {
            self.cpu.hardware_interrupt = true;
            self.cycle_read_pc();
            0
        } else {
            self.cycle_read_pc()
        };

        match op {
            0x00 => {
                // BRK
                self.cycle_read_pc();
                self.cycle_push_pc_hi();
                self.cycle_push_pc_lo();

                // poll address
                self.poll_interrupts();
                let addr = if self.cpu.nmi_sample {
                    self.cpu.nmi_sample = false;
                    0xFFFA
                } else if self.cpu.res_sample {
                    self.cpu.res_sample = false;
                    0xFFFC
                } else {
                    0xFFFE
                };

                let b = if self.cpu.hardware_interrupt {
                    // B flag clear
                    0b00100000
                } else {
                    // B flag set
                    0b00110000
                };
                self.cycle_push(self.cpu.P | b);

                // interrupt disable
                self.cpu.P |= 0b00000100;

                self.cpu.addr = addr;
                let lo = self.cycle_read();
                self.cpu.addr |= 1;
                let hi = self.cycle_read();

                let addr = u16::from_le_bytes([lo, hi]);
                self.cpu.PC = addr;
                self.cpu.hardware_interrupt = false;

                // no polling interrupts
                // immediately execute next instruction
            }
            0x4C => {
                // JMP abs
                let lo = self.cycle_read_pc();

                self.poll_interrupts();

                let hi = self.cycle_read_pc();
                let addr = u16::from_le_bytes([lo, hi]);
                self.cpu.PC = addr;
            }
            0x6C => {
                // JMP ind
                let lo = self.cycle_read_pc();
                let hi = self.cycle_read_pc();
                let addr_lo = u16::from_le_bytes([lo, hi]);
                let addr_hi = u16::from_le_bytes([lo.wrapping_add(1), hi]);

                self.cpu.addr = addr_lo;
                let lo = self.cycle_read();

                self.poll_interrupts();

                self.cpu.addr = addr_hi;
                let hi = self.cycle_read();
                let addr = u16::from_le_bytes([lo, hi]);
                self.cpu.PC = addr;
            }
            0x20 => {
                // JSR
                let lo = self.cycle_read_pc();
                self.cycle_peek();
                self.cycle_push_pc_hi();
                self.cycle_push_pc_lo();

                self.poll_interrupts();

                let hi = self.cycle_read_pc();
                let addr = u16::from_le_bytes([lo, hi]);
                self.cpu.PC = addr;
            }
            0x60 => {
                // RTS
                self.cycle_poke_pc();
                self.cycle_pop();
                let lo = self.cycle_pop();
                let hi = self.cycle_peek();
                let addr = u16::from_le_bytes([lo, hi]);
                self.cpu.PC = addr;

                self.poll_interrupts();

                self.cycle_read_pc();
            }
            0x40 => {
                // RTI
                self.cycle_poke_pc();
                self.cycle_pop();
                self.cpu.P = self.cycle_pop() & 0b11001111;
                let lo = self.cycle_pop();

                self.poll_interrupts();

                let hi = self.cycle_peek();
                let addr = u16::from_le_bytes([lo, hi]);
                self.cpu.PC = addr;
            }

            // NOP
            0xEA => self.imp(|_| ()),

            // STACK
            0x48 => self.imp_push(|cpu| cpu.A),
            0x08 => self.imp_push(|cpu| cpu.P | 0b00110000),
            0x68 => self.imp_pop(|cpu, data| cpu.A = cpu.flags(data)),
            0x28 => self.imp_pop(|cpu, data| cpu.P = data & 0b11001111),

            // TRANSFER
            0xAA => self.imp(|cpu| cpu.X = cpu.flags(cpu.A)),
            0xA8 => self.imp(|cpu| cpu.Y = cpu.flags(cpu.A)),
            0xBA => self.imp(|cpu| cpu.X = cpu.flags(cpu.SP)),
            0x8A => self.imp(|cpu| cpu.A = cpu.flags(cpu.X)),
            0x9A => self.imp(|cpu| cpu.SP = cpu.X),
            0x98 => self.imp(|cpu| cpu.A = cpu.flags(cpu.Y)),

            // INC
            0xE8 => self.imp(|cpu| cpu.X = cpu.flags(cpu.X.wrapping_add(1))),
            0xC8 => self.imp(|cpu| cpu.Y = cpu.flags(cpu.Y.wrapping_add(1))),

            0xE6 => self.zpg_rw(CPU::INC),
            0xF6 => self.zpg_x_rw(CPU::INC),
            0xEE => self.abs_rw(CPU::INC),
            0xFE => self.abs_x_rw(CPU::INC),

            0xCA => self.imp(|cpu| cpu.X = cpu.flags(cpu.X.wrapping_sub(1))),
            0x88 => self.imp(|cpu| cpu.Y = cpu.flags(cpu.Y.wrapping_sub(1))),

            0xC6 => self.zpg_rw(CPU::DEC),
            0xD6 => self.zpg_x_rw(CPU::DEC),
            0xCE => self.abs_rw(CPU::DEC),
            0xDE => self.abs_x_rw(CPU::DEC),

            // SHIFT
            0x0A => self.acc(CPU::ASL),
            0x06 => self.zpg_rw(CPU::ASL),
            0x16 => self.zpg_x_rw(CPU::ASL),
            0x0E => self.abs_rw(CPU::ASL),
            0x1E => self.abs_x_rw(CPU::ASL),

            0x4A => self.acc(CPU::LSR),
            0x46 => self.zpg_rw(CPU::LSR),
            0x56 => self.zpg_x_rw(CPU::LSR),
            0x4E => self.abs_rw(CPU::LSR),
            0x5E => self.abs_x_rw(CPU::LSR),

            0x2A => self.acc(CPU::ROL),
            0x26 => self.zpg_rw(CPU::ROL),
            0x36 => self.zpg_x_rw(CPU::ROL),
            0x2E => self.abs_rw(CPU::ROL),
            0x3E => self.abs_x_rw(CPU::ROL),

            0x6A => self.acc(CPU::ROR),
            0x66 => self.zpg_rw(CPU::ROR),
            0x76 => self.zpg_x_rw(CPU::ROR),
            0x6E => self.abs_rw(CPU::ROR),
            0x7E => self.abs_x_rw(CPU::ROR),

            // COMPARE
            0xC9 => self.imm(CPU::CMP),
            0xC5 => self.zpg_read(CPU::CMP),
            0xD5 => self.zpg_x_read(CPU::CMP),
            0xCD => self.abs_read(CPU::CMP),
            0xDD => self.abs_x_read(CPU::CMP),
            0xD9 => self.abs_y_read(CPU::CMP),
            0xC1 => self.x_ind_read(CPU::CMP),
            0xD1 => self.ind_y_read(CPU::CMP),

            0xE0 => self.imm(CPU::CPX),
            0xE4 => self.zpg_read(CPU::CPX),
            0xEC => self.abs_read(CPU::CPX),

            0xC0 => self.imm(CPU::CPY),
            0xC4 => self.zpg_read(CPU::CPY),
            0xCC => self.abs_read(CPU::CPY),

            // LOAD/STORE
            0xA9 => self.imm(CPU::LDA),
            0xA5 => self.zpg_read(CPU::LDA),
            0xB5 => self.zpg_x_read(CPU::LDA),
            0xAD => self.abs_read(CPU::LDA),
            0xBD => self.abs_x_read(CPU::LDA),
            0xB9 => self.abs_y_read(CPU::LDA),
            0xA1 => self.x_ind_read(CPU::LDA),
            0xB1 => self.ind_y_read(CPU::LDA),

            0x85 => self.zpg_write(CPU::STA),
            0x95 => self.zpg_x_write(CPU::STA),
            0x8D => self.abs_write(CPU::STA),
            0x9D => self.abs_x_write(CPU::STA),
            0x99 => self.abs_y_write(CPU::STA),
            0x81 => self.x_ind_write(CPU::STA),
            0x91 => self.ind_y_write(CPU::STA),

            0xA2 => self.imm(CPU::LDX),
            0xA6 => self.zpg_read(CPU::LDX),
            0xB6 => self.zpg_y_read(CPU::LDX),
            0xAE => self.abs_read(CPU::LDX),
            0xBE => self.abs_y_read(CPU::LDX),

            0x86 => self.zpg_write(CPU::STX),
            0x96 => self.zpg_y_write(CPU::STX),
            0x8E => self.abs_write(CPU::STX),

            0xA0 => self.imm(CPU::LDY),
            0xA4 => self.zpg_read(CPU::LDY),
            0xB4 => self.zpg_x_read(CPU::LDY),
            0xAC => self.abs_read(CPU::LDY),
            0xBC => self.abs_x_read(CPU::LDY),

            0x84 => self.zpg_write(CPU::STY),
            0x94 => self.zpg_x_write(CPU::STY),
            0x8C => self.abs_write(CPU::STY),

            // BITWISE
            0x29 => self.imm(CPU::AND),
            0x25 => self.zpg_read(CPU::AND),
            0x35 => self.zpg_x_read(CPU::AND),
            0x2D => self.abs_read(CPU::AND),
            0x3D => self.abs_x_read(CPU::AND),
            0x39 => self.abs_y_read(CPU::AND),
            0x21 => self.x_ind_read(CPU::AND),
            0x31 => self.ind_y_read(CPU::AND),

            0x09 => self.imm(CPU::ORA),
            0x05 => self.zpg_read(CPU::ORA),
            0x15 => self.zpg_x_read(CPU::ORA),
            0x0D => self.abs_read(CPU::ORA),
            0x1D => self.abs_x_read(CPU::ORA),
            0x19 => self.abs_y_read(CPU::ORA),
            0x01 => self.x_ind_read(CPU::ORA),
            0x11 => self.ind_y_read(CPU::ORA),

            0x49 => self.imm(CPU::EOR),
            0x45 => self.zpg_read(CPU::EOR),
            0x55 => self.zpg_x_read(CPU::EOR),
            0x4D => self.abs_read(CPU::EOR),
            0x5D => self.abs_x_read(CPU::EOR),
            0x59 => self.abs_y_read(CPU::EOR),
            0x41 => self.x_ind_read(CPU::EOR),
            0x51 => self.ind_y_read(CPU::EOR),

            0x24 => self.zpg_read(CPU::BIT),
            0x2C => self.abs_read(CPU::BIT),

            // ADD/SUB
            0x69 => self.imm(CPU::ADC),
            0x65 => self.zpg_read(CPU::ADC),
            0x75 => self.zpg_x_read(CPU::ADC),
            0x6D => self.abs_read(CPU::ADC),
            0x7D => self.abs_x_read(CPU::ADC),
            0x79 => self.abs_y_read(CPU::ADC),
            0x61 => self.x_ind_read(CPU::ADC),
            0x71 => self.ind_y_read(CPU::ADC),

            0xE9 => self.imm(CPU::SBC),
            0xE5 => self.zpg_read(CPU::SBC),
            0xF5 => self.zpg_x_read(CPU::SBC),
            0xED => self.abs_read(CPU::SBC),
            0xFD => self.abs_x_read(CPU::SBC),
            0xF9 => self.abs_y_read(CPU::SBC),
            0xE1 => self.x_ind_read(CPU::SBC),
            0xF1 => self.ind_y_read(CPU::SBC),

            // FLAGS
            0x18 => self.imp(|cpu| cpu.P &= 0b11111110),
            0x38 => self.imp(|cpu| cpu.P |= 0b00000001),
            0x58 => self.imp(|cpu| cpu.P &= 0b11111011),
            0x78 => self.imp(|cpu| cpu.P |= 0b00000100),
            0xB8 => self.imp(|cpu| cpu.P &= 0b10111111),
            0xD8 => self.imp(|cpu| cpu.P &= 0b11110111),
            0xF8 => self.imp(|cpu| cpu.P |= 0b00001000),

            0x10 => self.branch(|p| p & 0b10000000 == 0),
            0x30 => self.branch(|p| p & 0b10000000 != 0),
            0x50 => self.branch(|p| p & 0b01000000 == 0),
            0x70 => self.branch(|p| p & 0b01000000 != 0),
            0x90 => self.branch(|p| p & 0b00000001 == 0),
            0xB0 => self.branch(|p| p & 0b00000001 != 0),
            0xD0 => self.branch(|p| p & 0b00000010 == 0),
            0xF0 => self.branch(|p| p & 0b00000010 != 0),

            // ILLEGAL
            0x1A | 0x3A | 0x5A | 0x7A | 0xDA | 0xFA => self.imp(|_| ()),
            0x80 | 0x82 | 0x89 | 0xC2 | 0xE2 => self.imm(|_, _| ()),
            0x04 | 0x44 | 0x64 => self.zpg_read(|_, _| ()),
            0x14 | 0x34 | 0x54 | 0x74 | 0xD4 | 0xF4 => self.zpg_x_read(|_, _| ()),
            0x0C => self.abs_read(|_, _| ()),
            0x1C | 0x3C | 0x5C | 0x7C | 0xDC | 0xFC => self.abs_x_read(|_, _| ()),

            0xA7 => self.zpg_read(CPU::LAX),
            0xB7 => self.zpg_y_read(CPU::LAX),
            0xAF => self.abs_read(CPU::LAX),
            0xBF => self.abs_y_read(CPU::LAX),
            0xA3 => self.x_ind_read(CPU::LAX),
            0xB3 => self.ind_y_read(CPU::LAX),

            0x87 => self.zpg_write(CPU::SAX),
            0x97 => self.zpg_y_write(CPU::SAX),
            0x8F => self.abs_write(CPU::SAX),
            0x83 => self.x_ind_write(CPU::SAX),

            0xEB => self.imm(CPU::SBC),

            0xC7 => self.zpg_rw(CPU::DCP),
            0xD7 => self.zpg_x_rw(CPU::DCP),
            0xCF => self.abs_rw(CPU::DCP),
            0xDF => self.abs_x_rw(CPU::DCP),
            0xDB => self.abs_y_rw(CPU::DCP),
            0xC3 => self.x_ind_rw(CPU::DCP),
            0xD3 => self.ind_y_rw(CPU::DCP),

            0xE7 => self.zpg_rw(CPU::ISC),
            0xF7 => self.zpg_x_rw(CPU::ISC),
            0xEF => self.abs_rw(CPU::ISC),
            0xFF => self.abs_x_rw(CPU::ISC),
            0xFB => self.abs_y_rw(CPU::ISC),
            0xE3 => self.x_ind_rw(CPU::ISC),
            0xF3 => self.ind_y_rw(CPU::ISC),

            0x07 => self.zpg_rw(CPU::SLO),
            0x17 => self.zpg_x_rw(CPU::SLO),
            0x0F => self.abs_rw(CPU::SLO),
            0x1F => self.abs_x_rw(CPU::SLO),
            0x1B => self.abs_y_rw(CPU::SLO),
            0x03 => self.x_ind_rw(CPU::SLO),
            0x13 => self.ind_y_rw(CPU::SLO),

            0x27 => self.zpg_rw(CPU::RLA),
            0x37 => self.zpg_x_rw(CPU::RLA),
            0x2F => self.abs_rw(CPU::RLA),
            0x3F => self.abs_x_rw(CPU::RLA),
            0x3B => self.abs_y_rw(CPU::RLA),
            0x23 => self.x_ind_rw(CPU::RLA),
            0x33 => self.ind_y_rw(CPU::RLA),

            0x47 => self.zpg_rw(CPU::SRE),
            0x57 => self.zpg_x_rw(CPU::SRE),
            0x4F => self.abs_rw(CPU::SRE),
            0x5F => self.abs_x_rw(CPU::SRE),
            0x5B => self.abs_y_rw(CPU::SRE),
            0x43 => self.x_ind_rw(CPU::SRE),
            0x53 => self.ind_y_rw(CPU::SRE),

            0x67 => self.zpg_rw(CPU::RRA),
            0x77 => self.zpg_x_rw(CPU::RRA),
            0x6F => self.abs_rw(CPU::RRA),
            0x7F => self.abs_x_rw(CPU::RRA),
            0x7B => self.abs_y_rw(CPU::RRA),
            0x63 => self.x_ind_rw(CPU::RRA),
            0x73 => self.ind_y_rw(CPU::RRA),

            0x0B | 0x2B => self.imm(CPU::ANC),
            0x4B => self.imm(CPU::ALR),
            0x6B => self.imm(CPU::ARR),
            0xBB => self.abs_y_read(CPU::LAS),
            0xCB => self.imm(CPU::SBX),

            // UNSTABLE ILLEGAL
            0x9F => self.abs_y_write_unstable(CPU::SHA),
            0x93 => self.ind_y_write_unstable(CPU::SHA),
            0x9E => self.abs_y_write_unstable(CPU::SHX),
            0x9C => self.abs_x_write_unstable(CPU::SHY),
            0x9B => self.abs_y_write_unstable(CPU::TAS),

            // HIGHLY UNSTABLE ILLEGAL
            0x8B => self.imm(CPU::ANE),
            0xAB => self.imm(CPU::LXA),

            _ => todo!("instr 0x{:02X}", op),
        }
    }

    // ADDRESSING
    #[inline(always)]
    fn zpg(&mut self) -> u8 {
        // just the same as a single read cycle
        self.cycle_read_pc()
    }
    #[inline(always)]
    fn zpg_index(&mut self, index: u8) -> u8 {
        let addr = self.cycle_read_pc();
        self.cycle();
        addr.wrapping_add(index)
    }
    #[inline(always)]
    fn abs(&mut self) -> u16 {
        let lo = self.cycle_read_pc();
        let hi = self.cycle_read_pc();
        u16::from_le_bytes([lo, hi])
    }
    #[inline(always)]
    fn abs_index(&mut self, write: bool, index: u8) -> u16 {
        let lo = self.cycle_read_pc().wrapping_add(index);
        let hi = self.cycle_read_pc();
        let addr = u16::from_le_bytes([lo, hi]);

        // Fix high byte
        if lo < index {
            self.cpu.addr = addr;
            self.cycle_read();
            addr.wrapping_add(0x0100)
        } else {
            if write {
                self.cpu.addr = addr;
                self.cycle_read();
            }
            addr
        }
    }
    #[inline(always)]
    fn x_indirect(&mut self) -> u16 {
        let ptr = self.cycle_read_pc();
        self.cycle();
        let ptr = ptr.wrapping_add(self.cpu.X);
        let lo = self.cycle_read_ptr_lo(ptr);
        let hi = self.cycle_read_ptr_hi(ptr);
        u16::from_le_bytes([lo, hi])
    }
    #[inline(always)]
    fn indirect_y(&mut self, write: bool) -> u16 {
        let ptr = self.cycle_read_pc();
        let lo = self.cycle_read_ptr_lo(ptr).wrapping_add(self.cpu.Y);
        let hi = self.cycle_read_ptr_hi(ptr);
        let addr = u16::from_le_bytes([lo, hi]);

        // Fix high byte
        if lo < self.cpu.Y {
            self.cpu.addr = addr;
            self.cycle_read();
            addr.wrapping_add(0x0100)
        } else {
            if write {
                self.cpu.addr = addr;
                self.cycle_read();
            }
            addr
        }
    }

    // MODES
    fn branch(&mut self, p: fn(u8) -> bool) {
        if p(self.cpu.P) {
            let oper = self.cycle_read_pc();
            let [lo, hi] = self.cpu.PC.to_le_bytes();

            let lo = lo.wrapping_add(oper);
            let fixed = self.cpu.PC.wrapping_add_signed(i16::from(oper as i8));

            self.cycle_poke_pc();
            self.cpu.PC = u16::from_le_bytes([lo, hi]);

            if self.cpu.PC != fixed {
                // fix PC
                self.poll_interrupts();
                self.cycle_poke_pc();
                self.cpu.PC = fixed;
            } else {
                // no polling interrupts
                // immediately execute next instruction
            }
        } else {
            self.poll_interrupts();
            self.cycle_read_pc();
        }
    }

    fn imp(&mut self, f: fn(&mut CPU)) {
        self.poll_interrupts();
        self.cycle_poke_pc();
        f(&mut self.cpu);
    }
    fn imp_push(&mut self, f: fn(&mut CPU) -> u8) {
        self.cycle_poke_pc();
        self.poll_interrupts();
        let data = f(&mut self.cpu);
        self.cycle_push(data);
    }
    fn imp_pop(&mut self, f: fn(&mut CPU, u8)) {
        self.cycle_poke_pc();
        self.cycle_pop();
        self.poll_interrupts();
        let data = self.cycle_peek();
        f(&mut self.cpu, data);
    }
    fn imm(&mut self, f: fn(&mut CPU, u8)) {
        self.poll_interrupts();
        let data = self.cycle_read_pc();
        f(&mut self.cpu, data);
    }
    fn acc(&mut self, f: fn(&mut CPU, u8) -> u8) {
        self.poll_interrupts();
        self.cycle_poke_pc();
        let data = self.cpu.A;
        let data = f(&mut self.cpu, data);
        self.cpu.A = data;
    }

    mode_zpg!(zpg_read, zpg_write, zpg_rw, zpg());
    mode_zpg!(zpg_x_read, zpg_x_write, zpg_x_rw, zpg_index(X));
    mode_zpg!(zpg_y_read, zpg_y_write, _zpg_y_rw, zpg_index(Y));

    mode_abs!(abs_read, abs_write, abs_rw, abs());
    mode_abs!(abs_x_read, abs_x_write, abs_x_rw, abs_index(w, X));
    mode_abs!(abs_y_read, abs_y_write, abs_y_rw, abs_index(w, Y));
    mode_abs!(x_ind_read, x_ind_write, x_ind_rw, x_indirect());
    mode_abs!(ind_y_read, ind_y_write, ind_y_rw, indirect_y(w));

    #[cold]
    fn abs_y_write_unstable(&mut self, f: fn(&mut CPU) -> u8) {
        let addr = self.abs_index(true, self.cpu.Y);
        let page_cross = addr != self.cpu.addr;
        self.poll_interrupts();

        let [lo, hi] = self.cpu.addr.to_le_bytes();
        let data = f(&mut self.cpu) & hi.wrapping_add(1);
        if page_cross {
            self.cpu.addr = u16::from_le_bytes([lo, data]);
            self.cycle_write(data);
        } else {
            self.cycle_write(data);
        }
    }

    #[cold]
    fn abs_x_write_unstable(&mut self, f: fn(&mut CPU) -> u8) {
        let addr = self.abs_index(true, self.cpu.X);
        let page_cross = addr != self.cpu.addr;
        self.poll_interrupts();

        let [lo, hi] = self.cpu.addr.to_le_bytes();
        let data = f(&mut self.cpu) & hi.wrapping_add(1);
        if page_cross {
            self.cpu.addr = u16::from_le_bytes([lo, data]);
            self.cycle_write(data);
        } else {
            self.cycle_write(data);
        }
    }

    #[cold]
    fn ind_y_write_unstable(&mut self, f: fn(&mut CPU) -> u8) {
        let addr = self.indirect_y(true);
        let page_cross = addr != self.cpu.addr;
        self.poll_interrupts();

        let [lo, hi] = self.cpu.addr.to_le_bytes();
        let data = f(&mut self.cpu) & hi.wrapping_add(1);
        if page_cross {
            self.cpu.addr = u16::from_le_bytes([lo, data]);
            self.cycle_write(data);
        } else {
            self.cycle_write(data);
        }
    }

    // READ/WRITE CYCLES
    #[inline(always)]
    fn cycle_read(&mut self) -> u8 {
        self.cycle();
        self.read(self.cpu.addr)
    }
    #[inline(always)]
    fn cycle_write(&mut self, data: u8) {
        self.cycle();
        if !self.cpu.res_sample {
            self.write(self.cpu.addr, data);
        } else {
            self.read(self.cpu.addr);
        }
    }
    #[inline(always)]
    fn cycle_read_ptr_lo(&mut self, ptr: u8) -> u8 {
        self.cpu.addr = u16::from(ptr);
        self.cycle_read()
    }
    #[inline(always)]
    fn cycle_read_ptr_hi(&mut self, ptr: u8) -> u8 {
        self.cpu.addr = u16::from(ptr.wrapping_add(1));
        self.cycle_read()
    }
    #[inline(always)]
    fn cycle_read_pc(&mut self) -> u8 {
        self.cpu.addr = self.cpu.PC;
        let d = self.cycle_read();
        self.cpu.PC = self
            .cpu
            .PC
            .wrapping_add(u16::from(!self.cpu.hardware_interrupt));
        d
    }
    #[inline(always)]
    fn cycle_poke_pc(&mut self) {
        self.cycle();
        self.read(self.cpu.PC);
    }
    #[inline(always)]
    fn cycle_push(&mut self, data: u8) {
        self.cycle();
        if !self.cpu.res_sample {
            self.write_internal(0x0100 | u16::from(self.cpu.SP), data);
        }
        self.cpu.SP = self.cpu.SP.wrapping_sub(1);
    }

    #[inline(always)]
    fn cycle_push_pc_lo(&mut self) { self.cycle_push(self.cpu.PC as u8); }
    #[inline(always)]
    fn cycle_push_pc_hi(&mut self) { self.cycle_push((self.cpu.PC >> 8) as u8); }

    #[inline(always)]
    fn cycle_peek(&mut self) -> u8 {
        self.cycle();
        self.read_internal(0x0100 | u16::from(self.cpu.SP))
    }
    #[inline(always)]
    fn cycle_pop(&mut self) -> u8 {
        let data = self.cycle_peek();
        self.cpu.SP = self.cpu.SP.wrapping_add(1);
        data
    }
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            addr: 0,
            PC: 0,
            SP: 0,
            P: 0,
            A: 0,
            X: 0,
            Y: 0,
            irq_sample: false,
            nmi_sample: false,
            res_sample: true, // reset on startup
            hardware_interrupt: false,
        }
    }
    pub fn reset(&mut self) {
        self.res_sample = true;
        self.SP = 0;
    }

    pub fn nmi(&mut self) { self.nmi_sample = true; }
    pub fn irq(&mut self) { self.irq_sample = true; }
}

#[allow(non_snake_case)]
impl CPU {
    // HELPER FUNCTIONS
    fn flags(&mut self, val: u8) -> u8 {
        self.P &= 0b01111101;

        // negative flag
        self.P |= val & 0b10000000;

        // zero flag
        if val == 0 {
            self.P |= 0b00000010;
        }

        val
    }
    fn compare(&mut self, data: u8, reg: u8) {
        let (val, carry) = reg.overflowing_sub(data);
        self.P &= 0b11111110;
        if !carry {
            self.P |= 0b00000001;
        }
        self.flags(val);
    }

    // COMMON MICRO-INSTRUCTIONS
    fn LDA(&mut self, val: u8) { self.A = self.flags(val) }
    fn LDX(&mut self, val: u8) { self.X = self.flags(val) }
    fn LDY(&mut self, val: u8) { self.Y = self.flags(val) }
    fn STA(&mut self) -> u8 { self.A }
    fn STX(&mut self) -> u8 { self.X }
    fn STY(&mut self) -> u8 { self.Y }

    fn BIT(&mut self, val: u8) {
        self.P &= 0b00111101;
        if self.A & val == 0 {
            self.P |= 0b00000010;
        }
        self.P |= val & 0b11000000;
    }

    fn AND(&mut self, val: u8) { self.A = self.flags(self.A & val) }
    fn ORA(&mut self, val: u8) { self.A = self.flags(self.A | val) }
    fn EOR(&mut self, val: u8) { self.A = self.flags(self.A ^ val) }

    fn CMP(&mut self, val: u8) { self.compare(val, self.A) }
    fn CPX(&mut self, val: u8) { self.compare(val, self.X) }
    fn CPY(&mut self, val: u8) { self.compare(val, self.Y) }

    fn INC(&mut self, val: u8) -> u8 { self.flags(val.wrapping_add(1)) }
    fn DEC(&mut self, val: u8) -> u8 { self.flags(val.wrapping_sub(1)) }

    fn ADC(&mut self, val: u8) {
        let (v0, c0) = self.A.overflowing_add(val);
        let (v1, c1) = v0.overflowing_add(self.P & 1);

        self.P &= 0b10111110;

        // carry flag
        if c0 || c1 {
            self.P |= 0b00000001;
        }

        // overflow flag
        let sa = self.A & 0b10000000;
        let sb = val & 0b10000000;
        let sv = v1 & 0b10000000;
        if sa == sb && sb != sv {
            self.P |= 0b01000000;
        }

        self.A = self.flags(v1);
    }
    fn SBC(&mut self, val: u8) {
        let (v0, c0) = self.A.overflowing_sub(val);
        let (v1, c1) = v0.overflowing_sub(self.P & 1 ^ 1);

        self.P &= 0b10111110;

        // borrow flag
        if !(c0 || c1) {
            self.P |= 0b00000001;
        }

        // overflow flag
        let sa = self.A & 0b10000000;
        let sb = val & 0b10000000;
        let sv = v1 & 0b10000000;
        if sa != sb && sb == sv {
            self.P |= 0b01000000;
        }

        self.A = self.flags(v1);
    }

    fn ASL(&mut self, val: u8) -> u8 {
        self.P &= 0b11111110;
        self.P |= val >> 7;
        self.flags(val << 1)
    }
    fn LSR(&mut self, val: u8) -> u8 {
        self.P &= 0b11111110;
        self.P |= val & 1;
        self.flags(val >> 1)
    }
    fn ROL(&mut self, val: u8) -> u8 {
        let carry = self.P & 1;
        self.P &= 0b11111110;
        self.P |= val >> 7;
        self.flags(val << 1 | carry)
    }
    fn ROR(&mut self, val: u8) -> u8 {
        let carry = self.P << 7;
        self.P &= 0b11111110;
        self.P |= val & 1;
        self.flags(val >> 1 | carry)
    }

    // ILLEGAL
    fn LAX(&mut self, val: u8) {
        self.flags(val);
        self.A = val;
        self.X = val;
    }
    fn SAX(&mut self) -> u8 { self.A & self.X }
    fn DCP(&mut self, val: u8) -> u8 {
        let val = self.DEC(val);
        self.CMP(val);
        val
    }
    fn ISC(&mut self, val: u8) -> u8 {
        let val = self.INC(val);
        self.SBC(val);
        val
    }
    fn SLO(&mut self, val: u8) -> u8 {
        let val = self.ASL(val);
        self.ORA(val);
        val
    }
    fn RLA(&mut self, val: u8) -> u8 {
        let val = self.ROL(val);
        self.AND(val);
        val
    }
    fn SRE(&mut self, val: u8) -> u8 {
        let val = self.LSR(val);
        self.EOR(val);
        val
    }
    fn RRA(&mut self, val: u8) -> u8 {
        let val = self.ROR(val);
        self.ADC(val);
        val
    }
    fn ANC(&mut self, val: u8) {
        let val = self.A & val;
        self.A = self.flags(val);

        // carry flag
        self.P &= 0b11111110;
        self.P |= val >> 7;
    }
    fn ALR(&mut self, val: u8) {
        self.AND(val);
        self.A = self.LSR(self.A);
    }
    fn ARR(&mut self, val: u8) {
        self.A = self.ROR(self.A & val);

        self.P &= 0b10111110;
        if self.A.has_bit(6) {
            self.P |= 0b00000001;
        }
        if self.A.has_bit(5) != self.A.has_bit(6) {
            self.P |= 0b01000000;
        }
    }
    fn LAS(&mut self, val: u8) {
        self.A = self.flags(val & self.SP);
        self.X = self.A;
        self.SP = self.A;
    }
    fn SBX(&mut self, val: u8) {
        self.compare(val, self.A & self.X);
        self.X = (self.A & self.X).wrapping_sub(val);
    }

    // UNSTABLE ILLEGAL
    fn SHA(&mut self) -> u8 { self.A & self.X }
    fn SHX(&mut self) -> u8 { self.X }
    fn SHY(&mut self) -> u8 { self.Y }
    fn TAS(&mut self) -> u8 {
        self.SP = self.A & self.X;
        self.A & self.X
    }

    // HIGHLY UNSTABLE ILLEGAL
    // A base value in A is determined based on the contents of A and a constant, which may be typically $00, $ff, $ee, etc.
    // The value of this constant depends on temerature, the chip series, and maybe other factors, as well.
    fn ANE(&mut self, val: u8) {
        const BASE: u8 = 0;
        self.A = self.flags((self.A | BASE) & self.X & val);
    }
    fn LXA(&mut self, val: u8) {
        const BASE: u8 = 0;
        let val = self.flags((self.A | BASE) & val);
        self.A = val;
        self.X = val;
    }
}
