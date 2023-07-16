use crate::{cart::Cartridge, nes::NES, ppu::PPU};

#[allow(non_snake_case)]
#[derive(Clone)]
pub struct CPU {
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
        mode_full!($name_r, $name_w, $name_rw, $addr(w $(, $index)?), cycle_read_zpg, cycle_write_zpg);
    };
    ($name_r: ident, $name_w: ident, $name_rw: ident, $addr: ident ($($index: ident)?)) => {
        mode_full!($name_r, $name_w, $name_rw, $addr($($index)?), cycle_read_zpg, cycle_write_zpg);
    };
}

macro_rules! mode_full {
    ($name_r: ident, $name_w: ident, $name_rw: ident, $addr: ident (w $(, $index: ident)?), $read: ident, $write: ident) => {
        fn $name_r(&mut self, f: fn(&mut CPU, u8)) {
            let addr = self.$addr(false $(, self.cpu.$index)?);
            self.poll_interrupts();
            let data = self.$read(addr);
            f(&mut self.cpu, data);
        }
        fn $name_w(&mut self, f: fn(&mut CPU) -> u8) {
            let addr = self.$addr(true $(, self.cpu.$index)?);
            self.poll_interrupts();
            let data = f(&mut self.cpu);
            self.$write(addr, data);
        }
        fn $name_rw(&mut self, f: fn(&mut CPU, u8) -> u8) {
            let addr = self.$addr(true $(, self.cpu.$index)?);
            let data = self.$read(addr);
            self.$write(addr, data);
            self.poll_interrupts();
            let data = f(&mut self.cpu, data);
            self.$write(addr, data);
        }
    };
    ($name_r: ident, $name_w: ident, $name_rw: ident, $addr: ident ($($index: ident)?), $read: ident, $write: ident) => {
        fn $name_r(&mut self, f: fn(&mut CPU, u8)) {
            let addr = self.$addr($(self.cpu.$index)?);
            self.poll_interrupts();
            let data = self.$read(addr);
            f(&mut self.cpu, data);
        }
        fn $name_w(&mut self, f: fn(&mut CPU) -> u8) {
            let addr = self.$addr($(self.cpu.$index)?);
            self.poll_interrupts();
            let data = f(&mut self.cpu);
            self.$write(addr, data);
        }
        fn $name_rw(&mut self, f: fn(&mut CPU, u8) -> u8) {
            let addr = self.$addr($(self.cpu.$index)?);
            let data = self.$read(addr);
            self.$write(addr, data);
            self.poll_interrupts();
            let data = f(&mut self.cpu, data);
            self.$write(addr, data);
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

                let lo = self.cycle_read(addr);
                let hi = self.cycle_read(addr | 1);
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

            // NOP
            0xEA => self.imp(|_| ()),

            // NOP*
            0x1A | 0x3A | 0x5A | 0x7A | 0xDA | 0xFA => self.imp(|_| ()),
            0x80 | 0x82 | 0x89 | 0xC2 | 0xE2 => self.imm(|_, _| ()),
            0x04 | 0x44 | 0x64 => self.zpg_read(|_, _| ()),
            0x14 | 0x34 | 0x54 | 0x74 | 0xD4 | 0xF4 => self.zpg_x_read(|_, _| ()),
            0x0C => self.abs_read(|_, _| ()),
            0x1C | 0x3C | 0x5C | 0x7C | 0xDC | 0xFC => self.abs_x_read(|_, _| ()),

            // STACK
            0x48 => self.imp_push(|cpu| cpu.A),
            0x08 => self.imp_push(|cpu| cpu.P),
            0x68 => self.imp_pop(|cpu, data| cpu.A = cpu.flags(data)),
            0x28 => self.imp_pop(|cpu, data| cpu.P = data),

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
            0x95 => self.zpg_y_write(CPU::STA),
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

            // FLAGS
            0x18 => self.imp(|cpu| cpu.P &= 0b11111110),
            0x38 => self.imp(|cpu| cpu.P |= 0b00000100),
            0x58 => self.imp(|cpu| cpu.P &= 0b11111110),
            0x78 => self.imp(|cpu| cpu.P |= 0b00000100),
            0xB8 => self.imp(|cpu| cpu.P &= 0b10111111),
            0xD8 => self.imp(|cpu| cpu.P &= 0b11110111),
            0xF8 => self.imp(|cpu| cpu.P |= 0b00001000),

            0x24 => self.zpg_read(CPU::BIT),
            0x2C => self.abs_read(CPU::BIT),

            0x10 => self.branch(|p| p & 0b10000000 == 0),
            0x30 => self.branch(|p| p & 0b10000000 != 0),
            0x50 => self.branch(|p| p & 0b01000000 == 0),
            0x70 => self.branch(|p| p & 0b01000000 != 0),
            0x90 => self.branch(|p| p & 0b00000001 == 0),
            0xB0 => self.branch(|p| p & 0b00000001 != 0),
            0xD0 => self.branch(|p| p & 0b00000010 == 0),
            0xF0 => self.branch(|p| p & 0b00000010 != 0),

            _ => todo!("instr 0x{:02X}", op),
        }
    }

    // ADDRESSING
    fn zpg(&mut self) -> u8 {
        // just the same as a single read cycle
        self.cycle_read_pc()
    }
    fn zpg_index(&mut self, index: u8) -> u8 {
        let addr = self.cycle_read_pc();
        self.cycle();
        addr.wrapping_add(index)
    }
    fn abs(&mut self) -> u16 {
        let lo = self.cycle_read_pc();
        let hi = self.cycle_read_pc();
        u16::from_le_bytes([lo, hi])
    }
    fn abs_index(&mut self, write: bool, index: u8) -> u16 {
        let lo = self.cycle_read_pc().wrapping_add(index);
        let hi = self.cycle_read_pc();
        let addr = u16::from_le_bytes([lo, hi]);

        // Fix high byte
        if lo < index {
            self.cycle_read(addr);
            addr.wrapping_add(0x0100)
        } else {
            if write {
                self.cycle_read(addr);
            }
            addr
        }
    }
    fn x_indirect(&mut self) -> u16 {
        let ptr = self.cycle_read_pc();
        self.cycle();
        let ptr = ptr.wrapping_add(self.cpu.X);
        let lo = self.cycle_read_ptr_lo(ptr);
        let hi = self.cycle_read_ptr_hi(ptr);
        u16::from_le_bytes([lo, hi])
    }
    fn indirect_y(&mut self, write: bool) -> u16 {
        let ptr = self.cycle_read_pc();
        let lo = self.cycle_read_ptr_lo(ptr).wrapping_add(self.cpu.Y);
        let hi = self.cycle_read_ptr_hi(ptr);
        let addr = u16::from_le_bytes([lo, hi]);

        // Fix high byte
        if lo < self.cpu.Y {
            self.cycle_read(addr);
            addr.wrapping_add(0x0100)
        } else {
            if write {
                self.cycle_read(addr);
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

    mode_zpg!(zpg_read, zpg_write, zpg_rw, zpg());
    mode_zpg!(zpg_x_read, zpg_x_write, zpg_x_rw, zpg_index(X));
    mode_zpg!(zpg_y_read, zpg_y_write, zpg_y_rw, zpg_index(Y));

    mode_abs!(abs_read, abs_write, abs_rw, abs());
    mode_abs!(abs_x_read, abs_x_write, abs_x_rw, abs_index(w, X));
    mode_abs!(abs_y_read, abs_y_write, abs_y_rw, abs_index(w, Y));
    mode_abs!(x_ind_read, x_ind_write, x_ind_rw, x_indirect());
    mode_abs!(ind_y_read, ind_y_write, ind_y_rw, indirect_y(w));

    // READ/WRITE CYCLES
    fn cycle_read(&mut self, addr: u16) -> u8 {
        self.cycle();
        self.read(addr)
    }
    fn cycle_write(&mut self, addr: u16, data: u8) {
        self.cycle();
        if !self.cpu.res_sample {
            self.write(addr, data);
        } else {
            self.read(addr);
        }
    }
    fn cycle_read_zpg(&mut self, addr: u8) -> u8 {
        self.cycle();
        self.read_internal(u16::from(addr))
    }
    fn cycle_write_zpg(&mut self, addr: u8, data: u8) {
        self.cycle();
        if !self.cpu.res_sample {
            self.write_internal(u16::from(addr), data);
        }
    }
    fn cycle_read_ptr_lo(&mut self, ptr: u8) -> u8 {
        self.cycle();
        self.read_internal(u16::from(ptr))
    }
    fn cycle_read_ptr_hi(&mut self, ptr: u8) -> u8 {
        self.cycle();
        self.read_internal(u16::from(ptr.wrapping_add(1)))
    }
    fn cycle_read_pc(&mut self) -> u8 {
        self.cycle();
        let d = self.read(self.cpu.PC);
        if !self.cpu.hardware_interrupt {
            self.cpu.PC += 1;
        }
        d
    }
    fn cycle_poke_pc(&mut self) {
        self.cycle();
        self.read(self.cpu.PC);
    }
    fn cycle_push(&mut self, data: u8) {
        self.cycle();
        if !self.cpu.res_sample {
            self.write_internal(0x0100 | u16::from(self.cpu.SP), data);
        }
        self.cpu.SP = self.cpu.SP.wrapping_sub(1);
    }

    fn cycle_push_pc_lo(&mut self) { self.cycle_push(self.cpu.PC as u8); }
    fn cycle_push_pc_hi(&mut self) { self.cycle_push((self.cpu.PC >> 8) as u8); }

    fn cycle_peek(&mut self) -> u8 {
        self.cycle();
        self.read_internal(0x0100 | u16::from(self.cpu.SP))
    }
    fn cycle_pop(&mut self) -> u8 {
        let data = self.cycle_peek();
        self.cpu.SP = self.cpu.SP.wrapping_add(1);
        data
    }
}

impl CPU {
    pub fn new() -> Self {
        CPU {
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
    fn STA(&mut self) -> u8 { self.A }
    fn STX(&mut self) -> u8 { self.X }

    fn BIT(&mut self, val: u8) {
        self.flags(self.A & val);
        self.P |= val & 0b11000000;
    }
}
