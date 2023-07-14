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
                self.cpu.PC = u16::from(lo) | u16::from(hi) << 8;
            }

            // LOAD/STORE
            0xA9 => self.mode_imm(CPU::LDA),
            0xA5 => self.mode_zpg_read(CPU::LDA),
            0xB5 => self.mode_zpg_i_read(CPU::LDA, self.cpu.X),
            0xAD => self.mode_abs_read(CPU::LDA),
            0xBD => self.mode_abs_i_read(CPU::LDA, self.cpu.X),
            0xB9 => self.mode_abs_i_read(CPU::LDA, self.cpu.Y),
            0xA1 => self.mode_x_ind_read(CPU::LDA),
            0xB1 => self.mode_ind_y_read(CPU::LDA),

            // FLAGS
            0x18 => self.mode_imp(|cpu| cpu.P &= 0b11111110),
            0x38 => self.mode_imp(|cpu| cpu.P |= 0b00000100),
            0x58 => self.mode_imp(|cpu| cpu.P &= 0b11111110),
            0x78 => self.mode_imp(|cpu| cpu.P |= 0b00000100),
            0xB8 => self.mode_imp(|cpu| cpu.P &= 0b10111111),
            0xD8 => self.mode_imp(|cpu| cpu.P &= 0b11110111),
            0xF8 => self.mode_imp(|cpu| cpu.P |= 0b00001000),

            _ => todo!("instr 0x{:X}", op),
        }
    }

    // ADDRESSING
    fn mode_imp(&mut self, f: impl FnOnce(&mut CPU)) {
        self.poll_interrupts();
        self.cycle_poke_pc();
        f(&mut self.cpu);
    }
    fn mode_imm(&mut self, f: impl FnOnce(&mut CPU, u8)) {
        self.poll_interrupts();
        let data = self.cycle_read_pc();
        f(&mut self.cpu, data);
    }
    fn mode_zpg_read(&mut self, f: impl FnOnce(&mut CPU, u8)) {
        let addr = self.cycle_read_pc();

        self.poll_interrupts();
        let data = self.cycle_read_zpg(addr);
        f(&mut self.cpu, data);
    }
    fn mode_zpg_i_read(&mut self, f: impl FnOnce(&mut CPU, u8), index: u8) {
        let addr = self.cycle_read_pc();
        self.cycle();
        let addr = addr.wrapping_add(index);

        self.poll_interrupts();
        let data = self.cycle_read_zpg(addr);
        f(&mut self.cpu, data);
    }
    fn mode_abs_read(&mut self, f: impl FnOnce(&mut CPU, u8)) {
        let lo = self.cycle_read_pc();
        let hi = self.cycle_read_pc();
        let addr = u16::from_le_bytes([lo, hi]);

        self.poll_interrupts();
        let data = self.cycle_read(addr);
        f(&mut self.cpu, data);
    }
    fn mode_abs_i_read(&mut self, f: impl FnOnce(&mut CPU, u8), index: u8) {
        let lo = self.cycle_read_pc().wrapping_add(index);
        let hi = self.cycle_read_pc();
        let addr = u16::from_le_bytes([lo, hi]);

        let addr = if lo < index {
            // Fix high byte
            self.cycle_read(addr);
            addr.wrapping_add(0x0100)
        } else {
            addr
        };

        self.poll_interrupts();
        let data = self.cycle_read(addr);
        f(&mut self.cpu, data);
    }
    fn mode_x_ind_read(&mut self, f: impl FnOnce(&mut CPU, u8)) {
        let ptr = self.cycle_read_pc();
        self.cycle();
        let ptr = ptr.wrapping_add(self.cpu.X);
        let lo = self.cycle_read_ptr_lo(ptr);
        let hi = self.cycle_read_ptr_hi(ptr);
        let addr = u16::from_le_bytes([lo, hi]);

        self.poll_interrupts();
        let data = self.cycle_read(addr);
        f(&mut self.cpu, data);
    }
    fn mode_ind_y_read(&mut self, f: impl FnOnce(&mut CPU, u8)) {
        let ptr = self.cycle_read_pc();
        let lo = self.cycle_read_ptr_lo(ptr).wrapping_add(self.cpu.Y);
        let hi = self.cycle_read_ptr_hi(ptr);
        let addr = u16::from_le_bytes([lo, hi]);

        let addr = if lo < self.cpu.Y {
            // Fix high byte
            self.cycle_read(addr);
            addr.wrapping_add(0x0100)
        } else {
            addr
        };

        self.poll_interrupts();
        let data = self.cycle_read(addr);
        f(&mut self.cpu, data);
    }

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
    fn LDA(&mut self, val: u8) { self.A = self.flags(val); }
}
