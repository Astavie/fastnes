use std::{
    fmt::{Debug, Display},
    fs::read_to_string,
};

use crate::mem::NESTest;

#[derive(PartialEq, Eq, Clone, Copy)]
pub enum RW {
    Read(u16),
    Write(u16, u8),
}

#[allow(non_snake_case)]
pub struct MOS6502 {
    // instruction list
    instructions: [Instruction; 256],

    // in
    pub bus: u8,
    pub irq: u8,
    pub nmi: bool,
    pub halt: bool,
    pub max_cycles: usize,

    // rom
    cycles: usize,
    nestest: NESTest,

    // registers
    PC: u16,
    SP: u8,
    P: u8,
    A: u8,
    X: u8,
    Y: u8,

    // interrupts
    irq_sample: bool,
    nmi_sample: bool,
    res_sample: bool,
    hardware_interrupt: bool,
}

pub(crate) fn nestest<const TEST: bool>(rom: [u8; 16384], instr: [Instruction; 256]) {
    let mem = NESTest::from_rom(rom);
    let mut cpu = MOS6502::new(mem, instr);

    if TEST {
        let log = read_to_string("test/nestest/nestest.log").unwrap();
        let mut lines = log.split('\n');
        let mut expected_pc = 0x0000;
        let mut lineno = 1;

        let mut f = |cpu: &mut MOS6502, _: Option<RW>| {
            let line = lines.next().unwrap();
            let pc = u16::from_str_radix(&line[0..4], 16).unwrap();
            let cyc = usize::from_str_radix(line.rsplit_once(':').unwrap().1, 10).unwrap();

            // println!("cpu      : ${:04X} at CYC:{}", cpu.PC, cpu.cycles);
            assert_eq!(cpu.PC, expected_pc);
            // println!("line {:04}: ${:04X} at CYC:{}", lineno, pc, cyc);

            expected_pc = pc;
            lineno += 1;
            cpu.max_cycles = cyc;

            if lineno == 8992 {
                cpu.halt = true;
            }
        };

        f(&mut cpu, None);

        cpu.run(f);
    } else {
        cpu.max_cycles = 26546;
        cpu.run(|cpu, rw| match rw {
            Some(RW::Read(_)) => unreachable!(),
            Some(RW::Write(_, _)) => (),
            None => cpu.halt = true,
        });
    }

    assert_eq!(cpu.cycles, 26548);
    assert_eq!(cpu.PC, 0xC6A2);
}

pub fn instructions() -> [Instruction; 256] {
    (0..=255)
        .map(Instruction::from)
        .collect::<Vec<Instruction>>()
        .try_into()
        .unwrap()
}

impl Micro {
    fn run<F>(self, cpu: &mut MOS6502, addr: u16, mut cont: F)
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        match self {
            Micro::ASL => {
                let data = cpu.read_addr(addr, &mut cont);
                cpu.write_addr(addr, data, &mut cont);

                cpu.P &= 0b11111110;
                cpu.P |= data >> 7;
                let data = cpu.flags(data << 1);
                cpu.write_addr(addr, data, &mut cont);
            }
            Micro::LSR => {
                let data = cpu.read_addr(addr, &mut cont);
                cpu.write_addr(addr, data, &mut cont);

                cpu.P &= 0b11111110;
                cpu.P |= data & 1;
                let data = cpu.flags(data >> 1);
                cpu.write_addr(addr, data, &mut cont);
            }
            Micro::ROL => {
                let data = cpu.read_addr(addr, &mut cont);
                cpu.write_addr(addr, data, &mut cont);

                let carry = cpu.P & 1;
                cpu.P &= 0b11111110;
                cpu.P |= data >> 7;
                let data = cpu.flags(data << 1 | carry);
                cpu.write_addr(addr, data, &mut cont);
            }
            Micro::ROR => {
                let data = cpu.read_addr(addr, &mut cont);
                cpu.write_addr(addr, data, &mut cont);

                let carry = cpu.P << 7;
                cpu.P &= 0b11111110;
                cpu.P |= data & 1;
                let data = cpu.flags(data >> 1 | carry);
                cpu.write_addr(addr, data, &mut cont);
            }
            Micro::INC => {
                let data = cpu.read_addr(addr, &mut cont);
                cpu.write_addr(addr, data, &mut cont);
                let data = cpu.flags(data.wrapping_add(1));
                cpu.write_addr(addr, data, &mut cont);
            }
            Micro::DEC => {
                let data = cpu.read_addr(addr, &mut cont);
                cpu.write_addr(addr, data, &mut cont);
                let data = cpu.flags(data.wrapping_sub(1));
                cpu.write_addr(addr, data, &mut cont);
            }
            Micro::AND => {
                cpu.A &= cpu.read_addr(addr, cont);
                cpu.flags(cpu.A);
            }
            Micro::EOR => {
                cpu.A ^= cpu.read_addr(addr, cont);
                cpu.flags(cpu.A);
            }
            Micro::ORA => {
                cpu.A |= cpu.read_addr(addr, cont);
                cpu.flags(cpu.A);
            }
            Micro::BIT => {
                let data = cpu.read_addr(addr, cont);

                cpu.P &= 0b00111101;

                // Negative and oVerflow flags
                cpu.P |= data & 0b11000000;

                // Zero flag
                if cpu.A & data == 0 {
                    cpu.P |= 0b00000010;
                }
            }
            Micro::ADC => {
                let data = cpu.read_addr(addr, cont);
                let (v0, c0) = cpu.A.overflowing_add(data);
                let (v1, c1) = v0.overflowing_add(cpu.P & 1);

                cpu.P &= 0b10111110;

                // carry flag
                if c0 || c1 {
                    cpu.P |= 0b00000001;
                }

                // overflow flag
                let sa = cpu.A & 0b10000000;
                let sb = data & 0b10000000;
                let sv = v1 & 0b10000000;
                if sa == sb && sb != sv {
                    cpu.P |= 0b01000000;
                }

                cpu.A = v1;
                cpu.flags(cpu.A);
            }
            Micro::SBC => {
                let data = cpu.read_addr(addr, cont);
                let (v0, c0) = cpu.A.overflowing_sub(data);
                let (v1, c1) = v0.overflowing_sub(cpu.P & 1 ^ 1);

                cpu.P &= 0b10111110;

                // borrow flag
                if !(c0 || c1) {
                    cpu.P |= 0b00000001;
                }

                // overflow flag
                let sa = cpu.A & 0b10000000;
                let sb = data & 0b10000000;
                let sv = v1 & 0b10000000;
                if sa != sb && sb == sv {
                    cpu.P |= 0b01000000;
                }

                cpu.A = v1;
                cpu.flags(cpu.A);
            }
            Micro::STA => cpu.write_addr(addr, cpu.A, cont),
            Micro::STY => cpu.write_addr(addr, cpu.Y, cont),
            Micro::STX => cpu.write_addr(addr, cpu.X, cont),
            Micro::LDA => cpu.A = cpu.read_addr_flags(addr, cont),
            Micro::LDY => cpu.Y = cpu.read_addr_flags(addr, cont),
            Micro::LDX => cpu.X = cpu.read_addr_flags(addr, cont),
            Micro::CPX => cpu.compare(addr, cpu.X, cont),
            Micro::CPY => cpu.compare(addr, cpu.Y, cont),
            Micro::CMP => cpu.compare(addr, cpu.A, cont),
            Micro::NOP => _ = cpu.read_addr(addr, cont),
            Micro::SEC => cpu.P |= 0b00000001,
            Micro::SEI => cpu.P |= 0b00000100,
            Micro::SED => cpu.P |= 0b00001000,
            Micro::CLC => cpu.P &= 0b11111110,
            Micro::CLI => cpu.P &= 0b11111011,
            Micro::CLD => cpu.P &= 0b11110111,
            Micro::CLV => cpu.P &= 0b10111111,
            Micro::INX => cpu.X = cpu.flags(cpu.X.wrapping_add(1)),
            Micro::DEX => cpu.X = cpu.flags(cpu.X.wrapping_sub(1)),
            Micro::INY => cpu.Y = cpu.flags(cpu.Y.wrapping_add(1)),
            Micro::DEY => cpu.Y = cpu.flags(cpu.Y.wrapping_sub(1)),
            Micro::TAX => cpu.X = cpu.flags(cpu.A),
            Micro::TXA => cpu.A = cpu.flags(cpu.X),
            Micro::TAY => cpu.Y = cpu.flags(cpu.A),
            Micro::TYA => cpu.A = cpu.flags(cpu.Y),
            Micro::TSX => cpu.X = cpu.flags(cpu.SP),
            Micro::TXS => cpu.SP = cpu.X,
            Micro::SLO => {
                let data = cpu.read_addr(addr, &mut cont);

                // ASL
                cpu.write_addr(addr, data, &mut cont);

                cpu.P &= 0b11111110;
                cpu.P |= data >> 7;
                let data = data << 1;
                cpu.write_addr(addr, data, &mut cont);

                // ORA
                cpu.A |= data;
                cpu.flags(cpu.A);
            }
            Micro::SRE => {
                let data = cpu.read_addr(addr, &mut cont);

                // LSR
                cpu.write_addr(addr, data, &mut cont);

                cpu.P &= 0b11111110;
                cpu.P |= data & 1;
                let data = data >> 1;
                cpu.write_addr(addr, data, &mut cont);

                // EOR
                cpu.A ^= data;
                cpu.flags(cpu.A);
            }
            Micro::RLA => {
                let data = cpu.read_addr(addr, &mut cont);

                // ROL
                cpu.write_addr(addr, data, &mut cont);

                let carry = cpu.P & 1;
                cpu.P &= 0b11111110;
                cpu.P |= data >> 7;
                let data = data << 1 | carry;
                cpu.write_addr(addr, data, &mut cont);

                // AND
                cpu.A &= data;
                cpu.flags(cpu.A);
            }
            Micro::RRA => {
                let data = cpu.read_addr(addr, &mut cont);

                // ROR
                cpu.write_addr(addr, data, &mut cont);

                let carry = cpu.P << 7;
                cpu.P &= 0b11111110;
                cpu.P |= data & 1;
                let data = cpu.flags(data >> 1 | carry);
                cpu.write_addr(addr, data, &mut cont);

                // ADC
                let (v0, c0) = cpu.A.overflowing_add(data);
                let (v1, c1) = v0.overflowing_add(cpu.P & 1);

                cpu.P &= 0b10111110;

                // carry flag
                if c0 || c1 {
                    cpu.P |= 0b00000001;
                }

                // overflow flag
                let sa = cpu.A & 0b10000000;
                let sb = data & 0b10000000;
                let sv = v1 & 0b10000000;
                if sa == sb && sb != sv {
                    cpu.P |= 0b01000000;
                }

                cpu.A = v1;
                cpu.flags(cpu.A);
            }
            Micro::ISC => {
                let data = cpu.read_addr(addr, &mut cont);

                // INC
                cpu.write_addr(addr, data, &mut cont);
                let data = data.wrapping_add(1);
                cpu.write_addr(addr, data, &mut cont);

                // SBC
                let (v0, c0) = cpu.A.overflowing_sub(data);
                let (v1, c1) = v0.overflowing_sub(cpu.P & 1 ^ 1);

                cpu.P &= 0b10111110;

                // borrow flag
                if !(c0 || c1) {
                    cpu.P |= 0b00000001;
                }

                // overflow flag
                let sa = cpu.A & 0b10000000;
                let sb = data & 0b10000000;
                let sv = v1 & 0b10000000;
                if sa != sb && sb == sv {
                    cpu.P |= 0b01000000;
                }

                cpu.A = v1;
                cpu.flags(cpu.A);
            }
            Micro::DCP => {
                let data = cpu.read_addr(addr, &mut cont);

                // DEC
                cpu.write_addr(addr, data, &mut cont);
                let data = data.wrapping_sub(1);
                cpu.write_addr(addr, data, &mut cont);

                // CMP
                let (val, carry) = cpu.A.overflowing_sub(data);
                cpu.P &= 0b11111110;
                if !carry {
                    cpu.P |= 0b00000001;
                }
                cpu.flags(val);
            }
            Micro::LAX => {
                cpu.A = cpu.read_addr_flags(addr, cont);
                cpu.X = cpu.A;
            }
            Micro::SAX => cpu.write_addr(addr, cpu.A & cpu.X, cont),
            Micro::LAS => todo!(),
            Micro::TAS => todo!(),
            Micro::SHA => todo!(),
            Micro::SHY => todo!(),
            Micro::SHX => todo!(),
            Micro::ANE => todo!(),
            Micro::LXA => todo!(),
        }
    }
}

impl MOS6502 {
    fn poll(&mut self) {
        // sample irq and nmi (nmi stays on while irq gets reset every cycle)
        self.irq_sample = self.irq > 0;
        self.nmi_sample = self.nmi_sample || self.nmi;
        self.nmi = false;
    }
    pub fn run<F>(&mut self, mut cont: F)
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        self.reset();

        while !self.halt {
            // get instruction to perform
            let op = self.read_pc(&mut cont);
            let instr = self.instructions[if self.hardware_interrupt {
                0
            } else {
                usize::from(op)
            }];

            // run instruction
            match instr {
                Instruction::Mode(mode, mi) => match mode {
                    Mode::Accumulator => {
                        self.peek_pc(&mut cont);
                        match mi {
                            Micro::ASL => {
                                self.P &= 0b11111110;
                                self.P |= self.A >> 7;
                                self.A = self.flags(self.A << 1);
                            }
                            Micro::LSR => {
                                self.P &= 0b11111110;
                                self.P |= self.A & 1;
                                self.A = self.flags(self.A >> 1);
                            }
                            Micro::ROL => {
                                let carry = self.P & 1;
                                self.P &= 0b11111110;
                                self.P |= self.A >> 7;
                                self.A = self.flags(self.A << 1 | carry);
                            }
                            Micro::ROR => {
                                let carry = self.P << 7;
                                self.P &= 0b11111110;
                                self.P |= self.A & 1;
                                self.A = self.flags(self.A >> 1 | carry);
                            }
                            _ => unreachable!(),
                        }
                        self.poll();
                    }
                    Mode::Implied => {
                        self.peek_pc(&mut cont);
                        if mi != Micro::NOP {
                            mi.run(self, 0, &mut cont);
                        }
                        self.poll();
                    }
                    Mode::Immediate => {
                        mi.run(self, self.PC, &mut cont);
                        self.PC = self.PC.wrapping_add(1);
                        self.poll();
                    }
                    Mode::Absolute => {
                        let lo = self.read_pc(&mut cont);
                        let hi = self.read_pc(&mut cont);
                        let addr = u16::from(lo) | u16::from(hi) << 8;
                        mi.run(self, addr, &mut cont);
                        self.poll();
                    }
                    Mode::ZeroPage => {
                        let addr = u16::from(self.read_pc(&mut cont));
                        mi.run(self, addr, &mut cont);
                        self.poll();
                    }
                    Mode::ZeroPageIndexed(i) => {
                        let ptr = self.read_pc(&mut cont);
                        self.read_addr(u16::from(ptr), &mut cont);

                        let addr = u16::from(ptr.wrapping_add(i.get(self)));
                        mi.run(self, addr, &mut cont);
                        self.poll();
                    }
                    Mode::AbsoluteIndexed(i) => {
                        let reg = i.get(self);

                        let lo = self.read_pc(&mut cont);
                        let hi = self.read_pc(&mut cont);
                        let addr = u16::from(lo.wrapping_add(reg)) | u16::from(hi) << 8;

                        if (addr as u8) < reg {
                            self.read_addr(addr, &mut cont);
                            mi.run(self, addr.wrapping_add(0x0100), &mut cont);
                        } else {
                            match mi {
                                Micro::LDA
                                | Micro::LDX
                                | Micro::LDY
                                | Micro::EOR
                                | Micro::AND
                                | Micro::ORA
                                | Micro::ADC
                                | Micro::SBC
                                | Micro::CMP
                                | Micro::BIT
                                | Micro::LAX
                                | Micro::LAS
                                | Micro::TAS
                                | Micro::NOP => {} // skip fix cycle
                                _ => {
                                    self.read_addr(addr, &mut cont);
                                }
                            }
                            mi.run(self, addr, &mut cont);
                        }

                        self.poll();
                    }
                    Mode::IndexedIndirect => {
                        let ptr = self.read_pc(&mut cont);
                        self.read_ptr_lo(ptr, &mut cont);

                        let ptr = ptr.wrapping_add(self.X);
                        let lo = self.read_ptr_lo(ptr, &mut cont);
                        let hi = self.read_ptr_hi(ptr, &mut cont);
                        let addr = u16::from(lo) | u16::from(hi) << 8;
                        mi.run(self, addr, &mut cont);
                        self.poll();
                    }
                    Mode::IndirectIndexed => {
                        let ptr = self.read_pc(&mut cont);
                        let lo = self.read_ptr_lo(ptr, &mut cont);
                        let hi = self.read_ptr_hi(ptr, &mut cont);
                        let addr = u16::from(lo.wrapping_add(self.Y)) | u16::from(hi) << 8;

                        if (addr as u8) < self.Y {
                            self.read_addr(addr, &mut cont);
                            mi.run(self, addr.wrapping_add(0x0100), &mut cont);
                        } else {
                            match mi {
                                Micro::LDA
                                | Micro::LDX
                                | Micro::LDY
                                | Micro::EOR
                                | Micro::AND
                                | Micro::ORA
                                | Micro::ADC
                                | Micro::SBC
                                | Micro::CMP
                                | Micro::BIT
                                | Micro::LAX
                                | Micro::LAS
                                | Micro::TAS
                                | Micro::NOP => {} // skip fix cycle
                                _ => {
                                    self.read_addr(addr, &mut cont);
                                }
                            }
                            mi.run(self, addr, &mut cont);
                        }

                        self.poll();
                    }
                },
                Instruction::Branch => {
                    let oper = self.read_pc(&mut cont);
                    let addr = self.PC.wrapping_add_signed(i16::from(oper as i8));

                    if match op {
                        0x10 => self.P & 0b10000000 == 0, // BPL
                        0x30 => self.P & 0b10000000 != 0, // BMI

                        0x50 => self.P & 0b01000000 == 0, // BVC
                        0x70 => self.P & 0b01000000 != 0, // BVS

                        0x90 => self.P & 0b00000001 == 0, // BCC
                        0xB0 => self.P & 0b00000001 != 0, // BCS

                        0xD0 => self.P & 0b00000010 == 0, // BNE
                        0xF0 => self.P & 0b00000010 != 0, // BEQ

                        _ => unreachable!(),
                    } {
                        self.peek_pc(&mut cont);
                        self.PC = (self.PC & 0xFF00) | (addr & 0x00FF);

                        if self.PC != addr {
                            // fix PC
                            self.peek_pc(&mut cont);
                            self.PC = addr;

                            self.poll();
                        } else {
                            // no polling!
                        }
                    } else {
                        self.poll();
                    }
                }
                Instruction::JMPabs => {
                    let lo = self.read_pc(&mut cont);
                    let hi = self.read_pc(&mut cont);
                    self.PC = u16::from(lo) | u16::from(hi) << 8;

                    self.poll();
                }
                Instruction::JMPind => {
                    let lo = self.read_pc(&mut cont);
                    let hi = self.read_pc(&mut cont);
                    let addr_lo = u16::from(lo) | u16::from(hi) << 8;
                    let addr_hi = u16::from(lo.wrapping_add(1)) | u16::from(hi) << 8;

                    let lo = self.read_addr(addr_lo, &mut cont);
                    let hi = self.read_addr(addr_hi, &mut cont);
                    self.PC = u16::from(lo) | u16::from(hi) << 8;

                    self.poll();
                }
                Instruction::BRK => {
                    self.read_pc(&mut cont);
                    self.push_hi(&mut cont);
                    self.push_lo(&mut cont);

                    // poll address
                    self.poll();
                    let addr = if self.nmi_sample {
                        self.nmi_sample = false;
                        0xFFFA
                    } else if self.res_sample {
                        self.res_sample = false;
                        0xFFFC
                    } else {
                        0xFFFE
                    };

                    let b = if self.hardware_interrupt {
                        // B flag clear
                        0b00100000
                    } else {
                        // B flag set
                        0b00110000
                    };
                    self.push(self.P | b, &mut cont);

                    // interrupt disable
                    self.P |= 0b00000100;

                    let lo = self.read_addr(addr, &mut cont);
                    let hi = self.read_addr(addr | 1, &mut cont);
                    self.PC = u16::from(lo) | u16::from(hi) << 8;
                    self.hardware_interrupt = false;
                }
                Instruction::RTI => {
                    self.peek_pc(&mut cont);
                    self.pop(&mut cont);
                    self.P = self.pop(&mut cont) & 0b11001111;
                    let lo = self.pop(&mut cont);
                    let hi = self.peek(&mut cont);
                    self.PC = u16::from(lo) | u16::from(hi) << 8;

                    self.poll();
                }
                Instruction::JSR => {
                    let lo = self.read_pc(&mut cont);
                    self.peek(&mut cont);
                    self.push_hi(&mut cont);
                    self.push_lo(&mut cont);
                    let hi = self.read_pc(&mut cont);
                    self.PC = u16::from(lo) | u16::from(hi) << 8;

                    self.poll();
                }
                Instruction::RTS => {
                    self.peek_pc(&mut cont);
                    self.pop(&mut cont);
                    let lo = self.pop(&mut cont);
                    let hi = self.peek(&mut cont);
                    self.PC = u16::from(lo) | u16::from(hi) << 8;
                    self.read_pc(&mut cont);

                    self.poll();
                }
                Instruction::PHA => {
                    self.peek_pc(&mut cont);
                    self.push(self.A, &mut cont);

                    self.poll();
                }
                Instruction::PHP => {
                    self.peek_pc(&mut cont);
                    self.push(self.P | 0b00110000, &mut cont);

                    self.poll();
                }
                Instruction::PLA => {
                    self.peek_pc(&mut cont);
                    self.pop(&mut cont);
                    self.A = self.peek(&mut cont);
                    self.flags(self.A);

                    self.poll();
                }
                Instruction::PLP => {
                    self.peek_pc(&mut cont);
                    self.pop(&mut cont);
                    self.P = self.peek(&mut cont) & 0b11001111;

                    self.poll();
                }
                Instruction::JAM => todo!(),
            }
        }
    }

    pub fn reset(&mut self) {
        self.res_sample = true;
        self.hardware_interrupt = true;
    }
    pub fn new(test: NESTest, instructions: [Instruction; 256]) -> Self {
        MOS6502 {
            instructions,
            halt: false,
            bus: 0,
            PC: 0,
            SP: 0,
            P: 0,
            A: 0,
            X: 0,
            Y: 0,
            cycles: 0,
            max_cycles: 0,
            nestest: test,
            nmi: false,
            irq: 0,
            irq_sample: false,
            nmi_sample: false,
            res_sample: false,
            hardware_interrupt: false,
        }
    }
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
    fn compare<F>(&mut self, addr: u16, reg: u8, cont: F)
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        let data = self.read_addr(addr, cont);
        let (val, carry) = reg.overflowing_sub(data);
        self.P &= 0b11111110;
        if !carry {
            self.P |= 0b00000001;
        }
        self.flags(val);
    }
    fn write_addr<F>(&mut self, addr: u16, data: u8, mut cont: F)
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        if !self.res_sample {
            if addr < 0x0800 || addr >= 0x6000 {
                self.inc(cont);
                self.nestest.set(addr, data);
            } else {
                cont(self, Some(RW::Write(addr, data)));
                self.cycles += 1;
            }
        } else {
            self.read_addr(addr, cont);
        }
    }
    fn read_addr_flags<F>(&mut self, addr: u16, cont: F) -> u8
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        let data = self.read_addr(addr, cont);
        self.flags(data)
    }
    fn read_addr<F>(&mut self, addr: u16, mut cont: F) -> u8
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        if addr < 0x0800 || addr >= 0x6000 {
            self.inc(cont);
            self.nestest.get(addr)
        } else {
            cont(self, Some(RW::Read(addr)));
            self.cycles += 1;
            self.bus
        }
    }
    fn read_ptr_lo<F>(&mut self, ptr: u8, cont: F) -> u8
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        self.read_addr(u16::from(ptr), cont)
    }
    fn read_ptr_hi<F>(&mut self, ptr: u8, cont: F) -> u8
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        self.read_addr(u16::from(ptr.wrapping_add(1)), cont)
    }
    fn read_pc<F>(&mut self, cont: F) -> u8
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        let d = self.read_addr(self.PC, cont);
        if !self.hardware_interrupt {
            self.PC += 1;
        }
        d
    }
    fn peek_pc<F>(&mut self, cont: F) -> u8
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        self.read_addr(self.PC, cont)
    }
    fn push<F>(&mut self, data: u8, cont: F)
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        self.inc(cont);

        if !self.res_sample {
            self.nestest.set(0x0100 | u16::from(self.SP), data);
        }
        self.SP = self.SP.wrapping_sub(1);
    }
    fn push_lo<F>(&mut self, cont: F)
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        self.push(self.PC as u8, cont);
    }
    fn push_hi<F>(&mut self, cont: F)
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        self.push((self.PC >> 8) as u8, cont);
    }
    fn pop<F>(&mut self, cont: F) -> u8
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        self.inc(cont);

        let d = self.nestest.get(0x0100 | u16::from(self.SP));
        self.SP = self.SP.wrapping_add(1);
        d
    }
    fn peek<F>(&mut self, cont: F) -> u8
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        self.inc(cont);
        self.nestest.get(0x0100 | u16::from(self.SP))
    }
    fn inc<F>(&mut self, mut cont: F)
    where
        F: FnMut(&mut MOS6502, Option<RW>),
    {
        if self.cycles >= self.max_cycles {
            cont(self, None);
        }
        self.cycles += 1;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Micro {
    // legal
    ASL,
    LSR,
    ROL,
    ROR,
    INC,
    DEC,

    AND,
    EOR,
    ORA,
    BIT,
    ADC,
    SBC,
    STA,
    STY,
    STX,
    LDA,
    LDY,
    LDX,
    CPX,
    CPY,
    CMP,

    // implied
    NOP,

    CLC,
    SEC,
    CLI,
    SEI,
    CLV,
    CLD,
    SED,

    INX,
    DEX,
    INY,
    DEY,

    TAX,
    TXA,
    TAY,
    TYA,
    TSX,
    TXS,

    // illegal
    SLO,
    SRE,
    RLA,
    RRA,
    ISC,
    DCP,
    LAX,
    SAX,
    LAS,

    // unstable
    TAS,
    SHA,
    SHY,
    SHX,

    // highly unstable
    ANE,
    LXA,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Index {
    X,
    Y,
}

impl Index {
    fn op(op: u8) -> Self {
        if op & 0b11000010 == 0b10000010 {
            // exception for STX and LDX
            Self::Y
        } else {
            Self::X
        }
    }
    fn get(self, cpu: &MOS6502) -> u8 {
        match self {
            Self::X => cpu.X,
            Self::Y => cpu.Y,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Mode {
    Accumulator,
    Implied,
    Immediate,
    Absolute,
    ZeroPage,
    ZeroPageIndexed(Index),
    AbsoluteIndexed(Index),
    IndexedIndirect,
    IndirectIndexed,
}

impl Display for Mode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Accumulator => f.write_str("A"),
            Self::Implied => f.write_str("impl"),
            Self::Immediate => f.write_str("#"),
            Self::Absolute => f.write_str("abs"),
            Self::ZeroPage => f.write_str("zpg"),
            Self::ZeroPageIndexed(xy) => f.write_fmt(format_args!("zpg,{:?}", xy)),
            Self::AbsoluteIndexed(xy) => f.write_fmt(format_args!("abs,{:?}", xy)),
            Self::IndexedIndirect => f.write_str("X,ind"),
            Self::IndirectIndexed => f.write_str("ind,Y"),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Instruction {
    Mode(Mode, Micro),
    Branch,

    // outliers
    JMPabs,
    JMPind,
    BRK,
    RTI,
    JSR,
    RTS,
    PHA,
    PHP,
    PLA,
    PLP,

    JAM,
}

impl Display for Instruction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Mode(mode, mi) => f.write_fmt(format_args!("{:?} {}", mi, mode)),
            Self::Branch => f.write_str("rel"),
            Self::JMPabs => f.write_str("JMP abs"),
            Self::JMPind => f.write_str("JMP ind"),
            Self::JSR => f.write_str("JSR abs"),
            Self::JAM => f.write_str("JAM"),
            _ => f.write_fmt(format_args!("{:?} impl", self)),
        }
    }
}

impl From<u8> for Instruction {
    fn from(op: u8) -> Self {
        match op {
            0x10 | 0x30 | 0x50 | 0x70 => Instruction::Branch,
            0x90 | 0xB0 | 0xD0 | 0xF0 => Instruction::Branch,

            0x00 => Instruction::BRK,
            0x08 => Instruction::PHP,
            0x18 => Instruction::Mode(Mode::Implied, Micro::CLC),
            0x20 => Instruction::JSR,
            0x28 => Instruction::PLP,
            0x38 => Instruction::Mode(Mode::Implied, Micro::SEC),
            0x40 => Instruction::RTI,
            0x4C => Instruction::JMPabs,
            0x60 => Instruction::RTS,
            0x48 => Instruction::PHA,
            0x58 => Instruction::Mode(Mode::Implied, Micro::CLI),
            0x68 => Instruction::PLA,
            0x6C => Instruction::JMPind,
            0x78 => Instruction::Mode(Mode::Implied, Micro::SEI),
            0x88 => Instruction::Mode(Mode::Implied, Micro::DEY),
            0x8A => Instruction::Mode(Mode::Implied, Micro::TXA),
            0x98 => Instruction::Mode(Mode::Implied, Micro::TYA),
            0x9A => Instruction::Mode(Mode::Implied, Micro::TXS),
            0xA8 => Instruction::Mode(Mode::Implied, Micro::TAY),
            0xAA => Instruction::Mode(Mode::Implied, Micro::TAX),
            0xB8 => Instruction::Mode(Mode::Implied, Micro::CLV),
            0xBA => Instruction::Mode(Mode::Implied, Micro::TSX),
            0xC8 => Instruction::Mode(Mode::Implied, Micro::INY),
            0xCA => Instruction::Mode(Mode::Implied, Micro::DEX),
            0xD8 => Instruction::Mode(Mode::Implied, Micro::CLD),
            0xE8 => Instruction::Mode(Mode::Implied, Micro::INX),
            0xEA => Instruction::Mode(Mode::Implied, Micro::NOP),
            0xF8 => Instruction::Mode(Mode::Implied, Micro::SED),

            // illegal
            0xBB => Instruction::Mode(Mode::AbsoluteIndexed(Index::Y), Micro::LAS),
            0xEB => Instruction::Mode(Mode::Immediate, Micro::SBC),

            // unstable
            0x9B => Instruction::Mode(Mode::AbsoluteIndexed(Index::Y), Micro::TAS),
            0x93 => Instruction::Mode(Mode::IndirectIndexed, Micro::SHA),
            0x9C => Instruction::Mode(Mode::AbsoluteIndexed(Index::X), Micro::SHY),
            0x9E => Instruction::Mode(Mode::AbsoluteIndexed(Index::Y), Micro::SHX),
            0x9F => Instruction::Mode(Mode::AbsoluteIndexed(Index::Y), Micro::SHA),

            // highly unstable
            0xAB => Instruction::Mode(Mode::Immediate, Micro::LXA),
            0x8B => Instruction::Mode(Mode::Immediate, Micro::ANE),

            _ => Instruction::Mode(
                match op & 0b11101 {
                    0b00000 => {
                        if op & 0b10000000 == 0 {
                            return Instruction::JAM;
                        } else {
                            Mode::Immediate
                        }
                    }
                    0b00001 => Mode::IndexedIndirect,
                    0b00100 => Mode::ZeroPage,
                    0b00101 => Mode::ZeroPage,
                    0b01000 => Mode::Accumulator,
                    0b01001 => Mode::Immediate,
                    0b01100 => Mode::Absolute,
                    0b01101 => Mode::Absolute,
                    0b10000 => return Instruction::JAM,
                    0b10001 => Mode::IndirectIndexed,
                    0b10100 => Mode::ZeroPageIndexed(Index::op(op)),
                    0b10101 => Mode::ZeroPageIndexed(Index::op(op)),
                    0b11000 => return Instruction::Mode(Mode::Implied, Micro::NOP),
                    0b11001 => Mode::AbsoluteIndexed(Index::Y),
                    0b11100 => Mode::AbsoluteIndexed(Index::op(op)),
                    0b11101 => Mode::AbsoluteIndexed(Index::op(op)),
                    _ => unreachable!(),
                },
                match op & 0b11100011 {
                    0x00 => Micro::NOP,
                    0x01 => Micro::ORA,
                    0x02 => Micro::ASL,
                    0x20 => {
                        if op & 0b10000 == 0 {
                            Micro::BIT
                        } else {
                            Micro::NOP
                        }
                    }
                    0x21 => Micro::AND,
                    0x22 => Micro::ROL,
                    0x40 => Micro::NOP,
                    0x41 => Micro::EOR,
                    0x42 => Micro::LSR,
                    0x60 => Micro::NOP,
                    0x61 => Micro::ADC,
                    0x62 => Micro::ROR,
                    0x80 => Micro::STY,
                    0x81 => Micro::STA,
                    0x82 => {
                        if op & 0b100 == 0 {
                            Micro::NOP
                        } else {
                            Micro::STX
                        }
                    }
                    0xA0 => Micro::LDY,
                    0xA1 => Micro::LDA,
                    0xA2 => Micro::LDX,
                    0xC0 => {
                        if op & 0b10000 == 0 {
                            Micro::CPY
                        } else {
                            Micro::NOP
                        }
                    }
                    0xC1 => Micro::CMP,
                    0xC2 => {
                        if op & 0b100 == 0 {
                            Micro::NOP
                        } else {
                            Micro::DEC
                        }
                    }
                    0xE0 => {
                        if op & 0b10000 == 0 {
                            Micro::CPX
                        } else {
                            Micro::NOP
                        }
                    }
                    0xE1 => Micro::SBC,
                    0xE2 => {
                        if op & 0b100 == 0 {
                            Micro::NOP
                        } else {
                            Micro::INC
                        }
                    }

                    // illegal
                    0x03 => Micro::SLO,
                    0x23 => Micro::RLA,
                    0x43 => Micro::SRE,
                    0x63 => Micro::RRA,
                    0x83 => Micro::SAX,
                    0xA3 => Micro::LAX,
                    0xC3 => Micro::DCP,
                    0xE3 => Micro::ISC,
                    _ => unreachable!(),
                },
            ),
        }
    }
}
