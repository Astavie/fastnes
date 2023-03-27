use std::{
    fmt::{Debug, Display},
    fs::read_to_string,
};

pub trait PPU {
    fn write(&mut self, addr: u16, data: u8);
    fn read(&mut self, addr: u16) -> u8;
}

#[allow(non_snake_case)]
pub struct MOS6502<P: PPU> {
    // instruction list
    instructions: [Instruction; 256],

    // rom
    ram: [u8; 2048],
    rom: [u8; 16384],

    // ppu
    cycle: usize,
    ppu: P,

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

struct DummyPPU();

impl PPU for DummyPPU {
    fn write(&mut self, _addr: u16, _data: u8) {
        // todo!()
    }
    fn read(&mut self, _addr: u16) -> u8 {
        todo!()
    }
}

pub(crate) fn nestest<const TEST: bool>(rom: [u8; 16384], instr: [Instruction; 256]) {
    let mut cpu = MOS6502::new(rom, instr, DummyPPU());
    cpu.reset();

    if TEST {
        let log = read_to_string("test/nestest/nestest.log").unwrap();
        let mut lines = log.split('\n');
        let mut lineno = 1;

        loop {
            cpu.instruction();

            let line = lines.next().unwrap();
            let pc = u16::from_str_radix(&line[0..4], 16).unwrap();
            let cyc = usize::from_str_radix(line.rsplit_once(':').unwrap().1, 10).unwrap();

            // println!("cpu      : ${:04X} at CYC:{}", cpu.PC, cpu.cycle);
            // println!("line {:04}: ${:04X} at CYC:{}", lineno, pc, cyc);

            assert_eq!(cpu.PC, pc);
            assert_eq!(cpu.cycle, cyc);

            lineno += 1;

            if lineno == 8991 {
                break;
            }
        }
    } else {
        let max_cycles = 26548;
        while cpu.cycle < max_cycles {
            cpu.instruction();
        }
    }

    assert_eq!(cpu.cycle, 26548);
    assert_eq!(cpu.PC, 0xC6A2);
}

pub fn instructions() -> [Instruction; 256] {
    (0..=255)
        .map(Instruction::from)
        .collect::<Vec<Instruction>>()
        .try_into()
        .unwrap()
}

trait Addressable {
    fn poke<P: PPU>(&self, cpu: &mut MOS6502<P>);
    fn read<P: PPU>(&self, cpu: &mut MOS6502<P>) -> u8;
    fn write<P: PPU>(&self, cpu: &mut MOS6502<P>, data: u8);

    fn read_flags<P: PPU>(&self, cpu: &mut MOS6502<P>) -> u8 {
        let data = self.read(cpu);
        cpu.flags(data);
        data
    }

    fn compare<P: PPU>(&self, cpu: &mut MOS6502<P>, reg: u8) {
        let data = self.read(cpu);
        cpu.compare(data, reg);
    }
}

struct Accumulator();
struct Implied();

impl Addressable for Accumulator {
    fn poke<P: PPU>(&self, _cpu: &mut MOS6502<P>) {}
    fn read<P: PPU>(&self, cpu: &mut MOS6502<P>) -> u8 {
        cpu.A
    }
    fn write<P: PPU>(&self, cpu: &mut MOS6502<P>, data: u8) {
        cpu.A = data;
    }
}

impl Addressable for Implied {
    fn poke<P: PPU>(&self, _cpu: &mut MOS6502<P>) {}
    fn read<P: PPU>(&self, _cpu: &mut MOS6502<P>) -> u8 {
        unreachable!();
    }
    fn write<P: PPU>(&self, _cpu: &mut MOS6502<P>, _data: u8) {
        unreachable!();
    }
}

impl Addressable for u16 {
    fn poke<P: PPU>(&self, cpu: &mut MOS6502<P>) {
        cpu.read_addr(*self);
    }
    fn read<P: PPU>(&self, cpu: &mut MOS6502<P>) -> u8 {
        cpu.read_addr(*self)
    }
    fn write<P: PPU>(&self, cpu: &mut MOS6502<P>, data: u8) {
        cpu.write_addr(*self, data)
    }
}

impl Micro {
    fn run<P: PPU, A: Addressable>(self, cpu: &mut MOS6502<P>, addr: A) {
        match self {
            Micro::ASL => {
                let data = addr.read(cpu);
                addr.write(cpu, data);

                cpu.P &= 0b11111110;
                cpu.P |= data >> 7;
                let data = cpu.flags(data << 1);
                addr.write(cpu, data);
            }
            Micro::LSR => {
                let data = addr.read(cpu);
                addr.write(cpu, data);

                cpu.P &= 0b11111110;
                cpu.P |= data & 1;
                let data = cpu.flags(data >> 1);
                addr.write(cpu, data);
            }
            Micro::ROL => {
                let data = addr.read(cpu);
                addr.write(cpu, data);

                let carry = cpu.P & 1;
                cpu.P &= 0b11111110;
                cpu.P |= data >> 7;
                let data = cpu.flags(data << 1 | carry);
                addr.write(cpu, data);
            }
            Micro::ROR => {
                let data = addr.read(cpu);
                addr.write(cpu, data);

                let carry = cpu.P << 7;
                cpu.P &= 0b11111110;
                cpu.P |= data & 1;
                let data = cpu.flags(data >> 1 | carry);
                addr.write(cpu, data);
            }
            Micro::INC => {
                let data = addr.read(cpu);
                addr.write(cpu, data);
                let data = cpu.flags(data.wrapping_add(1));
                addr.write(cpu, data);
            }
            Micro::DEC => {
                let data = addr.read(cpu);
                addr.write(cpu, data);
                let data = cpu.flags(data.wrapping_sub(1));
                addr.write(cpu, data);
            }
            Micro::AND => {
                cpu.A &= addr.read(cpu);
                cpu.flags(cpu.A);
            }
            Micro::EOR => {
                cpu.A ^= addr.read(cpu);
                cpu.flags(cpu.A);
            }
            Micro::ORA => {
                cpu.A |= addr.read(cpu);
                cpu.flags(cpu.A);
            }
            Micro::BIT => {
                let data = addr.read(cpu);

                cpu.P &= 0b00111101;

                // Negative and oVerflow flags
                cpu.P |= data & 0b11000000;

                // Zero flag
                if cpu.A & data == 0 {
                    cpu.P |= 0b00000010;
                }
            }
            Micro::ADC => {
                let data = addr.read(cpu);
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
                let data = addr.read(cpu);
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
            Micro::STA => addr.write(cpu, cpu.A),
            Micro::STY => addr.write(cpu, cpu.Y),
            Micro::STX => addr.write(cpu, cpu.X),
            Micro::LDA => cpu.A = addr.read_flags(cpu),
            Micro::LDY => cpu.Y = addr.read_flags(cpu),
            Micro::LDX => cpu.X = addr.read_flags(cpu),
            Micro::CPX => addr.compare(cpu, cpu.X),
            Micro::CPY => addr.compare(cpu, cpu.Y),
            Micro::CMP => addr.compare(cpu, cpu.A),
            Micro::NOP => _ = addr.poke(cpu),
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
                let data = addr.read(cpu);

                // ASL
                addr.write(cpu, data);

                cpu.P &= 0b11111110;
                cpu.P |= data >> 7;
                let data = data << 1;
                addr.write(cpu, data);

                // ORA
                cpu.A |= data;
                cpu.flags(cpu.A);
            }
            Micro::SRE => {
                let data = addr.read(cpu);

                // LSR
                addr.write(cpu, data);

                cpu.P &= 0b11111110;
                cpu.P |= data & 1;
                let data = data >> 1;
                addr.write(cpu, data);

                // EOR
                cpu.A ^= data;
                cpu.flags(cpu.A);
            }
            Micro::RLA => {
                let data = addr.read(cpu);

                // ROL
                addr.write(cpu, data);

                let carry = cpu.P & 1;
                cpu.P &= 0b11111110;
                cpu.P |= data >> 7;
                let data = data << 1 | carry;
                addr.write(cpu, data);

                // AND
                cpu.A &= data;
                cpu.flags(cpu.A);
            }
            Micro::RRA => {
                let data = addr.read(cpu);

                // ROR
                addr.write(cpu, data);

                let carry = cpu.P << 7;
                cpu.P &= 0b11111110;
                cpu.P |= data & 1;
                let data = cpu.flags(data >> 1 | carry);
                addr.write(cpu, data);

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
                let data = addr.read(cpu);

                // INC
                addr.write(cpu, data);
                let data = data.wrapping_add(1);
                addr.write(cpu, data);

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
                let data = addr.read(cpu);

                // DEC
                addr.write(cpu, data);
                let data = data.wrapping_sub(1);
                addr.write(cpu, data);

                // CMP
                let (val, carry) = cpu.A.overflowing_sub(data);
                cpu.P &= 0b11111110;
                if !carry {
                    cpu.P |= 0b00000001;
                }
                cpu.flags(val);
            }
            Micro::LAX => {
                cpu.A = addr.read_flags(cpu);
                cpu.X = cpu.A;
            }
            Micro::SAX => addr.write(cpu, cpu.A & cpu.X),
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

impl<P: PPU> MOS6502<P> {
    // fn poll(&mut self) {
    //     // sample irq and nmi (nmi stays on while irq gets reset every cycle)
    //     self.irq_sample = self.irq > 0;
    //     self.nmi_sample = self.nmi_sample || self.nmi;
    //     self.nmi = false;
    // }
    pub fn instruction(&mut self) {
        // get instruction to perform
        let op = self.read_pc();
        let instr = self.instructions[if self.hardware_interrupt {
            0
        } else {
            usize::from(op)
        }];

        // run instruction
        match instr {
            Instruction::Mode(mode, mi) => match mode {
                Mode::Accumulator => {
                    self.peek_pc();
                    mi.run(self, Accumulator());
                    // self.poll();
                }
                Mode::Implied => {
                    self.peek_pc();
                    mi.run(self, Implied());
                    // self.poll();
                }
                Mode::Immediate => {
                    mi.run(self, self.PC);
                    self.PC = self.PC.wrapping_add(1);
                    // self.poll();
                }
                Mode::Absolute => {
                    let lo = self.read_pc();
                    let hi = self.read_pc();
                    let addr = u16::from(lo) | u16::from(hi) << 8;
                    mi.run(self, addr);
                    // self.poll();
                }
                Mode::ZeroPage => {
                    let addr = u16::from(self.read_pc());
                    mi.run(self, addr);
                    // self.poll();
                }
                Mode::ZeroPageIndexed(i) => {
                    let ptr = self.read_pc();
                    self.read_addr(u16::from(ptr));

                    let addr = u16::from(ptr.wrapping_add(i.get(self)));
                    mi.run(self, addr);
                    // self.poll();
                }
                Mode::AbsoluteIndexed(i) => {
                    let reg = i.get(self);

                    let lo = self.read_pc();
                    let hi = self.read_pc();
                    let addr = u16::from(lo.wrapping_add(reg)) | u16::from(hi) << 8;

                    if (addr as u8) < reg {
                        self.read_addr(addr);
                        mi.run(self, addr.wrapping_add(0x0100));
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
                                self.read_addr(addr);
                            }
                        }
                        mi.run(self, addr);
                    }

                    // self.poll();
                }
                Mode::IndexedIndirect => {
                    let ptr = self.read_pc();
                    self.read_ptr_lo(ptr);

                    let ptr = ptr.wrapping_add(self.X);
                    let lo = self.read_ptr_lo(ptr);
                    let hi = self.read_ptr_hi(ptr);
                    let addr = u16::from(lo) | u16::from(hi) << 8;
                    mi.run(self, addr);
                    // self.poll();
                }
                Mode::IndirectIndexed => {
                    let ptr = self.read_pc();
                    let lo = self.read_ptr_lo(ptr);
                    let hi = self.read_ptr_hi(ptr);
                    let addr = u16::from(lo.wrapping_add(self.Y)) | u16::from(hi) << 8;

                    if (addr as u8) < self.Y {
                        self.read_addr(addr);
                        mi.run(self, addr.wrapping_add(0x0100));
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
                                self.read_addr(addr);
                            }
                        }
                        mi.run(self, addr);
                    }

                    // self.poll();
                }
            },
            Instruction::Branch => {
                let oper = self.read_pc();
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
                    self.peek_pc();
                    self.PC = (self.PC & 0xFF00) | (addr & 0x00FF);

                    if self.PC != addr {
                        // fix PC
                        self.peek_pc();
                        self.PC = addr;

                        // self.poll();
                    } else {
                        // no polling!
                    }
                } else {
                    // self.poll();
                }
            }
            Instruction::JMPabs => {
                let lo = self.read_pc();
                let hi = self.read_pc();
                self.PC = u16::from(lo) | u16::from(hi) << 8;

                // self.poll();
            }
            Instruction::JMPind => {
                let lo = self.read_pc();
                let hi = self.read_pc();
                let addr_lo = u16::from(lo) | u16::from(hi) << 8;
                let addr_hi = u16::from(lo.wrapping_add(1)) | u16::from(hi) << 8;

                let lo = self.read_addr(addr_lo);
                let hi = self.read_addr(addr_hi);
                self.PC = u16::from(lo) | u16::from(hi) << 8;

                // self.poll();
            }
            Instruction::BRK => {
                self.read_pc();
                self.push_hi();
                self.push_lo();

                // poll address
                // self.poll();
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
                self.push(self.P | b);

                // interrupt disable
                self.P |= 0b00000100;

                let lo = self.read_addr(addr);
                let hi = self.read_addr(addr | 1);
                self.PC = u16::from(lo) | u16::from(hi) << 8;
                self.hardware_interrupt = false;
            }
            Instruction::RTI => {
                self.peek_pc();
                self.pop();
                self.P = self.pop() & 0b11001111;
                let lo = self.pop();
                let hi = self.peek();
                self.PC = u16::from(lo) | u16::from(hi) << 8;

                // self.poll();
            }
            Instruction::JSR => {
                let lo = self.read_pc();
                self.peek();
                self.push_hi();
                self.push_lo();
                let hi = self.read_pc();
                self.PC = u16::from(lo) | u16::from(hi) << 8;

                // self.poll();
            }
            Instruction::RTS => {
                self.peek_pc();
                self.pop();
                let lo = self.pop();
                let hi = self.peek();
                self.PC = u16::from(lo) | u16::from(hi) << 8;
                self.read_pc();

                // self.poll();
            }
            Instruction::PHA => {
                self.peek_pc();
                self.push(self.A);

                // self.poll();
            }
            Instruction::PHP => {
                self.peek_pc();
                self.push(self.P | 0b00110000);

                // self.poll();
            }
            Instruction::PLA => {
                self.peek_pc();
                self.pop();
                self.A = self.peek();
                self.flags(self.A);

                // self.poll();
            }
            Instruction::PLP => {
                self.peek_pc();
                self.pop();
                self.P = self.peek() & 0b11001111;

                // self.poll();
            }
            Instruction::JAM => todo!(),
        }
    }

    pub fn reset(&mut self) {
        self.res_sample = true;
        self.hardware_interrupt = true;
    }
    pub fn new(rom: [u8; 16384], instructions: [Instruction; 256], ppu: P) -> Self {
        MOS6502 {
            instructions,
            PC: 0,
            SP: 0,
            P: 0,
            A: 0,
            X: 0,
            Y: 0,
            cycle: 0,
            ram: [0; 2048],
            rom,
            ppu,
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
    fn compare(&mut self, data: u8, reg: u8) {
        let (val, carry) = reg.overflowing_sub(data);
        self.P &= 0b11111110;
        if !carry {
            self.P |= 0b00000001;
        }
        self.flags(val);
    }
    fn write_addr(&mut self, addr: u16, data: u8) {
        if !self.res_sample {
            self.cycle += 1;
            if addr < 0x0800 {
                self.ram[usize::from(addr)] = data;
            } else if addr < 0x6000 {
                self.ppu.write(addr, data);
            }
        } else {
            self.read_addr(addr);
        }
    }
    fn read_addr(&mut self, addr: u16) -> u8 {
        self.cycle += 1;
        if addr < 0x0800 {
            self.ram[usize::from(addr)]
        } else if addr >= 0x6000 {
            self.rom[usize::from(addr & 0x3FFF)]
        } else {
            self.ppu.read(addr)
        }
    }
    fn read_ptr_lo(&mut self, ptr: u8) -> u8 {
        self.read_addr(u16::from(ptr))
    }
    fn read_ptr_hi(&mut self, ptr: u8) -> u8 {
        self.read_addr(u16::from(ptr.wrapping_add(1)))
    }
    fn read_pc(&mut self) -> u8 {
        let d = self.read_addr(self.PC);
        if !self.hardware_interrupt {
            self.PC += 1;
        }
        d
    }
    fn peek_pc(&mut self) -> u8 {
        self.read_addr(self.PC)
    }
    fn push(&mut self, data: u8) {
        self.cycle += 1;
        if !self.res_sample {
            self.ram[usize::from(0x0100 | u16::from(self.SP))] = data;
        }
        self.SP = self.SP.wrapping_sub(1);
    }
    fn push_lo(&mut self) {
        self.push(self.PC as u8);
    }
    fn push_hi(&mut self) {
        self.push((self.PC >> 8) as u8);
    }
    fn pop(&mut self) -> u8 {
        let data = self.peek();
        self.SP = self.SP.wrapping_add(1);
        data
    }
    fn peek(&mut self) -> u8 {
        self.cycle += 1;
        self.ram[usize::from(0x0100 | u16::from(self.SP))]
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
    fn get<P: PPU>(self, cpu: &MOS6502<P>) -> u8 {
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
