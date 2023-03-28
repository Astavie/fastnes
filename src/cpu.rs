use std::{fmt::Debug, rc::Rc};

use repeated::repeated;

pub trait PPU {
    fn write(&mut self, cycle: usize, addr: u16, data: u8);
    fn read(&mut self, cycle: usize, addr: u16) -> u8;

    fn nmi(&mut self, cycle: usize) -> bool;
    fn new() -> Self;
}

type Instruction<P> = fn(&mut CPU<P>);
type Instructions<P> = Rc<[Instruction<P>; 256]>;

#[allow(non_snake_case)]
pub struct CPU<P: PPU> {
    // instruction list
    pub(crate) instructions: Instructions<P>,

    // rom
    ram_internal: [u8; 0x0800],
    pub(crate) ram_work: [u8; 0x2000],
    rom: [u8; 0x8000],

    // registers
    pub(crate) PC: u16,
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

    // ppu
    pub(crate) ppu_cycle: usize,
    ppu: P,
}

pub(crate) struct DummyPPU;

pub(crate) struct FastPPU {
    open: u8,
    odd_frame: bool,

    nmi_output: bool,
    background: bool,

    this_vbl: Option<usize>,
    next_vbl: usize,
    next_nmi: Option<usize>,
}

const SCANLINE: usize = 341;
const LINES: usize = 262;

impl PPU for DummyPPU {
    fn write(&mut self, _cycle: usize, _addr: u16, _data: u8) {}
    fn read(&mut self, _cycle: usize, _addr: u16) -> u8 {
        0
    }
    fn nmi(&mut self, _cycle: usize) -> bool {
        false
    }
    fn new() -> Self {
        DummyPPU
    }
}

impl FastPPU {
    fn sync(&mut self, cycle: usize) {
        // get next vblank
        while cycle >= self.next_vbl {
            self.this_vbl = Some(self.next_vbl);
            self.odd_frame = !self.odd_frame;
            self.next_vbl += LINES * SCANLINE - usize::from(self.background && self.odd_frame);
        }

        // end vblank after 20 scanlines
        self.this_vbl = self.this_vbl.filter(|i| cycle <= i + 20 * SCANLINE);

        // get next nmi signal
        self.next_nmi();
    }
    fn next_nmi(&mut self) {
        self.next_nmi = if self.nmi_output {
            Some(self.this_vbl.unwrap_or(self.next_vbl))
        } else {
            None
        };
    }
}

fn on_change(var: &mut bool, val: bool) -> Option<bool> {
    if *var != val {
        *var = val;
        Some(val)
    } else {
        None
    }
}

impl PPU for FastPPU {
    fn new() -> Self {
        FastPPU {
            next_vbl: 241 * SCANLINE + 1,
            this_vbl: None,
            next_nmi: None,
            nmi_output: false,
            odd_frame: false,
            background: false,
            open: 0,
        }
    }

    // we inline never because this tends to get inlined inside write_addr,
    // making every instruction slower just to make ppu writes a bit faster
    #[inline(never)]
    fn write(&mut self, cycle: usize, addr: u16, data: u8) {
        self.open = data;
        match addr & 7 {
            0 => {
                // TODO: the rest

                // NMI
                match on_change(&mut self.nmi_output, data & 0b10000000 != 0) {
                    Some(true) => {
                        // enable
                        self.sync(cycle);
                        self.next_nmi = Some(
                            // ignore last dot of vblank
                            self.this_vbl
                                .filter(|i| cycle < i + 20 * SCANLINE)
                                .unwrap_or(self.next_vbl),
                        );
                    }
                    Some(false) => {
                        // disable
                        self.next_nmi = None;
                    }
                    None => (),
                }
            }
            1 => {
                // TODO: the rest

                // background
                self.sync(cycle);
                match on_change(&mut self.background, data & 0b1000 != 0) {
                    Some(true) => {
                        // enable background
                        if self.odd_frame && cycle < self.next_vbl - 241 * SCANLINE - 3 {
                            self.next_vbl -= 1;
                            self.next_nmi();
                        }
                    }
                    Some(false) => {
                        // disable background
                        if self.odd_frame && cycle < self.next_vbl - 241 * SCANLINE - 2 {
                            self.next_vbl += 1;
                            self.next_nmi();
                        }
                    }
                    None => (),
                }
            }
            _ => {
                // TODO
            }
        }
    }

    // we inline never because this tends to get inlined inside read_addr,
    // making every instruction slower just to make ppu reads a bit faster
    #[inline(never)]
    fn read(&mut self, cycle: usize, addr: u16) -> u8 {
        match addr & 7 {
            2 => {
                // TODO: sprite overflow and sprite 0 hit
                self.open &= 0b00011111;

                // vblank
                self.sync(cycle);

                if self.this_vbl.is_some_and(|i| cycle != i) {
                    self.open |= 0b10000000;
                }

                if self.this_vbl.is_some() {
                    self.this_vbl = None;
                    self.next_nmi = None;
                }
            }
            _ => {
                // TODO
            }
        };
        self.open
    }

    fn nmi(&mut self, cycle: usize) -> bool {
        if self.next_nmi.is_some_and(|i| cycle >= i) {
            self.sync(cycle);
            self.next_nmi = Some(self.next_vbl);
            true
        } else {
            false
        }
    }
}

trait Addressable {
    fn poke<P: PPU>(&self, cpu: &mut CPU<P>);
    fn read<P: PPU>(&self, cpu: &mut CPU<P>) -> u8;
    fn write<P: PPU>(&self, cpu: &mut CPU<P>, data: u8);

    fn read_flags<P: PPU>(&self, cpu: &mut CPU<P>) -> u8 {
        let data = self.read(cpu);
        cpu.flags(data);
        data
    }

    fn compare<P: PPU>(&self, cpu: &mut CPU<P>, reg: u8) {
        let data = self.read(cpu);
        cpu.compare(data, reg);
    }
}

struct Accumulator();
struct Implied();

impl Addressable for Accumulator {
    fn poke<P: PPU>(&self, _cpu: &mut CPU<P>) {}
    fn read<P: PPU>(&self, cpu: &mut CPU<P>) -> u8 {
        cpu.A
    }
    fn write<P: PPU>(&self, cpu: &mut CPU<P>, data: u8) {
        cpu.A = data;
    }
}

impl Addressable for Implied {
    fn poke<P: PPU>(&self, _cpu: &mut CPU<P>) {}
    fn read<P: PPU>(&self, _cpu: &mut CPU<P>) -> u8 {
        unreachable!();
    }
    fn write<P: PPU>(&self, _cpu: &mut CPU<P>, _data: u8) {
        unreachable!();
    }
}

impl Addressable for u16 {
    fn poke<P: PPU>(&self, cpu: &mut CPU<P>) {
        cpu.read_addr(*self);
    }
    fn read<P: PPU>(&self, cpu: &mut CPU<P>) -> u8 {
        cpu.read_addr(*self)
    }
    fn write<P: PPU>(&self, cpu: &mut CPU<P>, data: u8) {
        cpu.write_addr(*self, data)
    }
}

impl Micro {
    const fn from<const OP: u8>() -> Micro {
        if OP & 0b11101 == 0b11000 {
            Micro::NOP
        } else {
            match OP & 0b11100011 {
                0x00 => Micro::NOP,
                0x01 => Micro::ORA,
                0x02 => Micro::ASL,
                0x20 => {
                    if OP & 0b10000 == 0 {
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
                    if OP & 0b100 == 0 {
                        Micro::NOP
                    } else {
                        Micro::STX
                    }
                }
                0xA0 => Micro::LDY,
                0xA1 => Micro::LDA,
                0xA2 => Micro::LDX,
                0xC0 => {
                    if OP & 0b10000 == 0 {
                        Micro::CPY
                    } else {
                        Micro::NOP
                    }
                }
                0xC1 => Micro::CMP,
                0xC2 => {
                    if OP & 0b100 == 0 {
                        Micro::NOP
                    } else {
                        Micro::DEC
                    }
                }
                0xE0 => {
                    if OP & 0b10000 == 0 {
                        Micro::CPX
                    } else {
                        Micro::NOP
                    }
                }
                0xE1 => Micro::SBC,
                0xE2 => {
                    if OP & 0b100 == 0 {
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
            }
        }
    }
    const fn instruction<P: PPU, A: Addressable, const S: Micro>() -> fn(&mut CPU<P>, A) {
        match S {
            Micro::ASL => |cpu, addr| {
                let data = addr.read(cpu);
                addr.write(cpu, data);

                cpu.P &= 0b11111110;
                cpu.P |= data >> 7;
                let data = cpu.flags(data << 1);
                addr.write(cpu, data);
            },
            Micro::LSR => |cpu, addr| {
                let data = addr.read(cpu);
                addr.write(cpu, data);

                cpu.P &= 0b11111110;
                cpu.P |= data & 1;
                let data = cpu.flags(data >> 1);
                addr.write(cpu, data);
            },
            Micro::ROL => |cpu, addr| {
                let data = addr.read(cpu);
                addr.write(cpu, data);

                let carry = cpu.P & 1;
                cpu.P &= 0b11111110;
                cpu.P |= data >> 7;
                let data = cpu.flags(data << 1 | carry);
                addr.write(cpu, data);
            },
            Micro::ROR => |cpu, addr| {
                let data = addr.read(cpu);
                addr.write(cpu, data);

                let carry = cpu.P << 7;
                cpu.P &= 0b11111110;
                cpu.P |= data & 1;
                let data = cpu.flags(data >> 1 | carry);
                addr.write(cpu, data);
            },
            Micro::INC => |cpu, addr| {
                let data = addr.read(cpu);
                addr.write(cpu, data);
                let data = cpu.flags(data.wrapping_add(1));
                addr.write(cpu, data);
            },
            Micro::DEC => |cpu, addr| {
                let data = addr.read(cpu);
                addr.write(cpu, data);
                let data = cpu.flags(data.wrapping_sub(1));
                addr.write(cpu, data);
            },
            Micro::AND => |cpu, addr| {
                cpu.A &= addr.read(cpu);
                cpu.flags(cpu.A);
            },
            Micro::EOR => |cpu, addr| {
                cpu.A ^= addr.read(cpu);
                cpu.flags(cpu.A);
            },
            Micro::ORA => |cpu, addr| {
                cpu.A |= addr.read(cpu);
                cpu.flags(cpu.A);
            },
            Micro::BIT => |cpu, addr| {
                let data = addr.read(cpu);

                cpu.P &= 0b00111101;

                // Negative and oVerflow flags
                cpu.P |= data & 0b11000000;

                // Zero flag
                if cpu.A & data == 0 {
                    cpu.P |= 0b00000010;
                }
            },
            Micro::ADC => |cpu, addr| {
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
            },
            Micro::SBC => |cpu, addr| {
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
            },
            Micro::STA => |cpu, addr| addr.write(cpu, cpu.A),
            Micro::STY => |cpu, addr| addr.write(cpu, cpu.Y),
            Micro::STX => |cpu, addr| addr.write(cpu, cpu.X),
            Micro::LDA => |cpu, addr| cpu.A = addr.read_flags(cpu),
            Micro::LDY => |cpu, addr| cpu.Y = addr.read_flags(cpu),
            Micro::LDX => |cpu, addr| cpu.X = addr.read_flags(cpu),
            Micro::CPX => |cpu, addr| addr.compare(cpu, cpu.X),
            Micro::CPY => |cpu, addr| addr.compare(cpu, cpu.Y),
            Micro::CMP => |cpu, addr| addr.compare(cpu, cpu.A),
            Micro::NOP => |cpu, addr| addr.poke(cpu),

            Micro::SEC => |cpu, _addr| cpu.P |= 0b00000001,
            Micro::SEI => |cpu, _addr| cpu.P |= 0b00000100,
            Micro::SED => |cpu, _addr| cpu.P |= 0b00001000,
            Micro::CLC => |cpu, _addr| cpu.P &= 0b11111110,
            Micro::CLI => |cpu, _addr| cpu.P &= 0b11111011,
            Micro::CLD => |cpu, _addr| cpu.P &= 0b11110111,
            Micro::CLV => |cpu, _addr| cpu.P &= 0b10111111,
            Micro::INX => |cpu, _addr| cpu.X = cpu.flags(cpu.X.wrapping_add(1)),
            Micro::DEX => |cpu, _addr| cpu.X = cpu.flags(cpu.X.wrapping_sub(1)),
            Micro::INY => |cpu, _addr| cpu.Y = cpu.flags(cpu.Y.wrapping_add(1)),
            Micro::DEY => |cpu, _addr| cpu.Y = cpu.flags(cpu.Y.wrapping_sub(1)),
            Micro::TAX => |cpu, _addr| cpu.X = cpu.flags(cpu.A),
            Micro::TXA => |cpu, _addr| cpu.A = cpu.flags(cpu.X),
            Micro::TAY => |cpu, _addr| cpu.Y = cpu.flags(cpu.A),
            Micro::TYA => |cpu, _addr| cpu.A = cpu.flags(cpu.Y),
            Micro::TSX => |cpu, _addr| cpu.X = cpu.flags(cpu.SP),
            Micro::TXS => |cpu, _addr| cpu.SP = cpu.X,

            Micro::SLO => |cpu, addr| {
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
            },
            Micro::SRE => |cpu, addr| {
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
            },
            Micro::RLA => |cpu, addr| {
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
            },
            Micro::RRA => |cpu, addr| {
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
            },
            Micro::ISC => |cpu, addr| {
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
            },
            Micro::DCP => |cpu, addr| {
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
            },
            Micro::LAX => |cpu, addr| {
                cpu.A = addr.read_flags(cpu);
                cpu.X = cpu.A;
            },
            Micro::SAX => |cpu, addr| addr.write(cpu, cpu.A & cpu.X),

            Micro::LAS => |_cpu, _addr| todo!(),
            Micro::TAS => |_cpu, _addr| todo!(),
            Micro::SHA => |_cpu, _addr| todo!(),
            Micro::SHY => |_cpu, _addr| todo!(),
            Micro::SHX => |_cpu, _addr| todo!(),
            Micro::ANE => |_cpu, _addr| todo!(),
            Micro::LXA => |_cpu, _addr| todo!(),
        }
    }
}

impl<P: PPU> CPU<P> {
    fn poll(&mut self) {
        // sample irq and nmi (nmi stays on while irq gets reset every cycle)
        // self.irq_sample = self.irq > 0;
        self.nmi_sample = self.nmi_sample || self.ppu.nmi(self.ppu_cycle);
    }
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
        self.ppu = P::new();
    }
    pub fn new(rom: [u8; 32768]) -> Self {
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
            ram_work: [0; 0x2000],
            rom,
            ppu: P::new(),
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
            self.ppu_cycle += 3;
            match addr & 0xE000 {
                0x0000 => self.ram_internal[usize::from(addr & 0x07FF)] = data,
                0x2000 => self.ppu.write(self.ppu_cycle, addr, data),
                0x4000 => (), // TODO
                0x6000 => self.ram_work[usize::from(addr & 0x1FFF)] = data,
                _ => (),
            }
        } else {
            self.read_addr(addr);
        }
    }
    fn read_addr(&mut self, addr: u16) -> u8 {
        self.ppu_cycle += 3;
        match addr & 0xE000 {
            0x0000 => self.ram_internal[usize::from(addr & 0x07FF)],
            0x2000 => self.ppu.read(self.ppu_cycle, addr),
            0x4000 => 0, // TODO
            0x6000 => self.ram_work[usize::from(addr & 0x1FFF)],
            _ => self.rom[usize::from(addr & 0x7FFF)],
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
        self.ppu_cycle += 3;
        if !self.res_sample {
            self.ram_internal[usize::from(0x0100 | u16::from(self.SP))] = data;
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
        self.ppu_cycle += 3;
        self.ram_internal[usize::from(0x0100 | u16::from(self.SP))]
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
    const fn op<const OP: u8>() -> Self {
        if OP & 0b11000010 == 0b10000010 {
            // exception for STX and LDX
            Self::Y
        } else {
            Self::X
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    Jam,
}

// weird generic shit
// see https://github.com/rust-lang/rust/issues/82509#issuecomment-1165533546
struct ModeEval<const N: Mode>;
struct MicroEval<const N: Micro>;

const fn instruction<P: PPU, const OP: u8>() -> Instruction<P>
where
    ModeEval<{ Mode::from::<OP>() }>: Sized,
    MicroEval<{ Micro::from::<OP>() }>: Sized,
{
    match OP {
        0x10 => branch::<0b10000000, true, P>(),
        0x30 => branch::<0b10000000, false, P>(),
        0x50 => branch::<0b01000000, true, P>(),
        0x70 => branch::<0b01000000, false, P>(),
        0x90 => branch::<0b00000001, true, P>(),
        0xB0 => branch::<0b00000001, false, P>(),
        0xD0 => branch::<0b00000010, true, P>(),
        0xF0 => branch::<0b00000010, false, P>(),

        0x00 => |cpu| {
            // BRK
            cpu.read_pc();
            cpu.push_hi();
            cpu.push_lo();

            // poll address
            cpu.poll();
            let addr = if cpu.nmi_sample {
                cpu.nmi_sample = false;
                0xFFFA
            } else if cpu.res_sample {
                cpu.res_sample = false;
                0xFFFC
            } else {
                0xFFFE
            };

            let b = if cpu.hardware_interrupt {
                // B flag clear
                0b00100000
            } else {
                // B flag set
                0b00110000
            };
            cpu.push(cpu.P | b);

            // interrupt disable
            cpu.P |= 0b00000100;

            let lo = cpu.read_addr(addr);
            let hi = cpu.read_addr(addr | 1);
            cpu.PC = u16::from(lo) | u16::from(hi) << 8;
            cpu.hardware_interrupt = false;

            // immediately execute next instruction
            cpu.instruction();
        },
        0x08 => |cpu| {
            // PHP
            cpu.peek_pc();

            cpu.poll();

            cpu.push(cpu.P | 0b00110000);
        },
        0x20 => |cpu| {
            // JSR
            let lo = cpu.read_pc();
            cpu.peek();
            cpu.push_hi();
            cpu.push_lo();

            cpu.poll();

            let hi = cpu.read_pc();
            cpu.PC = u16::from(lo) | u16::from(hi) << 8;
        },
        0x28 => |cpu| {
            // PLP
            cpu.peek_pc();
            cpu.pop();

            cpu.poll();

            cpu.P = cpu.peek() & 0b11001111;
        },
        0x40 => |cpu| {
            // RTI
            cpu.peek_pc();
            cpu.pop();
            cpu.P = cpu.pop() & 0b11001111;
            let lo = cpu.pop();

            cpu.poll();

            let hi = cpu.peek();
            cpu.PC = u16::from(lo) | u16::from(hi) << 8;
        },
        0x4C => |cpu| {
            // JMP abs
            let lo = cpu.read_pc();

            cpu.poll();

            let hi = cpu.read_pc();
            cpu.PC = u16::from(lo) | u16::from(hi) << 8;
        },
        0x60 => |cpu| {
            // RTS
            cpu.peek_pc();
            cpu.pop();
            let lo = cpu.pop();
            let hi = cpu.peek();
            cpu.PC = u16::from(lo) | u16::from(hi) << 8;

            cpu.poll();

            cpu.read_pc();
        },
        0x48 => |cpu| {
            // PHA
            cpu.peek_pc();

            cpu.poll();

            cpu.push(cpu.A);
        },
        0x68 => |cpu| {
            // PLA
            cpu.peek_pc();
            cpu.pop();

            cpu.poll();

            cpu.A = cpu.peek();
            cpu.flags(cpu.A);
        },
        0x6C => |cpu| {
            // JMP ind
            let lo = cpu.read_pc();
            let hi = cpu.read_pc();
            let addr_lo = u16::from(lo) | u16::from(hi) << 8;
            let addr_hi = u16::from(lo.wrapping_add(1)) | u16::from(hi) << 8;

            let lo = cpu.read_addr(addr_lo);

            cpu.poll();

            let hi = cpu.read_addr(addr_hi);
            cpu.PC = u16::from(lo) | u16::from(hi) << 8;
        },

        0x18 => Mode::instruction::<P, { Mode::Implied }, { Micro::CLC }>(),
        0x38 => Mode::instruction::<P, { Mode::Implied }, { Micro::SEC }>(),
        0x58 => Mode::instruction::<P, { Mode::Implied }, { Micro::CLI }>(),
        0x78 => Mode::instruction::<P, { Mode::Implied }, { Micro::SEI }>(),
        0x88 => Mode::instruction::<P, { Mode::Implied }, { Micro::DEY }>(),
        0x8A => Mode::instruction::<P, { Mode::Implied }, { Micro::TXA }>(),
        0x98 => Mode::instruction::<P, { Mode::Implied }, { Micro::TYA }>(),
        0x9A => Mode::instruction::<P, { Mode::Implied }, { Micro::TXS }>(),
        0xA8 => Mode::instruction::<P, { Mode::Implied }, { Micro::TAY }>(),
        0xAA => Mode::instruction::<P, { Mode::Implied }, { Micro::TAX }>(),
        0xB8 => Mode::instruction::<P, { Mode::Implied }, { Micro::CLV }>(),
        0xBA => Mode::instruction::<P, { Mode::Implied }, { Micro::TSX }>(),
        0xC8 => Mode::instruction::<P, { Mode::Implied }, { Micro::INY }>(),
        0xCA => Mode::instruction::<P, { Mode::Implied }, { Micro::DEX }>(),
        0xD8 => Mode::instruction::<P, { Mode::Implied }, { Micro::CLD }>(),
        0xE8 => Mode::instruction::<P, { Mode::Implied }, { Micro::INX }>(),
        0xEA => Mode::instruction::<P, { Mode::Implied }, { Micro::NOP }>(),
        0xF8 => Mode::instruction::<P, { Mode::Implied }, { Micro::SED }>(),

        // illegal
        0xBB => Mode::instruction::<P, { Mode::AbsoluteIndexed(Index::Y) }, { Micro::LAS }>(),
        0xEB => Mode::instruction::<P, { Mode::Immediate }, { Micro::SBC }>(),

        // unstable
        0x9B => Mode::instruction::<P, { Mode::AbsoluteIndexed(Index::Y) }, { Micro::TAS }>(),
        0x93 => Mode::instruction::<P, { Mode::IndirectIndexed }, { Micro::SHA }>(),
        0x9C => Mode::instruction::<P, { Mode::AbsoluteIndexed(Index::X) }, { Micro::SHY }>(),
        0x9E => Mode::instruction::<P, { Mode::AbsoluteIndexed(Index::Y) }, { Micro::SHX }>(),
        0x9F => Mode::instruction::<P, { Mode::AbsoluteIndexed(Index::Y) }, { Micro::SHA }>(),

        // highly unstable
        0xAB => Mode::instruction::<P, { Mode::Immediate }, { Micro::LXA }>(),
        0x8B => Mode::instruction::<P, { Mode::Immediate }, { Micro::ANE }>(),

        _ => Mode::instruction::<P, { Mode::from::<OP>() }, { Micro::from::<OP>() }>(),
    }
}

impl Mode {
    const fn from<const OP: u8>() -> Mode {
        match OP & 0b11101 {
            0b00000 => {
                if OP & 0b10000000 == 0 {
                    Mode::Jam
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
            0b10000 => Mode::Jam,
            0b10001 => Mode::IndirectIndexed,
            0b10100 => Mode::ZeroPageIndexed(Index::op::<OP>()),
            0b10101 => Mode::ZeroPageIndexed(Index::op::<OP>()),
            0b11000 => Mode::Implied,
            0b11001 => Mode::AbsoluteIndexed(Index::Y),
            0b11100 => Mode::AbsoluteIndexed(Index::op::<OP>()),
            0b11101 => Mode::AbsoluteIndexed(Index::op::<OP>()),
            _ => unreachable!(),
        }
    }
    const fn instruction<P: PPU, const S: Mode, const M: Micro>() -> Instruction<P> {
        match S {
            Mode::Accumulator => |cpu| {
                cpu.poll();

                cpu.peek_pc();
                Micro::instruction::<P, Accumulator, M>()(cpu, Accumulator());
            },
            Mode::Implied => |cpu| {
                cpu.poll();

                cpu.peek_pc();
                Micro::instruction::<P, Implied, M>()(cpu, Implied());
            },
            Mode::Immediate => |cpu| {
                cpu.poll();

                Micro::instruction::<P, u16, M>()(cpu, cpu.PC);
                cpu.PC = cpu.PC.wrapping_add(1);
            },
            Mode::Absolute => |cpu| {
                let lo = cpu.read_pc();
                let hi = cpu.read_pc();
                let addr = u16::from(lo) | u16::from(hi) << 8;

                cpu.poll();

                Micro::instruction::<P, u16, M>()(cpu, addr);
            },
            Mode::ZeroPage => |cpu| {
                let addr = u16::from(cpu.read_pc());

                cpu.poll();

                Micro::instruction::<P, u16, M>()(cpu, addr);
            },
            Mode::ZeroPageIndexed(i) => match i {
                Index::X => |cpu| {
                    let ptr = cpu.read_pc();
                    cpu.read_addr(u16::from(ptr));

                    let addr = u16::from(ptr.wrapping_add(cpu.X));

                    cpu.poll();

                    Micro::instruction::<P, u16, M>()(cpu, addr);
                },
                Index::Y => |cpu| {
                    let ptr = cpu.read_pc();
                    cpu.read_addr(u16::from(ptr));

                    let addr = u16::from(ptr.wrapping_add(cpu.Y));

                    cpu.poll();

                    Micro::instruction::<P, u16, M>()(cpu, addr);
                },
            },
            Mode::AbsoluteIndexed(i) => match i {
                Index::X => |cpu| {
                    let reg = cpu.X;

                    let lo = cpu.read_pc();
                    let hi = cpu.read_pc();
                    let addr = u16::from(lo.wrapping_add(reg)) | u16::from(hi) << 8;

                    if (addr as u8) < reg {
                        cpu.read_addr(addr);

                        cpu.poll();

                        Micro::instruction::<P, u16, M>()(cpu, addr.wrapping_add(0x0100));
                    } else {
                        match M {
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
                                cpu.read_addr(addr);
                            }
                        }

                        cpu.poll();

                        Micro::instruction::<P, u16, M>()(cpu, addr);
                    }
                },
                Index::Y => |cpu| {
                    let reg = cpu.Y;

                    let lo = cpu.read_pc();
                    let hi = cpu.read_pc();
                    let addr = u16::from(lo.wrapping_add(reg)) | u16::from(hi) << 8;

                    if (addr as u8) < reg {
                        cpu.read_addr(addr);

                        cpu.poll();

                        Micro::instruction::<P, u16, M>()(cpu, addr.wrapping_add(0x0100));
                    } else {
                        match M {
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
                                cpu.read_addr(addr);
                            }
                        }

                        cpu.poll();

                        Micro::instruction::<P, u16, M>()(cpu, addr);
                    }
                },
            },
            Mode::IndexedIndirect => |cpu| {
                let ptr = cpu.read_pc();
                cpu.read_ptr_lo(ptr);

                let ptr = ptr.wrapping_add(cpu.X);
                let lo = cpu.read_ptr_lo(ptr);
                let hi = cpu.read_ptr_hi(ptr);
                let addr = u16::from(lo) | u16::from(hi) << 8;

                cpu.poll();

                Micro::instruction::<P, u16, M>()(cpu, addr);
            },
            Mode::IndirectIndexed => |cpu| {
                let ptr = cpu.read_pc();
                let lo = cpu.read_ptr_lo(ptr);
                let hi = cpu.read_ptr_hi(ptr);
                let addr = u16::from(lo.wrapping_add(cpu.Y)) | u16::from(hi) << 8;

                if (addr as u8) < cpu.Y {
                    cpu.read_addr(addr);

                    cpu.poll();

                    Micro::instruction::<P, u16, M>()(cpu, addr.wrapping_add(0x0100));
                } else {
                    match M {
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
                            cpu.read_addr(addr);
                        }
                    }

                    cpu.poll();

                    Micro::instruction::<P, u16, M>()(cpu, addr);
                }
            },
            Mode::Jam => |_cpu| {
                panic!("jam instruction");
            },
        }
    }
}

const fn branch<const MASK: u8, const ZERO: bool, P: PPU>() -> Instruction<P> {
    |cpu| {
        if (cpu.P & MASK == 0) == ZERO {
            let oper = cpu.read_pc();
            let addr = cpu.PC.wrapping_add_signed(i16::from(oper as i8));

            cpu.peek_pc();
            cpu.PC = (cpu.PC & 0xFF00) | (addr & 0x00FF);

            if cpu.PC != addr {
                // fix PC
                cpu.poll();
                cpu.peek_pc();
                cpu.PC = addr;
            } else {
                // immediately execute next instruction
                cpu.instruction();
            }
        } else {
            cpu.poll();
            cpu.read_pc();
        }
    }
}

pub fn instructions<P: PPU>() -> Instructions<P> {
    Rc::new(repeated!(
        %%s prelude [ prelude s%%
        for op in [0;255] {
            instruction::<P, %%op%%>(),
        }
        %%e postlude ] postlude e%%
    ))
}
