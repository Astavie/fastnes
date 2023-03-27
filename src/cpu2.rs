use std::{
    cell::RefCell,
    fmt::{Debug, Display},
    fs::read_to_string,
    rc::Rc,
};

use crate::{mem::NESTest, Chip, Pin, Signal};

#[derive(Debug, Default)]
#[allow(non_snake_case)]
pub struct MOS6502 {
    // pins
    pub pin_PHI0: Pin<Signal>, // clock

    pub pin_RDY: Pin<Signal>,
    pub pin_SYNC: Pin<Signal>,

    pub pin_IRQ: Pin<Signal>,
    pub pin_NMI: Pin<Signal>,
    pub pin_RES: Pin<Signal>,

    pub pin_A: Pin<u16>,
    pub pin_D: Pin<u8>,
    pub pin_RW: Pin<Signal>,

    // instruction
    ptr: u8,
    addr: u16,

    data: u8,
    cycle: u8,
    opcode: u8,
    instr: Option<Instruction>,
    inc_pc: bool,

    // registers
    PC: u16,
    SP: u8,
    P: u8,
    A: u8,
    X: u8,
    Y: u8,

    // interrupts
    nmi_flipflop: bool,
    irq_sample: bool,
    nmi_sample: bool,
    res_sample: bool,
    hardware_interrupt: bool,
}

impl Chip for MOS6502 {
    fn subscribe(&self, rc: &Rc<RefCell<Self>>) {
        self.pin_PHI0.subscribe(rc, |s, val| match val {
            Signal::Low => s.cycle_start(),
            Signal::High => s.cycle_end(),
        });
        self.pin_RES.subscribe(rc, |s, val| {
            if val == Signal::High {
                // when a positive edge is detected on the input,
                // the microprocessor will immediately begin the reset sequence

                // NOTE: to investigate: it may be a flipflop like nmi that is polled on cycle end?
                s.instr = Some(Instruction::BRK);
                s.res_sample = true;
                s.hardware_interrupt = true;
                s.opcode = 0; // BRK
                s.cycle = 1;
            }
        });
        self.pin_NMI.subscribe(rc, |s, val| {
            if val == Signal::Low {
                s.nmi_flipflop = true;
            }
        });
    }
}

impl MOS6502 {
    pub fn new() -> Self {
        MOS6502 {
            pin_SYNC: Pin::new(Signal::Low),
            cycle: 1,
            ..Self::default()
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
    fn sub(&mut self, reg: u8) -> u8 {
        let (v0, c0) = reg.overflowing_sub(self.data);
        let (v1, c1) = v0.overflowing_sub(self.P & 1 ^ 1);

        self.P &= 0b10111110;

        // borrow flag
        if !(c0 || c1) {
            self.P |= 0b00000001;
        }

        // overflow flag
        let sa = reg & 0b10000000;
        let sb = self.data & 0b10000000;
        let sv = v1 & 0b10000000;
        if sa != sb && sb == sv {
            self.P |= 0b01000000;
        }

        self.flags(v1)
    }
    fn add(&mut self, reg: u8) -> u8 {
        let (v0, c0) = reg.overflowing_add(self.data);
        let (v1, c1) = v0.overflowing_add(self.P & 1);

        self.P &= 0b10111110;

        // carry flag
        if c0 || c1 {
            self.P |= 0b00000001;
        }

        // overflow flag
        let sa = reg & 0b10000000;
        let sb = self.data & 0b10000000;
        let sv = v1 & 0b10000000;
        if sa == sb && sb != sv {
            self.P |= 0b01000000;
        }

        self.flags(v1)
    }
    fn compare(&mut self, reg: u8) {
        let (val, carry) = reg.overflowing_sub(self.data);
        self.P &= 0b11111110;
        if !carry {
            self.P |= 0b00000001;
        }
        self.flags(val);
    }
    fn write_addr(&mut self, addr: u16) {
        if self.res_sample {
            // during reset, all writes are reads
            self.read_addr(addr);
        } else {
            self.pin_A.set(addr);
            self.pin_RW.set(Signal::Low);
        }
    }
    fn write(&mut self) {
        self.write_addr(self.addr);
    }
    fn read_addr(&mut self, addr: u16) {
        self.pin_RW.set(Signal::High);
        self.pin_A.set(addr);
    }
    fn read_hi(&mut self) {
        self.read_addr((self.addr & 0xFF00) | (self.addr.wrapping_add(1) & 0x00FF));
    }
    fn read_ptr_lo(&mut self) {
        self.read_addr(u16::from(self.ptr));
    }
    fn read_ptr_hi(&mut self) {
        self.read_addr(u16::from(self.ptr.wrapping_add(1)));
    }
    fn read(&mut self) {
        self.read_addr(self.addr);
    }
    fn read_pc(&mut self) {
        self.read_addr(self.PC);
        self.inc_pc = true;
    }
    fn push(&mut self, data: u8) {
        self.write_addr(0x0100 | u16::from(self.SP));
        self.data = data;
        self.SP = self.SP.wrapping_sub(1);
    }
    fn pop(&mut self) {
        self.read_addr(0x0100 | u16::from(self.SP));
        self.SP = self.SP.wrapping_add(1);
    }
    fn peek(&mut self) {
        self.read_addr(0x0100 | u16::from(self.SP));
    }
    pub fn dump_state(&self) {
        let mut cycle = self.cycle;
        let mut opcode = self.opcode;
        if cycle == 0 || cycle == 1 {
            cycle = 1;
            println!();
            if self.hardware_interrupt {
                if self.nmi_sample {
                    println!(" ---- NMI ---- ");
                } else if self.res_sample {
                    println!(" ---- RESET ---- ");
                } else if self.irq_sample {
                    println!(" ---- IRQ ---- ");
                }
                opcode = 0;
            } else {
                println!(" ---- {} ---- ", Instruction::from(self.pin_D.get()));
                opcode = self.pin_D.get();
            }
        }

        print!(
            "PC:${:04x} OP:{:02x} A:{:02x} x:{:02x} Y:{:02x} P:{:02x} SP:$01{:02x} CYC:{}",
            self.PC, opcode, self.A, self.X, self.Y, self.P, self.SP, cycle
        );
    }
    pub fn dump_rw(&self) {
        if self.pin_RW.get() == Signal::High {
            println!("    R ${:04x} = {:02x}", self.pin_A.get(), self.pin_D.get());
        } else {
            println!("    W ${:04x} = {:02x}", self.pin_A.get(), self.pin_D.get());
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum Micro {
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

impl Micro {
    fn cycle(self, cpu: &mut MOS6502) {
        match self {
            Self::NOP => (),
            Self::BIT => {
                cpu.P &= 0b00111101;

                // Negative and oVerflow flags
                cpu.P |= cpu.data & 0b11000000;

                // Zero flag
                if cpu.A & cpu.data == 0 {
                    cpu.P |= 0b00000010;
                }
            }
            Self::LDA => cpu.A = cpu.flags(cpu.data),
            Self::LDX => cpu.X = cpu.flags(cpu.data),
            Self::LDY => cpu.Y = cpu.flags(cpu.data),
            Self::LAX => {
                cpu.A = cpu.flags(cpu.data);
                cpu.X = cpu.A;
            }

            Self::STA => cpu.data = cpu.A,
            Self::STX => cpu.data = cpu.X,
            Self::STY => cpu.data = cpu.Y,
            Self::SAX => cpu.data = cpu.A & cpu.X,

            Self::SEC => cpu.P |= 0b00000001,
            Self::CLC => cpu.P &= 0b11111110,
            Self::SEI => cpu.P |= 0b00000100,
            Self::SED => cpu.P |= 0b00001000,
            Self::CLD => cpu.P &= 0b11110111,
            Self::CLV => cpu.P &= 0b10111111,

            Self::AND => cpu.A = cpu.flags(cpu.A & cpu.data),
            Self::ORA => cpu.A = cpu.flags(cpu.A | cpu.data),
            Self::EOR => cpu.A = cpu.flags(cpu.A ^ cpu.data),

            Self::ADC => cpu.A = cpu.add(cpu.A),
            Self::SBC => cpu.A = cpu.sub(cpu.A),

            Self::DEC => cpu.data = cpu.flags(cpu.data.wrapping_sub(1)),
            Self::DEY => cpu.Y = cpu.flags(cpu.Y.wrapping_sub(1)),
            Self::DEX => cpu.X = cpu.flags(cpu.X.wrapping_sub(1)),
            Self::INC => cpu.data = cpu.flags(cpu.data.wrapping_add(1)),
            Self::INY => cpu.Y = cpu.flags(cpu.Y.wrapping_add(1)),
            Self::INX => cpu.X = cpu.flags(cpu.X.wrapping_add(1)),

            Self::CMP => cpu.compare(cpu.A),
            Self::CPY => cpu.compare(cpu.Y),
            Self::CPX => cpu.compare(cpu.X),

            Self::TAY => cpu.Y = cpu.flags(cpu.A),
            Self::TAX => cpu.X = cpu.flags(cpu.A),
            Self::TYA => cpu.A = cpu.flags(cpu.Y),
            Self::TXA => cpu.A = cpu.flags(cpu.X),
            Self::TSX => cpu.X = cpu.flags(cpu.SP),
            Self::TXS => cpu.SP = cpu.X,

            Self::LSR => {
                cpu.P &= 0b11111110;
                cpu.P |= cpu.data & 1;
                cpu.data = cpu.flags(cpu.data >> 1);
            }
            Self::ROR => {
                let carry = cpu.P << 7;
                cpu.P &= 0b11111110;
                cpu.P |= cpu.data & 1;
                cpu.data = cpu.flags(cpu.data >> 1 | carry);
            }
            Self::ASL => {
                cpu.P &= 0b11111110;
                cpu.P |= cpu.data >> 7;
                cpu.data = cpu.flags(cpu.data << 1);
            }
            Self::ROL => {
                let carry = cpu.P & 1;
                cpu.P &= 0b11111110;
                cpu.P |= cpu.data >> 7;
                cpu.data = cpu.flags(cpu.data << 1 | carry);
            }

            Self::SLO => {
                Self::ASL.cycle(cpu);
                Self::ORA.cycle(cpu);
            }
            Self::RLA => {
                Self::ROL.cycle(cpu);
                Self::AND.cycle(cpu);
            }
            Self::SRE => {
                Self::LSR.cycle(cpu);
                Self::EOR.cycle(cpu);
            }
            Self::RRA => {
                Self::ROR.cycle(cpu);
                Self::ADC.cycle(cpu);
            }
            Self::DCP => {
                Self::DEC.cycle(cpu);
                Self::CMP.cycle(cpu);
            }
            Self::ISC => {
                Self::INC.cycle(cpu);
                Self::SBC.cycle(cpu);
            }
            Self::LAS => {
                let val = cpu.flags(cpu.data & cpu.SP);
                cpu.A = val;
                cpu.X = val;
                cpu.SP = val;
            }
            _ => panic!("unstable instruction {:?}", self),
        }
    }
    fn cpu_rw(self, cpu: &mut MOS6502) {
        match self {
            // write operations
            Self::STA | Self::STX | Self::STY | Self::SAX => cpu.write(),

            // other
            _ => cpu.read(),
        }
    }
    fn fix_cycle(self, cpu: &mut MOS6502, val: u8) -> bool {
        if (cpu.addr as u8) < val {
            // fix address
            cpu.addr = cpu.addr.wrapping_add(0x0100);
            false
        } else if match self {
            Self::LDA | Self::LDX | Self::LDY | Self::EOR | Self::AND | Self::ORA => true,
            Self::ADC | Self::SBC | Self::CMP | Self::BIT => true,
            Self::LAX | Self::LAS | Self::TAS | Self::NOP => true,
            _ => false,
        } {
            // no fix required, we end here
            self.cycle(cpu);
            true
        } else {
            false
        }
    }
    fn cycles(self) -> u8 {
        match self {
            // read-modify-write operations
            Self::ASL | Self::LSR | Self::ROL | Self::ROR | Self::INC | Self::DEC => 3,
            Self::SLO | Self::SRE | Self::RLA | Self::RRA | Self::ISC | Self::DCP => 3,

            // other
            _ => 1,
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum Index {
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
enum Mode {
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
enum Instruction {
    Mode(Mode, Micro),
    Branch(u8),

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
            Self::Branch(0x10) => f.write_str("BPL rel"),
            Self::Branch(0x30) => f.write_str("BMI rel"),
            Self::Branch(0x50) => f.write_str("BVC rel"),
            Self::Branch(0x70) => f.write_str("BVS rel"),
            Self::Branch(0x90) => f.write_str("BCC rel"),
            Self::Branch(0xB0) => f.write_str("BCS rel"),
            Self::Branch(0xD0) => f.write_str("BNE rel"),
            Self::Branch(0xF0) => f.write_str("BEQ rel"),
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
            0x10 | 0x30 | 0x50 | 0x70 => Instruction::Branch(op),
            0x90 | 0xB0 | 0xD0 | 0xF0 => Instruction::Branch(op),

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
                        if op & 0b00010000 == 0 {
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
                    0x82 => Micro::STX,
                    0xA0 => Micro::LDY,
                    0xA1 => Micro::LDA,
                    0xA2 => Micro::LDX,
                    0xC0 => {
                        if op & 0b00010000 == 0 {
                            Micro::CPY
                        } else {
                            Micro::NOP
                        }
                    }
                    0xC1 => Micro::CMP,
                    0xC2 => Micro::DEC,
                    0xE0 => {
                        if op & 0b00010000 == 0 {
                            Micro::CPX
                        } else {
                            Micro::NOP
                        }
                    }
                    0xE1 => Micro::SBC,
                    0xE2 => Micro::INC,

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

impl Instruction {
    fn cycle_start(self, cpu: &mut MOS6502) {
        // cycle 2+
        match (self, cpu.cycle) {
            // BRK
            (Self::BRK, 3) => cpu.push((cpu.PC >> 8) as u8),
            (Self::BRK, 4) => cpu.push((cpu.PC & 0xFF) as u8),
            (Self::BRK, 5) => {
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

                // decide vector
                // this is what causes NMI hijacking
                cpu.addr = if cpu.nmi_sample {
                    cpu.nmi_sample = false;
                    0xFFFA
                } else if cpu.res_sample {
                    cpu.res_sample = false;
                    0xFFFC
                } else {
                    0xFFFE
                };
            }
            (Self::BRK, 6) => cpu.read_addr(cpu.addr),
            (Self::BRK, 7) => cpu.read_addr(cpu.addr | 1),

            // PHP / PHA
            (Self::PHP, 2) => cpu.read_addr(cpu.PC),
            (Self::PHA, 2) => cpu.read_addr(cpu.PC),
            (Self::PHP, 3) => cpu.push(cpu.P | 0b00110000),
            (Self::PHA, 3) => cpu.push(cpu.A),

            // PLA / PLP
            (Self::PLA, 2) => cpu.read_addr(cpu.PC),
            (Self::PLP, 2) => cpu.read_addr(cpu.PC),
            (Self::PLA, 3) => cpu.pop(),
            (Self::PLP, 3) => cpu.pop(),
            (Self::PLA, 4) => cpu.peek(),
            (Self::PLP, 4) => cpu.peek(),

            // JSR
            (Self::JSR, 3) => cpu.peek(),
            (Self::JSR, 4) => cpu.push((cpu.PC >> 8) as u8),
            (Self::JSR, 5) => cpu.push((cpu.PC & 0xFF) as u8),
            (Self::JSR, 6) => cpu.read_pc(),

            // JMP abs
            (Self::JMPabs, 3) => cpu.read_pc(),

            // JMP ind
            (Self::JMPind, 3) => cpu.read_pc(),
            (Self::JMPind, 4) => cpu.read(),
            (Self::JMPind, 5) => cpu.read_hi(),

            // RTI
            (Self::RTI, 3) => cpu.pop(),
            (Self::RTI, 4) => cpu.pop(),
            (Self::RTI, 5) => cpu.pop(),
            (Self::RTI, 6) => cpu.peek(),

            // RTS
            (Self::RTS, 3) => cpu.pop(),
            (Self::RTS, 4) => cpu.pop(),
            (Self::RTS, 5) => cpu.peek(),
            (Self::RTS, 6) => cpu.read_pc(),

            // rel
            (Self::Branch(_), _) => cpu.read_pc(),

            // zpg
            (Self::Mode(Mode::ZeroPage, mi), 3) => mi.cpu_rw(cpu),
            (Self::Mode(Mode::ZeroPage, _), 4) => cpu.write(),
            (Self::Mode(Mode::ZeroPage, _), 5) => cpu.write(),

            // zpg,I
            (Self::Mode(Mode::ZeroPageIndexed(_), _), 3) => cpu.read(),
            (Self::Mode(Mode::ZeroPageIndexed(_), mi), 4) => mi.cpu_rw(cpu),
            (Self::Mode(Mode::ZeroPageIndexed(_), _), 5) => cpu.write(),
            (Self::Mode(Mode::ZeroPageIndexed(_), _), 6) => cpu.write(),

            // abs
            (Self::Mode(Mode::Absolute, _), 3) => cpu.read_pc(),
            (Self::Mode(Mode::Absolute, mi), 4) => mi.cpu_rw(cpu),
            (Self::Mode(Mode::Absolute, _), 5) => cpu.write(),
            (Self::Mode(Mode::Absolute, _), 6) => cpu.write(),

            // abs,I
            (Self::Mode(Mode::AbsoluteIndexed(_), _), 3) => cpu.read_pc(),
            (Self::Mode(Mode::AbsoluteIndexed(_), _), 4) => cpu.read(),
            (Self::Mode(Mode::AbsoluteIndexed(_), mi), 5) => mi.cpu_rw(cpu),
            (Self::Mode(Mode::AbsoluteIndexed(_), _), 6) => cpu.write(),
            (Self::Mode(Mode::AbsoluteIndexed(_), _), 7) => cpu.write(),

            // X,ind
            (Self::Mode(Mode::IndexedIndirect, _), 3) => cpu.read_ptr_lo(),
            (Self::Mode(Mode::IndexedIndirect, _), 4) => cpu.read_ptr_lo(),
            (Self::Mode(Mode::IndexedIndirect, _), 5) => cpu.read_ptr_hi(),
            (Self::Mode(Mode::IndexedIndirect, mi), 6) => mi.cpu_rw(cpu),
            (Self::Mode(Mode::IndexedIndirect, _), 7) => cpu.write(),
            (Self::Mode(Mode::IndexedIndirect, _), 8) => cpu.write(),

            // ind,Y
            (Self::Mode(Mode::IndirectIndexed, _), 3) => cpu.read_ptr_lo(),
            (Self::Mode(Mode::IndirectIndexed, _), 4) => cpu.read_ptr_hi(),
            (Self::Mode(Mode::IndirectIndexed, _), 5) => cpu.read(),
            (Self::Mode(Mode::IndirectIndexed, mi), 6) => mi.cpu_rw(cpu),
            (Self::Mode(Mode::IndirectIndexed, _), 7) => cpu.write(),
            (Self::Mode(Mode::IndirectIndexed, _), 8) => cpu.write(),

            // second cycle
            (Self::Mode(Mode::Implied, _), 2) => cpu.read_addr(cpu.PC),
            (Self::Mode(Mode::Accumulator, _), 2) => cpu.read_addr(cpu.PC),
            (_, 1) => cpu.read_pc(),
            (_, 2) => cpu.read_pc(),

            _ => unreachable!(),
        }
    }
    fn cycle_end(self, cpu: &mut MOS6502) {
        // cycle 2+
        let mut last = match self {
            Self::Mode(Mode::Accumulator, _) => cpu.cycle == 2,
            Self::Mode(Mode::Implied, _) => cpu.cycle == 2,
            Self::Mode(Mode::Immediate, _) => cpu.cycle == 2,
            Self::Mode(Mode::Absolute, m) => cpu.cycle == 3 + m.cycles(),
            Self::Mode(Mode::ZeroPage, m) => cpu.cycle == 2 + m.cycles(),
            Self::Mode(Mode::ZeroPageIndexed(_), m) => cpu.cycle == 3 + m.cycles(),
            Self::Mode(Mode::AbsoluteIndexed(_), m) => cpu.cycle == 4 + m.cycles(),
            Self::Mode(Mode::IndexedIndirect, m) => cpu.cycle == 5 + m.cycles(),
            Self::Mode(Mode::IndirectIndexed, m) => cpu.cycle == 5 + m.cycles(),

            Self::Branch(_) => cpu.cycle == 4,
            Self::JMPabs => cpu.cycle == 3,
            Self::JMPind => cpu.cycle == 5,
            Self::BRK => cpu.cycle == 7,
            Self::RTI | Self::RTS | Self::JSR => cpu.cycle == 6,
            Self::PHA | Self::PHP => cpu.cycle == 3,
            Self::PLA | Self::PLP => cpu.cycle == 4,

            Self::JAM => false,
        };

        match (self, cpu.cycle) {
            // BRK
            (Self::BRK, 6) => cpu.PC = (cpu.PC & 0xFF00) | u16::from(cpu.data),
            (Self::BRK, 7) => cpu.PC = (cpu.PC & 0x00FF) | u16::from(cpu.data) << 8,
            (Self::BRK, _) => (),

            // PHP / PHA
            (Self::PHP, _) => (),
            (Self::PHA, _) => (),

            // PLA / PLP
            (Self::PLA, 4) => cpu.A = cpu.flags(cpu.data),
            (Self::PLA, _) => (),
            (Self::PLP, 4) => cpu.P = cpu.data & 0b11001111,
            (Self::PLP, _) => (),

            // JSR
            (Self::JSR, 2) => cpu.addr = u16::from(cpu.data),
            (Self::JSR, 6) => cpu.PC = u16::from(cpu.data) << 8 | cpu.addr,
            (Self::JSR, _) => (),

            // JMP abs
            (Self::JMPabs, 2) => cpu.addr = u16::from(cpu.data),
            (Self::JMPabs, 3) => cpu.PC = u16::from(cpu.data) << 8 | cpu.addr,

            // JMP ind
            (Self::JMPind, 2) => cpu.addr = u16::from(cpu.data),
            (Self::JMPind, 3) => cpu.addr |= u16::from(cpu.data) << 8,
            (Self::JMPind, 4) => cpu.PC = u16::from(cpu.data),
            (Self::JMPind, 5) => cpu.PC |= u16::from(cpu.data) << 8,

            // RTI
            (Self::RTI, 4) => cpu.P = cpu.data & 0b11001111,
            (Self::RTI, 5) => cpu.addr = u16::from(cpu.data),
            (Self::RTI, 6) => cpu.PC = u16::from(cpu.data) << 8 | cpu.addr,
            (Self::RTI, _) => (),

            // RTS
            (Self::RTS, 4) => cpu.addr = u16::from(cpu.data),
            (Self::RTS, 5) => cpu.PC = u16::from(cpu.data) << 8 | cpu.addr,
            (Self::RTS, _) => (),

            // rel
            (Self::Branch(op), 2) => {
                // fetch operand
                cpu.addr = cpu.PC.wrapping_add_signed(i16::from(cpu.data as i8));

                if !match op {
                    0x10 => cpu.P & 0b10000000 == 0, // BPL
                    0x30 => cpu.P & 0b10000000 != 0, // BMI

                    0x50 => cpu.P & 0b01000000 == 0, // BVC
                    0x70 => cpu.P & 0b01000000 != 0, // BVS

                    0x90 => cpu.P & 0b00000001 == 0, // BCC
                    0xB0 => cpu.P & 0b00000001 != 0, // BCS

                    0xD0 => cpu.P & 0b00000010 == 0, // BNE
                    0xF0 => cpu.P & 0b00000010 != 0, // BEQ

                    _ => unreachable!(),
                } {
                    // not taken, skip to end of instruction
                    last = true;
                }
            }
            (Self::Branch(_), 3) => {
                // add operand to PCL
                cpu.PC = (cpu.PC & 0xFF00) | (cpu.addr & 0x00FF);

                if cpu.PC == cpu.addr {
                    // PCH does not need to be fixed
                    // we immediately go to the next instruction
                    // skipping interrupt checking
                    cpu.instr = None;
                    cpu.cycle = 1;
                    cpu.hardware_interrupt = false;

                    // do not pass go
                    // do not collect $200
                    return;
                }
            }
            (Self::Branch(_), 4) => {
                // fix PCH
                cpu.PC = cpu.addr;
            }

            // impl / # / A
            (Self::Mode(Mode::Implied, mi), 2) => mi.cycle(cpu),
            (Self::Mode(Mode::Immediate, mi), 2) => {
                if mi.cycles() == 1 {
                    mi.cycle(cpu);
                }
            }
            (Self::Mode(Mode::Accumulator, mi), 2) => {
                cpu.data = cpu.A;
                mi.cycle(cpu);
                cpu.A = cpu.data;
            }

            // zpg
            (Self::Mode(Mode::ZeroPage, _), 2) => cpu.addr = u16::from(cpu.data),
            (Self::Mode(Mode::ZeroPage, mi), _) if last => mi.cycle(cpu),
            (Self::Mode(Mode::ZeroPage, _), _) => (),

            // zpg,I
            (Self::Mode(Mode::ZeroPageIndexed(_), _), 2) => cpu.addr = u16::from(cpu.data),
            (Self::Mode(Mode::ZeroPageIndexed(i), _), 3) => {
                cpu.addr = u16::from((cpu.addr as u8).wrapping_add(i.get(cpu)))
            }
            (Self::Mode(Mode::ZeroPageIndexed(_), mi), _) if last => mi.cycle(cpu),
            (Self::Mode(Mode::ZeroPageIndexed(_), _), _) => (),

            // abs
            (Self::Mode(Mode::Absolute, _), 2) => cpu.addr = u16::from(cpu.data),
            (Self::Mode(Mode::Absolute, _), 3) => cpu.addr = cpu.addr | u16::from(cpu.data) << 8,
            (Self::Mode(Mode::Absolute, mi), _) if last => mi.cycle(cpu),
            (Self::Mode(Mode::Absolute, _), _) => (),

            // abs,I
            (Self::Mode(Mode::AbsoluteIndexed(i), _), 2) => {
                cpu.addr = u16::from(cpu.data.wrapping_add(i.get(cpu)))
            }
            (Self::Mode(Mode::AbsoluteIndexed(_), _), 3) => cpu.addr |= u16::from(cpu.data) << 8,
            (Self::Mode(Mode::AbsoluteIndexed(i), mi), 4) => last = mi.fix_cycle(cpu, i.get(cpu)),
            (Self::Mode(Mode::AbsoluteIndexed(_), mi), _) if last => mi.cycle(cpu),
            (Self::Mode(Mode::AbsoluteIndexed(_), _), _) => (),

            // X,ind
            (Self::Mode(Mode::IndexedIndirect, _), 2) => cpu.ptr = cpu.data,
            (Self::Mode(Mode::IndexedIndirect, _), 3) => cpu.ptr = cpu.ptr.wrapping_add(cpu.X),
            (Self::Mode(Mode::IndexedIndirect, _), 4) => cpu.addr = u16::from(cpu.data),
            (Self::Mode(Mode::IndexedIndirect, _), 5) => cpu.addr |= u16::from(cpu.data) << 8,
            (Self::Mode(Mode::IndexedIndirect, mi), _) if last => mi.cycle(cpu),
            (Self::Mode(Mode::IndexedIndirect, _), _) => (),

            // ind,Y
            (Self::Mode(Mode::IndirectIndexed, _), 2) => cpu.ptr = cpu.data,
            (Self::Mode(Mode::IndirectIndexed, _), 3) => {
                cpu.addr = u16::from(cpu.data.wrapping_add(cpu.Y))
            }
            (Self::Mode(Mode::IndirectIndexed, _), 4) => cpu.addr |= u16::from(cpu.data) << 8,
            (Self::Mode(Mode::IndirectIndexed, mi), 5) => last = mi.fix_cycle(cpu, cpu.Y),
            (Self::Mode(Mode::IndirectIndexed, mi), _) if last => mi.cycle(cpu),
            (Self::Mode(Mode::IndirectIndexed, _), _) => (),

            _ => unreachable!(),
        }

        // last instruction?
        if last {
            // next instruction (check for interrupts)
            if (cpu.irq_sample && (cpu.P & 0b00000100 == 0)) || cpu.nmi_sample {
                cpu.instr = Some(Instruction::BRK);
                cpu.opcode = 0; // BRK
                cpu.cycle = 1;
                cpu.hardware_interrupt = true;
            } else {
                cpu.instr = None;
                cpu.cycle = 1;
                cpu.hardware_interrupt = false;
            }
        } else {
            cpu.cycle += 1;
        }
    }
}

impl MOS6502 {
    fn cycle_start(&mut self) {
        if let Some(i) = self.instr {
            // instruction logic
            i.cycle_start(self);
            self.pin_SYNC.set(Signal::Low);
        } else {
            // fetch opcode
            self.read_pc();
            self.pin_SYNC.set(Signal::High);
        }
    }
    fn cycle_end(&mut self) {
        // increment pc
        if self.inc_pc {
            self.inc_pc = false;

            // don't increment during interrupts
            if !self.hardware_interrupt {
                self.PC = self.PC.wrapping_add(1);
            }
        }

        // read
        if self.pin_RW.get() == Signal::High {
            self.data = self.pin_D.get();
        }

        // instruction
        if let Some(i) = self.instr {
            i.cycle_end(self);
        } else {
            self.opcode = self.data;
            self.instr = Some(Instruction::from(self.opcode));
            self.cycle += 1;
        }

        // write
        if self.pin_RW.get() == Signal::Low {
            self.pin_D.set(self.data);
        }

        // sample irq and nmi (nmi stays on while irq gets reset every cycle)
        self.irq_sample = self.pin_IRQ.get() == Signal::Low;
        self.nmi_sample = self.nmi_sample || self.nmi_flipflop;
        self.nmi_flipflop = false;
    }
}

pub(crate) fn nestest<const TEST: bool>() {
    let clk = Pin::new(Signal::High);
    let res = Pin::new(Signal::High);

    // create chips and connect pins
    let mut mem = NESTest::new("test/nestest/nestest.nes");
    let mut cpu = MOS6502::new();

    cpu.pin_PHI0.connect(&clk);
    cpu.pin_RES.connect(&res);

    mem.pin_A.connect(&cpu.pin_A);
    mem.pin_D.connect(&cpu.pin_D);
    mem.pin_RW.connect(&cpu.pin_RW);

    // start chips
    let cpu = cpu.start();
    let mem = mem.start();

    // trigger reset
    res.assert();
    res.release();

    // RESET instruction
    for _ in 0..7 {
        clk.assert();
        // cpu.borrow().dump_state();
        clk.release();
        // cpu.borrow().dump_rw();
    }

    if TEST {
        // load log file
        let log = read_to_string("test/nestest/nestest.log").unwrap();
        let mut lines = log.split('\n');

        let mut lineno = 1;
        let mut cycle: u64 = 7;

        // loop
        loop {
            clk.assert();

            // test against log file
            if cpu.borrow().pin_SYNC.get() == Signal::High {
                if cpu.borrow().PC == 0x0001 {
                    break;
                }

                let line = lines.next().unwrap();
                assert_ne!(line, "");

                let pc = u16::from_str_radix(&line[0..4], 16).unwrap();
                let cyc = u64::from_str_radix(line.rsplit_once(':').unwrap().1, 10).unwrap();

                assert_eq!(cpu.borrow().PC, pc, "[line {}] PC:{:04X}", lineno, pc);
                assert_eq!(cycle, cyc, "[line {}] CYC:{}", lineno, cyc);
                lineno += 1;
            }

            clk.release();
            cycle += 1;
        }

        // we should now be at the end of the file
        assert_eq!(lines.next().unwrap(), "");
    } else {
        for _ in 0..26554 {
            clk.assert();
            clk.release();
        }
    }
}
