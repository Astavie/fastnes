use std::{
    cell::RefCell,
    fmt::{Debug, Display},
    fs::read_to_string,
    rc::Rc,
};

use crate::{mem::NESTest, Chip, Pin, Signal};

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

    // instruction list
    instructions: Rc<[Vec<fn(&mut MOS6502) -> bool>; 256]>,

    // instruction
    ptr: u8,
    addr: u16,

    data: u8,
    cycle: Option<u8>,
    opcode: u8,
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
                s.reset();
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
    pub fn reset(&mut self) {
        self.res_sample = true;
        self.hardware_interrupt = true;
        self.opcode = 0; // BRK
        self.cycle = Some(0); // force
    }
    pub fn new(instr: Rc<[Vec<fn(&mut MOS6502) -> bool>; 256]>) -> Self {
        MOS6502 {
            pin_PHI0: Pin::default(),
            pin_RDY: Pin::default(),
            pin_SYNC: Pin::new(Signal::Low),
            pin_IRQ: Pin::default(),
            pin_NMI: Pin::default(),
            pin_RES: Pin::default(),
            pin_A: Pin::default(),
            pin_D: Pin::default(),
            pin_RW: Pin::default(),
            instructions: instr,
            ptr: 0,
            addr: 0,
            data: 0,
            cycle: None,
            opcode: 0,
            inc_pc: false,
            PC: 0,
            SP: 0,
            P: 0,
            A: 0,
            X: 0,
            Y: 0,
            nmi_flipflop: false,
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
    fn write(&mut self) -> bool {
        self.write_addr(self.addr);
        false
    }
    fn read_addr(&mut self, addr: u16) {
        self.pin_RW.set(Signal::High);
        self.pin_A.set(addr);
    }
    fn read_hi(&mut self) -> bool {
        self.read_addr((self.addr & 0xFF00) | (self.addr.wrapping_add(1) & 0x00FF));
        false
    }
    fn read_ptr_lo(&mut self) -> bool {
        self.read_addr(u16::from(self.ptr));
        false
    }
    fn read_ptr_hi(&mut self) -> bool {
        self.read_addr(u16::from(self.ptr.wrapping_add(1)));
        false
    }
    fn read(&mut self) -> bool {
        self.read_addr(self.addr);
        false
    }
    fn read_pc(&mut self) -> bool {
        self.read_addr(self.PC);
        self.inc_pc = true;
        false
    }
    fn get_pc_hi(&mut self) -> bool {
        self.PC = self.PC | u16::from(self.data) << 8;
        false
    }
    fn get_pc_lo(&mut self) -> bool {
        self.PC = u16::from(self.data);
        false
    }
    fn get_hi_forward(&mut self) -> bool {
        self.get_hi();
        self.PC = self.addr;
        false
    }
    fn get_hi(&mut self) -> bool {
        self.addr = self.addr | u16::from(self.data) << 8;
        false
    }
    fn get_lo(&mut self) -> bool {
        self.addr = u16::from(self.data);
        false
    }
    fn get_lo_x(&mut self) -> bool {
        self.addr = u16::from(self.data.wrapping_add(self.X));
        false
    }
    fn get_lo_y(&mut self) -> bool {
        self.addr = u16::from(self.data.wrapping_add(self.Y));
        false
    }
    fn peek_pc(&mut self) -> bool {
        self.read_addr(self.PC);
        false
    }
    fn push(&mut self, data: u8) {
        self.write_addr(0x0100 | u16::from(self.SP));
        self.data = data;
        self.SP = self.SP.wrapping_sub(1);
    }
    fn push_lo(&mut self) -> bool {
        self.push(self.PC as u8);
        false
    }
    fn push_hi(&mut self) -> bool {
        self.push((self.PC >> 8) as u8);
        false
    }
    fn pop(&mut self) -> bool {
        self.read_addr(0x0100 | u16::from(self.SP));
        self.SP = self.SP.wrapping_add(1);
        false
    }
    fn peek(&mut self) -> bool {
        self.read_addr(0x0100 | u16::from(self.SP));
        false
    }
    pub fn dump_state(&self) {
        let cycle = self.cycle.unwrap_or(1);
        let mut opcode = self.opcode;
        if cycle == 1 {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    fn cycle_generic<const S: Micro>() -> fn(&mut MOS6502) -> bool {
        match S {
            Self::NOP => |_| true,
            Self::BIT => |cpu| {
                cpu.P &= 0b00111101;

                // Negative and oVerflow flags
                cpu.P |= cpu.data & 0b11000000;

                // Zero flag
                if cpu.A & cpu.data == 0 {
                    cpu.P |= 0b00000010;
                }
                true
            },
            Self::LDA => |cpu| {
                cpu.A = cpu.flags(cpu.data);
                true
            },
            Self::LDX => |cpu| {
                cpu.X = cpu.flags(cpu.data);
                true
            },
            Self::LDY => |cpu| {
                cpu.Y = cpu.flags(cpu.data);
                true
            },
            Self::LAX => |cpu| {
                cpu.A = cpu.flags(cpu.data);
                cpu.X = cpu.A;
                true
            },

            Self::STA => |cpu| {
                cpu.data = cpu.A;
                true
            },
            Self::STX => |cpu| {
                cpu.data = cpu.X;
                true
            },
            Self::STY => |cpu| {
                cpu.data = cpu.Y;
                true
            },
            Self::SAX => |cpu| {
                cpu.data = cpu.A & cpu.X;
                true
            },

            Self::SEC => |cpu| {
                cpu.P |= 0b00000001;
                true
            },
            Self::CLC => |cpu| {
                cpu.P &= 0b11111110;
                true
            },
            Self::SEI => |cpu| {
                cpu.P |= 0b00000100;
                true
            },
            Self::SED => |cpu| {
                cpu.P |= 0b00001000;
                true
            },
            Self::CLD => |cpu| {
                cpu.P &= 0b11110111;
                true
            },
            Self::CLV => |cpu| {
                cpu.P &= 0b10111111;
                true
            },

            Self::AND => |cpu| {
                cpu.A = cpu.flags(cpu.A & cpu.data);
                true
            },
            Self::ORA => |cpu| {
                cpu.A = cpu.flags(cpu.A | cpu.data);
                true
            },
            Self::EOR => |cpu| {
                cpu.A = cpu.flags(cpu.A ^ cpu.data);
                true
            },

            Self::ADC => |cpu| {
                cpu.A = cpu.add(cpu.A);
                true
            },
            Self::SBC => |cpu| {
                cpu.A = cpu.sub(cpu.A);
                true
            },

            Self::DEC => |cpu| {
                cpu.data = cpu.flags(cpu.data.wrapping_sub(1));
                true
            },
            Self::DEY => |cpu| {
                cpu.Y = cpu.flags(cpu.Y.wrapping_sub(1));
                true
            },
            Self::DEX => |cpu| {
                cpu.X = cpu.flags(cpu.X.wrapping_sub(1));
                true
            },
            Self::INC => |cpu| {
                cpu.data = cpu.flags(cpu.data.wrapping_add(1));
                true
            },
            Self::INY => |cpu| {
                cpu.Y = cpu.flags(cpu.Y.wrapping_add(1));
                true
            },
            Self::INX => |cpu| {
                cpu.X = cpu.flags(cpu.X.wrapping_add(1));
                true
            },

            Self::CMP => |cpu| {
                cpu.compare(cpu.A);
                true
            },
            Self::CPY => |cpu| {
                cpu.compare(cpu.Y);
                true
            },
            Self::CPX => |cpu| {
                cpu.compare(cpu.X);
                true
            },

            Self::TAY => |cpu| {
                cpu.Y = cpu.flags(cpu.A);
                true
            },
            Self::TAX => |cpu| {
                cpu.X = cpu.flags(cpu.A);
                true
            },
            Self::TYA => |cpu| {
                cpu.A = cpu.flags(cpu.Y);
                true
            },
            Self::TXA => |cpu| {
                cpu.A = cpu.flags(cpu.X);
                true
            },
            Self::TSX => |cpu| {
                cpu.X = cpu.flags(cpu.SP);
                true
            },
            Self::TXS => |cpu| {
                cpu.SP = cpu.X;
                true
            },

            Self::LSR => |cpu| {
                cpu.P &= 0b11111110;
                cpu.P |= cpu.data & 1;
                cpu.data = cpu.flags(cpu.data >> 1);
                true
            },
            Self::ROR => |cpu| {
                let carry = cpu.P << 7;
                cpu.P &= 0b11111110;
                cpu.P |= cpu.data & 1;
                cpu.data = cpu.flags(cpu.data >> 1 | carry);
                true
            },
            Self::ASL => |cpu| {
                cpu.P &= 0b11111110;
                cpu.P |= cpu.data >> 7;
                cpu.data = cpu.flags(cpu.data << 1);
                true
            },
            Self::ROL => |cpu| {
                let carry = cpu.P & 1;
                cpu.P &= 0b11111110;
                cpu.P |= cpu.data >> 7;
                cpu.data = cpu.flags(cpu.data << 1 | carry);
                true
            },

            Self::SLO => |cpu| {
                Self::cycle_generic::<{ Self::ASL }>()(cpu);
                Self::cycle_generic::<{ Self::ORA }>()(cpu);
                true
            },
            Self::RLA => |cpu| {
                Self::cycle_generic::<{ Self::ROL }>()(cpu);
                Self::cycle_generic::<{ Self::AND }>()(cpu);
                true
            },
            Self::SRE => |cpu| {
                Self::cycle_generic::<{ Self::LSR }>()(cpu);
                Self::cycle_generic::<{ Self::EOR }>()(cpu);
                true
            },
            Self::RRA => |cpu| {
                Self::cycle_generic::<{ Self::ROR }>()(cpu);
                Self::cycle_generic::<{ Self::ADC }>()(cpu);
                true
            },
            Self::DCP => |cpu| {
                Self::cycle_generic::<{ Self::DEC }>()(cpu);
                Self::cycle_generic::<{ Self::CMP }>()(cpu);
                true
            },
            Self::ISC => |cpu| {
                Self::cycle_generic::<{ Self::INC }>()(cpu);
                Self::cycle_generic::<{ Self::SBC }>()(cpu);
                true
            },
            Self::LAS => |cpu| {
                let val = cpu.flags(cpu.data & cpu.SP);
                cpu.A = val;
                cpu.X = val;
                cpu.SP = val;
                true
            },
            _ => |cpu| {
                panic!("unstable instruction {:02X}", cpu.opcode);
            },
        }
    }
    fn cycle_generic_acc<const S: Micro, const ACC: bool>() -> fn(&mut MOS6502) -> bool {
        if ACC {
            |cpu| {
                cpu.data = cpu.A;
                Self::cycle_generic::<S>()(cpu);
                cpu.A = cpu.data;
                true
            }
        } else {
            Self::cycle_generic::<S>()
        }
    }
    fn cycle<const ACC: bool>(self) -> fn(&mut MOS6502) -> bool {
        match self {
            Micro::ASL => Self::cycle_generic_acc::<{ Micro::ASL }, ACC>(),
            Micro::LSR => Self::cycle_generic_acc::<{ Micro::LSR }, ACC>(),
            Micro::ROL => Self::cycle_generic_acc::<{ Micro::ROL }, ACC>(),
            Micro::ROR => Self::cycle_generic_acc::<{ Micro::ROR }, ACC>(),
            Micro::INC => Self::cycle_generic_acc::<{ Micro::INC }, ACC>(),
            Micro::DEC => Self::cycle_generic_acc::<{ Micro::DEC }, ACC>(),
            Micro::AND => Self::cycle_generic_acc::<{ Micro::AND }, ACC>(),
            Micro::EOR => Self::cycle_generic_acc::<{ Micro::EOR }, ACC>(),
            Micro::ORA => Self::cycle_generic_acc::<{ Micro::ORA }, ACC>(),
            Micro::BIT => Self::cycle_generic_acc::<{ Micro::BIT }, ACC>(),
            Micro::ADC => Self::cycle_generic_acc::<{ Micro::ADC }, ACC>(),
            Micro::SBC => Self::cycle_generic_acc::<{ Micro::SBC }, ACC>(),
            Micro::STA => Self::cycle_generic_acc::<{ Micro::STA }, ACC>(),
            Micro::STY => Self::cycle_generic_acc::<{ Micro::STY }, ACC>(),
            Micro::STX => Self::cycle_generic_acc::<{ Micro::STX }, ACC>(),
            Micro::LDA => Self::cycle_generic_acc::<{ Micro::LDA }, ACC>(),
            Micro::LDY => Self::cycle_generic_acc::<{ Micro::LDY }, ACC>(),
            Micro::LDX => Self::cycle_generic_acc::<{ Micro::LDX }, ACC>(),
            Micro::CPX => Self::cycle_generic_acc::<{ Micro::CPX }, ACC>(),
            Micro::CPY => Self::cycle_generic_acc::<{ Micro::CPY }, ACC>(),
            Micro::CMP => Self::cycle_generic_acc::<{ Micro::CMP }, ACC>(),
            Micro::NOP => Self::cycle_generic_acc::<{ Micro::NOP }, ACC>(),
            Micro::CLC => Self::cycle_generic_acc::<{ Micro::CLC }, ACC>(),
            Micro::SEC => Self::cycle_generic_acc::<{ Micro::SEC }, ACC>(),
            Micro::CLI => Self::cycle_generic_acc::<{ Micro::CLI }, ACC>(),
            Micro::SEI => Self::cycle_generic_acc::<{ Micro::SEI }, ACC>(),
            Micro::CLV => Self::cycle_generic_acc::<{ Micro::CLV }, ACC>(),
            Micro::CLD => Self::cycle_generic_acc::<{ Micro::CLD }, ACC>(),
            Micro::SED => Self::cycle_generic_acc::<{ Micro::SED }, ACC>(),
            Micro::INX => Self::cycle_generic_acc::<{ Micro::INX }, ACC>(),
            Micro::DEX => Self::cycle_generic_acc::<{ Micro::DEX }, ACC>(),
            Micro::INY => Self::cycle_generic_acc::<{ Micro::INY }, ACC>(),
            Micro::DEY => Self::cycle_generic_acc::<{ Micro::DEY }, ACC>(),
            Micro::TAX => Self::cycle_generic_acc::<{ Micro::TAX }, ACC>(),
            Micro::TXA => Self::cycle_generic_acc::<{ Micro::TXA }, ACC>(),
            Micro::TAY => Self::cycle_generic_acc::<{ Micro::TAY }, ACC>(),
            Micro::TYA => Self::cycle_generic_acc::<{ Micro::TYA }, ACC>(),
            Micro::TSX => Self::cycle_generic_acc::<{ Micro::TSX }, ACC>(),
            Micro::TXS => Self::cycle_generic_acc::<{ Micro::TXS }, ACC>(),
            Micro::SLO => Self::cycle_generic_acc::<{ Micro::SLO }, ACC>(),
            Micro::SRE => Self::cycle_generic_acc::<{ Micro::SRE }, ACC>(),
            Micro::RLA => Self::cycle_generic_acc::<{ Micro::RLA }, ACC>(),
            Micro::RRA => Self::cycle_generic_acc::<{ Micro::RRA }, ACC>(),
            Micro::ISC => Self::cycle_generic_acc::<{ Micro::ISC }, ACC>(),
            Micro::DCP => Self::cycle_generic_acc::<{ Micro::DCP }, ACC>(),
            Micro::LAX => Self::cycle_generic_acc::<{ Micro::LAX }, ACC>(),
            Micro::SAX => Self::cycle_generic_acc::<{ Micro::SAX }, ACC>(),
            Micro::LAS => Self::cycle_generic_acc::<{ Micro::LAS }, ACC>(),
            Micro::TAS => Self::cycle_generic_acc::<{ Micro::TAS }, ACC>(),
            Micro::SHA => Self::cycle_generic_acc::<{ Micro::SHA }, ACC>(),
            Micro::SHY => Self::cycle_generic_acc::<{ Micro::SHY }, ACC>(),
            Micro::SHX => Self::cycle_generic_acc::<{ Micro::SHX }, ACC>(),
            Micro::ANE => Self::cycle_generic_acc::<{ Micro::ANE }, ACC>(),
            Micro::LXA => Self::cycle_generic_acc::<{ Micro::LXA }, ACC>(),
        }
    }
    fn cpu_rw(self) -> fn(&mut MOS6502) -> bool {
        match self {
            // write operations
            Self::STA | Self::STX | Self::STY | Self::SAX => MOS6502::write,

            // other
            _ => MOS6502::read,
        }
    }
    fn fix_cycle(self, x: Index) -> fn(&mut MOS6502) -> bool {
        match (self, x) {
            (Micro::ASL, Index::X) => Micro::fix_cycle_generic::<{ Micro::ASL }, { Index::X }>(),
            (Micro::ASL, Index::Y) => Micro::fix_cycle_generic::<{ Micro::ASL }, { Index::Y }>(),
            (Micro::LSR, Index::X) => Micro::fix_cycle_generic::<{ Micro::LSR }, { Index::X }>(),
            (Micro::LSR, Index::Y) => Micro::fix_cycle_generic::<{ Micro::LSR }, { Index::Y }>(),
            (Micro::ROL, Index::X) => Micro::fix_cycle_generic::<{ Micro::ROL }, { Index::X }>(),
            (Micro::ROL, Index::Y) => Micro::fix_cycle_generic::<{ Micro::ROL }, { Index::Y }>(),
            (Micro::ROR, Index::X) => Micro::fix_cycle_generic::<{ Micro::ROR }, { Index::X }>(),
            (Micro::ROR, Index::Y) => Micro::fix_cycle_generic::<{ Micro::ROR }, { Index::Y }>(),
            (Micro::INC, Index::X) => Micro::fix_cycle_generic::<{ Micro::INC }, { Index::X }>(),
            (Micro::INC, Index::Y) => Micro::fix_cycle_generic::<{ Micro::INC }, { Index::Y }>(),
            (Micro::DEC, Index::X) => Micro::fix_cycle_generic::<{ Micro::DEC }, { Index::X }>(),
            (Micro::DEC, Index::Y) => Micro::fix_cycle_generic::<{ Micro::DEC }, { Index::Y }>(),
            (Micro::AND, Index::X) => Micro::fix_cycle_generic::<{ Micro::AND }, { Index::X }>(),
            (Micro::AND, Index::Y) => Micro::fix_cycle_generic::<{ Micro::AND }, { Index::Y }>(),
            (Micro::EOR, Index::X) => Micro::fix_cycle_generic::<{ Micro::EOR }, { Index::X }>(),
            (Micro::EOR, Index::Y) => Micro::fix_cycle_generic::<{ Micro::EOR }, { Index::Y }>(),
            (Micro::ORA, Index::X) => Micro::fix_cycle_generic::<{ Micro::ORA }, { Index::X }>(),
            (Micro::ORA, Index::Y) => Micro::fix_cycle_generic::<{ Micro::ORA }, { Index::Y }>(),
            (Micro::BIT, Index::X) => Micro::fix_cycle_generic::<{ Micro::BIT }, { Index::X }>(),
            (Micro::BIT, Index::Y) => Micro::fix_cycle_generic::<{ Micro::BIT }, { Index::Y }>(),
            (Micro::ADC, Index::X) => Micro::fix_cycle_generic::<{ Micro::ADC }, { Index::X }>(),
            (Micro::ADC, Index::Y) => Micro::fix_cycle_generic::<{ Micro::ADC }, { Index::Y }>(),
            (Micro::SBC, Index::X) => Micro::fix_cycle_generic::<{ Micro::SBC }, { Index::X }>(),
            (Micro::SBC, Index::Y) => Micro::fix_cycle_generic::<{ Micro::SBC }, { Index::Y }>(),
            (Micro::STA, Index::X) => Micro::fix_cycle_generic::<{ Micro::STA }, { Index::X }>(),
            (Micro::STA, Index::Y) => Micro::fix_cycle_generic::<{ Micro::STA }, { Index::Y }>(),
            (Micro::STY, Index::X) => Micro::fix_cycle_generic::<{ Micro::STY }, { Index::X }>(),
            (Micro::STY, Index::Y) => Micro::fix_cycle_generic::<{ Micro::STY }, { Index::Y }>(),
            (Micro::STX, Index::X) => Micro::fix_cycle_generic::<{ Micro::STX }, { Index::X }>(),
            (Micro::STX, Index::Y) => Micro::fix_cycle_generic::<{ Micro::STX }, { Index::Y }>(),
            (Micro::LDA, Index::X) => Micro::fix_cycle_generic::<{ Micro::LDA }, { Index::X }>(),
            (Micro::LDA, Index::Y) => Micro::fix_cycle_generic::<{ Micro::LDA }, { Index::Y }>(),
            (Micro::LDY, Index::X) => Micro::fix_cycle_generic::<{ Micro::LDY }, { Index::X }>(),
            (Micro::LDY, Index::Y) => Micro::fix_cycle_generic::<{ Micro::LDY }, { Index::Y }>(),
            (Micro::LDX, Index::X) => Micro::fix_cycle_generic::<{ Micro::LDX }, { Index::X }>(),
            (Micro::LDX, Index::Y) => Micro::fix_cycle_generic::<{ Micro::LDX }, { Index::Y }>(),
            (Micro::CPX, Index::X) => Micro::fix_cycle_generic::<{ Micro::CPX }, { Index::X }>(),
            (Micro::CPX, Index::Y) => Micro::fix_cycle_generic::<{ Micro::CPX }, { Index::Y }>(),
            (Micro::CPY, Index::X) => Micro::fix_cycle_generic::<{ Micro::CPY }, { Index::X }>(),
            (Micro::CPY, Index::Y) => Micro::fix_cycle_generic::<{ Micro::CPY }, { Index::Y }>(),
            (Micro::CMP, Index::X) => Micro::fix_cycle_generic::<{ Micro::CMP }, { Index::X }>(),
            (Micro::CMP, Index::Y) => Micro::fix_cycle_generic::<{ Micro::CMP }, { Index::Y }>(),
            (Micro::NOP, Index::X) => Micro::fix_cycle_generic::<{ Micro::NOP }, { Index::X }>(),
            (Micro::NOP, Index::Y) => Micro::fix_cycle_generic::<{ Micro::NOP }, { Index::Y }>(),
            (Micro::CLC, Index::X) => Micro::fix_cycle_generic::<{ Micro::CLC }, { Index::X }>(),
            (Micro::CLC, Index::Y) => Micro::fix_cycle_generic::<{ Micro::CLC }, { Index::Y }>(),
            (Micro::SEC, Index::X) => Micro::fix_cycle_generic::<{ Micro::SEC }, { Index::X }>(),
            (Micro::SEC, Index::Y) => Micro::fix_cycle_generic::<{ Micro::SEC }, { Index::Y }>(),
            (Micro::CLI, Index::X) => Micro::fix_cycle_generic::<{ Micro::CLI }, { Index::X }>(),
            (Micro::CLI, Index::Y) => Micro::fix_cycle_generic::<{ Micro::CLI }, { Index::Y }>(),
            (Micro::SEI, Index::X) => Micro::fix_cycle_generic::<{ Micro::SEI }, { Index::X }>(),
            (Micro::SEI, Index::Y) => Micro::fix_cycle_generic::<{ Micro::SEI }, { Index::Y }>(),
            (Micro::CLV, Index::X) => Micro::fix_cycle_generic::<{ Micro::CLV }, { Index::X }>(),
            (Micro::CLV, Index::Y) => Micro::fix_cycle_generic::<{ Micro::CLV }, { Index::Y }>(),
            (Micro::CLD, Index::X) => Micro::fix_cycle_generic::<{ Micro::CLD }, { Index::X }>(),
            (Micro::CLD, Index::Y) => Micro::fix_cycle_generic::<{ Micro::CLD }, { Index::Y }>(),
            (Micro::SED, Index::X) => Micro::fix_cycle_generic::<{ Micro::SED }, { Index::X }>(),
            (Micro::SED, Index::Y) => Micro::fix_cycle_generic::<{ Micro::SED }, { Index::Y }>(),
            (Micro::INX, Index::X) => Micro::fix_cycle_generic::<{ Micro::INX }, { Index::X }>(),
            (Micro::INX, Index::Y) => Micro::fix_cycle_generic::<{ Micro::INX }, { Index::Y }>(),
            (Micro::DEX, Index::X) => Micro::fix_cycle_generic::<{ Micro::DEX }, { Index::X }>(),
            (Micro::DEX, Index::Y) => Micro::fix_cycle_generic::<{ Micro::DEX }, { Index::Y }>(),
            (Micro::INY, Index::X) => Micro::fix_cycle_generic::<{ Micro::INY }, { Index::X }>(),
            (Micro::INY, Index::Y) => Micro::fix_cycle_generic::<{ Micro::INY }, { Index::Y }>(),
            (Micro::DEY, Index::X) => Micro::fix_cycle_generic::<{ Micro::DEY }, { Index::X }>(),
            (Micro::DEY, Index::Y) => Micro::fix_cycle_generic::<{ Micro::DEY }, { Index::Y }>(),
            (Micro::TAX, Index::X) => Micro::fix_cycle_generic::<{ Micro::TAX }, { Index::X }>(),
            (Micro::TAX, Index::Y) => Micro::fix_cycle_generic::<{ Micro::TAX }, { Index::Y }>(),
            (Micro::TXA, Index::X) => Micro::fix_cycle_generic::<{ Micro::TXA }, { Index::X }>(),
            (Micro::TXA, Index::Y) => Micro::fix_cycle_generic::<{ Micro::TXA }, { Index::Y }>(),
            (Micro::TAY, Index::X) => Micro::fix_cycle_generic::<{ Micro::TAY }, { Index::X }>(),
            (Micro::TAY, Index::Y) => Micro::fix_cycle_generic::<{ Micro::TAY }, { Index::Y }>(),
            (Micro::TYA, Index::X) => Micro::fix_cycle_generic::<{ Micro::TYA }, { Index::X }>(),
            (Micro::TYA, Index::Y) => Micro::fix_cycle_generic::<{ Micro::TYA }, { Index::Y }>(),
            (Micro::TSX, Index::X) => Micro::fix_cycle_generic::<{ Micro::TSX }, { Index::X }>(),
            (Micro::TSX, Index::Y) => Micro::fix_cycle_generic::<{ Micro::TSX }, { Index::Y }>(),
            (Micro::TXS, Index::X) => Micro::fix_cycle_generic::<{ Micro::TXS }, { Index::X }>(),
            (Micro::TXS, Index::Y) => Micro::fix_cycle_generic::<{ Micro::TXS }, { Index::Y }>(),
            (Micro::SLO, Index::X) => Micro::fix_cycle_generic::<{ Micro::SLO }, { Index::X }>(),
            (Micro::SLO, Index::Y) => Micro::fix_cycle_generic::<{ Micro::SLO }, { Index::Y }>(),
            (Micro::SRE, Index::X) => Micro::fix_cycle_generic::<{ Micro::SRE }, { Index::X }>(),
            (Micro::SRE, Index::Y) => Micro::fix_cycle_generic::<{ Micro::SRE }, { Index::Y }>(),
            (Micro::RLA, Index::X) => Micro::fix_cycle_generic::<{ Micro::RLA }, { Index::X }>(),
            (Micro::RLA, Index::Y) => Micro::fix_cycle_generic::<{ Micro::RLA }, { Index::Y }>(),
            (Micro::RRA, Index::X) => Micro::fix_cycle_generic::<{ Micro::RRA }, { Index::X }>(),
            (Micro::RRA, Index::Y) => Micro::fix_cycle_generic::<{ Micro::RRA }, { Index::Y }>(),
            (Micro::ISC, Index::X) => Micro::fix_cycle_generic::<{ Micro::ISC }, { Index::X }>(),
            (Micro::ISC, Index::Y) => Micro::fix_cycle_generic::<{ Micro::ISC }, { Index::Y }>(),
            (Micro::DCP, Index::X) => Micro::fix_cycle_generic::<{ Micro::DCP }, { Index::X }>(),
            (Micro::DCP, Index::Y) => Micro::fix_cycle_generic::<{ Micro::DCP }, { Index::Y }>(),
            (Micro::LAX, Index::X) => Micro::fix_cycle_generic::<{ Micro::LAX }, { Index::X }>(),
            (Micro::LAX, Index::Y) => Micro::fix_cycle_generic::<{ Micro::LAX }, { Index::Y }>(),
            (Micro::SAX, Index::X) => Micro::fix_cycle_generic::<{ Micro::SAX }, { Index::X }>(),
            (Micro::SAX, Index::Y) => Micro::fix_cycle_generic::<{ Micro::SAX }, { Index::Y }>(),
            (Micro::LAS, Index::X) => Micro::fix_cycle_generic::<{ Micro::LAS }, { Index::X }>(),
            (Micro::LAS, Index::Y) => Micro::fix_cycle_generic::<{ Micro::LAS }, { Index::Y }>(),
            (Micro::TAS, Index::X) => Micro::fix_cycle_generic::<{ Micro::TAS }, { Index::X }>(),
            (Micro::TAS, Index::Y) => Micro::fix_cycle_generic::<{ Micro::TAS }, { Index::Y }>(),
            (Micro::SHA, Index::X) => Micro::fix_cycle_generic::<{ Micro::SHA }, { Index::X }>(),
            (Micro::SHA, Index::Y) => Micro::fix_cycle_generic::<{ Micro::SHA }, { Index::Y }>(),
            (Micro::SHY, Index::X) => Micro::fix_cycle_generic::<{ Micro::SHY }, { Index::X }>(),
            (Micro::SHY, Index::Y) => Micro::fix_cycle_generic::<{ Micro::SHY }, { Index::Y }>(),
            (Micro::SHX, Index::X) => Micro::fix_cycle_generic::<{ Micro::SHX }, { Index::X }>(),
            (Micro::SHX, Index::Y) => Micro::fix_cycle_generic::<{ Micro::SHX }, { Index::Y }>(),
            (Micro::ANE, Index::X) => Micro::fix_cycle_generic::<{ Micro::ANE }, { Index::X }>(),
            (Micro::ANE, Index::Y) => Micro::fix_cycle_generic::<{ Micro::ANE }, { Index::Y }>(),
            (Micro::LXA, Index::X) => Micro::fix_cycle_generic::<{ Micro::LXA }, { Index::X }>(),
            (Micro::LXA, Index::Y) => Micro::fix_cycle_generic::<{ Micro::LXA }, { Index::Y }>(),
        }
    }
    fn fix_cycle_generic<const S: Micro, const I: Index>() -> fn(&mut MOS6502) -> bool {
        return |cpu| {
            if (cpu.addr as u8) < I.get(cpu) {
                cpu.addr = cpu.addr.wrapping_add(0x0100);
                false
            } else if match S {
                Self::LDA | Self::LDX | Self::LDY | Self::EOR | Self::AND | Self::ORA => true,
                Self::ADC | Self::SBC | Self::CMP | Self::BIT => true,
                Self::LAX | Self::LAS | Self::TAS | Self::NOP => true,
                _ => false,
            } {
                // no fix required, we end here
                Self::cycle_generic::<S>()(cpu)
            } else {
                false
            }
        };
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

pub(crate) fn cycles() -> [Vec<fn(&mut MOS6502) -> bool>; 256] {
    let v: Vec<_> = (0..=255)
        .map(|i| {
            let mut vec = vec![];
            let instr = Instruction::from(i);
            for (start, (end, last)) in
                (1..).map(|cycle| (instr.cycle_start(cycle), instr.cycle_end(cycle)))
            {
                vec.push(start);
                vec.push(end);
                if last {
                    break;
                }
            }
            vec
        })
        .collect();
    v.try_into().unwrap_or_else(|_| panic!())
}

impl Instruction {
    fn cycle_start(self, cycle: u8) -> fn(&mut MOS6502) -> bool {
        // cycle 2+
        match (self, cycle) {
            // BRK
            (Self::BRK, 3) => MOS6502::push_hi,
            (Self::BRK, 4) => MOS6502::push_lo,
            (Self::BRK, 5) => |cpu| {
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
                false
            },
            (Self::BRK, 6) => MOS6502::read,
            (Self::BRK, 7) => MOS6502::read_hi,

            // PHP / PHA
            (Self::PHP, 2) => MOS6502::peek_pc,
            (Self::PHA, 2) => MOS6502::peek_pc,
            (Self::PHP, 3) => |cpu| {
                cpu.push(cpu.P | 0b00110000);
                false
            },
            (Self::PHA, 3) => |cpu| {
                cpu.push(cpu.A);
                false
            },

            // PLA / PLP
            (Self::PLA, 2) => MOS6502::peek_pc,
            (Self::PLP, 2) => MOS6502::peek_pc,
            (Self::PLA, 3) => MOS6502::pop,
            (Self::PLP, 3) => MOS6502::pop,
            (Self::PLA, 4) => MOS6502::peek,
            (Self::PLP, 4) => MOS6502::peek,

            // JSR
            (Self::JSR, 3) => MOS6502::peek,
            (Self::JSR, 4) => MOS6502::push_hi,
            (Self::JSR, 5) => MOS6502::push_lo,
            (Self::JSR, 6) => MOS6502::read_pc,

            // JMP abs
            (Self::JMPabs, 3) => MOS6502::read_pc,

            // JMP ind
            (Self::JMPind, 3) => MOS6502::read_pc,
            (Self::JMPind, 4) => MOS6502::read,
            (Self::JMPind, 5) => MOS6502::read_hi,

            // RTI
            (Self::RTI, 3) => MOS6502::pop,
            (Self::RTI, 4) => MOS6502::pop,
            (Self::RTI, 5) => MOS6502::pop,
            (Self::RTI, 6) => MOS6502::peek,

            // RTS
            (Self::RTS, 3) => MOS6502::pop,
            (Self::RTS, 4) => MOS6502::pop,
            (Self::RTS, 5) => MOS6502::peek,
            (Self::RTS, 6) => MOS6502::read_pc,

            // rel
            (Self::Branch, _) => MOS6502::read_pc,

            // zpg
            (Self::Mode(Mode::ZeroPage, mi), 3) => mi.cpu_rw(),
            (Self::Mode(Mode::ZeroPage, _), 4) => MOS6502::write,
            (Self::Mode(Mode::ZeroPage, _), 5) => MOS6502::write,

            // zpg,I
            (Self::Mode(Mode::ZeroPageIndexed(_), _), 3) => MOS6502::read,
            (Self::Mode(Mode::ZeroPageIndexed(_), mi), 4) => mi.cpu_rw(),
            (Self::Mode(Mode::ZeroPageIndexed(_), _), 5) => MOS6502::write,
            (Self::Mode(Mode::ZeroPageIndexed(_), _), 6) => MOS6502::write,

            // abs
            (Self::Mode(Mode::Absolute, _), 3) => MOS6502::read_pc,
            (Self::Mode(Mode::Absolute, mi), 4) => mi.cpu_rw(),
            (Self::Mode(Mode::Absolute, _), 5) => MOS6502::write,
            (Self::Mode(Mode::Absolute, _), 6) => MOS6502::write,

            // abs,I
            (Self::Mode(Mode::AbsoluteIndexed(_), _), 3) => MOS6502::read_pc,
            (Self::Mode(Mode::AbsoluteIndexed(_), _), 4) => MOS6502::read,
            (Self::Mode(Mode::AbsoluteIndexed(_), mi), 5) => mi.cpu_rw(),
            (Self::Mode(Mode::AbsoluteIndexed(_), _), 6) => MOS6502::write,
            (Self::Mode(Mode::AbsoluteIndexed(_), _), 7) => MOS6502::write,

            // X,ind
            (Self::Mode(Mode::IndexedIndirect, _), 3) => MOS6502::read_ptr_lo,
            (Self::Mode(Mode::IndexedIndirect, _), 4) => MOS6502::read_ptr_lo,
            (Self::Mode(Mode::IndexedIndirect, _), 5) => MOS6502::read_ptr_hi,
            (Self::Mode(Mode::IndexedIndirect, mi), 6) => mi.cpu_rw(),
            (Self::Mode(Mode::IndexedIndirect, _), 7) => MOS6502::write,
            (Self::Mode(Mode::IndexedIndirect, _), 8) => MOS6502::write,

            // ind,Y
            (Self::Mode(Mode::IndirectIndexed, _), 3) => MOS6502::read_ptr_lo,
            (Self::Mode(Mode::IndirectIndexed, _), 4) => MOS6502::read_ptr_hi,
            (Self::Mode(Mode::IndirectIndexed, _), 5) => MOS6502::read,
            (Self::Mode(Mode::IndirectIndexed, mi), 6) => mi.cpu_rw(),
            (Self::Mode(Mode::IndirectIndexed, _), 7) => MOS6502::write,
            (Self::Mode(Mode::IndirectIndexed, _), 8) => MOS6502::write,

            // second cycle
            (Self::Mode(Mode::Implied, _), 2) => MOS6502::peek_pc,
            (Self::Mode(Mode::Accumulator, _), 2) => MOS6502::peek_pc,
            (_, 1) => MOS6502::read_pc,
            (_, 2) => MOS6502::read_pc,

            _ => unreachable!(),
        }
    }
    fn cycle_end(self, cycle: u8) -> (fn(&mut MOS6502) -> bool, bool) {
        // cycle 2+
        let last = match self {
            Self::Mode(Mode::Accumulator, _) => cycle == 2,
            Self::Mode(Mode::Implied, _) => cycle == 2,
            Self::Mode(Mode::Immediate, _) => cycle == 2,
            Self::Mode(Mode::Absolute, m) => cycle == 3 + m.cycles(),
            Self::Mode(Mode::ZeroPage, m) => cycle == 2 + m.cycles(),
            Self::Mode(Mode::ZeroPageIndexed(_), m) => cycle == 3 + m.cycles(),
            Self::Mode(Mode::AbsoluteIndexed(_), m) => cycle == 4 + m.cycles(),
            Self::Mode(Mode::IndexedIndirect, m) => cycle == 5 + m.cycles(),
            Self::Mode(Mode::IndirectIndexed, m) => cycle == 5 + m.cycles(),

            Self::Branch => cycle == 4,
            Self::JMPabs => cycle == 3,
            Self::JMPind => cycle == 5,
            Self::BRK => cycle == 7,
            Self::RTI | Self::RTS | Self::JSR => cycle == 6,
            Self::PHA | Self::PHP => cycle == 3,
            Self::PLA | Self::PLP => cycle == 4,

            Self::JAM => true,
        };

        let f: fn(&mut MOS6502) -> bool = match (self, cycle) {
            (_, 1) => |_| false,

            // BRK
            (Self::BRK, 6) => MOS6502::get_pc_lo,
            (Self::BRK, 7) => MOS6502::get_pc_hi,
            (Self::BRK, _) => |_| false,

            // PHP / PHA
            (Self::PHP, _) => |_| false,
            (Self::PHA, _) => |_| false,

            // PLA / PLP
            (Self::PLA, 4) => |cpu| {
                cpu.A = cpu.flags(cpu.data);
                false
            },
            (Self::PLA, _) => |_| false,
            (Self::PLP, 4) => |cpu| {
                cpu.P = cpu.data & 0b11001111;
                false
            },
            (Self::PLP, _) => |_| false,

            // JSR
            (Self::JSR, 2) => MOS6502::get_lo,
            (Self::JSR, 6) => MOS6502::get_hi_forward,
            (Self::JSR, _) => |_| false,

            // JMP abs
            (Self::JMPabs, 2) => MOS6502::get_lo,
            (Self::JMPabs, 3) => MOS6502::get_hi_forward,

            // JMP ind
            (Self::JMPind, 2) => MOS6502::get_lo,
            (Self::JMPind, 3) => MOS6502::get_hi,
            (Self::JMPind, 4) => MOS6502::get_pc_lo,
            (Self::JMPind, 5) => MOS6502::get_pc_hi,

            // RTI
            (Self::RTI, 4) => |cpu| {
                cpu.P = cpu.data & 0b11001111;
                false
            },
            (Self::RTI, 5) => MOS6502::get_pc_lo,
            (Self::RTI, 6) => MOS6502::get_pc_hi,
            (Self::RTI, _) => |_| false,

            // RTS
            (Self::RTS, 4) => MOS6502::get_pc_lo,
            (Self::RTS, 5) => MOS6502::get_pc_hi,
            (Self::RTS, _) => |_| false,

            // rel
            (Self::Branch, 2) => |cpu| {
                // fetch operand
                cpu.addr = cpu.PC.wrapping_add_signed(i16::from(cpu.data as i8));

                if match cpu.opcode {
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
                    false
                } else {
                    // not taken, skip to end of instruction
                    true
                }
            },
            (Self::Branch, 3) => |cpu| {
                // add operand to PCL
                cpu.PC = (cpu.PC & 0xFF00) | (cpu.addr & 0x00FF);

                if cpu.PC == cpu.addr {
                    // PCH does not need to be fixed
                    // we immediately go to the next instruction
                    // skipping interrupt checking
                    cpu.cycle = None;
                    cpu.hardware_interrupt = false;
                }
                false
            },
            (Self::Branch, 4) => |cpu| {
                // fix PCH
                cpu.PC = cpu.addr;
                false
            },

            // impl / # / A
            (Self::Mode(Mode::Implied, mi), 2) => mi.cycle::<false>(),
            (Self::Mode(Mode::Immediate, mi), 2) => {
                if mi.cycles() == 1 {
                    mi.cycle::<false>()
                } else {
                    |_| false
                }
            }
            (Self::Mode(Mode::Accumulator, mi), 2) => mi.cycle::<true>(),

            // zpg
            (Self::Mode(Mode::ZeroPage, _), 2) => MOS6502::get_lo,
            (Self::Mode(Mode::ZeroPage, mi), _) if last => mi.cycle::<false>(),
            (Self::Mode(Mode::ZeroPage, _), _) => |_| false,

            // zpg,I
            (Self::Mode(Mode::ZeroPageIndexed(_), _), 2) => MOS6502::get_lo,
            (Self::Mode(Mode::ZeroPageIndexed(Index::X), _), 3) => |cpu| {
                cpu.addr = u16::from((cpu.addr as u8).wrapping_add(cpu.X));
                false
            },
            (Self::Mode(Mode::ZeroPageIndexed(Index::Y), _), 3) => |cpu| {
                cpu.addr = u16::from((cpu.addr as u8).wrapping_add(cpu.Y));
                false
            },
            (Self::Mode(Mode::ZeroPageIndexed(_), mi), _) if last => mi.cycle::<false>(),
            (Self::Mode(Mode::ZeroPageIndexed(_), _), _) => |_| false,

            // abs
            (Self::Mode(Mode::Absolute, _), 2) => MOS6502::get_lo,
            (Self::Mode(Mode::Absolute, _), 3) => MOS6502::get_hi,
            (Self::Mode(Mode::Absolute, mi), _) if last => mi.cycle::<false>(),
            (Self::Mode(Mode::Absolute, _), _) => |_| false,

            // abs,I
            (Self::Mode(Mode::AbsoluteIndexed(Index::X), _), 2) => MOS6502::get_lo_x,
            (Self::Mode(Mode::AbsoluteIndexed(Index::Y), _), 2) => MOS6502::get_lo_y,
            (Self::Mode(Mode::AbsoluteIndexed(________), _), 3) => MOS6502::get_hi,
            (Self::Mode(Mode::AbsoluteIndexed(i), mi), 4) => mi.fix_cycle(i),
            (Self::Mode(Mode::AbsoluteIndexed(_), mi), _) if last => mi.cycle::<false>(),
            (Self::Mode(Mode::AbsoluteIndexed(_), _), _) => |_| false,

            // X,ind
            (Self::Mode(Mode::IndexedIndirect, _), 2) => |cpu| {
                cpu.ptr = cpu.data;
                false
            },
            (Self::Mode(Mode::IndexedIndirect, _), 3) => |cpu| {
                cpu.ptr = cpu.ptr.wrapping_add(cpu.X);
                false
            },
            (Self::Mode(Mode::IndexedIndirect, _), 4) => MOS6502::get_lo,
            (Self::Mode(Mode::IndexedIndirect, _), 5) => MOS6502::get_hi,
            (Self::Mode(Mode::IndexedIndirect, mi), _) if last => mi.cycle::<false>(),
            (Self::Mode(Mode::IndexedIndirect, _), _) => |_| false,

            // ind,Y
            (Self::Mode(Mode::IndirectIndexed, _), 2) => |cpu| {
                cpu.ptr = cpu.data;
                false
            },
            (Self::Mode(Mode::IndirectIndexed, _), 3) => MOS6502::get_lo_y,
            (Self::Mode(Mode::IndirectIndexed, _), 4) => MOS6502::get_hi,
            (Self::Mode(Mode::IndirectIndexed, mi), 5) => mi.fix_cycle(Index::Y),
            (Self::Mode(Mode::IndirectIndexed, mi), _) if last => mi.cycle::<false>(),
            (Self::Mode(Mode::IndirectIndexed, _), _) => |_| false,

            _ => unreachable!("{} {}", self, cycle),
        };

        return (f, last);
    }
}

impl MOS6502 {
    pub(crate) fn cycle_start(&mut self) {
        if let Some(cycle) = self.cycle {
            let vec = &self.instructions[usize::from(self.opcode)];
            let idx = usize::from(cycle);

            // instruction logic
            vec[idx](self);
            self.pin_SYNC.set(Signal::Low);
            self.cycle = Some(cycle + 1);
        } else {
            // fetch opcode
            self.read_pc();
            self.pin_SYNC.set(Signal::High);
            self.cycle = None;
        }
    }
    pub(crate) fn cycle_end(&mut self) {
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
        if let Some(cycle) = self.cycle {
            let vec = &self.instructions[usize::from(self.opcode)];
            let len = vec.len();
            let idx = usize::from(cycle);

            // instruction logic
            let skip = vec[idx](self);
            self.pin_SYNC.set(Signal::Low);

            // last instruction?
            if skip || idx == len - 1 {
                // next instruction (check for interrupts)
                if (self.irq_sample && (self.P & 0b00000100 == 0)) || self.nmi_sample {
                    self.opcode = 0; // BRK
                    self.cycle = Some(0); // force
                    self.hardware_interrupt = true;
                } else {
                    self.cycle = None; // unknown opcode
                    self.hardware_interrupt = false;
                }
            } else if self.cycle.is_some() {
                self.cycle = Some(cycle + 1);
            }
        } else {
            // fetch opcode
            self.opcode = self.data;
            self.cycle = Some(2);
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

pub(crate) fn nestest<const TEST: bool>(instr: Rc<[Vec<fn(&mut MOS6502) -> bool>; 256]>) {
    let clk = Pin::new(Signal::High);
    let res = Pin::new(Signal::High);

    // create chips and connect pins
    let mut mem = NESTest::new("test/nestest/nestest.nes");
    let mut cpu = MOS6502::new(instr);

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

    if TEST {
        // RESET instruction
        for _ in 0..7 {
            clk.assert();
            clk.release();
        }

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
