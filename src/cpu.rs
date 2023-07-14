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

pub trait Bus {
    fn read_cycle(&mut self, addr: u16) -> u8;
    fn write_cycle(&mut self, addr: u16, data: u8);
    fn poll_interrupts(&mut self) -> (bool, bool);
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
            res_sample: false,
            hardware_interrupt: false,
        }
    }
    pub fn reset(&mut self) {
        self.res_sample = true;
        self.SP = 0;
    }
    pub fn nmi(&mut self) {
        self.nmi_sample = true;
    }
    pub fn irq(&mut self) {
        self.irq_sample = true;
    }
    pub fn instruction(&mut self, bus: &mut impl Bus) {
        // get instruction to perform
        let op = if self.nmi_sample || self.irq_sample || self.res_sample {
            self.hardware_interrupt = true;
            self.read_pc(bus);
            0
        } else {
            self.read_pc(bus)
        };

        todo!("instr {}", op);
        // let instr = INSTRUCTIONS[usize::from(op)];
        // instr(self);
    }

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
    fn poll(&mut self, bus: &mut impl Bus) {
        // sample irq and nmi (nmi stays on while irq gets reset every cycle)
        let (irq, nmi) = bus.poll_interrupts();
        self.irq_sample = irq;
        self.nmi_sample = self.nmi_sample || nmi;
    }

    // READ/WRITE CYCLES
    pub fn read_cycle(&self, addr: u16, bus: &mut impl Bus) -> u8 {
        bus.read_cycle(addr)
    }
    pub fn write_cycle(&self, addr: u16, data: u8, bus: &mut impl Bus) {
        if !self.res_sample {
            bus.write_cycle(addr, data);
        } else {
            bus.read_cycle(addr);
        }
    }

    fn read_zeropage(&self, addr: u8, bus: &mut impl Bus) -> u8 {
        self.read_cycle(u16::from(addr), bus)
    }
    fn write_zeropage(&self, addr: u8, data: u8, bus: &mut impl Bus) {
        self.write_cycle(u16::from(addr), data, bus);
    }
    fn read_ptr_lo(&self, ptr: u8, bus: &mut impl Bus) -> u8 {
        self.read_cycle(u16::from(ptr), bus)
    }
    fn read_ptr_hi(&self, ptr: u8, bus: &mut impl Bus) -> u8 {
        self.read_cycle(u16::from(ptr.wrapping_add(1)), bus)
    }
    fn read_pc(&mut self, bus: &mut impl Bus) -> u8 {
        let d = bus.read_cycle(self.PC);
        if !self.hardware_interrupt {
            self.PC += 1;
        }
        d
    }
    fn poke_pc(&self, bus: &mut impl Bus) {
        self.read_cycle(self.PC, bus);
    }
    fn push(&mut self, data: u8, bus: &mut impl Bus) {
        self.write_cycle(0x0100 | u16::from(self.SP), data, bus);
        self.SP = self.SP.wrapping_sub(1);
    }
    fn push_lo(&mut self, bus: &mut impl Bus) {
        self.push(self.PC as u8, bus);
    }
    fn push_hi(&mut self, bus: &mut impl Bus) {
        self.push((self.PC >> 8) as u8, bus);
    }
    fn peek(&self, bus: &mut impl Bus) -> u8 {
        self.read_cycle(0x0100 | u16::from(self.SP), bus)
    }
    fn pop(&mut self, bus: &mut impl Bus) -> u8 {
        let data = self.peek(bus);
        self.SP = self.SP.wrapping_add(1);
        data
    }
}
