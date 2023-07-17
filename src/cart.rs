use std::sync::Arc;

pub trait Cartridge: Clone {
    fn read_prg_rom(&self, addr: u16) -> Option<u8>;
    fn write_prg_rom(&mut self, addr: u16, data: u8);

    fn read_prg_ram(&self, addr: u16) -> Option<u8>;
    fn write_prg_ram(&mut self, addr: u16, data: u8);

    fn read_chr(&self, addr: u16) -> u8;
    fn write_chr(&mut self, addr: u16, data: u8);

    fn read_nametable(&self, addr: u16) -> u8;
    fn write_nametable(&mut self, addr: u16, data: u8);
}

#[derive(Clone)]
pub struct Empty;

impl Cartridge for Empty {
    fn read_prg_rom(&self, _addr: u16) -> Option<u8> { None }
    fn read_prg_ram(&self, _addr: u16) -> Option<u8> { None }
    fn read_chr(&self, _addr: u16) -> u8 { 0 }
    fn read_nametable(&self, _addr: u16) -> u8 { 0 }

    fn write_prg_rom(&mut self, _addr: u16, _dataa: u8) {}
    fn write_prg_ram(&mut self, _addr: u16, _dataa: u8) {}
    fn write_chr(&mut self, _addr: u16, _dataa: u8) {}
    fn write_nametable(&mut self, _addr: u16, _dataa: u8) {}
}

macro_rules! generate_cartridge_enum {
    (
        $($mapper: ident),*
    ) => {
        #[derive(Clone)]
        pub enum CartridgeEnum {
            Empty,
            $($mapper($mapper),)*
        }

        impl Cartridge for CartridgeEnum {
            fn read_prg_rom(&self, addr: u16) -> Option<u8> {
                match self {
                    Self::Empty => Empty.read_prg_rom(addr),
                    $(Self::$mapper(c) => c.read_prg_rom(addr),)*
                }
            }

            fn write_prg_rom(&mut self, addr: u16, data: u8) {
                match self {
                    Self::Empty => Empty.write_prg_rom(addr, data),
                    $(Self::$mapper(c) => c.write_prg_rom(addr, data),)*
                }
            }

            fn read_prg_ram(&self, addr: u16) -> Option<u8> {
                match self {
                    Self::Empty => Empty.read_prg_ram(addr),
                    $(Self::$mapper(c) => c.read_prg_ram(addr),)*
                }
            }

            fn write_prg_ram(&mut self, addr: u16, data: u8) {
                match self {
                    Self::Empty => Empty.write_prg_ram(addr, data),
                    $(Self::$mapper(c) => c.write_prg_ram(addr, data),)*
                }
            }

            fn read_chr(&self, addr: u16) -> u8 {
                match self {
                    Self::Empty => Empty.read_chr(addr),
                    $(Self::$mapper(c) => c.read_chr(addr),)*
                }
            }

            fn write_chr(&mut self, addr: u16, data: u8) {
                match self {
                    Self::Empty => Empty.write_chr(addr, data),
                    $(Self::$mapper(c) => c.write_chr(addr, data),)*
                }
            }

            fn read_nametable(&self, addr: u16) -> u8 {
                match self {
                    Self::Empty => Empty.read_nametable(addr),
                    $(Self::$mapper(c) => c.read_nametable(addr),)*
                }
            }

            fn write_nametable(&mut self, addr: u16, data: u8) {
                match self {
                    Self::Empty => Empty.write_nametable(addr, data),
                    $(Self::$mapper(c) => c.write_nametable(addr, data),)*
                }
            }
        }
    };
}

generate_cartridge_enum!(NROM);

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mirroring {
    SingleA,
    SingleB,
    Vertical,
    Horizontal,
}

#[derive(Clone)]
pub struct Nametable {
    mirroring: Mirroring,
    a: [u8; 0x0400],
    b: [u8; 0x0400],
}

impl Nametable {
    pub fn new(mirroring: Mirroring) -> Self {
        Self {
            mirroring,
            a: [0; 0x0400],
            b: [0; 0x0400],
        }
    }
    pub fn read(&self, addr: u16) -> u8 {
        let spot = usize::from(addr & 0x3FF);
        match (self.mirroring, addr & 0xC00) {
            (Mirroring::SingleA, _) => self.a[spot],
            (Mirroring::SingleB, _) => self.b[spot],
            (Mirroring::Vertical, 0x000) => self.a[spot],
            (Mirroring::Vertical, 0x400) => self.b[spot],
            (Mirroring::Vertical, 0x800) => self.a[spot],
            (Mirroring::Vertical, 0xC00) => self.b[spot],
            (Mirroring::Horizontal, 0x000) => self.a[spot],
            (Mirroring::Horizontal, 0x400) => self.a[spot],
            (Mirroring::Horizontal, 0x800) => self.b[spot],
            (Mirroring::Horizontal, 0xC00) => self.b[spot],
            _ => unreachable!(),
        }
    }
    pub fn write(&mut self, addr: u16, data: u8) {
        let spot = usize::from(addr & 0x3FF);
        match (self.mirroring, addr & 0xC00) {
            (Mirroring::SingleA, _) => self.a[spot] = data,
            (Mirroring::SingleB, _) => self.b[spot] = data,
            (Mirroring::Vertical, 0x000) => self.a[spot] = data,
            (Mirroring::Vertical, 0x400) => self.b[spot] = data,
            (Mirroring::Vertical, 0x800) => self.a[spot] = data,
            (Mirroring::Vertical, 0xC00) => self.b[spot] = data,
            (Mirroring::Horizontal, 0x000) => self.a[spot] = data,
            (Mirroring::Horizontal, 0x400) => self.a[spot] = data,
            (Mirroring::Horizontal, 0x800) => self.b[spot] = data,
            (Mirroring::Horizontal, 0xC00) => self.b[spot] = data,
            _ => unreachable!(),
        }
    }
}

#[derive(Clone)]
pub struct NROM {
    // these are reado-only and can be shared among cartridges of the same game
    chr: Arc<[u8; 0x2000]>,
    rom: Arc<[u8; 0x8000]>,

    // technically the data is stored on the NES itself but we store it here for convenience
    nametable: Nametable,

    // ram is not included on hardware but some emulator tests use it
    // to save memory when unused we store it optionally on the heap
    ram: Option<Box<[u8; 0x2000]>>,
}

fn concat<const A: usize, const B: usize>(a: [u8; A], b: [u8; B]) -> [u8; A + B] {
    let mut res = [0; A + B];
    res[0..A].copy_from_slice(&a);
    res[A..A + B].copy_from_slice(&b);
    res
}

impl NROM {
    pub fn new_256(mirroring: Mirroring, chr: [u8; 0x2000], rom: [u8; 0x8000]) -> Self {
        Self {
            nametable: Nametable::new(mirroring),
            ram: None,
            rom: Arc::new(rom),
            chr: Arc::new(chr),
        }
    }
    pub fn new_128(mirroring: Mirroring, chr: [u8; 0x2000], rom: [u8; 0x4000]) -> Self {
        Self {
            nametable: Nametable::new(mirroring),
            ram: None,
            rom: Arc::new(concat(rom, rom)),
            chr: Arc::new(chr),
        }
    }
}

impl Cartridge for NROM {
    fn read_chr(&self, addr: u16) -> u8 { self.chr[usize::from(addr)] }

    fn read_nametable(&self, addr: u16) -> u8 { self.nametable.read(addr) }

    fn write_nametable(&mut self, addr: u16, data: u8) { self.nametable.write(addr, data) }

    fn read_prg_ram(&self, addr: u16) -> Option<u8> {
        // return 0 if PRGRAM is uninitialized
        Some(
            self.ram
                .as_ref()
                .map_or(0, |ram| ram[usize::from(addr & 0x1FFF)]),
        )
    }

    fn write_prg_ram(&mut self, addr: u16, data: u8) {
        // if PRGRAM appears to be used,
        // we simply create it on the fly
        self.ram.get_or_insert_with(|| Box::new([0; 0x2000]))[usize::from(addr & 0x1FFF)] = data;
    }

    fn read_prg_rom(&self, addr: u16) -> Option<u8> { Some(self.rom[usize::from(addr & 0x7FFF)]) }

    // Cannot write to CHR ROM
    fn write_chr(&mut self, _addr: u16, _data: u8) {}

    // Cannot write to PRG ROM
    fn write_prg_rom(&mut self, _addr: u16, _data: u8) {}
}
