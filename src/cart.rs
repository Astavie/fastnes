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
    fn read_prg_rom(&self, addr: u16) -> Option<u8> {
        None
    }
    fn read_prg_ram(&self, addr: u16) -> Option<u8> {
        None
    }
    fn read_chr(&self, addr: u16) -> u8 {
        0
    }
    fn read_nametable(&self, addr: u16) -> u8 {
        0
    }

    fn write_prg_rom(&mut self, addr: u16, data: u8) {}
    fn write_prg_ram(&mut self, addr: u16, data: u8) {}
    fn write_chr(&mut self, addr: u16, data: u8) {}
    fn write_nametable(&mut self, addr: u16, data: u8) {}
}

#[derive(Clone)]
pub enum CartridgeEnum {
    Empty,
    NROM(NROM),
}

impl Cartridge for CartridgeEnum {
    fn read_prg_rom(&self, addr: u16) -> Option<u8> {
        match self {
            Self::Empty => None,
            Self::NROM(n) => n.read_prg_rom(addr),
        }
    }

    fn write_prg_rom(&mut self, addr: u16, data: u8) {
        match self {
            Self::Empty => (),
            Self::NROM(n) => n.write_prg_rom(addr, data),
        }
    }

    fn read_prg_ram(&self, addr: u16) -> Option<u8> {
        match self {
            Self::Empty => None,
            Self::NROM(n) => n.read_prg_ram(addr),
        }
    }

    fn write_prg_ram(&mut self, addr: u16, data: u8) {
        match self {
            Self::Empty => (),
            Self::NROM(n) => n.write_prg_ram(addr, data),
        }
    }

    fn read_chr(&self, addr: u16) -> u8 {
        match self {
            Self::Empty => 0,
            Self::NROM(n) => n.read_chr(addr),
        }
    }

    fn write_chr(&mut self, addr: u16, data: u8) {
        match self {
            Self::Empty => (),
            Self::NROM(n) => n.write_chr(addr, data),
        }
    }

    fn read_nametable(&self, addr: u16) -> u8 {
        match self {
            Self::Empty => 0,
            Self::NROM(n) => n.read_nametable(addr),
        }
    }

    fn write_nametable(&mut self, addr: u16, data: u8) {
        match self {
            Self::Empty => (),
            Self::NROM(n) => n.write_nametable(addr, data),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mirroring {
    SingleA,
    SingleB,
    Vertical,
    Horizontal,
}

#[derive(Clone)]
pub struct MirroredNametable {
    mirroring: Mirroring,
    a: [u8; 0x0400],
    b: [u8; 0x0400],
}

impl MirroredNametable {
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
    nametable: MirroredNametable,

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
            nametable: MirroredNametable::new(mirroring),
            ram: None,
            rom: Arc::new(rom),
            chr: Arc::new(chr),
        }
    }
    pub fn new_128(mirroring: Mirroring, chr: [u8; 0x2000], rom: [u8; 0x4000]) -> Self {
        Self {
            nametable: MirroredNametable::new(mirroring),
            ram: None,
            rom: Arc::new(concat(rom, rom)),
            chr: Arc::new(chr),
        }
    }
}

impl Cartridge for NROM {
    fn read_chr(&self, addr: u16) -> u8 {
        self.chr[usize::from(addr)]
    }

    fn write_chr(&mut self, _addr: u16, _data: u8) {
        // Cannot write to CHR ROM
    }

    fn read_nametable(&self, addr: u16) -> u8 {
        self.nametable.read(addr)
    }

    fn write_nametable(&mut self, addr: u16, data: u8) {
        self.nametable.write(addr, data)
    }

    fn read_prg_ram(&self, addr: u16) -> Option<u8> {
        // return open bus if PRGRAM is uninitialized
        self.ram.as_ref().map(|ram| ram[usize::from(addr & 0x1FFF)])
    }

    fn write_prg_ram(&mut self, addr: u16, data: u8) {
        // if PRGRAM appears to be used,
        // we simply create it on the fly
        self.ram.get_or_insert_with(|| Box::new([0; 0x2000]))[usize::from(addr & 0x1FFF)] = data;
    }

    fn read_prg_rom(&self, addr: u16) -> Option<u8> {
        Some(self.rom[usize::from(addr & 0x7FFF)])
    }

    fn write_prg_rom(&mut self, _addr: u16, _data: u8) {
        // Cannot write to PRG ROM
    }
}
