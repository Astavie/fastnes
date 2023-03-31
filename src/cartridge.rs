pub trait Cartridge {
    fn read_chr(&self, addr: u16) -> u8;
    fn write_chr(&mut self, addr: u16, data: u8);

    fn cache_prg_rom(&self) -> [u8; 0x8000];
    fn write_prg_rom(&mut self, addr: u16, data: u8, cache: &mut [u8; 0x8000]);

    fn read_prg_ram(&self, addr: u16) -> Option<u8>;
    fn write_prg_ram(&mut self, addr: u16, data: u8);

    fn read_nametable(&self, addr: u16) -> u8;
    fn write_nametable(&mut self, addr: u16, data: u8);
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mirroring {
    SingleA,
    SingleB,
    Vertical,
    Horizontal,
}

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

pub struct NROM {
    nametable: MirroredNametable,
    chr: [u8; 0x2000],
    ram: [u8; 0x2000], // not included on hardware but some emulator tests use it
    rom: [u8; 0x8000],
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
            ram: [0; 0x2000],
            rom,
            chr,
        }
    }
    pub fn new_128(mirroring: Mirroring, chr: [u8; 0x2000], rom: [u8; 0x4000]) -> Self {
        Self {
            nametable: MirroredNametable::new(mirroring),
            ram: [0; 0x2000],
            rom: concat(rom, rom),
            chr,
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

    fn cache_prg_rom(&self) -> [u8; 0x8000] {
        self.rom
    }

    fn write_prg_rom(&mut self, _addr: u16, _data: u8, _cache: &mut [u8; 0x8000]) {
        // Cannot write to PRG ROM
    }

    fn read_prg_ram(&self, addr: u16) -> Option<u8> {
        Some(self.ram[usize::from(addr & 0x1FFF)])
    }

    fn write_prg_ram(&mut self, addr: u16, data: u8) {
        self.ram[usize::from(addr & 0x1FFF)] = data;
    }
}
