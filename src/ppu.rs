use enumset::{EnumSet, EnumSetType};

pub trait PPU {
    fn write(&mut self, cycle: usize, addr: u16, data: u8);
    fn read(&mut self, cycle: usize, addr: u16) -> u8;

    fn nmi(&mut self, cycle: usize) -> bool;
    fn new() -> Self;
}

pub(crate) struct DummyPPU;

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

#[derive(Clone, Copy, PartialEq, Eq)]
enum Direction {
    Across,
    Down,
}

impl Direction {
    fn increment(&self, addr: u16) -> u16 {
        match self {
            Self::Across => addr + 1,
            Self::Down => addr + 32,
        }
    }
}

impl From<bool> for Direction {
    fn from(value: bool) -> Self {
        if value {
            Self::Down
        } else {
            Self::Across
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Quadrant {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
}

impl From<u8> for Quadrant {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::TopLeft,
            1 => Self::TopRight,
            2 => Self::BottomLeft,
            3 => Self::BottomRight,
            _ => panic!(),
        }
    }
}

impl Quadrant {
    fn nametable(&self) -> u16 {
        match self {
            Quadrant::TopLeft => 0x2000,
            Quadrant::TopRight => 0x2400,
            Quadrant::BottomLeft => 0x2800,
            Quadrant::BottomRight => 0x2C00,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Half {
    Left,
    Right,
}

impl From<bool> for Half {
    fn from(value: bool) -> Self {
        if value {
            Self::Right
        } else {
            Self::Left
        }
    }
}

impl Half {
    fn pattern_table(&self) -> u16 {
        match self {
            Half::Left => 0x0000,
            Half::Right => 0x1000,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SpriteSize {
    Small, // 8x8
    Tall,  // 8x16
}

impl From<bool> for SpriteSize {
    fn from(value: bool) -> Self {
        if value {
            Self::Tall
        } else {
            Self::Small
        }
    }
}

#[derive(EnumSetType, Debug)]
enum PPUMASK {
    Greyscale,
    ShowBackgroundLeft,
    ShowSpritesLeft,
    ShowBackground,
    ShowSprites,
    EmphasizeRed,
    EmphasizeGreen,
    EmphasizeBlue,
}

#[allow(non_snake_case)]
pub(crate) struct FastPPU {
    open: u8,
    odd_frame: bool,

    this_vbl: Option<usize>,
    next_vbl: usize,
    next_nmi: Option<usize>,

    // registers
    PPUADDR: u16,
    PPUMASK: EnumSet<PPUMASK>,

    // PPUCTRL
    nametable: Quadrant,
    vram_direction: Direction,
    sprite_table: Half,
    background_table: Half,
    sprite_size: SpriteSize,
    nmi_output: bool,
}

const SCANLINE: usize = 341;
const FRAME: usize = 262 * SCANLINE;

impl FastPPU {
    fn sync(&mut self, cycle: usize) {
        // get next vblank
        while cycle >= self.next_vbl {
            self.this_vbl = Some(self.next_vbl);
            self.odd_frame = !self.odd_frame;
            self.next_vbl += FRAME
                - usize::from(self.PPUMASK.contains(PPUMASK::ShowBackground) && self.odd_frame);
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
            odd_frame: false,
            open: 0,

            // registers
            PPUADDR: 0,
            PPUMASK: EnumSet::new(),

            // PPUCTRL
            nametable: Quadrant::TopLeft,
            vram_direction: Direction::Across,
            sprite_table: Half::Left,
            background_table: Half::Left,
            sprite_size: SpriteSize::Small,
            nmi_output: false,
        }
    }

    // we inline never because this tends to get inlined inside write_addr,
    // making every instruction slower just to make ppu writes a bit faster
    #[inline(never)]
    fn write(&mut self, cycle: usize, addr: u16, data: u8) {
        self.open = data;
        match addr & 7 {
            // PPUCTRL
            0 => {
                self.nametable = Quadrant::from(data & 0b11);
                self.vram_direction = Direction::from(data & 0b00000100 != 0);
                self.sprite_table = Half::from(data & 0b00001000 != 0);
                self.background_table = Half::from(data & 0b00010000 != 0);
                self.sprite_size = SpriteSize::from(data & 0b00100000 != 0);

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
            // PPUMASK
            1 => {
                let old = self.PPUMASK.contains(PPUMASK::ShowBackground);
                let val = EnumSet::from_u8(data);
                let new = self.PPUMASK.contains(PPUMASK::ShowBackground);

                // background
                if old != new {
                    self.sync(cycle);
                    if new {
                        // enable background
                        if self.odd_frame && cycle < self.next_vbl - 241 * SCANLINE - 3 {
                            if let Some(next) = self.next_nmi.as_mut() {
                                if *next == self.next_vbl {
                                    *next -= 1;
                                }
                            }
                            self.next_vbl -= 1;
                        }
                    } else {
                        // disable background
                        if self.odd_frame && cycle < self.next_vbl - 241 * SCANLINE - 2 {
                            if let Some(next) = self.next_nmi.as_mut() {
                                if *next == self.next_vbl {
                                    *next += 1;
                                }
                            }
                            self.next_vbl += 1;
                        }
                    }
                }

                self.PPUMASK = val;
            }
            // PPUADDR
            6 => {
                self.PPUADDR = (self.PPUADDR << 8) | u16::from(data);
            }
            _ => todo!("write ${:04X} = {:08b}", addr, data),
        }
    }

    // we inline never because this tends to get inlined inside read_addr,
    // making every instruction slower just to make ppu reads a bit faster
    #[inline(never)]
    fn read(&mut self, cycle: usize, addr: u16) -> u8 {
        match addr & 7 {
            // PPUSTATUS
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
            _ => todo!("read ${:04X}", addr),
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
