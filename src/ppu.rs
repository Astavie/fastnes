use enumset::{EnumSet, EnumSetType};

use crate::cpu::DynCartridge;

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

pub trait PPU {
    fn write(&mut self, cycle: usize, addr: u16, data: u8, cart: &mut DynCartridge);
    fn read(&mut self, cycle: usize, addr: u16, cart: &DynCartridge) -> u8;

    fn nmi(&mut self, cycle: usize) -> bool;
    fn reset(&mut self);

    fn frame(&self, cart: &DynCartridge) -> [Color; 61440]; // 256 * 240 pixels
}

pub(crate) struct DummyPPU;

impl PPU for DummyPPU {
    fn write(&mut self, _cycle: usize, _addr: u16, _data: u8, _cart: &mut DynCartridge) {}
    fn read(&mut self, _cycle: usize, _addr: u16, _cart: &DynCartridge) -> u8 {
        0
    }

    fn nmi(&mut self, _cycle: usize) -> bool {
        false
    }
    fn reset(&mut self) {}

    fn frame(&self, _cart: &DynCartridge) -> [Color; 61440] {
        [Color { r: 0, g: 0, b: 0 }; 61440]
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Direction {
    Across,
    Down,
}

impl Direction {
    fn increment(&self, addr: &mut u16) {
        *addr = (match self {
            Self::Across => *addr + 1,
            Self::Down => *addr + 32,
        }) & 0x3FFF;
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
enum Nametable {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
}

impl From<u8> for Nametable {
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

impl Nametable {
    fn addr(&self) -> u16 {
        match self {
            Nametable::TopLeft => 0x2000,
            Nametable::TopRight => 0x2400,
            Nametable::BottomLeft => 0x2800,
            Nametable::BottomRight => 0x2C00,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PatternTable {
    Left,
    Right,
}

impl From<bool> for PatternTable {
    fn from(value: bool) -> Self {
        if value {
            Self::Right
        } else {
            Self::Left
        }
    }
}

impl PatternTable {
    fn addr(&self) -> u16 {
        match self {
            PatternTable::Left => 0x0000,
            PatternTable::Right => 0x1000,
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
    OAMADDR: u8,
    PPUSCROLL: u16,

    // PPUCTRL
    nametable: Nametable,
    vram_direction: Direction,
    sprite_table: PatternTable,
    background_table: PatternTable,
    sprite_size: SpriteSize,
    nmi_output: bool,

    // memory
    palette_ram: [u8; 32],
    OAM: [u8; 256],
}

const SCANLINE: usize = 341;
const FRAME: usize = 262 * SCANLINE;

impl FastPPU {
    pub fn new() -> Self {
        FastPPU {
            next_vbl: 241 * SCANLINE + 1,
            this_vbl: None,
            next_nmi: None,
            odd_frame: false,
            open: 0,

            // registers
            PPUADDR: 0,
            PPUMASK: EnumSet::new(),
            OAMADDR: 0,
            PPUSCROLL: 0,

            // PPUCTRL
            nametable: Nametable::TopLeft,
            vram_direction: Direction::Across,
            sprite_table: PatternTable::Left,
            background_table: PatternTable::Left,
            sprite_size: SpriteSize::Small,
            nmi_output: false,

            // memory
            palette_ram: [0; 32],
            OAM: [0; 256],
        }
    }
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
    fn write(&mut self, cycle: usize, addr: u16, data: u8, cart: &mut DynCartridge) {
        self.open = data;
        match addr & 7 {
            // PPUCTRL
            0 => {
                self.nametable = Nametable::from(data & 0b11);
                self.vram_direction = Direction::from(data & 0b00000100 != 0);
                self.sprite_table = PatternTable::from(data & 0b00001000 != 0);
                self.background_table = PatternTable::from(data & 0b00010000 != 0);
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
            // OAMADDR
            3 => {
                self.OAMADDR = data;
            }
            // OAMDATA
            4 => {
                self.OAM[usize::from(self.OAMADDR)] = data;
                self.OAMADDR = self.OAMADDR.wrapping_add(1);
            }
            // PPUSCROLL
            5 => {
                self.PPUSCROLL = (self.PPUSCROLL << 8) | u16::from(data);
            }
            // PPUADDR
            6 => {
                self.PPUADDR = ((self.PPUADDR << 8) | u16::from(data)) & 0x3FFF;
            }
            // PPUDATA
            7 => {
                if self.PPUADDR >= 0x3F00 && self.PPUADDR < 0x3F20 {
                    // palette ram
                    self.palette_ram[usize::from(self.PPUADDR as u8)] = data;
                } else if let Some(cart) = cart.as_mut() {
                    match self.PPUADDR & 0x2000 {
                        // pattern table
                        0x0000 => cart.write_chr(self.PPUADDR, data),

                        // nametable
                        0x2000 => cart.write_nametable(self.PPUADDR, data),

                        _ => unreachable!(),
                    }
                }

                // increment PPUADDR
                self.vram_direction.increment(&mut self.PPUADDR);
            }
            _ => (),
        }
    }

    fn read(&mut self, cycle: usize, addr: u16, cart: &DynCartridge) -> u8 {
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

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn frame(&self, cart: &DynCartridge) -> [Color; 61440] {
        let mut frame = [Color { r: 0, g: 0, b: 0 }; 61440];

        if let Some(cart) = cart {
            let nametable = self.nametable.addr();
            let bg_table = self.background_table.addr();
            let scroll_x = self.PPUSCROLL >> 8;
            let scroll_y = self.PPUSCROLL & 0xFF;

            for screen_y in 0..240 {
                for screen_x in 0..256 {
                    let mut x = screen_x + scroll_x;
                    let mut y = screen_y + scroll_y;
                    let mut nametable = nametable;

                    if x >= 256 {
                        // nametable to the right
                        x -= 256;
                        nametable += 0x400;
                    }

                    if y >= 240 {
                        // nametable to the bottom
                        y -= 240;
                        nametable += 0x800;
                    }

                    let tile_addr = nametable + (x / 8) + (y / 8) * 32;
                    let attribute_addr = nametable + 0x3C0 + (x / 32) + (y / 32) * 8;

                    let tile = cart.read_nametable(tile_addr);
                    let attribute = cart.read_nametable(attribute_addr);

                    let palette = match (x & 16 == 0, y & 16 == 0) {
                        (true, true) => (attribute >> 0) & 0b11,   // top left
                        (false, true) => (attribute >> 2) & 0b11,  // top right
                        (true, false) => (attribute >> 4) & 0b11,  // bottom left
                        (false, false) => (attribute >> 6) & 0b11, // bottom right
                    };

                    let tile_x = 7 - (x & 7);
                    let tile_y = y & 7;

                    let pattern_addr = bg_table + u16::from(tile) * 16 + tile_y;

                    let color_lo = (cart.read_chr(pattern_addr) >> tile_x) & 1;
                    let color_hi = (cart.read_chr(pattern_addr + 8) >> tile_x) & 1;
                    let color_indx = color_lo | (color_hi << 1);

                    let color = if color_indx == 0 {
                        self.palette_ram[0]
                    } else {
                        self.palette_ram[usize::from(palette * 4 + color_indx)]
                    };

                    frame[usize::from(screen_x + screen_y * 256)] =
                        PALETTE[usize::from(color & 0x3F)];
                }
            }
        }

        frame
    }
}

const PALETTE: [Color; 64] = [
    Color {
        r: 84,
        g: 84,
        b: 84,
    },
    Color {
        r: 0,
        g: 30,
        b: 116,
    },
    Color {
        r: 8,
        g: 16,
        b: 144,
    },
    Color {
        r: 48,
        g: 0,
        b: 136,
    },
    Color {
        r: 68,
        g: 0,
        b: 100,
    },
    Color { r: 92, g: 0, b: 48 },
    Color { r: 84, g: 4, b: 0 },
    Color { r: 60, g: 24, b: 0 },
    Color { r: 32, g: 42, b: 0 },
    Color { r: 8, g: 58, b: 0 },
    Color { r: 0, g: 64, b: 0 },
    Color { r: 0, g: 60, b: 0 },
    Color { r: 0, g: 50, b: 60 },
    Color { r: 0, g: 0, b: 0 },
    Color { r: 0, g: 0, b: 0 },
    Color { r: 0, g: 0, b: 0 },
    Color {
        r: 152,
        g: 150,
        b: 152,
    },
    Color {
        r: 8,
        g: 76,
        b: 196,
    },
    Color {
        r: 48,
        g: 50,
        b: 236,
    },
    Color {
        r: 92,
        g: 30,
        b: 228,
    },
    Color {
        r: 136,
        g: 20,
        b: 176,
    },
    Color {
        r: 160,
        g: 20,
        b: 100,
    },
    Color {
        r: 152,
        g: 34,
        b: 32,
    },
    Color {
        r: 120,
        g: 60,
        b: 0,
    },
    Color { r: 84, g: 90, b: 0 },
    Color {
        r: 40,
        g: 114,
        b: 0,
    },
    Color { r: 8, g: 124, b: 0 },
    Color {
        r: 0,
        g: 118,
        b: 40,
    },
    Color {
        r: 0,
        g: 102,
        b: 120,
    },
    Color { r: 0, g: 0, b: 0 },
    Color { r: 0, g: 0, b: 0 },
    Color { r: 0, g: 0, b: 0 },
    Color {
        r: 236,
        g: 238,
        b: 236,
    },
    Color {
        r: 76,
        g: 154,
        b: 236,
    },
    Color {
        r: 120,
        g: 124,
        b: 236,
    },
    Color {
        r: 176,
        g: 98,
        b: 236,
    },
    Color {
        r: 228,
        g: 84,
        b: 236,
    },
    Color {
        r: 236,
        g: 88,
        b: 180,
    },
    Color {
        r: 236,
        g: 106,
        b: 100,
    },
    Color {
        r: 212,
        g: 136,
        b: 32,
    },
    Color {
        r: 160,
        g: 170,
        b: 0,
    },
    Color {
        r: 116,
        g: 196,
        b: 0,
    },
    Color {
        r: 76,
        g: 208,
        b: 32,
    },
    Color {
        r: 56,
        g: 204,
        b: 108,
    },
    Color {
        r: 56,
        g: 180,
        b: 204,
    },
    Color {
        r: 60,
        g: 60,
        b: 60,
    },
    Color { r: 0, g: 0, b: 0 },
    Color { r: 0, g: 0, b: 0 },
    Color {
        r: 236,
        g: 238,
        b: 236,
    },
    Color {
        r: 168,
        g: 204,
        b: 236,
    },
    Color {
        r: 188,
        g: 188,
        b: 236,
    },
    Color {
        r: 212,
        g: 178,
        b: 236,
    },
    Color {
        r: 236,
        g: 174,
        b: 236,
    },
    Color {
        r: 236,
        g: 174,
        b: 212,
    },
    Color {
        r: 236,
        g: 180,
        b: 176,
    },
    Color {
        r: 228,
        g: 196,
        b: 144,
    },
    Color {
        r: 204,
        g: 210,
        b: 120,
    },
    Color {
        r: 180,
        g: 222,
        b: 120,
    },
    Color {
        r: 168,
        g: 226,
        b: 144,
    },
    Color {
        r: 152,
        g: 226,
        b: 180,
    },
    Color {
        r: 160,
        g: 214,
        b: 228,
    },
    Color {
        r: 160,
        g: 162,
        b: 160,
    },
    Color { r: 0, g: 0, b: 0 },
    Color { r: 0, g: 0, b: 0 },
];
