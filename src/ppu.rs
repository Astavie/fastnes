use enumset::{EnumSet, EnumSetType};

use crate::cpu::DynCartridge;

pub trait PPU {
    fn write(&mut self, cycle: usize, addr: u16, data: u8, cart: &mut DynCartridge);
    fn read(&mut self, cycle: usize, addr: u16, cart: &DynCartridge) -> u8;

    fn nmi(&mut self, cycle: usize) -> bool;
    fn reset(&mut self);
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

    // PPUCTRL
    nametable: Nametable,
    vram_direction: Direction,
    sprite_table: PatternTable,
    background_table: PatternTable,
    sprite_size: SpriteSize,
    nmi_output: bool,

    // memory
    palette_ram: [u8; 0x0020],
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

            // PPUCTRL
            nametable: Nametable::TopLeft,
            vram_direction: Direction::Across,
            sprite_table: PatternTable::Left,
            background_table: PatternTable::Left,
            sprite_size: SpriteSize::Small,
            nmi_output: false,

            // memory
            palette_ram: [0; 0x0020],
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
                    match self.PPUADDR & 0x3000 {
                        // pattern table
                        0x0000 | 0x1000 => cart.write_chr(self.PPUADDR, data),

                        // nametable
                        0x2000 => cart.write_nametable(self.PPUADDR, data),
                        0x3000 => cart.write_nametable(self.PPUADDR - 0x1000, data),

                        _ => unreachable!(),
                    }
                }

                // increment PPUADDR
                self.vram_direction.increment(&mut self.PPUADDR);
            }
            _ => todo!("write ${:04X} = {:08b}", addr, data),
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
            // _ => todo!("read ${:04X}", addr),
            _ => (),
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
}
