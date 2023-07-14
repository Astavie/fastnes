use enumset::{EnumSet, EnumSetType};

use crate::cart::Cartridge;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

pub trait PPU: Clone {
    fn write(&mut self, cycle: usize, addr: u16, data: u8, cart: &mut impl Cartridge);
    fn read(&mut self, cycle: usize, addr: u16, cart: &impl Cartridge) -> u8;

    fn nmi(&mut self, cycle: usize) -> bool;
    fn reset(&mut self);

    fn frame(&self, cart: &impl Cartridge, options: DrawOptions) -> [Color; 61440]; // 256 * 240 pixels
    fn frame_number(&mut self, cycle: usize) -> usize;
}

#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum DrawOptions {
    #[default]
    All,
    Background,
    Sprites,
}

#[derive(Clone)]
pub(crate) struct DummyPPU;

impl PPU for DummyPPU {
    fn write(&mut self, _cycle: usize, _addr: u16, _data: u8, _cart: &mut impl Cartridge) {}
    fn read(&mut self, _cycle: usize, _addr: u16, _cart: &impl Cartridge) -> u8 {
        0
    }

    fn nmi(&mut self, _cycle: usize) -> bool {
        false
    }
    fn reset(&mut self) {}

    fn frame(&self, _cart: &impl Cartridge, _options: DrawOptions) -> [Color; 61440] {
        [Color {
            r: 0,
            g: 0,
            b: 0,
            a: 255,
        }; 61440]
    }
    fn frame_number(&mut self, _cycle: usize) -> usize {
        0
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
        }) & 0x7FFF;
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

impl From<Nametable> for u8 {
    fn from(value: Nametable) -> Self {
        match value {
            Nametable::TopLeft => 0,
            Nametable::TopRight => 1,
            Nametable::BottomLeft => 2,
            Nametable::BottomRight => 3,
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
    Single, // 8x8
    Double, // 8x16
}

impl From<bool> for SpriteSize {
    fn from(value: bool) -> Self {
        if value {
            Self::Double
        } else {
            Self::Single
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
#[derive(Clone)]
pub struct FastPPU {
    open: u8,

    next_vbl: usize,
    next_nmi: Option<usize>,
    next_sprite_0: Option<usize>,
    vbl_started: Option<usize>,
    last_cycle: usize,

    frame: usize,

    // registers
    PPUMASK: EnumSet<PPUMASK>,
    OAMADDR: u8,
    PPUDATA: u8,
    scroll_x: u8,
    scroll_y: u8,
    w: u8,
    v: u16,

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
            next_nmi: None,
            frame: 0,
            open: 0,
            vbl_started: None,
            next_sprite_0: None,
            last_cycle: 0,

            // registers
            PPUMASK: EnumSet::new(),
            OAMADDR: 0,
            PPUDATA: 0,
            scroll_x: 0,
            scroll_y: 0,
            w: 0,
            v: 0,

            // PPUCTRL
            nametable: Nametable::TopLeft,
            vram_direction: Direction::Across,
            sprite_table: PatternTable::Left,
            background_table: PatternTable::Left,
            sprite_size: SpriteSize::Single,
            nmi_output: false,

            // memory
            palette_ram: [0; 32],
            OAM: [0; 256],
        }
    }
    fn sync(&mut self, cycle: usize) {
        // get next vblank
        while cycle >= self.next_vbl {
            self.frame += 1;
            self.vbl_started = Some(self.next_vbl);
            self.next_vbl += FRAME
                - usize::from(
                    self.PPUMASK.contains(PPUMASK::ShowBackground) && (self.frame & 1 == 1),
                );
        }

        // end vblank after 20 scanlines
        self.vbl_started = self.vbl_started.filter(|i| cycle <= i + 20 * SCANLINE);
    }
    fn next_sprite_0(&mut self, cycle: usize, cart: &impl Cartridge) {
        // check if previous sprite0 check was still this frame
        self.next_sprite_0 = self.next_sprite_0.filter(|i| {
            *i >= self
                .vbl_started
                .filter(|i| cycle < i + 20 * SCANLINE)
                .unwrap_or(self.next_vbl)
                - 241 * SCANLINE
                + 1
        });

        if !self.next_sprite_0.is_some()
            && self.PPUMASK.contains(PPUMASK::ShowBackground)
            && self.PPUMASK.contains(PPUMASK::ShowSprites)
        {
            let frame_start = self
                .vbl_started
                .filter(|i| cycle < i + 20 * SCANLINE)
                .unwrap_or(self.next_vbl)
                - 241 * SCANLINE
                + 1;

            let x = self.OAM[3];
            let y = self.OAM[0];

            let tile = self.OAM[1];
            let flip_hor = self.OAM[2] & 0b01000000 != 0;
            let flip_ver = self.OAM[2] & 0b10000000 != 0;

            let pattern_addr = match self.sprite_size {
                SpriteSize::Single => self.sprite_table.addr() + u16::from(tile) * 16,
                SpriteSize::Double => {
                    (u16::from(tile & 1) * 0x1000) + u16::from(tile & 0b11111110) * 16
                }
            };

            let height = match self.sprite_size {
                SpriteSize::Single => 8,
                SpriteSize::Double => 16,
            };

            let nametable = self.nametable.addr();

            for ypos in 0..height {
                for xpos in 0..8 {
                    let screen_x = u16::from(x) + xpos as u16;
                    let screen_y = u16::from(y) + 1 + ypos as u16;

                    // check if off the screen
                    if screen_x < 8
                        && (!self.PPUMASK.contains(PPUMASK::ShowBackgroundLeft)
                            || !self.PPUMASK.contains(PPUMASK::ShowSpritesLeft))
                    {
                        continue;
                    }

                    if screen_x >= 255 || screen_y >= 240 {
                        continue;
                    }

                    // check if after last cycle
                    let dot_cycle =
                        frame_start + usize::from(screen_y) * SCANLINE + usize::from(screen_x);

                    if dot_cycle <= self.last_cycle {
                        continue;
                    }

                    // get if opaque
                    let sprite_x: u8 = if flip_hor { 7 - xpos } else { xpos };
                    let sprite_y: u8 = if flip_ver { height - 1 - ypos } else { ypos };

                    let pattern_addr = if sprite_y >= 8 {
                        pattern_addr + u16::from(sprite_y) + 8
                    } else {
                        pattern_addr + u16::from(sprite_y)
                    };

                    let color_lo = (cart.read_chr(pattern_addr) >> (7 - sprite_x)) & 1;
                    let color_hi = (cart.read_chr(pattern_addr + 8) >> (7 - sprite_x)) & 1;
                    let color_indx = color_lo | (color_hi << 1);

                    if color_indx == 0 {
                        continue;
                    }

                    // get if tile underneath is opaque
                    let mut x = screen_x + u16::from(self.scroll_x);
                    let mut y = screen_y + u16::from(self.scroll_y);
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
                    let tile = cart.read_nametable(tile_addr);

                    let tile_x = x & 7;
                    let tile_y = y & 7;

                    let pattern_addr = self.background_table.addr() + u16::from(tile) * 16 + tile_y;

                    let color_lo = (cart.read_chr(pattern_addr) >> (7 - tile_x)) & 1;
                    let color_hi = (cart.read_chr(pattern_addr + 8) >> (7 - tile_x)) & 1;
                    let color_indx = color_lo | (color_hi << 1);

                    if color_indx == 0 {
                        continue;
                    }

                    // sprite 0 hit!
                    self.next_sprite_0 = Some(dot_cycle);
                    return;
                }
            }
            self.last_cycle = cycle;
        }
    }

    fn draw_sprites(&self, background: bool, cart: &impl Cartridge, frame: &mut [Color]) {
        for sprite in 0..64 {
            let data = &self.OAM[sprite * 4..];

            // ignore other sprites
            if (data[2] & 0b00100000 == 0) == background {
                continue;
            }

            // ignore below the screen
            if data[0] >= 0xEF {
                continue;
            }

            let tile = data[1];
            let pattern_addr = match self.sprite_size {
                SpriteSize::Single => self.sprite_table.addr() + u16::from(tile) * 16,
                SpriteSize::Double => {
                    (u16::from(tile & 1) * 0x1000) + u16::from(tile & 0b11111110) * 16
                }
            };

            let height = match self.sprite_size {
                SpriteSize::Single => 8,
                SpriteSize::Double => 16,
            };

            let palette = data[2] & 0b11;
            let flip_hor = data[2] & 0b01000000 != 0;
            let flip_ver = data[2] & 0b10000000 != 0;

            let x = data[3];
            let y = data[0];

            for sprite_y in 0..height {
                for sprite_x in 0..8 {
                    let screen_x = u16::from(x)
                        + if flip_hor {
                            u16::from(7 - u8::from(sprite_x))
                        } else {
                            u16::from(sprite_x)
                        };

                    let screen_y = u16::from(y)
                        + 1
                        + if flip_ver {
                            u16::from(height - 1 - u8::from(sprite_y))
                        } else {
                            u16::from(sprite_y)
                        };

                    if screen_x < 8 && !self.PPUMASK.contains(PPUMASK::ShowSpritesLeft) {
                        continue;
                    }

                    if screen_x >= 256 || screen_y >= 240 {
                        continue;
                    }

                    let pattern_addr = if sprite_y >= 8 {
                        pattern_addr + u16::from(sprite_y) + 8
                    } else {
                        pattern_addr + u16::from(sprite_y)
                    };

                    let color_lo = (cart.read_chr(pattern_addr) >> (7 - sprite_x)) & 1;
                    let color_hi = (cart.read_chr(pattern_addr + 8) >> (7 - sprite_x)) & 1;
                    let color_indx = color_lo | (color_hi << 1);

                    let color = if color_indx == 0 {
                        continue;
                    } else {
                        self.palette_ram[usize::from(0x10 + palette * 4 + color_indx)]
                    };

                    frame[usize::from(screen_x + screen_y * 256)] =
                        PALETTE[usize::from(color & 0x3F)];
                }
            }
        }
    }

    fn draw_tiles(&self, cart: &impl Cartridge, frame: &mut [Color]) {
        let nametable = self.nametable.addr();

        let x_start = if self.PPUMASK.contains(PPUMASK::ShowBackgroundLeft) {
            0
        } else {
            8
        };

        // background
        for screen_y in 0..240 {
            for screen_x in x_start..256 {
                let mut x = screen_x + u16::from(self.scroll_x);
                let mut y = screen_y + u16::from(self.scroll_y);
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

                let tile_x = x & 7;
                let tile_y = y & 7;

                let pattern_addr = self.background_table.addr() + u16::from(tile) * 16 + tile_y;

                let color_lo = (cart.read_chr(pattern_addr) >> (7 - tile_x)) & 1;
                let color_hi = (cart.read_chr(pattern_addr + 8) >> (7 - tile_x)) & 1;
                let color_indx = color_lo | (color_hi << 1);

                let color = if color_indx == 0 {
                    continue;
                } else {
                    self.palette_ram[usize::from(palette * 4 + color_indx)]
                };

                frame[usize::from(screen_x + screen_y * 256)] = PALETTE[usize::from(color & 0x3F)];
            }
        }
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
    fn write(&mut self, cycle: usize, addr: u16, data: u8, cart: &mut impl Cartridge) {
        self.sync(cycle);

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
                        self.next_nmi = Some(
                            // ignore last dot of vblank
                            self.vbl_started
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
                let new = val.contains(PPUMASK::ShowBackground);

                // background
                if old != new {
                    if new {
                        // enable background
                        if (self.frame & 1 == 1) && cycle < self.next_vbl - 241 * SCANLINE - 3 {
                            if let Some(next) = self.next_nmi.as_mut() {
                                if *next == self.next_vbl {
                                    *next -= 1;
                                }
                            }
                            self.next_vbl -= 1;
                        }
                    } else {
                        // disable background
                        if (self.frame & 1 == 1) && cycle < self.next_vbl - 241 * SCANLINE - 2 {
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
                if self.w == 0 {
                    self.scroll_x = data;
                } else {
                    self.scroll_y = data;
                }
                self.w ^= 1;
            }
            // PPUADDR
            6 => {
                if self.w == 0 {
                    self.scroll_y &= 0b00111000;
                    self.scroll_y |= (data >> 4) & 0b111;
                    self.nametable = Nametable::from((data >> 2) & 0b11);
                    self.scroll_y |= (data << 6) & 0b11000000;
                } else {
                    self.scroll_x &= 0b00000111;
                    self.scroll_x |= data << 3;
                    self.scroll_y &= 0b11000111;
                    self.scroll_y |= (data >> 2) & 0b00111000;

                    let mut high: u8 = 0;
                    high |= (self.scroll_y & 0b111) << 4;
                    high |= (self.scroll_y >> 6) & 0b11;
                    high |= u8::from(self.nametable) << 2;
                    self.v = u16::from(high) << 8 | u16::from(data);
                }
                self.w ^= 1;
            }
            // PPUDATA
            7 => {
                let addr = self.v & 0x3FFF;
                if addr >= 0x3F00 && addr < 0x3F20 {
                    // palette ram
                    self.palette_ram[usize::from(addr as u8)] = data;

                    // FIXME: hacky way to implement mirrors
                    if addr as u8 >= 0x10 {
                        self.palette_ram[0x00] = self.palette_ram[0x10];
                        self.palette_ram[0x04] = self.palette_ram[0x14];
                        self.palette_ram[0x08] = self.palette_ram[0x18];
                        self.palette_ram[0x0C] = self.palette_ram[0x1C];
                    } else {
                        self.palette_ram[0x10] = self.palette_ram[0x00];
                        self.palette_ram[0x14] = self.palette_ram[0x04];
                        self.palette_ram[0x18] = self.palette_ram[0x08];
                        self.palette_ram[0x1C] = self.palette_ram[0x0C];
                    }
                } else {
                    match addr & 0x2000 {
                        // pattern table
                        0x0000 => cart.write_chr(addr, data),

                        // nametable
                        0x2000 => cart.write_nametable(addr, data),

                        _ => unreachable!(),
                    }
                }

                // increment PPUADDR
                self.vram_direction.increment(&mut self.v);
            }
            _ => (),
        }

        // this write could have affected sprite zero
        // FIXME: this sprite 0 system is incredibly scuffed
        // jesus christ, please make this better
        self.next_sprite_0.filter(|i| cycle >= *i);
        // self.next_sprite_0(cycle, cart);
    }

    fn read(&mut self, cycle: usize, addr: u16, cart: &impl Cartridge) -> u8 {
        match addr & 7 {
            // PPUSTATUS
            2 => {
                self.w = 0;
                self.open &= 0b00011111;

                self.sync(cycle);
                self.next_sprite_0(cycle, cart);

                if self.next_sprite_0.is_some_and(|i| cycle >= i) {
                    self.open |= 0b01000000;
                }

                // vblank
                if self.vbl_started.is_some_and(|i| cycle != i) {
                    self.open |= 0b10000000;
                }

                if self.vbl_started.is_some() {
                    self.vbl_started = None;
                    self.next_nmi = None;
                }
            }
            // PPUDATA
            7 => {
                let addr = self.v & 0x3FFF;

                // palette data is paced immediately on the data bus
                self.open = if addr >= 0x3F00 && addr < 0x3F20 {
                    // palette ram
                    self.palette_ram[usize::from(addr as u8)]
                } else {
                    self.PPUDATA
                };

                // reading PPUDATA *always* updates the internal buffer
                self.PPUDATA = match addr & 0x2000 {
                    // pattern table
                    0x0000 => cart.read_chr(addr),

                    // nametable
                    0x2000 => cart.read_nametable(addr),

                    _ => unreachable!(),
                };

                // increment PPUADDR
                self.vram_direction.increment(&mut self.v);
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

    fn frame(&self, cart: &impl Cartridge, options: DrawOptions) -> [Color; 61440] {
        let sprites = options == DrawOptions::All || options == DrawOptions::Sprites;
        let background = options == DrawOptions::All || options == DrawOptions::Background;

        let mut frame = [if background {
            PALETTE[usize::from(self.palette_ram[0])]
        } else {
            Color {
                r: 0,
                g: 0,
                b: 0,
                a: 0,
            }
        }; 61440];

        if sprites && self.PPUMASK.contains(PPUMASK::ShowSprites) {
            self.draw_sprites(true, cart, &mut frame);
        }
        if background && self.PPUMASK.contains(PPUMASK::ShowBackground) {
            self.draw_tiles(cart, &mut frame);
        }
        if sprites && self.PPUMASK.contains(PPUMASK::ShowSprites) {
            self.draw_sprites(false, cart, &mut frame);
        }

        frame
    }

    fn frame_number(&mut self, cycle: usize) -> usize {
        self.sync(cycle);
        self.frame
    }
}

const PALETTE: [Color; 64] = [
    Color {
        r: 84,
        g: 84,
        b: 84,
        a: 255,
    },
    Color {
        r: 0,
        g: 30,
        b: 116,
        a: 255,
    },
    Color {
        r: 8,
        g: 16,
        b: 144,
        a: 255,
    },
    Color {
        r: 48,
        g: 0,
        b: 136,
        a: 255,
    },
    Color {
        r: 68,
        g: 0,
        b: 100,
        a: 255,
    },
    Color {
        r: 92,
        g: 0,
        b: 48,
        a: 255,
    },
    Color {
        r: 84,
        g: 4,
        b: 0,
        a: 255,
    },
    Color {
        r: 60,
        g: 24,
        b: 0,
        a: 255,
    },
    Color {
        r: 32,
        g: 42,
        b: 0,
        a: 255,
    },
    Color {
        r: 8,
        g: 58,
        b: 0,
        a: 255,
    },
    Color {
        r: 0,
        g: 64,
        b: 0,
        a: 255,
    },
    Color {
        r: 0,
        g: 60,
        b: 0,
        a: 255,
    },
    Color {
        r: 0,
        g: 50,
        b: 60,
        a: 255,
    },
    Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    },
    Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    },
    Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    },
    Color {
        r: 152,
        g: 150,
        b: 152,
        a: 255,
    },
    Color {
        r: 8,
        g: 76,
        b: 196,
        a: 255,
    },
    Color {
        r: 48,
        g: 50,
        b: 236,
        a: 255,
    },
    Color {
        r: 92,
        g: 30,
        b: 228,
        a: 255,
    },
    Color {
        r: 136,
        g: 20,
        b: 176,
        a: 255,
    },
    Color {
        r: 160,
        g: 20,
        b: 100,
        a: 255,
    },
    Color {
        r: 152,
        g: 34,
        b: 32,
        a: 255,
    },
    Color {
        r: 120,
        g: 60,
        b: 0,
        a: 255,
    },
    Color {
        r: 84,
        g: 90,
        b: 0,
        a: 255,
    },
    Color {
        r: 40,
        g: 114,
        b: 0,
        a: 255,
    },
    Color {
        r: 8,
        g: 124,
        b: 0,
        a: 255,
    },
    Color {
        r: 0,
        g: 118,
        b: 40,
        a: 255,
    },
    Color {
        r: 0,
        g: 102,
        b: 120,
        a: 255,
    },
    Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    },
    Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    },
    Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    },
    Color {
        r: 236,
        g: 238,
        b: 236,
        a: 255,
    },
    Color {
        r: 76,
        g: 154,
        b: 236,
        a: 255,
    },
    Color {
        r: 120,
        g: 124,
        b: 236,
        a: 255,
    },
    Color {
        r: 176,
        g: 98,
        b: 236,
        a: 255,
    },
    Color {
        r: 228,
        g: 84,
        b: 236,
        a: 255,
    },
    Color {
        r: 236,
        g: 88,
        b: 180,
        a: 255,
    },
    Color {
        r: 236,
        g: 106,
        b: 100,
        a: 255,
    },
    Color {
        r: 212,
        g: 136,
        b: 32,
        a: 255,
    },
    Color {
        r: 160,
        g: 170,
        b: 0,
        a: 255,
    },
    Color {
        r: 116,
        g: 196,
        b: 0,
        a: 255,
    },
    Color {
        r: 76,
        g: 208,
        b: 32,
        a: 255,
    },
    Color {
        r: 56,
        g: 204,
        b: 108,
        a: 255,
    },
    Color {
        r: 56,
        g: 180,
        b: 204,
        a: 255,
    },
    Color {
        r: 60,
        g: 60,
        b: 60,
        a: 255,
    },
    Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    },
    Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    },
    Color {
        r: 236,
        g: 238,
        b: 236,
        a: 255,
    },
    Color {
        r: 168,
        g: 204,
        b: 236,
        a: 255,
    },
    Color {
        r: 188,
        g: 188,
        b: 236,
        a: 255,
    },
    Color {
        r: 212,
        g: 178,
        b: 236,
        a: 255,
    },
    Color {
        r: 236,
        g: 174,
        b: 236,
        a: 255,
    },
    Color {
        r: 236,
        g: 174,
        b: 212,
        a: 255,
    },
    Color {
        r: 236,
        g: 180,
        b: 176,
        a: 255,
    },
    Color {
        r: 228,
        g: 196,
        b: 144,
        a: 255,
    },
    Color {
        r: 204,
        g: 210,
        b: 120,
        a: 255,
    },
    Color {
        r: 180,
        g: 222,
        b: 120,
        a: 255,
    },
    Color {
        r: 168,
        g: 226,
        b: 144,
        a: 255,
    },
    Color {
        r: 152,
        g: 226,
        b: 180,
        a: 255,
    },
    Color {
        r: 160,
        g: 214,
        b: 228,
        a: 255,
    },
    Color {
        r: 160,
        g: 162,
        b: 160,
        a: 255,
    },
    Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    },
    Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    },
];
