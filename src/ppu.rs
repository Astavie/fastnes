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

pub(crate) struct FastPPU {
    open: u8,
    odd_frame: bool,

    nmi_output: bool,
    background: bool,

    this_vbl: Option<usize>,
    next_vbl: usize,
    next_nmi: Option<usize>,
}

const SCANLINE: usize = 341;
const FRAME: usize = 262 * SCANLINE;

impl FastPPU {
    fn sync(&mut self, cycle: usize) {
        // get next vblank
        while cycle >= self.next_vbl {
            self.this_vbl = Some(self.next_vbl);
            self.odd_frame = !self.odd_frame;
            self.next_vbl += FRAME - usize::from(self.background && self.odd_frame);
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
            nmi_output: false,
            odd_frame: false,
            background: false,
            open: 0,
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
                // TODO: the rest

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
                // TODO: the rest

                // background
                self.sync(cycle);
                match on_change(&mut self.background, data & 0b1000 != 0) {
                    Some(true) => {
                        // enable background
                        if self.odd_frame && cycle < self.next_vbl - 241 * SCANLINE - 3 {
                            if let Some(next) = self.next_nmi.as_mut() {
                                if *next == self.next_vbl {
                                    *next -= 1;
                                }
                            }
                            self.next_vbl -= 1;
                        }
                    }
                    Some(false) => {
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
                    None => (),
                }
            }
            _ => {
                // TODO
            }
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
            _ => {
                // TODO
            }
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
