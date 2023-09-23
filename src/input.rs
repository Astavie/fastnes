use std::sync::{
    atomic::{AtomicU8, Ordering},
    Arc,
};

pub struct Controllers {
    pub left: Box<dyn Controller + Send>,
    pub right: Box<dyn Controller + Send>,
    pub polled: bool,
}

impl Controllers {
    pub(crate) fn read_left(&mut self, open: u8) -> u8 {
        self.polled = true;
        self.left.read(open)
    }
    pub(crate) fn read_right(&mut self, open: u8) -> u8 {
        self.polled = true;
        self.right.read(open)
    }
    pub(crate) fn write(&mut self, data: u8) {
        self.left.write(data);
        self.right.write(data);
    }
    pub fn standard(input: &Arc<AtomicU8>) -> Controllers {
        let left = StandardController {
            input: Arc::clone(input),
            shift: None,
        };
        let right = Unconnected;
        Controllers {
            left: Box::new(left),
            right: Box::new(right),
            polled: false,
        }
    }
    pub fn disconnected() -> Controllers {
        Controllers {
            left: Box::new(Unconnected),
            right: Box::new(Unconnected),
            polled: false,
        }
    }
    pub fn was_polled(&mut self) -> bool {
        let ret = self.polled;
        self.polled = false;
        ret
    }
}

pub trait Controller {
    fn read(&mut self, open: u8) -> u8;
    fn write(&mut self, data: u8);
}

struct StandardController {
    pub input: Arc<AtomicU8>,
    shift: Option<u8>,
}

struct Unconnected;

impl Controller for Unconnected {
    fn read(&mut self, open: u8) -> u8 { open }
    fn write(&mut self, _data: u8) {}
}

impl Controller for StandardController {
    fn read(&mut self, open: u8) -> u8 {
        let shift = self.shift.unwrap_or(0);
        let data = (self.input.load(Ordering::Relaxed) >> shift) & 1;
        self.shift = self.shift.map(|s| s + 1);
        (open & 0b11111110) | data
    }
    fn write(&mut self, data: u8) {
        if data & 1 == 1 {
            self.shift = None;
        } else {
            self.shift = Some(0);
        }
    }
}
