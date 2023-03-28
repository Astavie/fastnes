use std::{cell::Cell, rc::Rc};

pub struct Controllers {
    left: Box<dyn Controller>,
    right: Box<dyn Controller>,
}

impl Controllers {
    pub(crate) fn read_left(&mut self) -> u8 {
        self.left.read()
    }
    pub(crate) fn read_right(&mut self) -> u8 {
        self.right.read()
    }
    pub(crate) fn write(&mut self, data: u8) {
        self.left.write(data);
        self.right.write(data);
    }
    pub fn standard() -> (Controllers, Rc<Cell<u8>>) {
        let input = Rc::new(Cell::new(0));
        let left = StandardController {
            input: Rc::clone(&input),
            shift: None,
        };
        let right = Unconnected;
        (
            Controllers {
                left: Box::new(left),
                right: Box::new(right),
            },
            input,
        )
    }
    pub fn disconnected() -> Controllers {
        Controllers {
            left: Box::new(Unconnected),
            right: Box::new(Unconnected),
        }
    }
}

trait Controller {
    fn read(&mut self) -> u8;
    fn write(&mut self, data: u8);
}

struct StandardController {
    pub input: Rc<Cell<u8>>,
    shift: Option<u8>,
}

struct Unconnected;

impl Controller for Unconnected {
    fn read(&mut self) -> u8 {
        0
    }
    fn write(&mut self, data: u8) {}
}

impl Controller for StandardController {
    fn read(&mut self) -> u8 {
        let shift = self.shift.unwrap_or(0);
        let data = (self.input.get() >> shift) & 1;
        println!("{:08b} {:08b}", data, self.input.get());
        self.shift = self.shift.map(|s| s + 1);
        data
    }
    fn write(&mut self, data: u8) {
        if data & 1 == 1 {
            self.shift = None;
        } else {
            self.shift = Some(0);
        }
    }
}
