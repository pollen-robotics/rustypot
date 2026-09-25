//! A serial port that answers from a script, for tests that have no bus.

use std::collections::VecDeque;
use std::io;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// What the code under test did to the port's settings.
///
/// A controller owns its port, so a test keeps sight of these through the `Arc` that
/// [`FakePort::settings`] hands out before the port is given away.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Settings {
    pub baud_rate: u32,
    /// Every timeout set on the port, oldest first; the last one is in force.
    pub timeouts: Vec<Duration>,
}

/// A serial port that replays canned status packets and throws away what is written.
///
/// Each instruction written loads the next scripted answer. An empty answer, or a
/// script that has run out, reads as nothing on the wire: the timeout case, without the
/// wait. `bytes_to_read` always answers 0, so the pre-send flush never eats a queued
/// answer. Everything the protocol does not call is left unimplemented.
pub(crate) struct FakePort {
    answers: VecDeque<Vec<u8>>,
    to_read: io::Cursor<Vec<u8>>,
    settings: Arc<Mutex<Settings>>,
}

impl FakePort {
    pub(crate) fn new(answers: Vec<Vec<u8>>) -> Self {
        FakePort {
            answers: answers.into(),
            to_read: io::Cursor::new(Vec::new()),
            settings: Arc::new(Mutex::new(Settings {
                baud_rate: 1_000_000,
                timeouts: vec![Duration::from_millis(10)],
            })),
        }
    }

    pub(crate) fn settings(&self) -> Arc<Mutex<Settings>> {
        Arc::clone(&self.settings)
    }
}

impl io::Read for FakePort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.to_read.read(buf)
    }
}

impl io::Write for FakePort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.to_read = io::Cursor::new(self.answers.pop_front().unwrap_or_default());
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl serialport::SerialPort for FakePort {
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn timeout(&self) -> Duration {
        *self.settings.lock().unwrap().timeouts.last().unwrap()
    }
    fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
        self.settings.lock().unwrap().timeouts.push(timeout);
        Ok(())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(self.settings.lock().unwrap().baud_rate)
    }
    fn set_baud_rate(&mut self, baud_rate: u32) -> serialport::Result<()> {
        self.settings.lock().unwrap().baud_rate = baud_rate;
        Ok(())
    }

    fn name(&self) -> Option<String> {
        None
    }
    fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
        unimplemented!()
    }
    fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
        unimplemented!()
    }
    fn parity(&self) -> serialport::Result<serialport::Parity> {
        unimplemented!()
    }
    fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
        unimplemented!()
    }
    fn set_data_bits(&mut self, _: serialport::DataBits) -> serialport::Result<()> {
        unimplemented!()
    }
    fn set_flow_control(&mut self, _: serialport::FlowControl) -> serialport::Result<()> {
        unimplemented!()
    }
    fn set_parity(&mut self, _: serialport::Parity) -> serialport::Result<()> {
        unimplemented!()
    }
    fn set_stop_bits(&mut self, _: serialport::StopBits) -> serialport::Result<()> {
        unimplemented!()
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        unimplemented!()
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        unimplemented!()
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        unimplemented!()
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        unimplemented!()
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        unimplemented!()
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        unimplemented!()
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        unimplemented!()
    }
    fn clear(&self, _: serialport::ClearBuffer) -> serialport::Result<()> {
        unimplemented!()
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn serialport::SerialPort>> {
        unimplemented!()
    }
    fn set_break(&self) -> serialport::Result<()> {
        unimplemented!()
    }
    fn clear_break(&self) -> serialport::Result<()> {
        unimplemented!()
    }
}
