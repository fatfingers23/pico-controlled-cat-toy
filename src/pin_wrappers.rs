use embedded_hal::digital::v2::OutputPin;
use rp_pico_w::hal;
use rp_pico_w::hal::gpio::DynPin;

/// Generic wrapper for the RP2040 GPIO pins.
pub(crate) struct InOutPin {
    inner: DynPin,
}

impl InOutPin {
    pub(crate) fn new(inner: DynPin) -> Self {
        Self { inner }
    }
}

impl OutputPin for InOutPin {
    type Error = hal::gpio::Error;
    fn set_low(&mut self) -> Result<(), <Self as embedded_hal::digital::v2::OutputPin>::Error> {
        self.inner.into_push_pull_output();
        self.inner.set_low()?;
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), <Self as embedded_hal::digital::v2::OutputPin>::Error> {
        self.inner.into_push_pull_output();
        self.inner.set_high()?;
        Ok(())
    }
}