use drone_cortexm::{reg::prelude::*};

use drone_stm32_map::{
    reg,
};

/// Station Management Interface
pub struct SMI<'a> {
    mac_miiar: &'a mut reg::ethernet_mac::Macmiiar<Srt>,
    mac_miidr: &'a mut reg::ethernet_mac::Macmiidr<Srt>,
}

impl<'a> SMI<'a> {
    /// Allocate
    pub fn new(
        mac_miiar: &'a mut reg::ethernet_mac::Macmiiar<Srt>,
        mac_miidr: &'a mut reg::ethernet_mac::Macmiidr<Srt>) -> Self {

        SMI {
            mac_miiar,
            mac_miidr,
        }
    }

    /// Wait for not busy
    fn wait_ready(&self) {
        while self.mac_miiar.mb.read_bit() {}
    }

    fn read_data(&self) -> u16 {
        self.mac_miidr.td.read_bits() as u16
    }

    /// Read an SMI register
    pub fn read(&self, phy: u8, reg: u8) -> u16 {
        self.mac_miiar.modify(|r| {
            r.write_pa(phy.into())
                .write_mr(reg.into())
                /* Read operation MW=0 */
                .clear_mw()
                .set_mb()
        });
        self.wait_ready();

        // Return value:
        self.read_data()
    }

    fn write_data(&self, data: u16) {
        self.mac_miidr.modify(|r| {
            r.write_td(data.into())
        });
    }

    /// Write an SMI register
    pub fn write(&self, phy: u8, reg: u8, data: u16) {
        self.write_data(data);
        self.mac_miiar.modify(|r| {
            r.write_pa(phy.into())
                .write_mr(reg.into())
                /* Write operation MW=1*/
                .set_mw()
                .set_mb()
        });
        self.wait_ready();
    }

    /// Helper: `read()` and `write()` by OR-ing the current value of
    /// the register `reg` with `mask`.
    pub fn set_bits(&self, phy: u8, reg: u8, mask: u16) {
        let value = self.read(phy, reg);
        self.write(phy, reg, value | mask);
    }
}
