#![no_std]
pub mod phy;
use phy::{Phy, PhyStatus};
mod smi;
mod ring;
pub use ring::RingEntry;
mod rx;
use rx::{RxRing, RxRingEntry, RxPacket};
pub use rx::{RxDescriptor, RxError};
mod tx;
use tx::{TxRing, TxRingEntry};
pub use tx::{TxDescriptor, TxError};
mod setup;
pub use setup::setup;

use drone_cortex_m::{reg::prelude::*};

use drone_stm32_map::{
    reg,
};

pub use smoltcp;
mod smoltcp_phy;
pub use smoltcp_phy::{EthRxToken, EthTxToken};


const PHY_ADDR: u8 = 0;
/// From the datasheet: *VLAN Frame maxsize = 1522*
const MTU: usize = 1522;

#[allow(dead_code)]
mod consts {
    /* For HCLK 60-100 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_42: u8 = 0;
    /* For HCLK 100-150 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_62: u8 = 1;
    /* For HCLK 20-35 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_16: u8 = 2;
    /* For HCLK 35-60 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_26: u8 = 3;
    /* For HCLK 150-168 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_102: u8 = 4;
}
use self::consts::*;

/// Ethernet driver for *STM32* chips with a *LAN8742*
/// [`Phy`](phy/struct.Phy.html) like they're found on STM Nucleo-144
/// boards.
pub struct Eth<'rx, 'tx> {
    mac_miiar: reg::ethernet_mac::Macmiiar<Srt>,
    mac_miidr: reg::ethernet_mac::Macmiidr<Srt>,
    mac_cr: reg::ethernet_mac::Maccr<Srt>,
    mac_ffr: reg::ethernet_mac::Macffr<Srt>,
    mac_fcr: reg::ethernet_mac::Macfcr<Srt>,

    dma_omr: reg::ethernet_dma::Dmaomr<Srt>,
    dma_bmr: reg::ethernet_dma::Dmabmr<Srt>,
    dma_ier: reg::ethernet_dma::Dmaier<Srt>,
    dma_sr: reg::ethernet_dma::Dmasr<Srt>,
    dma_tpdr: reg::ethernet_dma::Dmatpdr<Srt>,
    dma_rpdr: reg::ethernet_dma::Dmarpdr<Srt>,

    rx_ring: RxRing<'rx>,
    tx_ring: TxRing<'tx>,
}

impl<'rx, 'tx> Eth<'rx, 'tx> {
    /// Initialize and start tx and rx DMA engines.
    ///
    /// You must call [`setup()`](fn.setup.html) before to initialize
    /// the hardware!
    ///
    /// Make sure that the buffers reside in a memory region that is
    /// accessible by the peripheral. Core-Coupled Memory (CCM) is
    /// usually not.
    ///
    /// Other than that, initializes and starts the Ethernet hardware
    /// so that you can [`send()`](#method.send) and
    /// [`recv_next()`](#method.recv_next).
    pub fn new(
        mac_miiar: reg::ethernet_mac::Macmiiar<Srt>,
        mac_miidr: reg::ethernet_mac::Macmiidr<Srt>,
        mac_cr: reg::ethernet_mac::Maccr<Srt>,
        mac_ffr: reg::ethernet_mac::Macffr<Srt>,
        mac_fcr: reg::ethernet_mac::Macfcr<Srt>,

        dma_omr: reg::ethernet_dma::Dmaomr<Srt>,
        dma_bmr: reg::ethernet_dma::Dmabmr<Srt>,
        dma_ier: reg::ethernet_dma::Dmaier<Srt>,
        dma_sr: reg::ethernet_dma::Dmasr<Srt>,
        dma_rpdr: reg::ethernet_dma::Dmarpdr<Srt>,
        dma_tpdr: reg::ethernet_dma::Dmatpdr<Srt>,
        dma_rdlar: reg::ethernet_dma::Dmardlar<Srt>,
        dma_tdlar: reg::ethernet_dma::Dmatdlar<Srt>,

        rx_buffer: &'rx mut [RxRingEntry], 
        tx_buffer: &'tx mut [TxRingEntry]
    ) -> Self {
        let mut eth = Eth {
            mac_miiar,
            mac_miidr,
            mac_cr,
            mac_ffr,
            mac_fcr,
            dma_omr,
            dma_bmr,
            dma_ier,
            dma_sr,
            dma_rpdr,
            dma_tpdr,
            rx_ring: RxRing::new(rx_buffer),
            tx_ring: TxRing::new(tx_buffer),
        };
        eth.init();
        eth.rx_ring.start(&mut dma_rpdr, dma_rdlar, &mut dma_omr);
        eth.tx_ring.start(dma_tdlar, &mut dma_omr);
        eth
    }

    fn init(&mut self) -> &Self {
        self.reset_dma_and_wait();

        // set clock range in MAC MII address register
        let clock_range = ETH_MACMIIAR_CR_HCLK_DIV_16;
        self.mac_miiar.modify(|r| r.write_cr(clock_range.into()));

        self.get_phy()
            .reset()
            .set_autoneg();

        // Configuration Register
        self.mac_cr.modify(|r| {
            // CRC stripping for Type frames
            r.set_cstf()
                // Fast Ethernet speed
                .set_fes()
                // Duplex mode
                .set_dm()
                // Automatic pad/CRC stripping
                .set_apcs()
                // Retry disable in half-duplex mode
                .set_rd()
                // Receiver enable
                .set_re()
                // Transmitter enable
                .set_te()
        });
        // frame filter register
        self.mac_ffr.modify(|r| {
            // Receive All
            r.set_ra()
                // Promiscuous mode
                .set_pm()
        });
        // Flow Control Register
        self.mac_fcr.modify(|r| {
            // Pause time
            r.write_pt(0x100)
        });
        // operation mode register
        self.dma_omr.modify(|r| {
            // Dropping of TCP/IP checksum error frames disable
            r.set_dtcefd()
                // Receive store and forward
                .set_rsf()
                // Disable flushing of received frames
                .set_dfrf()
                // Transmit store and forward
                .set_tsf()
                // Forward error frames
                .set_fef()
                // Operate on second frame
                .set_osf()
        });
        // bus mode register
        self.dma_bmr.modify(|r| {
            // Address-aligned beats
            r.set_aab()
                // Fixed burst
                .set_fb()
                // Rx DMA PBL
                .write_rdp(32)
                // Programmable burst length
                .write_pbl(32)
                // Rx Tx priority ratio 2:1
                .write_rtpr(0b01)
                // Use separate PBL
                .set_usp()
        });

        self
    }

    /// reset DMA bus mode register
    fn reset_dma_and_wait(&self) {
        self.dma_bmr.modify(|r| r.set_sr());

        // Wait until done
        while self.dma_bmr.sr.read_bit() {}
    }

    /// Enable RX and TX interrupts
    ///
    /// In your handler you must call
    /// [`eth_interrupt_handler()`](fn.eth_interrupt_handler.html) to
    /// clear interrupt pending bits. Otherwise the interrupt will
    /// reoccur immediately.
    pub fn enable_interrupt(&self, nvic: &mut NVIC) {
        self.dma_ier.modify(|r|
                // Normal interrupt summary enable
                r.set_nise()
                    // Receive Interrupt Enable
                    .set_rie()
                    // Transmit Interrupt Enable
                    .set_tie()
        );

        // Enable ethernet interrupts
        let interrupt = Interrupt::ETH;

        nvic.enable(interrupt);
    }

    /// Calls [`eth_interrupt_handler()`](fn.eth_interrupt_handler.html)
    pub fn interrupt_handler(&self) {
        eth_interrupt_handler(&self.eth_dma);
    }

    /// Construct a PHY driver
    pub fn get_phy(&self) -> Phy {
        Phy::new(self.mac_miiar, self.mac_miidr, PHY_ADDR)
    }

    /// Obtain PHY status
    pub fn status(&self) -> PhyStatus {
        self.get_phy().status()
    }

    /// Is Rx DMA currently running?
    ///
    /// It stops if the ring is full. Call `recv_next()` to free an
    /// entry and to demand poll from the hardware.
    pub fn rx_is_running(&self) -> bool {
        self.rx_ring.running_state(&mut self.dma_sr).is_running()
    }

    /// Receive the next packet (if any is ready), or return `None`
    /// immediately.
    pub fn recv_next(&mut self) -> Result<RxPacket, RxError> {
        self.rx_ring.recv_next(&mut self.dma_rpdr, &mut self.dma_sr)
    }

    /// Is Tx DMA currently running?
    pub fn tx_is_running(&self) -> bool {
        self.tx_ring.is_running(self.dma_sr)
    }

    /// Send a packet
    pub fn send<F: FnOnce(&mut [u8]) -> R, R>(&mut self, length: usize, f: F) -> Result<R, TxError> {
        let result = self.tx_ring.send(length, f);
        self.tx_ring.demand_poll(self.dma_tpdr);
        result
    }
}

/// Call in interrupt handler to clear interrupt reason, when
/// [`enable_interrupt()`](struct.Eth.html#method.enable_interrupt).
///
/// There are two ways to call this:
///
/// * Via the [`Eth`](struct.Eth.html) driver instance that your interrupt handler has access to.
/// * By unsafely getting `Peripherals`.
///
/// TODO: could return interrupt reason
pub fn eth_interrupt_handler(dma_sr: reg::ethernet_dma::Dmasr<Srt>) {
    dma_sr.modify(|r|
        r.set_nis()
            .set_rs()
            .set_ts()
    );
}
