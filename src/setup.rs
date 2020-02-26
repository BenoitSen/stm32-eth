use drone_cortex_m::{reg::prelude::*};

use drone_stm32_map::{
    reg,
};

/// Initialize GPIO pins. Enable syscfg and ethernet clocks. Reset the
/// Ethernet MAC.
///
/// If supported, you should also call `setup_pins()`.
pub fn setup(
    rcc_apb2enr: &mut reg::rcc::Apb2Enr<Srt>,
    rcc_ahb1enr: &mut reg::rcc::Ahb1Enr<Srt>,
    rcc_ahb1rstr: &mut reg::rcc::Ahb1Rstr<Srt>, 
    syscfg_pmc: &mut reg::syscfg::Pmc<Srt>,
    ) {
    // enable syscfg clock
    rcc_apb2enr.modify(|r| r.set_syscfgen());

    // select MII or RMII mode
    // 0 = MII, 1 = RMII
    syscfg_pmc.modify(|r| r.set_mii_rmii_sel());

    // enable ethernet clocks
    rcc_ahb1enr.modify(|r| {
        r.set_ethmacen()
         .set_ethmactxen()
         .set_ethmacrxen()
    });

    reset_pulse(rcc_ahb1rstr);
}

fn reset_pulse(
    rcc_ahb1rstr: &mut reg::rcc::Ahb1Rstr<Srt>,
    ) {
    rcc_ahb1rstr.modify(|r| r.set_ethmacrst());
    rcc_ahb1rstr.modify(|r| r.clear_ethmacrst());
}

/// Pin setup for the **OLIMEX STM32-E407** dev board
///
/// Set RMII pins to
/// * Alternate function 11
/// * High-speed
pub fn setup_pins(
    rcc_ahb1enr: &mut reg::rcc::Ahb1Enr<Srt>,

    gpioa_ospeedr: &mut reg::gpioa::Ospeedr<Srt>,
    gpioa_moder: &mut reg::gpioa::Moder<Srt>,
    gpioa_afrl: &mut reg::gpioa::Afrl<Srt>,

    gpiob_ospeedr: &mut reg::gpiob::Ospeedr<Srt>,
    gpiob_moder: &mut reg::gpiob::Moder<Srt>,
    gpiob_afrh: &mut reg::gpiob::Afrh<Srt>,

    gpioc_ospeedr: &mut reg::gpioc::Ospeedr<Srt>,
    gpioc_moder: &mut reg::gpioc::Moder<Srt>,
    gpioc_afrl: &mut reg::gpioc::Afrl<Srt>,

    gpiog_ospeedr: &mut reg::gpiog::Ospeedr<Srt>,
    gpiog_moder: &mut reg::gpiog::Moder<Srt>,
    gpiog_afrh: &mut reg::gpiog::Afrh<Srt>,
) {
    // GPIOA 1 -> ETH_RMII_REF_CLK
    // GPIOA 2 -> ETH_MDIO
    // GPIOA 7 -> ETH_RMII_CRS_DV
    // GPIOB 13 -> ETH _RMII_TXD1
    // GPIOC 1 -> ETH_MDC
    // GPIOC 4 -> ETH_RMII_RXD0
    // GPIOC 5 -> ETH_RMII_RXD1
    // GPIOG 11 -> ETH _RMII_TX_EN
    // GPIOG 13 -> ETH _RMII_TXD0

    rcc_ahb1enr.modify(|r| {
        r.set_gpioaen()
            .set_gpioben()
            .set_gpiocen()
            .set_gpiogen()
    });

    // GPIOA 1 - 2 - 7
    // speed to Very high
    // AF 11
    gpioa_ospeedr.modify(|r| {
        r.write_ospeedr1(0b11)
            .write_ospeedr2(0b11)
            .write_ospeedr7(0b11)
    });
    gpioa_moder.modify(|r| {
        r.write_moder1(0b10)
            .write_moder2(0b10)
            .write_moder7(0b10)
    });  
    gpioa_afrl.modify(|r| {
        r.write_afrl1(11)
            .write_afrl2(11)
            .write_afrl7(11)
    });

    // GPIOB 13
    // speed to Very high
    // AF 11
    gpiob_ospeedr.modify(|r| {
        r.write_ospeedr13(0b11)
    });
    gpiob_moder.modify(|r| {
        r.write_moder13(0b10)
    });  
    gpiob_afrh.modify(|r| {
        r.write_afrh13(11)
    });

    // GPIOC 1 - 4 - 5
    // speed to Very high
    // AF 11
    gpioc_ospeedr.modify(|r| {
        r.write_ospeedr1(0b11)
            .write_ospeedr4(0b11)
            .write_ospeedr5(0b11)
    });
    gpioc_moder.modify(|r| {
        r.write_moder1(0b10)
            .write_moder4(0b10)
            .write_moder5(0b10)
    });  
    gpioc_afrl.modify(|r| {
        r.write_afrl1(11)
            .write_afrl4(11)
            .write_afrl5(11)
    });

    // GPIOG 11 - 13
    // speed to Very high
    // AF 11
    gpiog_ospeedr.modify(|r| {
        r.write_ospeedr11(0b11)
            .write_ospeedr13(0b11)
    });
    gpiog_moder.modify(|r| {
        r.write_moder11(0b10)
            .write_moder13(0b10)
    });  
    gpiog_afrh.modify(|r| {
        r.write_afrh11(11)
            .write_afrh13(11)
    });
}
