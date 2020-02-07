use drone_cortex_m::{reg::prelude::*};

use drone_stm32_map::{
    periph::{
        gpio::{GpioPortPeriph, GpioA, GpioB, GpioC, GpioG},
    },
    reg,
};

/// Initialize GPIO pins. Enable syscfg and ethernet clocks. Reset the
/// Ethernet MAC.
///
/// If supported, you should also call `setup_pins()`.
pub fn setup(
    rcc_apb2enr: reg::rcc::Apb2Enr<Srt>,
    rcc_ahb1enr: reg::rcc::Ahb1Enr<Srt>,
    rcc_ahb1rstr: reg::rcc::Ahb1Rstr<Srt>, 
    syscfg_pmc: reg::syscfg::Pmc<Srt>,
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
    rcc_ahb1rstr: reg::rcc::Ahb1Rstr<Srt>,
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
    gpio_a: &mut GpioPortPeriph<GpioA>,
    gpio_b: &mut GpioPortPeriph<GpioB>,
    gpio_c: &mut GpioPortPeriph<GpioC>,
    gpio_g: &mut GpioPortPeriph<GpioG>
) {
    // GPIOA 1 - 2 - 7
    // speed to Very high
    // AF 11
    gpio_a.gpio_ospeedr.modify(|r| {
        r.write_ospeedr1(0b11)
            .write_ospeedr2(0b11)
            .write_ospeedr7(0b11)
    });
    gpio_a.gpio_moder.modify(|r| {
        r.write_moder1(0b10)
            .write_moder2(0b10)
            .write_moder7(0b10)
    });  
    gpio_a.gpio_afrl.modify(|r| {
        r.write_afrl1(11)
            .write_afrl2(11)
            .write_afrl7(11)
    });

    // GPIOB 13
    // speed to Very high
    // AF 11
    gpio_b.gpio_ospeedr.modify(|r| {
        r.write_ospeedr13(0b11)
    });
    gpio_b.gpio_moder.modify(|r| {
        r.write_moder13(0b10)
    });  
    gpio_b.gpio_afrh.modify(|r| {
        r.write_afrh13(11)
    });

    // GPIOC 1 - 4 - 5
    // speed to Very high
    // AF 11
    gpio_c.gpio_ospeedr.modify(|r| {
        r.write_ospeedr1(0b11)
            .write_ospeedr4(0b11)
            .write_ospeedr5(0b11)
    });
    gpio_c.gpio_moder.modify(|r| {
        r.write_moder1(0b10)
            .write_moder4(0b10)
            .write_moder5(0b10)
    });  
    gpio_c.gpio_afrl.modify(|r| {
        r.write_afrl1(11)
            .write_afrl4(11)
            .write_afrl5(11)
    });

    // GPIOG 11 - 13
    // speed to Very high
    // AF 11
    gpio_g.gpio_ospeedr.modify(|r| {
        r.write_ospeedr11(0b11)
            .write_ospeedr13(0b11)
    });
    gpio_g.gpio_moder.modify(|r| {
        r.write_moder11(0b10)
            .write_moder13(0b10)
    });  
    gpio_g.gpio_afrh.modify(|r| {
        r.write_afrh11(11)
            .write_afrh13(11)
    });
}
