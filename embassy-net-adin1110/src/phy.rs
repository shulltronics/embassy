use crate::mdio::MdioBus;

#[allow(dead_code, non_camel_case_types, clippy::upper_case_acronyms)]
#[repr(u8)]
/// Clause 22 Registers
pub enum RegsC22 {
    CONTROL = 0x00,
    STATUS = 0x01,
    PHY_ID1 = 0x02,
    PHY_ID2 = 0x03,
}

#[allow(non_snake_case, dead_code)]
pub mod RegsC45 {
    #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
    #[repr(u16)]
    pub enum DA1 {
        PMA_PMD_CNTRL1 = 0x0000,
        PMA_PMD_STAT1 = 0x0001,
        MSE_VAL = 0x830B,
    }

    impl DA1 {
        pub fn into(self) -> (u8, u16) {
            (0x01, self as u16)
        }
    }

    #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
    #[repr(u16)]
    pub enum DA3 {
        PCS_CNTRL1 = 0x0000,
        PCS_STAT1 = 0x0001,
        PCS_STAT2 = 0x0008,
    }

    impl DA3 {
        pub fn into(self) -> (u8, u16) {
            (0x03, self as u16)
        }
    }

    #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
    #[repr(u16)]
    pub enum DA1E {
        CRSM_IRQ_STATUS = 0x0010,
        CRSM_IRQ_MASK = 0x0020,
        DIGIO_PINMUX = 0x8c56,
        LED_CNTRL = 0x8C82,
        LED_POLARITY = 0x8C83,
    }

    impl DA1E {
        pub fn into(self) -> (u8, u16) {
            (0x1e, self as u16)
        }
    }

    #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
    #[repr(u16)]
    pub enum DA1F {
        PHY_SYBSYS_IRQ_STATUS = 0x0011,
        PHY_SYBSYS_IRQ_MASK = 0x0021,
    }

    impl DA1F {
        pub fn into(self) -> (u8, u16) {
            (0x1f, self as u16)
        }
    }
}

pub struct Phy10BaseT1x(u8);

impl Default for Phy10BaseT1x {
    fn default() -> Self {
        Self(0x01)
    }
}

impl Phy10BaseT1x {
    /// Get the both parts of the PHYID.
    pub async fn get_id<MDIOBUS, MDE>(&self, mdiobus: &mut MDIOBUS) -> Result<u32, MDE>
    where
        MDIOBUS: MdioBus<Error = MDE>,
        MDE: core::fmt::Debug,
    {
        let mut phyid = (mdiobus.read_cl22(self.0, RegsC22::PHY_ID1 as u8).await? as u32) << 16;
        phyid |= mdiobus.read_cl22(self.0, RegsC22::PHY_ID2 as u8).await? as u32;
        Ok(phyid)
    }

    /// Get the Mean Squared Error Value.
    pub async fn get_sqi<MDIOBUS, MDE>(&self, mdiobus: &mut MDIOBUS) -> Result<u16, MDE>
    where
        MDIOBUS: MdioBus<Error = MDE>,
        MDE: core::fmt::Debug,
    {
        mdiobus.read_cl45(self.0, RegsC45::DA1::MSE_VAL.into()).await
    }
}
