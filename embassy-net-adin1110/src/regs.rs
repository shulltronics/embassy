use core::ops::{BitAnd, BitOr, BitOrAssign};

#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
/// SPI REGISTER DETAILS
/// Table 38.
pub enum SpiRegisters {
    IDVER = 0x00,
    PHYID = 0x01,
    CAPABILITY = 0x02,
    RESET = 0x03,
    CONFIG0 = 0x04,
    CONFIG2 = 0x06,
    STATUS0 = 0x08,
    STATUS1 = 0x09,
    IMASK0 = 0x0C,
    IMASK1 = 0x0D,
    MDIO_ACC = 0x20,
    TX_FSIZE = 0x30,
    TX = 0x31,
    TX_SPACE = 0x32,
    FIFO_CLR = 0x36,
    ADDR_FILT_UPR0 = 0x50,
    ADDR_FILT_LWR0 = 0x51,
    ADDR_FILT_UPR1 = 0x52,
    ADDR_FILT_LWR1 = 0x53,
    ADDR_MSK_LWR0 = 0x70,
    ADDR_MSK_UPR0 = 0x71,
    ADDR_MSK_LWR1 = 0x72,
    ADDR_MSK_UPR1 = 0x73,
    RX_FSIZE = 0x90,
    RX = 0x91,
}

impl SpiRegisters {
    pub fn from(&self) -> u16 {
        *self as u16
    }
}

/// Status0 Register bits
#[allow(clippy::upper_case_acronyms, dead_code)]
#[repr(u32)]
pub enum STATUS0 {
    CDPE = 12,
    TXFCSE = 11,
    TTSCAC = 10,
    TTSCAB = 9,
    TTSCAA = 8,
    PHYINT = 7,
    RESETC = 6,
    HDRE = 5,
    LOFE = 4,
    RXBOE = 3,
    TXBUE = 2,
    TXBOE = 1,
    TXPE = 0,
}

/// Status1 Register bits
#[allow(non_camel_case_types, dead_code)]
#[repr(u32)]
pub enum STATUS1 {
    TX_ECC_ERR = 12,
    RX_ECC_ERR = 11,
    SPI_ERR = 10,
    P1_RX_IFG_ERR = 8,
    P1_RX_RDY_HI = 5,
    P1_RX_RDY = 4,
    TX_RDY = 3,
    LINK_CHANGE = 1,
    P1_LINK_STATUS = 0,
}

impl BitAnd<STATUS1> for u32 {
    type Output = u32;

    fn bitand(self, rhs: STATUS1) -> Self::Output {
        self & (1 << rhs as u32)
    }
}

impl BitOr<STATUS1> for u32 {
    type Output = u32;

    fn bitor(self, rhs: STATUS1) -> Self::Output {
        self | (1 << rhs as u32)
    }
}

impl BitAnd<STATUS0> for u32 {
    type Output = u32;

    fn bitand(self, rhs: STATUS0) -> Self::Output {
        self & (1 << rhs as u32)
    }
}

impl BitOrAssign<STATUS0> for u32 {
    fn bitor_assign(&mut self, rhs: STATUS0) {
        *self |= 1 << rhs as u32;
    }
}

impl BitOrAssign<STATUS1> for u32 {
    fn bitor_assign(&mut self, rhs: STATUS1) {
        *self |= 1 << rhs as u32;
    }
}

/// Config0 Register bits
#[allow(clippy::upper_case_acronyms, dead_code)]
#[repr(u32)]
pub enum CONFIG0 {
    SYNC = 15,
    TXFCSVE = 14,
    CSARFE = 13,
    ZARFE = 12,
    TCXTHRESH1 = 11,
    TCXTHRESH0 = 10,
    TXCTE = 9,
    RXCTE = 8,
    FTSE = 7,
    FTSS = 6,
    PROTE = 5,
    SEQE = 4,
    CPS2 = 2,
    CPS1 = 1,
    CPS0 = 0,
}

impl BitOrAssign<CONFIG0> for u32 {
    fn bitor_assign(&mut self, rhs: CONFIG0) {
        *self |= 1 << rhs as u32;
    }
}

impl BitOr<CONFIG0> for u32 {
    type Output = u32;

    fn bitor(self, rhs: CONFIG0) -> Self::Output {
        self | (1 << (rhs as u32))
    }
}

/// Config2 Register bits
#[allow(clippy::upper_case_acronyms, dead_code, non_camel_case_types)]
#[repr(u32)]
pub enum CONFIG2 {
    TX_RDY_ON_EMPTY = 8,
    SDF_DETECT_SRC = 7,
    STATS_CLR_ON_RD = 6,
    CRC_APPEND = 5,
    P1_RCV_IFG_ERR_FRM = 4,
    P1_FWD_UNK2HOST = 2,
    MSPEED = 0,
}

impl BitOrAssign<CONFIG2> for u32 {
    fn bitor_assign(&mut self, rhs: CONFIG2) {
        *self |= 1 << rhs as u32;
    }
}

impl BitOr<CONFIG2> for u32 {
    type Output = u32;

    fn bitor(self, rhs: CONFIG2) -> Self::Output {
        self | (1 << (rhs as u32))
    }
}

/// IMASK0 Register bits
#[allow(clippy::upper_case_acronyms, dead_code, non_camel_case_types)]
#[repr(u32)]
pub enum IMASK0 {
    CPPEM = 12,
    TXFCSEM = 11,
    TTSCACM = 10,
    TTSCABM = 9,
    TTSCAAM = 8,
    PHYINTM = 7,
    RESETCM = 6,
    HDREM = 5,
    LOFEM = 4,
    RXBOEM = 3,
    TXBUEM = 2,
    TXBOEM = 1,
    TXPEM = 0,
    RESET_VALUE = 0x0000_1FBF,
}

impl BitOrAssign<IMASK0> for u32 {
    fn bitor_assign(&mut self, rhs: IMASK0) {
        *self |= 1 << rhs as u32;
    }
}

impl BitOr<IMASK0> for u32 {
    type Output = u32;

    fn bitor(self, rhs: IMASK0) -> Self::Output {
        self | (1 << (rhs as u32))
    }
}

/// IMASK1 Register bits
#[allow(clippy::upper_case_acronyms, dead_code, non_camel_case_types)]
#[repr(u32)]
pub enum IMASK1 {
    TX_ECC_ERR_MASK = 12,
    RX_ECC_ERR_MASK = 11,
    SPI_ERR_MASK = 10,
    P1_RX_IFG_ERR_MASK = 8,
    P1_RX_RDY_MASK = 4,
    TX_RDY_MASK = 3,
    LINK_CHANGE_MASK = 1,
    RESET_VALUE = 0x43FA_1F1A,
}

impl BitOrAssign<IMASK1> for u32 {
    fn bitor_assign(&mut self, rhs: IMASK1) {
        *self |= 1 << rhs as u32;
    }
}

impl BitOr<IMASK1> for u32 {
    type Output = u32;

    fn bitor(self, rhs: IMASK1) -> Self::Output {
        self | (1 << (rhs as u32))
    }
}
