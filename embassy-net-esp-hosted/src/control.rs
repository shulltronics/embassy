use ch::driver::LinkState;
use embassy_net_driver_channel as ch;
use heapless::String;

use crate::ioctl::Shared;
use crate::proto::{self, CtrlMsg};

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Failed(u32),
    Timeout,
    Internal,
}

pub struct Control<'a> {
    state_ch: ch::StateRunner<'a>,
    shared: &'a Shared,
}

#[allow(unused)]
enum WifiMode {
    None = 0,
    Sta = 1,
    Ap = 2,
    ApSta = 3,
}

macro_rules! ioctl {
    ($self:ident, $req_variant:ident, $resp_variant:ident, $req:ident, $resp:ident) => {
        let mut msg = proto::CtrlMsg {
            msg_id: proto::CtrlMsgId::$req_variant as _,
            msg_type: proto::CtrlMsgType::Req as _,
            payload: Some(proto::CtrlMsgPayload::$req_variant($req)),
        };
        $self.ioctl(&mut msg).await?;
        let Some(proto::CtrlMsgPayload::$resp_variant($resp)) = msg.payload else {
            warn!("unexpected response variant");
            return Err(Error::Internal);
        };
        if $resp.resp != 0 {
            return Err(Error::Failed($resp.resp));
        }
    };
}

impl<'a> Control<'a> {
    pub(crate) fn new(state_ch: ch::StateRunner<'a>, shared: &'a Shared) -> Self {
        Self { state_ch, shared }
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        debug!("wait for init event...");
        self.shared.init_wait().await;

        debug!("set heartbeat");
        self.set_heartbeat(10).await?;

        debug!("set wifi mode");
        self.set_wifi_mode(WifiMode::Sta as _).await?;

        let mac_addr = self.get_mac_addr().await?;
        debug!("mac addr: {:02x}", mac_addr);
        self.state_ch.set_ethernet_address(mac_addr);

        Ok(())
    }

    pub async fn connect(&mut self, ssid: &str, password: &str) -> Result<(), Error> {
        let req = proto::CtrlMsgReqConnectAp {
            ssid: String::from(ssid),
            pwd: String::from(password),
            bssid: String::new(),
            listen_interval: 3,
            is_wpa3_supported: false,
        };
        ioctl!(self, ReqConnectAp, RespConnectAp, req, resp);
        self.state_ch.set_link_state(LinkState::Up);
        Ok(())
    }

    pub async fn disconnect(&mut self) -> Result<(), Error> {
        let req = proto::CtrlMsgReqGetStatus {};
        ioctl!(self, ReqDisconnectAp, RespDisconnectAp, req, resp);
        self.state_ch.set_link_state(LinkState::Down);
        Ok(())
    }

    /// duration in seconds, clamped to [10, 3600]
    async fn set_heartbeat(&mut self, duration: u32) -> Result<(), Error> {
        let req = proto::CtrlMsgReqConfigHeartbeat { enable: true, duration };
        ioctl!(self, ReqConfigHeartbeat, RespConfigHeartbeat, req, resp);
        Ok(())
    }

    async fn get_mac_addr(&mut self) -> Result<[u8; 6], Error> {
        let req = proto::CtrlMsgReqGetMacAddress {
            mode: WifiMode::Sta as _,
        };
        ioctl!(self, ReqGetMacAddress, RespGetMacAddress, req, resp);

        // WHY IS THIS A STRING? WHYYYY
        fn nibble_from_hex(b: u8) -> u8 {
            match b {
                b'0'..=b'9' => b - b'0',
                b'a'..=b'f' => b + 0xa - b'a',
                b'A'..=b'F' => b + 0xa - b'A',
                _ => panic!("invalid hex digit {}", b),
            }
        }

        let mac = resp.mac.as_bytes();
        let mut res = [0; 6];
        if mac.len() != 17 {
            warn!("unexpected MAC respnse length");
            return Err(Error::Internal);
        }
        for (i, b) in res.iter_mut().enumerate() {
            *b = (nibble_from_hex(mac[i * 3]) << 4) | nibble_from_hex(mac[i * 3 + 1])
        }
        Ok(res)
    }

    async fn set_wifi_mode(&mut self, mode: u32) -> Result<(), Error> {
        let req = proto::CtrlMsgReqSetMode { mode };
        ioctl!(self, ReqSetWifiMode, RespSetWifiMode, req, resp);

        Ok(())
    }

    async fn ioctl(&mut self, msg: &mut CtrlMsg) -> Result<(), Error> {
        debug!("ioctl req: {:?}", &msg);

        let mut buf = [0u8; 128];

        let req_len = noproto::write(msg, &mut buf).map_err(|_| {
            warn!("failed to serialize control request");
            Error::Internal
        })?;

        struct CancelOnDrop<'a>(&'a Shared);

        impl CancelOnDrop<'_> {
            fn defuse(self) {
                core::mem::forget(self);
            }
        }

        impl Drop for CancelOnDrop<'_> {
            fn drop(&mut self) {
                self.0.ioctl_cancel();
            }
        }

        let ioctl = CancelOnDrop(self.shared);

        let resp_len = ioctl.0.ioctl(&mut buf, req_len).await;

        ioctl.defuse();

        *msg = noproto::read(&buf[..resp_len]).map_err(|_| {
            warn!("failed to serialize control request");
            Error::Internal
        })?;
        debug!("ioctl resp: {:?}", msg);

        Ok(())
    }
}
