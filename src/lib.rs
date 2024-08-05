//! A platform agnostic driver to interface with the TMC5160 (Trinamic integrated stepper motor controller)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/0.2
//!
#![no_std]
#![allow(dead_code)]
#![deny(missing_docs)]
#![deny(warnings)]

use core::fmt;
use core::result::Result;


use embassy_time::Duration;
use embedded_hal_async::spi::{SpiDevice, Mode, Phase, Polarity};
use fixed::types::{I8F8, I48F16};
use crate::registers::*;

pub mod registers;

fn swap_bytes(input: [u8; 4]) -> [u8; 4] {
    let mut output = [0; 4];
    for i in 0..4 {
        output[4 - 1 - i] = input[i];
    }
    output
}

/// SPI mode
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

#[derive(Debug)]
/// Error type for the TMC5160
pub enum Error<E> {
    /// SPI bus error
    Spi(E),
    /// Pin error
    PinError,
}

/// Data Exchange packet
pub struct DataPacket {
    /// Status returned from last communication
    pub status: SpiStatus,
    /// Data received from TMC5160
    pub data: u32,
    /// debug
    pub debug: [u8; 5],
}

impl fmt::Display for DataPacket {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "0x{:x}:0x{:x}", self.status.into_bytes()[0], self.data)
    }
}


/// TMC5160 driver
pub struct Tmc5160<SPIDEV> {
    spi: SPIDEV,
    /// the max velocity that is set
    pub v_max: f32,
    /// status register of the driver
    pub status: SpiStatus,
    /// debug info of the last transmission
    pub debug: [u8; 5],
    _clock: f32,
    /// number of steps per revolution
    pub _step_count: f32,
    _en_inverted: bool,
    /// value of the GCONF register
    pub g_conf: GConf,
    /// value of the NODECONF register
    pub node_conf: NodeConf,
    /// value of the OTPPROG register
    pub otp_prog: OtpProg,
    /// value of the SHORT_CONF register
    pub short_conf: ShortConf,
    /// value of the DRV_CONF register
    pub drv_conf: DrvConf,
    /// value of the IHOLD_IRUN register
    pub ihold_irun: IHoldIRun,
    /// value of the SWMODE register
    pub sw_mode: SwMode,
    /// value of the ENCMODE register
    pub enc_mode: EncMode,
    /// value of the MSLUTSEL register
    pub ms_lut_sel: MsLutSel,
    /// value of the CHOPCONF register
    pub chop_conf: ChopConf,
    /// value of the COOLCONF register
    pub cool_conf: CoolConf,
    /// value of the PWMCONF register
    pub pwm_conf: PwmConf,
}

impl <SPIDEV> fmt::Debug for Tmc5160<SPIDEV> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "TMC5160")
    }
}

impl<SPIDEV, E> Tmc5160<SPIDEV>
    where
        SPIDEV: SpiDevice<Error = E>,
{
    /// Create a new driver from a SPI peripheral and a NCS pin
    pub fn new(spi: SPIDEV) -> Self {
        Tmc5160 {
            spi,
            v_max: 0.0,
            status: SpiStatus::new(),
            debug: [0; 5],
            _clock: 12000000.0,
            _step_count: 256.0,
            _en_inverted: false,
            g_conf: GConf::new(),
            node_conf: NodeConf::new(),
            otp_prog: OtpProg::new(),
            short_conf: ShortConf::new(),
            drv_conf: DrvConf::new(),
            ihold_irun: IHoldIRun::new(),
            sw_mode: SwMode::new(),
            enc_mode: EncMode::new(),
            ms_lut_sel: MsLutSel::new(),
            chop_conf: ChopConf::new(),
            cool_conf: CoolConf::new(),
            pwm_conf: PwmConf::new(),
        }
    }

    /// specify clock speed of the Tmc5160 (Default is 12 MHz)
    pub fn clock(mut self, clock: f32) -> Self {
        self._clock = clock;
        self
    }

    /// specify step count of the motor (Default is 256)
    pub fn step_count(mut self, step_count: f32) -> Self {
        self._step_count = step_count;
        self
    }

    fn speed_from_hz(&mut self, speed_hz: f32) -> u32 {
        (speed_hz / (self._clock / 16_777_216.0) * self._step_count) as u32
    }

    fn speed_to_hz(&mut self, speed: u32) -> f32 {
        (speed as f32 / self._step_count) * (self._clock / 16_777_216.0)
    }

    fn accel_from_hz(&mut self, accel_hz_per_s: f32) -> u32 {
        (accel_hz_per_s / (self._clock * self._clock)
            * (512.0 * 256.0)
            * 16_777_216.0
            * self._step_count) as u32
    }

    fn accel_to_hz(&mut self, accel: u32) -> f32 {
        (accel as f32 / self._step_count) * (self._clock * self._clock)
            / (512.0 * 256.0)
            / 16_777_216.0
    }


    /// Convert revolutions to microsteps
    pub fn convert_revs_to_microsteps(&self, revs: I48F16) -> Option<i32> {
        let revs: f64 = revs.to_num();
        let microsteps = revs * self._step_count as f64;
        if microsteps as i64 > i32::MAX as i64 {
            None
        } else {
            Some(microsteps as i32)
        }
    }

    /// Convert revolutions to microsteps
    pub fn convert_microsteps_to_revs(&self, steps: i32) -> I48F16 {
        let esteps_per_rev = I48F16::from_num(self._step_count);
        I48F16::from_num(steps) / esteps_per_rev
    }

    /// read a specified register
    pub async fn read_register<T>(&mut self, reg: T) -> Result<DataPacket, Error<E>>
        where
            T: Address + Copy,
    {
        // Process cmd to read, return previous (dummy) state
        let _dummy = self.read_io(reg).await?;
        // Repeat cmd to read, return state
        self.read_io(reg).await
    }

    
    async fn read_io<T>(&mut self, reg: T) -> Result<DataPacket, Error<E>>
        where
            T: Address + Copy,
    {

        let mut buffer = [reg.addr(), 0, 0, 0, 0];

        let mut response: [u8; 5] = [0; 5];
        
        self.spi.transfer(&mut response, &mut buffer).await.map_err(Error::Spi)?;


        let mut ret_val: [u8; 4] = [0; 4];

        for i in 0..4 {
            ret_val[i] = response[i + 1];
        }

        let mut debug_val: [u8; 5] = [0; 5];

        for i in 0..5 {
            debug_val[i] = response[i];
        }

        Ok(DataPacket { status: SpiStatus::from_bytes([response[0]]), data: u32::from_be_bytes(ret_val), debug: debug_val })
    }

    /// write value to a specified register
    pub async fn write_register<T>(&mut self, reg: T, val: &mut [u8; 4]) -> Result<DataPacket, Error<E>>
        where
            T: Address + Copy,
    {

        let mut buffer = [reg.addr() | 0x80, val[0], val[1], val[2], val[3]];

        let debug_val = buffer.clone();

        let mut response: [u8; 5] = [0; 5];
        
        self.spi.transfer(&mut response, &mut buffer).await.map_err(Error::Spi)?;

        let mut ret_val: [u8; 4] = [0; 4];

        for i in 0..4 {
            ret_val[i] = response[i + 1];
        }

        Ok(DataPacket { status: SpiStatus::from_bytes([response[0]]), data: u32::from_be_bytes(ret_val), debug: debug_val })
    }

    /// clear G_STAT register
    pub async fn clear_g_stat(&mut self) -> Result<DataPacket, Error<E>> {
        let mut value = 0b111_u32.to_be_bytes();
        //let mut value= (!0_u32).to_be_bytes();
        self.write_register(Registers::GSTAT, &mut value).await
    }

    /// clear ENC_STATUS register
    pub async fn clear_enc_status(&mut self) -> Result<DataPacket, Error<E>> {
        let mut value = 0b111_u32.to_be_bytes();
        //let mut value= (!0_u32).to_be_bytes();
        self.write_register(Registers::ENC_STATUS, &mut value).await
    }

    /// write value to SW_MODE register
    pub async fn update_sw_mode(&mut self) -> Result<DataPacket, Error<E>> {
        let mut value = swap_bytes(self.sw_mode.into_bytes());
        self.write_register(Registers::SW_MODE, &mut value).await
    }

    /// write value to G_CONF register
    pub async fn update_g_conf(&mut self) -> Result<DataPacket, Error<E>> {
        let mut value = swap_bytes(self.g_conf.into_bytes());
        self.write_register(Registers::GCONF, &mut value).await
    }

    /// write value to CHOP_CONF register
    pub async fn update_chop_conf(&mut self) -> Result<DataPacket, Error<E>> {
        let mut value = swap_bytes(self.chop_conf.into_bytes());
        self.write_register(Registers::CHOPCONF, &mut value).await
    }

    /// write value to COOL_CONF register
    pub async fn update_cool_conf(&mut self) -> Result<DataPacket, Error<E>> {
        let mut value = swap_bytes(self.cool_conf.into_bytes());
        self.write_register(Registers::COOLCONF, &mut value).await
    }

    /// write value to IHOLD_IRUN register
    pub async fn update_ihold_irun(&mut self) -> Result<DataPacket, Error<E>> {
        let mut value = swap_bytes(self.ihold_irun.into_bytes());
        self.write_register(Registers::IHOLD_IRUN, &mut value).await
    }

    /// write value to PWM_CONF register
    pub async fn update_pwm_conf(&mut self) -> Result<DataPacket, Error<E>> {
        let mut value = swap_bytes(self.pwm_conf.into_bytes());
        self.write_register(Registers::PWMCONF, &mut value).await
    }

    /// write value to ENC_MODE register
    pub async fn update_enc_mode(&mut self) -> Result<DataPacket, Error<E>> {
        let mut value = swap_bytes(self.enc_mode.into_bytes());
        self.write_register(Registers::ENCMODE, &mut value).await
    }

    /// write value to GLOBALSCALER register
    pub async fn set_global_scaler(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::GLOBALSCALER, &mut value).await
    }

    /// write value to TPOWERDOWN register
    pub async fn set_tpowerdown(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::TPOWERDOWN, &mut value).await
    }

    /// write value to TPWMTHRS register
    pub async fn set_tpwmthrs(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::TPWMTHRS, &mut value).await
    }

    /// write value to TCOOLTHRS register
    pub async fn set_tcoolthrs(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::TCOOLTHRS, &mut value).await
    }

    /// write value to A1 register
    pub async fn set_a1(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::A1, &mut value).await
    }

    /// write value to V1 register
    pub async fn set_v1(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::V1, &mut value).await
    }

    /// write value to AMAX register
    pub async fn set_amax(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::AMAX, &mut value).await
    }

    /// write value to VMAX register
    pub async fn set_vmax(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::VMAX, &mut value).await
    }

    /// write value to DMAX register
    pub async fn set_dmax(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::DMAX, &mut value).await
    }

    /// write value to D1 register
    pub async fn set_d1(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::D1, &mut value).await
    }

    /// write value to VSTART register
    pub async fn set_vstart(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::VSTART, &mut value).await
    }

    /// write value to VSTOP register
    pub async fn set_vstop(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::VSTOP, &mut value).await
    }

    /// write value to PWM_AUTO register
    pub async fn set_pwm_auto(&mut self, val: u32) -> Result<DataPacket, Error<E>> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::PWM_AUTO, &mut value).await
    }

    /// write value to RAMPMODE register
    pub async fn set_rampmode(&mut self, val: RampMode) -> Result<DataPacket, Error<E>> {
        let mut value = (val as u32).to_be_bytes();
        self.write_register(Registers::RAMPMODE, &mut value).await
    }

    /// read offset register
    pub async fn read_offset(&mut self) -> Result<u32, Error<E>> {
        self.read_register(Registers::OFFSET_READ).await.map(|packet| packet.data)
    }

    /// read TSTEP register
    pub async fn read_tstep(&mut self) -> Result<u32, Error<E>> {
        self.read_register(Registers::TSTEP).await.map(|packet| packet.data)
    }

    /// read DRV_STATUS register
    pub async fn read_drv_status(&mut self) -> Result<DrvStatus, Error<E>> {
        let packet = self.read_register(Registers::DRV_STATUS).await?;
        self.status = packet.status;
        Ok(DrvStatus::from_bytes(packet.data.to_le_bytes()))
    }

    /// read GSTAT register
    pub async fn read_gstat(&mut self) -> Result<GStat, Error<E>> {
        let packet = self.read_register(Registers::GSTAT).await?;
        self.status = packet.status;
        self.debug = packet.debug;
        Ok(GStat::from_bytes(packet.data.to_le_bytes()))
    }

    /// read GCONF register
    pub async fn read_gconf(&mut self) -> Result<GConf, Error<E>> {
        let packet = self.read_register(Registers::GCONF).await?;
        self.status = packet.status;
        Ok(GConf::from_bytes(packet.data.to_le_bytes()))
    }

    /// read RAMP_STAT register
    pub async fn read_ramp_status(&mut self) -> Result<RampStat, Error<E>> {
        let packet = self.read_register(Registers::RAMP_STAT).await?;
        self.status = packet.status;
        Ok(RampStat::from_bytes(packet.data.to_le_bytes()))
    }

    /// read ENC_STATUS register
    pub async fn read_enc_status(&mut self) -> Result<EncStatus, Error<E>> {
        let packet = self.read_register(Registers::ENC_STATUS).await?;
        self.status = packet.status;
        Ok(EncStatus::from_bytes(packet.data.to_le_bytes()))
    }

    /// read encoder position
    pub async fn read_enc_pos(&mut self) -> Result<i32, Error<E>> {
        self.read_register(Registers::X_ENC).await.map(|packet| packet.data as i32)
    }

    /// read encoder position
    pub async fn home_enc(&mut self) -> Result<DataPacket, Error<E>> {
        let mut val = 0_u32.to_be_bytes();
        let packet = self.write_register(Registers::X_ENC, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// set the position to 0 / home
    pub async fn set_home(&mut self) -> Result<DataPacket, Error<E>> {
        let mut val = 0_u32.to_be_bytes();
        self.write_register(Registers::XACTUAL, &mut val).await?;
        let packet = self.write_register(Registers::XTARGET, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// stop the motor now
    pub async fn stop(&mut self) -> Result<DataPacket, Error<E>> {
        //self.disable()?;
        let mut val = 0_u32.to_be_bytes();
        self.write_register(Registers::VSTART, &mut val).await?;
        self.write_register(Registers::VMAX, &mut val).await?;
        // TODO: check how we can restart the movement afterwards
        let mut position = self.get_position().await?.to_be_bytes();
        let packet = self.write_register(Registers::XTARGET, &mut position).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// pauses the motion by writing 0 to VMAX
    pub async fn pause(&mut self) -> Result<DataPacket, Error<E>> {
        let mut val = 0_u32.to_be_bytes();
        let packet = self.write_register(Registers::VMAX, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// check if the motor is moving
    pub async fn is_moving(&mut self) -> Result<bool, Error<E>> {
        self.read_drv_status().await.map(|packet| !packet.standstill())
    }

    /// check if the motor has reached the target position
    pub async fn position_is_reached(&mut self) -> Result<bool, Error<E>> {
        self.read_ramp_status().await.map(|packet| packet.position_reached())
    }

    /// check if the motor has reached the constant velocity
    pub async fn velocity_is_reached(&mut self) -> Result<bool, Error<E>> {
        self.read_ramp_status().await.map(|packet| packet.velocity_reached())
    }

    /// check if motor is at right limit
    pub async fn is_at_limit_r(&mut self) -> Result<bool, Error<E>> {
        self.read_ramp_status().await.map(|packet| packet.status_stop_r())
    }

    /// check if motor is at left limit
    pub async fn is_at_limit_l(&mut self) -> Result<bool, Error<E>> {
        self.read_ramp_status().await.map(|packet| packet.status_stop_l())
    }

    /// set the max velocity (VMAX)
    pub async fn set_velocity(&mut self, velocity: f32) -> Result<DataPacket, Error<E>> {
        self.v_max = velocity;
        let v_max = self.speed_from_hz(velocity);
        let mut val = v_max.to_be_bytes();
        let packet = self.write_register(Registers::VMAX, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// set the max velocity (VMAX)
    pub async fn set_velocity_raw(&mut self, velocity: u32) -> Result<DataPacket, Error<E>> {
        self.v_max = velocity as f32 / self._step_count * (self._clock / 16_777_216.0);
        let mut val = velocity.to_be_bytes();
        let packet = self.write_register(Registers::VMAX, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// set the max acceleration (AMAX, DMAX, A1, D1)
    pub async fn set_acceleration(&mut self, acceleration: f32) -> Result<DataPacket, Error<E>> {
        let a_max = self.accel_from_hz(acceleration);
        let mut val = a_max.to_be_bytes();
        self.write_register(Registers::AMAX, &mut val).await?;
        self.write_register(Registers::DMAX, &mut val).await?;
        self.write_register(Registers::A1, &mut val).await?;
        let packet = self.write_register(Registers::D1, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// move to a specific location
    pub async fn move_to(&mut self, target: f32) -> Result<DataPacket, Error<E>> {
        //self.enable()?;
        let target = (target * self._step_count) as i32;
        let mut val = target.to_be_bytes();
        let packet = self.write_register(Registers::XTARGET, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// move to a specific location by microsteps
    pub async fn move_to_steps(&mut self, target: i32) -> Result<DataPacket, Error<E>> {
        //self.enable()?;
        let target = (target) as i32;
        let mut val = target.to_be_bytes();
        let packet = self.write_register(Registers::XTARGET, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// get the latched position
    pub async fn get_latched_position(&mut self) -> Result<f32, Error<E>> {
        self.read_register(Registers::XLATCH).await
        .map(|val| (val.data as i32) as f32 / self._step_count)
    }

    /// get the current position
    pub async fn get_position(&mut self) -> Result<f32, Error<E>> {
        self.read_register(Registers::XACTUAL).await
        .map(|val| (val.data as i32) as f32 / self._step_count)
    }

    /// get encoder and motor position as a tuple
    /// This function tries to do things in an optimized way, 
    /// blocking the thread so that both values are read at nearly the same time
    pub async fn get_position_enc(&mut self) -> Result<(i32, i32), Error<E>> {
        let mut mot_buf = [Registers::XACTUAL.addr(), 0, 0, 0, 0];
        let mut enc_buf = [Registers::X_ENC.addr(), 0, 0, 0, 0];

        let mut motor_resp: [u8; 5] = [0; 5];
        let mut enc_resp: [u8; 5] = [0; 5];
        
        embassy_futures::block_on(async {
            self.spi.transfer(&mut motor_resp, &mut mot_buf).await.map_err(Error::Spi)?;
            self.spi.transfer(&mut enc_resp, &mut enc_buf).await.map_err(Error::Spi)
        })?;

        let mut motor_val: [u8; 4] = [0; 4];
        let mut enc_val: [u8; 4] = [0; 4];
        for i in 0..4 {
            motor_val[i] = motor_resp[i + 1];
            enc_val[i] = enc_resp[i + 1];
        }
        
        let motor =  u32::from_be_bytes(motor_val);
        let encoder =  u32::from_be_bytes(enc_val);
        Ok(((motor as i32), (encoder as i32)))
    }

    /// get the current position in microsteps
    pub async fn get_position_steps(&mut self) -> Result<i32, Error<E>> {
        self.read_register(Registers::XACTUAL).await
        .map(|val| val.data as i32)
    }

    /// set the current position
    pub async fn set_position(&mut self, target_signed: i32) -> Result<DataPacket, Error<E>> {
        let target = target_signed;
        let mut val = (target * self._step_count as i32).to_be_bytes();
        self.write_register(Registers::XACTUAL, &mut val).await
    }

    /// get the current velocity
    pub async fn get_velocity(&mut self) -> Result<f32, Error<E>> {
        self.read_register(Registers::VACTUAL).await.map(|target| {
            if (target.data & 0b100000000000000000000000) == 0b100000000000000000000000 {
                ((16777216 - target.data as i32) as f64 / self._step_count as f64) as f32
            } else {
                ((target.data as i32) as f64 / self._step_count as f64) as f32
            }
        })
    }

    /// get the set maximum velocity (VMAX)
    pub fn get_velocity_max(&mut self) -> f32 {
        self.v_max
    }

    /// get the current target position (XTARGET)
    pub async fn get_target(&mut self) -> Result<f32, Error<E>> {
        self.read_register(Registers::XTARGET).await
        .map(|packet| packet.data as f32 / self._step_count)
    }

    /// get the current target position (XTARGET) in microsteps
    pub async fn get_target_steps(&mut self) -> Result<i32, Error<E>> {
        self.read_register(Registers::XTARGET).await
        .map(|packet| packet.data as i32)
    }
    /// relative positional movement
    pub async fn move_to_relative(&mut self, speed: f32, microsteps: i32) -> Result<(), Error<E>>  {
        self.set_rampmode(RampMode::PositioningMode).await?;
        self.set_velocity(speed).await?;
        self.set_home().await?;
        self.home_enc().await?;
        self.move_to_steps(microsteps).await?;
        Ok(())
    }
    /// directional velocity movement
    pub async fn move_velocity(&mut self, velocity: I8F8 ) -> Result<(), Error<E>>  {
        if velocity < 0 {
            self.set_rampmode(RampMode::VelocityModeNeg).await?;
        } else {
            self.set_rampmode(RampMode::VelocityModePos).await?;
        }
        self.set_velocity(velocity.abs().to_num()).await?;
        Ok(())
    }

    /// Wait for the motor to reach the target speed
    pub async fn wait_for_velocity(&mut self, delay_ms: u64) -> Result<(), Error<E>> {
        while !self.velocity_is_reached().await? {
            embassy_time::Timer::after(Duration::from_millis(delay_ms)).await;
        }
        Ok(())
    }

    /// Wait for the motor to reach the target position
    pub async fn wait_for_target(&mut self) -> Result<(), Error<E>> {
        let mode = self.read_register(Registers::RAMPMODE).await?.data;
        if mode != RampMode::PositioningMode as u32 {
            return Err(Error::PinError);
        }
        let est_end = self.calculate_finish().await?;
        embassy_time::Timer::after(est_end).await;
        while !self.position_is_reached().await? {
            embassy_time::Timer::after(Duration::from_micros(50)).await;
        }
        Ok(())
    }
    /// Calculate the remaining time in a motion in positioning mode
    pub async fn calculate_finish(&mut self) -> Result<Duration, Error<E>> {
        let status = self.read_ramp_status().await?;
        if status.position_reached() {
            return Ok(Duration::from_millis(0));
        }
        let position = self.get_position().await?;
        let target = self.get_target().await?;
        let distance = abs(target - position);
        let max_speed = abs(self.get_velocity_max());    
        if distance < 1.0 {
            // We underestimate the time by a bit here by not accounting for acceleration.
            // This is fine, saves a bunch of register reads & math and is good enough for most cases.
            // Similarly, we ignore the possibility of the motor being in the deceleration phase, because
            // it's hard to know for sure, and this case here covers it for 99% of all scenarios.
            let millis = max_speed * 1000.0 * distance;
            return Ok(Duration::from_millis(millis as u64));
        }
        let v1 = self.read_register(Registers::V1).await?.data;
        let v1 = self.speed_to_hz(v1);
        let (speed, phase) = if status.vzero() {
            (0.0 , MovePhase::Stopped)
        } else if status.velocity_reached() {
            (max_speed, MovePhase::VMax)
        } else {
            let s = abs(self.get_velocity().await?);
            if s < v1 {
                (s, MovePhase::AStart)
            } else {
                (s, MovePhase::A2Max)
            }
        };
        let v_start = self.read_register(Registers::VSTART).await?.data;
        let v_start = self.speed_to_hz(v_start);
        let v_stop = self.read_register(Registers::VSTOP).await?.data;
        let v_stop = self.speed_to_hz(v_stop);
        
        let a_1 = self.read_register(registers::Registers::A1).await?.data;
        let a_1 = self.accel_to_hz(a_1);
        let d_1 = self.read_register(Registers::D1).await?.data;
        let d_1 = self.accel_to_hz(d_1);
        let d_max = self.read_register(registers::Registers::DMAX).await?.data;
        let d_max = self.accel_to_hz(d_max);
        let a_max = self.read_register(registers::Registers::AMAX).await?.data;
        let a_max = self.accel_to_hz(a_max);

        let t_accel = (max_speed - v_start) / a_1;
        let d_accel = (v1 * v1 - v_start * v_start) / (2.0 * a_1);
        let t_decel = (max_speed - v_stop) / d_1;
        let d_decel = (v1 * v1 - v_stop * v_stop) / (2.0 * d_1);
        let d_a_max = (max_speed * max_speed - v1 * v1) / (2.0 * a_max);
        let d_d_max = (max_speed * max_speed - v1 * v1) / (2.0 * d_max);
        let t_a_max = (max_speed - v1) / a_max;
        let t_d_max = (max_speed - v1) / d_max;
        let d_const = match phase {
            MovePhase::Stopped => distance - (d_accel + d_decel + d_a_max + d_d_max),
            MovePhase::AStart => {
                let d = d_accel - (speed * speed - v_start * v_start) / (2.0 * a_1);
                distance - (d + d_decel + d_d_max + d_a_max)
            },
            MovePhase::A2Max => {
                let d = d_a_max - (speed * speed - v1 * v1) / (2.0 * a_max);
                distance - (d + d_decel + d_d_max)
            },
            MovePhase::VMax => distance - (d_decel + d_d_max),
        };
        let t_const = d_const / max_speed;
        let t_total = t_const + match phase {
            MovePhase::Stopped => t_accel + t_decel + t_a_max + t_d_max,
            MovePhase::AStart => {
                let t = (speed - v_start) / a_1;
                t + t_decel + t_a_max + t_d_max
            },
            MovePhase::A2Max => {
                let t = (speed - v1) / a_max;
                t + t_decel + t_d_max
            },
            MovePhase::VMax => t_decel + t_d_max,
        };
        let micros = t_total * 1000000.0;
        // This is also a slight underestimate, because the motor doesn't ACTUALLY
        // accelerate from 0 to V_START in 0 time, but it's good enough,
        // and we WANT to underestimate rather than overestimate.
        Ok(Duration::from_micros(micros as u64))
    }
}

#[repr(u8)]
#[derive(Debug, PartialEq, Clone, Copy)]
enum MovePhase {
    Stopped,
    AStart,
    A2Max,
    VMax,
}

fn abs(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}   