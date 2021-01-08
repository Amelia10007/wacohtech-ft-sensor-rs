//! ワコーテック製6軸力覚センサと通信するためのライブラリ．

use dimensioned::si::Unitless;
pub use dimensioned::si::{Meter, Newton};
use dimensioned::typenum::{Prod, Quot};
pub use pair_macro::Triplet;
use std::fmt::{self, Display, Formatter};
use std::ops::{Add, Sub};
use std::time::Duration;

pub type NewtonMeter<T> = Prod<Newton<T>, Meter<T>>;

/// レンチ(力とトルクのペア)を表す．
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wrench {
    /// 力．
    pub force: Triplet<Newton<f64>>,
    /// トルク．
    pub torque: Triplet<NewtonMeter<f64>>,
}

impl Wrench {
    pub const fn new(force: Triplet<Newton<f64>>, torque: Triplet<NewtonMeter<f64>>) -> Wrench {
        Wrench { force, torque }
    }

    /// 力とトルクが0である`Wrench`を返す．
    pub fn zeroed() -> Wrench {
        let force = Triplet::from_cloned(0.0).map(Newton::new);
        let torque = Triplet::from_cloned(0.0).map(NewtonMeter::<f64>::new);
        Wrench { force, torque }
    }
}

impl Add for Wrench {
    type Output = Wrench;

    fn add(self, rhs: Self) -> Self::Output {
        let force = self.force + rhs.force;
        let torque = self.torque + rhs.torque;
        Wrench { force, torque }
    }
}

impl Sub for Wrench {
    type Output = Wrench;

    fn sub(self, rhs: Self) -> Self::Output {
        let force = self.force - rhs.force;
        let torque = self.torque - rhs.torque;
        Wrench { force, torque }
    }
}

/// WDF-6M200-3 Wacohtech 6-axis force/touque sensor
pub struct Wdf6m200 {
    /// センサに接続されたシリアルポート．
    serial_port: Box<dyn serialport::SerialPort>,
    /// 現在のセンサ出力値．
    raw_wrench: Wrench,
    /// センサ出力値から減ずる補正値．
    /// センサは力がはたらいていない場合も0ではない出力を出す．
    /// そのため，センサからの生の出力からこのオフセット値を減じて補正してやる必要がある．
    offset: Wrench,
}

impl Wdf6m200 {
    /// コンピュータに接続されたセンサとの通信を確立する．
    /// # Params
    /// 1. `read_timeout_duration`: シリアル通信の読み取り操作がこの時間経過しても完了していない場合，タイムアウトとなる．
    ///
    /// # Returns
    /// センサとの通信が確立できた場合，センサのインスタンス`sensor`を`Ok(sensor)`として返す．
    /// 通信に失敗した場合，その内容を表すエラー`e`を`Err(e)`として返す．
    pub fn open(read_timeout_duration: Duration) -> Result<Wdf6m200, SensorError> {
        // PCに接続されているデバイスの中から力覚センサを探し，そのデバイスへのパスを取得する
        let sensor_port_path = serial_ports::ListPorts::new()
            .iter()
            // デバイスのうち，USB接続されているものをみつける
            .filter_map(|port| {
                if let serial_ports::ListPortType::UsbPort(info) = &port.port_type {
                    Some((port, info))
                } else {
                    None
                }
            })
            // IDが力覚センサと一致するデバイスをみつける
            .filter(|(_, info)| info.vid == SENSOR_DEVICE_VENDOR_ID)
            .filter(|(_, info)| info.pid == SENSOR_DEVICE_PRODUCT_ID)
            .map(|(port, _)| port.device.clone())
            .next()
            .ok_or(SensorError::SensorNotFound)?;

        // ハードウェアの仕様に合わせて通信設定を作る．
        // センサの仕様書を見て，ここの通信設定を決めた．
        let settings = serialport::SerialPortSettings {
            baud_rate: 921600,
            data_bits: serialport::DataBits::Eight,
            flow_control: serialport::FlowControl::None,
            parity: serialport::Parity::None,
            stop_bits: serialport::StopBits::One,
            timeout: read_timeout_duration,
        };

        // シリアル通信確立
        let serial_port =
            serialport::open_with_settings(&sensor_port_path.into_os_string(), &settings)?;

        let mut sensor = Self {
            serial_port,
            raw_wrench: Wrench::zeroed(),
            offset: Wrench::zeroed(),
        };

        // 最初のupdate()に備えて，データを送信するようにセンサに要求する
        sensor.request_next_data()?;

        Ok(sensor)
    }

    /// 最後にこのセンサから取得した測定値を返す．
    /// このメソッドでは，センサとの直接の通信は行わない．
    /// センサと通信して観測値を更新するには`update`メソッドを利用する．
    pub fn last_measurement(&self) -> Wrench {
        self.raw_wrench - self.offset
    }

    /// センサと通信して，測定値情報を更新する．
    /// 更新した測定値を得るには`last_measurement`メソッドを利用する．
    pub fn update(&mut self) -> Result<(), SensorError> {
        self.raw_wrench = self
            .read_bytes()
            .and_then(Self::convert_reception_to_raw_wrench)?;

        // 次の観測に備えて，センサに力を送信するように命令しておく
        self.request_next_data()?;

        Ok(())
    }

    /// 指定した期間センサからの出力を受信し，その平均をゼロ点とすることでキャリブレーションを行う．
    /// # Panics
    /// `measurement_times`が0の場合．
    pub fn calibrate(&mut self, measurement_period: Duration, measurement_times: usize) {
        assert!(measurement_times > 0);

        let mut raw_wrenches = vec![];

        // 指定回数，センサからの生データを収集する
        for _ in 0..measurement_times {
            if let Err(_) = self.update() {}
            raw_wrenches.push(self.raw_wrench);
            // 次の取得時刻まで待機
            std::thread::sleep(measurement_period);
        }
        // 生データの平均をとり，補正後の値が0となるようにオフセットを定める．
        let raw_wrench_sum = raw_wrenches
            .iter()
            .fold(Wrench::zeroed(), |acc, &cur| acc + cur);
        let raw_force_average = raw_wrench_sum.force.map(|e| e / raw_wrenches.len() as f64);
        let raw_torque_average = raw_wrench_sum.torque.map(|e| e / raw_wrenches.len() as f64);

        self.offset = Wrench::new(raw_force_average, raw_torque_average);
    }

    /// 次の出力値を送信するようセンサに指令する．
    /// センサからデータを受信するには，前もってこのメソッドを呼び出す必要がある．
    fn request_next_data(&mut self) -> Result<(), SensorError> {
        // Read命令を送信
        const WRITE_DATA: [u8; 1] = ['R' as u8];
        let write_count = self.serial_port.write(&WRITE_DATA)?;
        // 送信できたデータサイズで成否判定
        match write_count {
            c if c == WRITE_DATA.len() => Ok(()),
            c => Err(SensorError::Write(WRITE_DATA.len(), c)),
        }
    }

    /// センサから受信したデータを読み出して返す．
    fn read_bytes(&mut self) -> Result<[u8; RESPONSE_BYTES], SensorError> {
        let mut read_bytes = [0; RESPONSE_BYTES];
        let read_count = self.serial_port.read(&mut read_bytes)?;
        // 送信できたデータサイズで成否判定
        match read_count {
            RESPONSE_BYTES => Ok(read_bytes),
            c => Err(SensorError::Read(RESPONSE_BYTES, c)),
        }
    }

    /// センサから受信したデータをレンチ情報に変換して返す．
    fn convert_reception_to_raw_wrench(
        reception: [u8; RESPONSE_BYTES],
    ) -> Result<Wrench, SensorError> {
        let digitals = {
            // 受信データを文字列として解釈する
            let text = std::str::from_utf8(&reception)?;
            let mut array = [0; AXIS_COUNT];
            // 各軸別々にデータを抽出
            for i in 0..AXIS_COUNT {
                // 該当する軸のデータが生バイト列のどの範囲にあるのか計算
                let start = AXIS_DATA_START_INDEX + i * AXIS_DATUM_LENGTH;
                let end = 1 + (i + 1) * AXIS_DATUM_LENGTH;
                // 該当部分の文字列を読み，16進数テキストから整数へ変換
                let axis_text = text.get(start..end).ok_or(SensorError::InvalidTextLength)?;
                let digital = u16::from_str_radix(axis_text, 16)?;
                array[i] = digital;
            }

            // このデジタル出力値の配列は，x,y,z方向の力，x,y,z方向のトルクの順に情報が格納されている．
            array
        };

        // デジタル出力値からレンチへ変換
        let raw_wrench = {
            let force = {
                let digital = Triplet::new(digitals[0], digitals[1], digitals[2]).map(|i| i as f64);
                let sensitivity = force_sensitivity();
                digital.map_entrywise(sensitivity, |d, s| d / s)
            };
            let torque = {
                let digital = Triplet::new(digitals[3], digitals[4], digitals[5]).map(|i| i as f64);
                let sensitivity = torque_sensitivity();
                digital.map_entrywise(sensitivity, |d, s| d / s)
            };
            Wrench::new(force, torque)
        };

        Ok(raw_wrench)
    }
}

/// 力覚センサとの通信で発生したエラーを表す．
#[derive(Debug)]
pub enum SensorError {
    /// 力覚センサが見つからない。
    SensorNotFound,
    /// シリアル通信開始時に発生したエラー．
    SerialPortOpen(serialport::Error),
    /// センサから受信したデータサイズが期待されるサイズと一致しない．
    Read(usize, usize),
    /// センサに送信したデータサイズが期待されるサイズと一致しない．
    Write(usize, usize),
    /// センサとのI/Oで発生したエラー．
    Io(std::io::Error),
    /// センサからの受信データをUTF8文字列にパースできない．
    Utf8(std::str::Utf8Error),
    /// センサから受信した文字列の長さが期待される長さと一致しない．
    InvalidTextLength,
    /// センサから受信した文字列を整数に変換できない．
    ParseInt(std::num::ParseIntError),
}

impl Display for SensorError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            SensorError::SensorNotFound => write!(f, "Sensor not found"),
            SensorError::SerialPortOpen(x) => x.fmt(f),
            SensorError::Io(x) => x.fmt(f),
            SensorError::Utf8(x) => x.fmt(f),
            SensorError::InvalidTextLength => {
                write!(f, "Received text's length from the sensor does not match the expected one")
            }
            SensorError::ParseInt(x) => x.fmt(f),
            SensorError::Read(desired, actual) => write!(
                f,
                "The driver should read {} bytes from the sensor, but actually {} bytes read",
                desired, actual
            ),
            SensorError::Write(desired, actual) => write!(
                f,
                "The driver should write {} bytes to the sensor, but actually {} bytes written",
                desired, actual
            ),
        }
    }
}

impl std::error::Error for SensorError {}

impl From<serialport::Error> for SensorError {
    fn from(err: serialport::Error) -> Self {
        SensorError::SerialPortOpen(err)
    }
}

impl From<std::io::Error> for SensorError {
    fn from(err: std::io::Error) -> Self {
        SensorError::Io(err)
    }
}

impl From<std::str::Utf8Error> for SensorError {
    fn from(err: std::str::Utf8Error) -> Self {
        SensorError::Utf8(err)
    }
}

impl From<std::num::ParseIntError> for SensorError {
    fn from(err: std::num::ParseIntError) -> Self {
        SensorError::ParseInt(err)
    }
}

// 力覚センサから返ってくるバイト列の解釈方法:
// ---------------------------
// X111122223333444455556666++
// X: 受信データの先頭1バイトはレコード番号．
// 1111...6666: 次に各軸に対応した電圧が4バイト (合計で6*4=24バイト)．
// ++: 最後に改行コード(CR+LF)が2バイト

/// 各軸に関するデジタル出力値のバイト数．
const AXIS_DATUM_LENGTH: usize = 4;
/// 各軸に関するデジタル出力が何バイトめから始まるか．
const AXIS_DATA_START_INDEX: usize = 1;
/// 軸数．
const AXIS_COUNT: usize = 6;
/// 改行コードの記述に要するバイト数．
const NEWLINE_BYTES: usize = 2;
/// センサから受信されるべきバイト数．
const RESPONSE_BYTES: usize =
    AXIS_DATA_START_INDEX + AXIS_DATUM_LENGTH * AXIS_COUNT + NEWLINE_BYTES;

/// センサデバイスの開発元ID
const SENSOR_DEVICE_VENDOR_ID: u16 = 0x10C4;
/// センサデバイスの製品ID
const SENSOR_DEVICE_PRODUCT_ID: u16 = 0xEA60;

type PerNewton<T> = Quot<Unitless<T>, Newton<T>>;
type PerNewtonMeter<T> = Quot<Unitless<T>, NewtonMeter<T>>;

/// センサ各軸について，1Nあたりデジタル出力値がいくつ変化するか．
/// これはセンサの仕様表から取ってきた値．
fn force_sensitivity() -> Triplet<PerNewton<f64>> {
    Triplet::new(24.9, 24.6, 24.5).map(PerNewton::<f64>::new)
}

/// センサ各軸について，1Nmあたりデジタル出力値がいくつ変化するか．
/// これはセンサの仕様表から取ってきた値．
fn torque_sensitivity() -> Triplet<PerNewtonMeter<f64>> {
    Triplet::new(1664.7, 1639.7, 1638.0).map(PerNewtonMeter::<f64>::new)
}
