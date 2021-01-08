use std::time::Duration;
use wacohtech_force_torque_sensor::Wdf6m200;

fn main() {
    println!("Demonstration started!");
    let period = Duration::from_millis(10);

    // 力覚センサと接続
    let mut sensor = Wdf6m200::open(period).unwrap();

    // 力覚センサに力がはたらいていなくても，力覚センサからの値は0とはならない．
    // そこで，力覚センサから何度か値を取得して，その平均をオフセットとしてとることでゼロ点を設定する．
    println!("Performing calibration. Do not touch the sensor...");
    sensor.calibrate(period, 100);
    println!("Calibration done!");

    // 1000回力の測定を行う．
    let count = 1000;

    for i in 0..count {
        // まずセンサからの情報を更新．
        // エラーが発生したらその内容を表示する．
        if let Err(err) = sensor.update() {
            println!("{}", err);
        }

        // レンチを取得
        let wrench = sensor.last_measurement();
        // レンチを表示
        println!("[{}/{}]: {:?}", i + 1, count, wrench);

        // 次の観測時刻まで待機
        std::thread::sleep(period);
    }

    println!("Demonstration finished!");
}
