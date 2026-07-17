#ifndef CONFIG_H_
#define CONFIG_H_

#include <cstdint>

/**
 * 走行体のチューニング用定数を一元管理するクラス。
 * 値の調整はこのファイルだけで完結させ、各クラスのコード中に定数を直接書かないようにする。
 */
class Config {
public:
    // ── 機体の寸法 ─────────────────────────────────────
    static constexpr float WHEEL_RADIUS_MM = 28.0f;  // ホイール半径[mm]
    static constexpr float TREAD_MM = 112.0f;        // 左右ホイール間の距離[mm]
    static constexpr float PI = 3.14159f;
    static constexpr float DISTANCE_FROM_COLORCENSOR_TO_WHEEL = 40.0f;  // [mm] カラーセンサーからホイール軸までの距離（概数）

    // ── Robot: 走行機能 ────────────────────────────────
    // driveStraight
    static constexpr int DRIVE_DEFAULT_SPEED_DEG_PER_SEC = 300;  // 直進の既定速度[°/秒]
    static constexpr int DRIVE_TIMEOUT_LOOP_COUNT = 2000;        // 直進のタイムアウト(周期の回数)
    // turn
    static constexpr int TURN_DEFAULT_SPEED_DEG_PER_SEC = 300;  // 旋回の既定速度[°/秒]
    static constexpr int TURN_TIMEOUT_LOOP_COUNT = 500;         // 旋回のタイムアウト(周期の回数)
    static constexpr float TURN_IMU_STOP_TOLERANCE_DEG = 0.5f;  // IMU旋回の停止許容誤差[°]
    static constexpr float TURN_IMU_KP = 4.0f;                  // IMU旋回の比例ゲイン
    static constexpr int TURN_IMU_MIN_SPEED_DEG_PER_SEC = 80;   // IMU旋回の最低速度[°/秒]
    // runUntilColor
    static constexpr int RUC_DEFAULT_SPEED_DEG_PER_SEC = 300;  // 既定速度[°/秒]
    static constexpr int RUC_SWING_MAX_COUNT = 50;             // 蛇行/最大旋回回数
    static constexpr float RUC_SWING_DEFAULT_DEG = 50.0f;      // 蛇行/1旋回における旋回角度[°]
    static constexpr int RUC_SWING_TIMEOUT_LOOP_COUNT = 500;   // 蛇行/1旋回におけるタイムアウト(周期の回数)
    // 共通
    static constexpr int MOTION_POLL_INTERVAL_US = 10 * 1000;  // 直進・旋回・蛇行中のエンコーダー確認周期[us]

    // turnByImuUntilUltrasonic
    // 超音波センサのポーリング周期は他の移動系（turn/driveStraight等）と共通のMOTION_POLL_INTERVAL_USではなく、
    // 検知精度を上げたいため専用に短い周期を使う。周期が短くなる分、同じタイムアウトのループ回数上限を確保
    // しないと実時間でのタイムアウトが短くなりすぎる（低速走行だと完了前に打ち切られる）。
    static constexpr int TURN_ULTRASONIC_POLL_INTERVAL_US = 1 * 1000;  // 超音波センサの確認周期[us]
    static constexpr int TURN_ULTRASONIC_TIMEOUT_LOOP_COUNT = 15000;   // 走行のタイムアウト(周期の回数。上記周期で約30秒分)
    static constexpr int TURN_ULTRASONIC_LOG_INTERVAL_LOOPS = 50;      // 走行中、距離とIMU角度をsyslog出力する周期(ループ回数。上記周期で約50ms間隔)

    // ── Robot: 起動時の設定 / HMI ────────────────────────
    static constexpr int SPEAKER_VOLUME = 50;    // スピーカー音量[0-100]
    static constexpr int BEEP_DEFAULT_MS = 100;  // ビープ音のデフォルト再生時間[ms]

    // ── Tracer（ライントレース）───────────────────────────
    static constexpr float TRACER_KP = 0.33f;                // 反射率PID制御の比例ゲイン
    static constexpr float TRACER_KI = 0.01f;                // 反射率PID制御の積分ゲイン
    static constexpr float TRACER_KD = 0.03f;                // 反射率PID制御の微分ゲイン
    static constexpr int32_t TRACER_TARGET_REFLECTION = 60;  // 黒白の中間反射率
    static constexpr int8_t TRACER_PWM = 50;                 // 基準パワー

    // ── ColorJudge（色判定）───────────────────────────────
    // 彩度がこれ未満なら無彩色(黒/白)とみなす（実測でS=22〜27前後のノイズが乗るため、それより高い値にする）
    static constexpr uint8_t COLOR_CHROMATIC_MIN_SATURATION = 30;
    // 無彩色のとき、反射率がこれ未満なら黒、以上なら白（TRACER_TARGET_REFLECTIONと同じ考え方）
    static constexpr int COLOR_ACHROMATIC_REFLECTION_THRESHOLD = 60;
    static constexpr uint16_t COLOR_RED_HUE = 0;
    static constexpr uint16_t COLOR_YELLOW_HUE = 60;
    static constexpr uint16_t COLOR_GREEN_HUE = 120;
    static constexpr uint16_t COLOR_BLUE_HUE = 240;
    static constexpr int COLOR_HUE_TOLERANCE = 40;         // 各色のHueからの許容誤差[度]
    static constexpr int COLOR_DETECTED_STABLE_COUNT = 3;  // 色判定を行う際、信頼するカラーセンサーの連続判定回数

    // ── Pid（PID制御共通）─────────────────────────────────
    static constexpr float PID_INTEGRAL_LIMIT = 100.0f;         // 積分項の暴走を防ぐ上下限
    static constexpr float PID_DERIVATIVE_FILTER_ALPHA = 0.8f;  // 微分項のローパスフィルタ係数

    // ── Calibrator（起動準備）────────────────────────────
    static constexpr int CALIBRATOR_BLE_WAIT_US = 3 * 1000 * 1000;  // BLE接続待ち時間[us]
    static constexpr int CALIBRATOR_POLL_INTERVAL_US = 50 * 1000;   // ボタン確認周期[us]
    static constexpr int CALIBRATOR_BEEP_MS = 300;                  // 起動ビープの再生時間[ms]

    // ── GameRunner（全体フロー）───────────────────────────
    static constexpr int LINE_TRACE_POLL_INTERVAL_US = 100 * 1000;  // ライントレースの制御周期[us]
    static constexpr int LABEL_CHANGE_CYCLES = 3;                   // 表示文字を切り替える周期(制御周期の何回分か)

    // ── DeliveryTask（ボトルデリバリー）───────────────────────
    static constexpr int8_t DELIVERY_TRACER_PWM = 30;       // ボトル接近時のライントレース速度
    static constexpr int DELIVERY_TARGET_DISTANCE_MM = 95;  // ボトル手前で停止する目標距離[mm]

    // ── Arm (アーム制御) ───────────────────────────────────
    static constexpr int ARM_RAISE_DEG = 155;   // アームを上げる角度
    static constexpr int ARM_LOWER_DEG = 160;   // アームを下げる角度
    static constexpr int ARM_RAISE_PWM = -100;  // アームを上げる速度
    static constexpr int ARM_LOWER_PWM = 100;   // アームを下げる速度
    // ── ET-Rally（課題）───────────────────────────────────
    // 走行速度
    static constexpr int ETRALLY_DEFAULT_SPEED = 200;
    static constexpr int ETRALLY_FAST_SPEED = 500;
    static constexpr int ETRALLY_SLOW_SPEED = 100;
    static constexpr int ETRALLY_WAVING_SPEED = 150;
    static constexpr int ETRALLY_LINE_TRACE_DEFAULT_POWER = 34;
    static constexpr int ETRALLY_LINE_TRACE_FAST_POWER = 40;
    // コースの寸法
    static constexpr int COLOR_CIRCLE_RADIUS = 35;                       // [mm] ラリーフィールド横のライン上にある色付きの円の半径（概数）
    static constexpr int BLUE_LINE_DISTANCE = 100;                       // [mm] 青いラインの長さ（概数）
    static constexpr int ETRALLY_UNIT_DISTANCE = 240;                    // [mm] ETラリーフィールドのQRからQRまでの距離を単位距離としている。（概数）
    static constexpr int ETRALLY_THROUGH_GATE_ADJUSTMENT_DISTANCE = 25;  // [mm] ラリーフィールド横のラインからQRコードまでの距離と単位距離の差
    // 調整値
    static constexpr float ETRALLY_NARROW_SWING_DEG = 20.0f;  // [°]
    static constexpr int ETRALLY_DELAY = 100 * 1000;
    /*
     * ETラリーフィールド上のゲート配置例
     *
     *          col
     *        1 2 3 4 5
     * row 1  . . . . .
     *     2  Y Y . . .
     *     3  . . . . B
     *     4  . . . . B
     *     5  . R R . .
     *
     * R: 赤ゲート
     * B: 青ゲート
     * Y: 黄ゲート
     */
    // 赤ゲート
    static constexpr int ETRALLY_RED_GATE_LEFT_ROW = 5;
    static constexpr int ETRALLY_RED_GATE_LEFT_COL = 2;
    static constexpr int ETRALLY_RED_GATE_RIGHT_ROW = 5;
    static constexpr int ETRALLY_RED_GATE_RIGHT_COL = 3;

    // 青ゲート
    static constexpr int ETRALLY_BLUE_GATE_LEFT_ROW = 3;
    static constexpr int ETRALLY_BLUE_GATE_LEFT_COL = 5;
    static constexpr int ETRALLY_BLUE_GATE_RIGHT_ROW = 4;
    static constexpr int ETRALLY_BLUE_GATE_RIGHT_COL = 5;

    // 黄ゲート
    static constexpr int ETRALLY_YELLOW_GATE_LEFT_ROW = 2;
    static constexpr int ETRALLY_YELLOW_GATE_LEFT_COL = 1;
    static constexpr int ETRALLY_YELLOW_GATE_RIGHT_ROW = 2;
    static constexpr int ETRALLY_YELLOW_GATE_RIGHT_COL = 2;

private:
    Config() = delete;  // インスタンス化しない、定数の名前空間として使う
};

#endif  // !CONFIG_H_
