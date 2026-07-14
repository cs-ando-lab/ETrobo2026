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

    // ── Robot: 走行機能 ────────────────────────────────
    // driveStraight
    static constexpr int DRIVE_DEFAULT_SPEED_DEG_PER_SEC = 300;  // 直進の既定速度[°/秒]
    static constexpr int DRIVE_TIMEOUT_LOOP_COUNT = 500;         // 直進のタイムアウト(周期の回数)
    // turn
    static constexpr int TURN_DEFAULT_SPEED_DEG_PER_SEC = 300;  // 旋回の既定速度[°/秒]
    static constexpr int TURN_TIMEOUT_LOOP_COUNT = 500;         // 旋回のタイムアウト(周期の回数)
    // runUntilColor
    static constexpr int RUC_DEFAULT_SPEED_DEG_PER_SEC = 300;  // 既定速度[°/秒]
    static constexpr int RUC_SWING_MAX_COUNT = 50;             // 蛇行/最大旋回回数
    static constexpr float RUC_SWING_DEFAULT_DEG = 40.0f;      // 蛇行/1旋回における旋回角度[°]
    static constexpr int RUC_SWING_TIMEOUT_LOOP_COUNT = 500;   // 蛇行/1旋回におけるタイムアウト(周期の回数)
    // 共通
    static constexpr int MOTION_POLL_INTERVAL_US = 10 * 1000;  // 直進・旋回・蛇行中のエンコーダー確認周期[us]

    // ── Robot: 起動時の設定 / HMI ────────────────────────
    static constexpr int SPEAKER_VOLUME = 50;    // スピーカー音量[0-100]
    static constexpr int BEEP_DEFAULT_MS = 100;  // ビープ音のデフォルト再生時間[ms]

    // ── Tracer（ライントレース）───────────────────────────
    static constexpr float TRACER_KP = 0.33f;                 // 反射率PID制御の比例ゲイン
    static constexpr float TRACER_KI = 0.01f;                 // 反射率PID制御の積分ゲイン
    static constexpr float TRACER_KD = 0.03f;                 // 反射率PID制御の微分ゲイン
    static constexpr int32_t TRACER_TARGET_REFLECTION = 60;   // 黒白の中間反射率
    static constexpr int8_t TRACER_PWM = 50;                  // 基準パワー
    static constexpr int8_t TRACER_BLUE_DETECTION_COUNT = 3;  // 青判定に必要な連続検出回数

    // ── ColorJudge（色判定）───────────────────────────────
    // 彩度がこれ未満なら無彩色(黒/白)とみなす（実測でS=22〜27前後のノイズが乗るため、それより高い値にする）
    static constexpr uint8_t COLOR_CHROMATIC_MIN_SATURATION = 30;
    // 無彩色のとき、反射率がこれ未満なら黒、以上なら白（TRACER_TARGET_REFLECTIONと同じ考え方）
    static constexpr int COLOR_ACHROMATIC_REFLECTION_THRESHOLD = 60;
    static constexpr uint16_t COLOR_RED_HUE = 0;
    static constexpr uint16_t COLOR_YELLOW_HUE = 60;
    static constexpr uint16_t COLOR_GREEN_HUE = 120;
    static constexpr uint16_t COLOR_BLUE_HUE = 240;
    static constexpr int COLOR_HUE_TOLERANCE = 40;  // 各色のHueからの許容誤差[度]

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
    static constexpr int8_t DELIVERY_TRACER_PWM = 30;        // ボトル接近時のライントレース速度
    static constexpr int DELIVERY_TARGET_DISTANCE_MM = 120;  // ボトル手前で停止する目標距離[mm]

private:
    Config() = delete;  // インスタンス化しない、定数の名前空間として使う
};

#endif  // !CONFIG_H_
