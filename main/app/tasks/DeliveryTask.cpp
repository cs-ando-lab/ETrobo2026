#include "DeliveryTask.h"
#include "AreaBlueGate.h"
#include "ColorNotifier.h"
#include "SettleGate.h"
#include "TraceEntryLog.h"
#include "Tracer.h"
#include "Config.h"
#include "CourseConfig.h"
#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/syslog.h"
#include <cmath>

namespace {
    // 停止後のまとめ出力専用。32件の共有バッファに一括投入すると古い診断が消える。
    // 余裕を残して送信タスクへ実行時間を渡す。通信停止時は待ち続けず詳細を打ち切る。
    bool diagnosticOutputBlocked = false;
    int diagnosticSkippedLines = 0;

    // 後続タスクの大量ログに最後の要約が押し出されないよう、復帰前に消費を待つ。
    // シリアル/BLEの受信完了を保証するものではなく、共有ログキューの排出だけを待つ。
    void drainStoppedDiagnostics() {
        if(diagnosticOutputBlocked) return;
        SYSTIM started, now;
        get_tim(&started);
        for(;;) {
            T_SYSLOG_RLOG status;
            if(syslog_ref_log(&status) != E_OK || status.count == 0) return;
            get_tim(&now);
            if(now - started >= 1000000) return;
            dly_tsk(1000);
        }
    }

    template <typename... Args>
    void stoppedDiagnosticLog(uint_t priority, const char* format, Args... args) {
        if(diagnosticOutputBlocked) {
            ++diagnosticSkippedLines;
            return;
        }
        SYSTIM started, now;
        get_tim(&started);
        for(;;) {
            T_SYSLOG_RLOG status;
            const ER result = syslog_ref_log(&status);
            if(result == E_OK && status.count <= 8) {
                syslog(priority, format, args...);
                return;
            }
            get_tim(&now);
            if(result != E_OK || now - started >= 1000000) {
                diagnosticOutputBlocked = true;
                ++diagnosticSkippedLines;
                return;
            }
            dly_tsk(1000);
        }
    }

    // ── ライントレース速度 ──────────────────────────────
    constexpr int kApproachPwm = 50;         // ボトル接近中。Config::DELIVERY_TRACER_PWM(30)だとカーブ減速でほぼ動けなくなる
    constexpr int kReacquireLinePwm = 50;    // ライン復帰直後。ラインに対するズレが大きくカーブ減速が効きやすいので高め
    constexpr int kPostSlowTracePwm = 93;    // 曲線前半。後半は78/Kp0.55
    constexpr int kOnFirstBlueLinePwm = 78;  // 青1本目に乗っている間だけ落とす速度

    // ── ボトルの色判定 ────────────────────────────────
    // 離れた位置から読むため反射光が弱く、実測で反射率2〜3・明度7〜9しかない。それでも彩度88〜91・
    // 色相210〜218と色味は安定して出るので、彩度がこれ以上あればHueを信用する
    constexpr int kBottleMinSaturation = 40;
    // 走行全体の目標本数を決める判定なので、1サンプルでは決めない。停止・整定した状態で読むため
    // 連続一致で確定できるはず。決まらないまま走り出すより止まる方が安全
    // アームを上げた直後の整定待ち。confirmBottleColor()が5回連続一致を要求し、決まらなければ
    // 1秒まで読み直すので、長い固定待ちは不要。車体の揺れが収まる分だけ置く
    constexpr int kArmSettleBeforeColorMs = 200;

    // ── アームを上げる前の停止と整定 ─────────────────────
    // 接近ループはtracer.terminate()(=stop()=coast)で抜けるだけで、止まるのを待っていなかった。
    // 実測では1回目のアーム上げの時点で左307/右285°/秒出ており、車体が煽られたまま上げている。
    // 2回目(raiseArmWhileDriving)は停止状態から200°/秒で5mmなので、条件がまったく違う
    constexpr int kBottleStopSpeedDegPerSec = 40;  // これ未満なら止まったとみなす
    constexpr int kBottleStopTimeoutMs = 600;
    constexpr int kBottleSettleMinMs = 150;  // 止まってから車体の揺れが収まるのを待つ最低時間
    constexpr int kBottleSettleMaxMs = 500;  // 収まらない場合の打ち切り
    // IMU::isStationary()は実測で毎回550msのタイムアウトまで真にならなかった（判定窓が長いとみられる）。
    // 代わりに角速度そのものを見る。実測では静止時に0/0/0が出ている
    constexpr int kBottleSettleGyroDegPerSec = 3;  // 全軸これ未満なら揺れが収まったとみなす
    constexpr int kBottleSettleStableCount = 5;    // それが続く回数（約50ms）

    // 診断: アームを上げ切った後、角度がまだ動いていないかを見る
    constexpr int kArmPostRaiseLogCount = 5;      // 記録する回数
    constexpr int kArmPostRaiseLogIntervalMs = 40;

    // アームの角度はraiseArm/lowerArmの開始時にresetCount()されるので、ログの値は「その動作での移動量」
    // であって姿勢ではない。1回目と2回目を比べるには、各動作の移動量を積算した値が要る。
    // 符号は上げが負・下げが正（ARM_RAISE_SPEED_DEG_PER_SECが負のため）。1回目の上げ直前を原点とする。
    //
    // 位置制御のis_done()は目標に届く前に真になる。実測では165/166で「reached」と言った後、
    // 読み取り時点で171まで届いた回は色が読めて(v=52)、167で止まった回は読めなかった(v=1)。
    // 保持は続いているので、実際に目標角に入るまで待つ
    constexpr int kBottleColorStableCount = 5;
    // 色が読めなかったとき、アームを下げて少し前に出ながら上げ直し、読めるまで繰り返す。
    // 角度と距離の組み合わせでセンサーの当たる位置が変わるので、前に出るたびに読める可能性がある
    constexpr int kBottleRetryAdvanceMm = 5;
    constexpr int kBottleRetryAdvanceSpeedDegPerSec = 200;  // ボトルの目の前なのでゆっくり
    constexpr int kBottleRetryExtraArmDeg = 2;              // 角度不足で読めないことがあるので、再試行ごとに上げ角を足す
    // 保険。前に出続けるとボトルに当たるため、この回数で打ち切る（5mm×5回＝25mm）
    constexpr int kBottleRetryMaxCount = 5;
    constexpr int kBottleColorSampleIntervalMs = 20;
    constexpr int kBottleColorTimeoutMs = 1000;

    // ── ボトル色の通知 ─────────────────────────────────
    // 以前は色が決まった直後に黄1回・青2回・赤3回を鳴らし、鳴り終わるまでアームの下降へ
    // 進まなかった（黄100/青300/赤500msがそのまま走行時間に乗る）。
    // 鳴らすのはアームを下げた後の1秒直進の中にして、走行を止めずに通知する。
    // OFFにすれば一切鳴らさない。正のdurationのbeep()はこの区間から呼ばない
    enum class ColorNotifyMode { OFF,
                                 DEFERRED_NONBLOCKING };
    constexpr ColorNotifyMode kColorNotifyMode = ColorNotifyMode::DEFERRED_NONBLOCKING;
    constexpr int kColorNotifyToneMs = 100;  // 1回の長さ
    constexpr int kColorNotifyGapMs = 100;   // 2回目以降の前に空ける時間

    // ── アームを下げた直後の直進（これだけでラインへ復帰させる）──
    constexpr int kAfterArmStraightLeftPwm = 35;
    constexpr int kAfterArmStraightRightPwm = 40;
    constexpr float kAfterArmStraightSec = 1.0f;

    // ── 青ライン判定 ──────────────────────────────────
    // 白黒のグラデーション帯は色相が青寄り(実測195〜215度)で、彩度がときどき40〜56まで跳ねるため
    // ColorJudgeでは青と判定される。実測の本物の青ラインは彩度70〜88あるので、彩度で切り分ける
    constexpr int kBlueLineMinSaturation = 65;
    // 確定はサンプル数ではなくms基準にする。制御周期が変わっても意図した時間幅を保つため
    // 実測: コーナー後の青は条件を満たす最大連続が12〜20サンプルしかない。100ms(10サンプル)だと
    // マージンが2〜3しか残らないため70msまで詰める。誤検知は彩度ゲート(kBlueLineMinSaturation)で切る
    constexpr int kBlueEntryConfirmMs = 70;    // 青に乗ったと確定するまでの時間
    constexpr int kBluePassedConfirmMs = 400;  // 青を通過した（完全に降りた）と確定するまでの時間
    // エリアへ向かう最後の1本だけ短くする。300msだと確定までに約60mm進み、入口ではなく出口で抜けてしまうため
    constexpr int kBlueFinalEntryConfirmMs = 50;

    // ── 青1本目の後の移動（右エッジへの持ち替え） ──────────────
    // 基準角度から80度へのturnByImuは約1秒かかり、青の上で膨らんだ分も戻せなかった。弧を描いて線を横切り、
    // 黒を踏んだ時点で止める。時間だけで止めると、青を降りた位置のばらつきがそのまま残る。Lコース基準
    // 実測の所要は121ms・275msとばらつく。時間ではなく進んだ距離で見たいので距離もログに出す。
    // 50/30では遅いので上げる（比はそのまま。線を渡る向きの回頭量を変えないため）
    constexpr int kAfterBlue1OuterPwm = 65;
    constexpr int kAfterBlue1InnerPwm = 39;
    constexpr int kAfterBlue1MoveMs = 450;          // 上限。実測では143〜187msで黒を踏む
    constexpr int kAfterBlue1BlackReflection = 35;  // 黒15/青37/グラデーション43〜55
    constexpr int kAfterBlue1BlackRunCount = 2;
    // 以前は「動き出しで左エッジの黒を拾って即終了しない」ために最低100ms動かしていたが、実測の所要は
    // 132〜154msで、そのうち100msがこの固定待ちだった。懸念は時間ではなく開始時の状態で切り分ける。
    //   開始時が黒   → すでに線をまたいでいるので動かさない
    //   開始時が黒でない → 黒でない状態から始まるので、最初の黒で止めてよい（最低時間は不要）
    constexpr int kAfterBlue1StartBlackCount = 3;  // 開始時に黒とみなす連続サンプル数（約30ms）
    // 開始時の反射率が黒(35)の近くだと、動き出して1〜2サンプルで黒に入り、渡り切る前に止まる恐れがある。
    // 実測の開始値は52・59で、この値より十分明るければ最低時間は要らない
    constexpr int kAfterBlue1ClearOfBlackReflection = 45;
    constexpr int kAfterBlue1AmbiguousMinMoveMs = 40;  // 35〜45の中間だったときだけ待つ

    // ── 青1本目での線の踏み越え ─────────────────────────────
    // 青の手前の曲線は成功時でも白が最大91回続くので判定に使えない。青1本目の上なら成功時の白の連続は
    // 11〜15回、踏み越えた3回は35回以上と分かれる。踏み越えると左エッジのTracerは白を見て右へ切り、線から離れ続ける
    constexpr int kBlue1OvershootWhiteRunCount = 20;
    // 踏み越えたら元の方式（基準角度から80度へ旋回→右エッジ）に切り替える。
    // 内側へピボットして線を探す方式は、線を見つけても向きが行き過ぎて見失った
    constexpr float kBlue1OvershootTargetHeadingDeg = 80.0f;
    // 踏み越えた位置によっては、旋回後に青1本目をもう一度跨ぐ。2本目と数えるとエリアの本数がずれるので、
    // この間は青を数えず、明けたら1本目を通過済みとして揃える。コーナー後に青1本目は無いので、曲がりきったら早めに明ける
    constexpr int kBlue1OvershootBlueIgnoreMs = 5000;

    // ── 行きの90度コーナー手前の減速 ─────────────────────────
    // TRACER_PWM(90)のままピボットに入るとボトルを落とす。トレース開始からコーナー検知までは
    // 1194〜1226mm（3回）と安定しているので、距離で手前から落とす。検知は線の終わりの約40mm先
    // 直線でのPWMは93が最良だった（実測の平均速度/平均誤差: 100で615〜635/27、95で818〜865/9〜10だが1本だけ638/18、
    // 93で788〜831/5〜6、97で604〜623/23）。100や97では「基準PWM±操舵量」の余裕がなくなり、
    // 狙った左右差が出ない分だけ蛇行する。93は2本とも白黒の端に一度も張り付かなかった
    constexpr int kCornerTracePwm = 97;          // 手前で減速する前提なので、そこまでは速く走る
    constexpr int kCornerSlowdownStartMm = 950;  // 実機調整。線の終わりの約210mm手前
    // 踏み越え時は80度旋回の後から測るので、起点が通常時とずれる。通常時より早めに落とす
    constexpr int kCornerSlowdownStartMmAfterOvershoot = 800;
    constexpr int kCornerApproachPwm = 65;  // 行き・帰りのコーナー手前の減速後の速度

    // ── 帰りの90度コーナーの後 ────────────────────────────
    constexpr int kReturnAfterCornerFastPwm = 97;
    constexpr int kReturnFinishAfterCornerMm = 900;  // 曲がりきってからこの距離を走ったら終了し、ラリーへ引き渡す

    // ── 帰りの90度コーナーの手前 ──────────────────────────
    // 帰りのトレース開始からコーナー検知までは 黄467〜473 / 青777 / 赤1064〜1089mm で、色が1つ変わるごとに約305mm。
    // 時間で判定を始めるとエリア付近のトレースの揺れで白が9回続き、黄では356mmで誤検知したので、距離で始める
    constexpr int kReturnCornerDetectStartMmYellow = 400;    // 誤検知した356mmより後ろ、線の終わり(約430mm)より手前
    constexpr int kReturnCornerSlowdownStartMmYellow = 270;  // 実機調整。線の終わりの約160mm手前
    constexpr int kReturnCornerColorStepMm = 305;            // 黄→青→赤で1段ずつ遠くなる
    constexpr int kReturnTracePwm = 97;                      // 減速位置まで。手前で減速する前提なので速く走る

    // ── エリアに入る青の手前 ─────────────────────────────
    // 速いまま斜め移動に入るとボトルを落とす。行きのコーナーを曲がりきってからエリアに入る青までは
    // 黄450〜452 / 赤1023〜1026mm（各3回）で、青2〜4本目の間隔は約287mm（青は青3本目の約735mm）
    // 最終青の検知位置予測。黄/青は従来実測、赤は直近1007〜1008mmを基にする。
    // 物理的な青の先端位置ではなく、連続一致が成立する位置の目安。
    constexpr int kAreaExpectedBlueMm[] = {450, 735, 1005}; // 黄・青・赤
    constexpr int kAreaSlowLeadMm = 250;
    // 黄は減速から最終青までが短く、加速と減速の間に姿勢が乱れて立て直しに距離を使う。黄だけ100mm早く落とす
    constexpr int kAreaSlowLeadMmYellow = 350;  // 450-350=100mm地点で減速（従来は200mm地点）
    constexpr int kAreaSearchLeadMm = 120; // 減速後、前の青より先で最終青の探索を有効化
    constexpr int kAreaMissingBlueLimitMm = 200;
    constexpr int kAreaFastPwm = 97;
    constexpr int kAreaApproachPwm = 65;

    // ── 直角コーナー対策 ───────────────────────────────
    // Tracerのカーブ減速はEMA(TRACER_CURVE_TURN_FILTER_ALPHA=0.05 → 時定数約200ms)で
    // ステップ状の変化には間に合わないため、白の連続で「線を見失った」を検知しピボット旋回で曲がり直す
    constexpr int kCornerWhiteReflection = 85;  // これ以上を白とみなす（実測の白は約99）
    constexpr int kCornerBlackReflection = 35;  // これ以下を黒（実測 黒15/青37）。45では境目のグラデーション(43〜55)を誤検知した
    // 直線部でも白は6〜7回連続する（TRACER_PWM=80・旧PIDでの実測）。10では帰りの手前で9まで来て誤検知もしたので12。
    // コーナー手前で減速するので、検知が2サンプル遅れても行き過ぎは小さい。Tracerの設定を触ったら測り直すこと
    constexpr int kCornerWhiteRunCount = 12;

    constexpr int kCornerPivotOuterPwm = 57;       // 両輪逆転は回転ジャークでボトルを落とすため片輪駆動。強すぎると線を踏み抜く
    constexpr int kCornerPivotInnerPwm = 0;        // 0で片輪旋回。負にすると半径は縮むがボトルへの負荷が増える
    constexpr int kCornerPivotRampLoopCount = 15;  // 立ち上がりでボトルを振らないよう150msかけて上げる
    constexpr int kCornerPivotBlackRunCount = 3;   // 1サンプルのノイズで抜けないための連続回数
    // 行き過ぎた直後は元の線の方が近く先に当たる。元の線だと18度で終わるが正解時は72〜86度なので40度で分離できる
    constexpr float kCornerPivotMinTurnDeg = 40.0f;
    // 復路は事情が違う。実測では旋回開始から40度に達する前に線を横切っており（黒17〜21サンプル）、
    // 40度のゲートがその線を握りつぶして120度まで空回りしていた。ゲートを外して最初の交差で拾う。
    // 掴んだのがコーナー手前の線でも正味70度に届かずREACQUIREDになるので、Tracerが進んでから再挑戦になる
    constexpr float kCornerPivotReturnMinTurnDeg = 0.0f;
    // 線がどこにあっても減速済みで到達するよう、ゲートが開く角度から線形に出力を落とす
    constexpr float kCornerPivotTaperStartDeg = 40.0f;  // ゲートとは独立。ゲートを下げても減速の形は変えない
    constexpr float kCornerPivotTaperEndDeg = 100.0f;
    constexpr float kCornerPivotTaperMinRatio = 0.6f;  // 57×0.6≒34。低すぎると動かなくなる
    constexpr float kCornerDoneTurnDeg = 70.0f;        // これだけ回って復帰できたら曲がりきったとみなし、以降の判定を止める
    constexpr float kCornerPivotMaxTurnDeg = 120.0f;   // 暴走を止める保険
    constexpr int kCornerPivotTimeoutLoopCount = 300;  // 保険その2（10ms周期なので3秒）
    constexpr int kCornerSuppressAfterBlueMs = 600;    // 青を跨ぐとき脇の白を踏むため、青の上と直後は判定を止める
    constexpr int kCornerRetryAfterFailMs = 1500;      // 線を見つけられなかったときに次の検知まで置く間隔
    // 線に復帰した後、Tracerが曲がりきるのを待つ上限。周期数ではなく内部時計で測る。
    // 旧実装は150周期で、実周期が約11msだったため実際には約1650msだった
    constexpr int kCornerConfirmTimeoutMs = 1500;
    constexpr int kCornerSuppressAfterBlueCount = (kCornerSuppressAfterBlueMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    constexpr int kCornerRetryAfterFailCount = (kCornerRetryAfterFailMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

    // ── エリア配置 ────────────────────────────────────
    constexpr int kDiagonalPwmHigh = 85;  // 斜め移動の外輪PWM
    // 内輪PWM。旋回半径 R = TREAD/2 * (外輪+内輪)/(外輪-内輪) なので、上げるほど弧が大きくなり
    // 同じ回頭量でも進む距離が伸びる。回頭量を3等分して段階的に切り替える
    constexpr int kAreaDiagonalInnerPwms[] = { 5, 10, 55 };
    constexpr float kDiagonalTurnDeg = 80.0f;  // 斜め移動での回頭量[度]
    // 斜め移動の出だし。いきなりkDiagonalPwmHighで動くとボトルが倒れかけるため、立ち上がりだけ段階的に上げる
    constexpr int kDiagonalRampPwms[] = { 50, 70 };
    constexpr int kDiagonalRampStageMs = 90;

    // 惰性ぶんの行き過ぎは実測で12mm（間引きブレーキで止め切った後）。指令60mmで実効72mmだった。
    // ブレーキで行き過ぎが読めるようになったので、指令そのものを増やして余裕を取る
    constexpr int kAreaBackwardMm = -75;                // ボトルから抜ける後退量（実効で約87mm）
    constexpr int kAreaBackwardSpeedDegPerSec = 10000;  // 常に飽和させて最速で後退（pbio側でモーターの上限にクランプされる）
    // 車体が浮くのは速度ではなく立ち上がりのトルクが原因なので、必要ならデューティ上限でトルクの頭を押さえる（100で無効）
    // 後退の指令速度は常に飽和させているので、立ち上がりでモーターが全トルクを出す。
    // 実測のモーター速度は後退直後で-667/-716°/秒。ここが車体が浮く原因なので、デューティ上限で頭を押さえる。
    // 100で無効。浮きが残るなら下げる／60mmに時間が掛かりすぎるなら上げる
    constexpr int kAreaBackwardDutyLimit = 60;

    // 惰性を残したまま逆を指令すると初速が出ないため、後退前に止め切る
    constexpr int kSettleSpeedDegPerSec = 60;  // これ未満なら止まったとみなす
    constexpr int kSettleTimeoutMs = 400;
    // brake()は二択で全掛けだと車体が煽られるため、周期の一部だけ掛けて平均の制動力を落とす
    constexpr int kSettleBrakeCycleLoops = 3;  // 制動の1周期（30ms）
    constexpr int kSettleBrakeOnLoops = 1;     // そのうちブレーキを掛ける回数。0で完全なcoast、周期数と同じで全掛け

    constexpr float kAreaTurnDeg = 70.0f;  // 後退後、帰りの線を探す向きへの旋回量
    constexpr int kAreaTurnPwm = 70;       // 両輪逆転のその場旋回PWM（ボトル配置済みなのでジャークは気にしない）
    // IMUが動かなくなった場合に回り続けるのを防ぐ保険。実測は斜め移動600ms・その場旋回308ms
    constexpr int kDiagonalTimeoutMs = 3000;
    constexpr int kAreaTurnTimeoutMs = 2000;

    // ── 帰りの線探し ──────────────────────────────────
    // 色判定ではなく反射率で判定する（TRACER_TARGET_REFLECTIONとCOLOR_ACHROMATIC_REFLECTION_THRESHOLDが
    // 同値のため、ライントレース中の白黒判定が最も不安定になるため）
    constexpr int kSearchReflectionThreshold = 55;
    constexpr int kSearchPwm = 65;
    constexpr float kSearchMaxTurnDeg = 180.0f;   // これ以上回っても線から離れるだけなので打ち切る
    constexpr int kSearchTimeoutLoopCount = 500;  // 保険（10ms周期なので5秒）
    // 発見後の固定30/90・100ms踏み込みは廃止。反射率を見ながら接続する。

    // 往路の詳細ログを出す位置。falseが旧位置（最終青→terminate→往路診断→斜め移動）。
    // 良好だった版はログ出力中にcoastしてから配置へ進んでいた。今は最終青検知と斜め移動開始が
    // 同時刻で、実測の開始速度が423/412 deg/sある。この初期速度・位置の違いは配置後の姿勢へ
    // 波及し得るので、姿勢の再現を優先して旧位置に戻す。食い込みの原因と確定したわけではない。
    // ログ保護（欠落対策付きの待ち）は通したままなので、待ち時間は旧版と一致しない。
    // 詳細ログが増えた分と通信状況で変わる
    constexpr bool kDeferOutboundDiagnostics = false;

    // 帰還後に出すために、往路終了時点の値を確定して持っておく。
    // 後から現在距離・現在時刻を読んで復元しない（車輪カウントは配置でリセットされる）
    struct OutboundSummary {
        bool pending = false;
        int areaExpectedBlueMm = 0, areaSlowdownStartMm = 0, areaSlowActualMm = -1, finalBlueActualMm = -1;
        int areaSearchStartMm = 0, areaGateOpenedMm = -1, areaFirstRawBlueMm = -1, areaMaxMm = 0;
        int slowToFinalMm = -1, slowToFinalMs = -1;
        int curveSlowMm = 0, curveSlowMs = 0, outboundElapsedMs = 0;
        int detectedBlueCount = 0, targetBlueLineCount = 0, cornerDone = 0, aborted = 0;
        int maxWhiteRunBeforeTrigger = 0;
        int cornerDoneReason = 0, cornerDoneTurnDeg = 0, cornerConfirmedWhileSuppressed = 0, cornerConfirmExpired = 0;
    };
    OutboundSummary outboundSummary;

    // 最終青が成立した瞬間と、斜め移動を始める瞬間。ログ待ちの惰性が配置位置に効いていたかを見る。
    // 往路の詳細ログとは別に持ち、kDeferOutboundDiagnosticsの値に関わらず取って出す
    struct PlacementSummary {
        bool pending = false;
        int finalBlueMs = -1, finalBlueMm = 0, finalBlueLeftSpeed = 0, finalBlueRightSpeed = 0, finalBlueHeading10 = 0;
        int diagonalStartMs = -1, diagonalStartMm = 0, diagonalStartLeftSpeed = 0, diagonalStartRightSpeed = 0, diagonalStartHeading10 = 0;
    };
    PlacementSummary placementSummary;

    // コーナーを曲がりきって設定を切り替えた直後の、最初の制御の内訳。
    // 経路別の引き継ぎ（restart か keep-state か）が効いているかを見る。往路・復路で1件ずつ
    struct PostCornerSample {
        bool armed = false, captured = false;
        const char* label = "";
        int reason = 0;  // 1=ピボットから直接 2=Tracerの追従中
        int pwm = 0;     // 切り替えた基準PWM。走行中にsyslogせず、ここへ入れて停止後に出す
        int p100 = 0, i100 = 0, d100 = 0;
        int leftPwm = 0, rightPwm = 0;
        int ms = 0;
        // コーナー完了から最初のTracer出力までの内部時刻差。走行中の同期処理が制御を
        // どれだけ遅らせたかを、CSVの受信時刻ではなく内部時計で見るため
        SYSTIM doneUs = 0;
        int gapToFirstControlUs = -1;
    };
    PostCornerSample postCornerSamples[2];
    bool postCornerSamplesPrinted = false;
    // コーナーが完了した瞬間の内部時刻。SYSTIMはヘッダから見えないのでここで持つ。
    // 完了した周期と同じ周期でarmPostCornerSample()が消費するので、行きと帰りで取り違えない
    SYSTIM cornerDoneUs = 0;

    // 接続の診断。成功時は走りながら貯めて、止まった後でまとめて出す。
    // タスクスタック(4KiB)を食わないようここに置く。走行ごとにrun()の先頭で初期化する
    TraceEntryLogBuffer<4> traceEntryLog;
    int traceEntryPendingIndex = -1;  // 高速側の最初の制御を待っている記録
    SYSTIM traceEntryLastControl = 0;  // その記録の、低速側の最後の制御の時刻

    // 診断ログ用。区間ごとの所要時間を出すため
    int nowMs() {
        SYSTIM now;
        get_tim(&now);
        return static_cast<int>(now / 1000);
    }

}  // namespace


namespace {
// 診断は固定容量。走行中は集計のみ、出力は停止後。
struct DeliveryTraceLog {
    struct Part {
        int n=0, clips=0, white=0, black=0, maxWhite=0, maxBlack=0;
        float err=0, turn=0, lo=0, hi=0;
        void add(Tracer& t, float kp, float heading) {
            const float e=t.getLastP()/kp;
            if(n==0) lo=hi=heading;
            ++n; err+=std::fabs(e);
            turn+=std::fabs(t.getLastP()+t.getLastI()+t.getLastD());
            if(heading<lo)lo=heading;
            if(heading>hi)hi=heading;
            if(std::abs(t.getLastLeftPwm())>100 || std::abs(t.getLastRightPwm())>100)++clips;
            float r=Config::TRACER_TARGET_REFLECTION-e;
            white=r>=93?white+1:0; black=r<=27?black+1:0;
            if(white>maxWhite)maxWhite=white;
            if(black>maxBlack)maxBlack=black;
        }
void print(const char* name, const char* section) const {
            if(!n)return;
            stoppedDiagnosticLog(LOG_NOTICE,"[DTrace] %s/%s n %d err100 %d turn100 %d",name,section,n,(int)(100*err/n),(int)(100*turn/n));
            stoppedDiagnosticLog(LOG_NOTICE,"[DTrace] %s/%s clips %d white %d black %d",name,section,clips,maxWhite,maxBlack);
            stoppedDiagnosticLog(LOG_NOTICE,"[DTrace] %s/%s heading10 min %d max %d",name,section,(int)(lo*10),(int)(hi*10));
        }
    };
    const char* name;
    int startMs=0,startMm=0,endMs=0,endMm=0,entryMs=-1,entryMm=0;
    float startHeading=0;
    bool active=false, started=false;
    Part entry, rest;
    explicit DeliveryTraceLog(const char* label):name(label){}
    void begin(int ms,int mm,float heading) {
        startMs=endMs=ms; startMm=endMm=mm; startHeading=heading;
        active=started=true;
    }
    void end(int ms,int mm) { if(active){endMs=ms;endMm=mm;active=false;} }
    void sample(Tracer& t,float kp,int ms,int mm,float heading) {
        if(!active)return;
        endMs=ms; endMm=mm;
        if(mm-startMm>=150 && entryMs<0){entryMs=ms;entryMm=mm;}
        (entryMs<0?entry:rest).add(t,kp,heading-startHeading);
    }
void print() const {
        if(!started)return;
        stoppedDiagnosticLog(LOG_NOTICE,"[DTrace] %s total %d mm %d ms startHeading10 %d",name,endMm-startMm,endMs-startMs,(int)(startHeading*10));
        if(entryMs>=0) {
            stoppedDiagnosticLog(LOG_NOTICE,"[DTrace] %s entry %d mm %d ms",name,entryMm-startMm,entryMs-startMs);
            stoppedDiagnosticLog(LOG_NOTICE,"[DTrace] %s rest %d mm %d ms",name,endMm-entryMm,endMs-entryMs);
        }
        entry.print(name,"entry");rest.print(name,"rest");
    }
};

    // 往路の区間ログ。帰還後に出すので、run()のローカルではなくここに置く。
    // 走行ごとにrun()の先頭で作り直す
    DeliveryTraceLog curveLog("curve"), outLog("blue1-to-corner"), areaLog("corner-to-area");
    DeliveryTraceLog areaApproachLog("area-slow");
}

DeliveryTask::DeliveryTask(Robot& robot)
    : robot(robot) {
}

// ボトルの色判定。ColorJudge::judge()は反射率20未満または明度30未満だとHueを信用せず無彩色に倒すが、
// これは黒ラインを青と誤検知しないための対策で、離れたボトルを読むこの場面には合わない
// （実測の反射率2〜3・明度7〜9は常にこの条件に掛かり、必ず黒＝UNKNOWNになってしまう）。
// ここは彩度とHueだけで判定し、候補もデリバリーで使う赤・黄・青の3色に絞る
ColorJudge::Color DeliveryTask::judgeBottleColor() const {
    ColorJudge::Reading reading = robot.getColorReading();

    const ColorJudge::Color candidates[] = { ColorJudge::Color::RED, ColorJudge::Color::YELLOW, ColorJudge::Color::BLUE };
    const int candidateHues[] = { Config::COLOR_RED_HUE, Config::COLOR_YELLOW_HUE, Config::COLOR_BLUE_HUE };
    const int candidateCount = static_cast<int>(sizeof(candidateHues) / sizeof(candidateHues[0]));

    int minDist = 360;
    ColorJudge::Color nearest = ColorJudge::Color::UNKNOWN;
    for(int i = 0; i < candidateCount; i++) {
        // Hueは円環なので最短距離で比べる
        int diff = std::abs(static_cast<int>(reading.hsv.h) - candidateHues[i]);
        if(diff > 180) {
            diff = 360 - diff;
        }
        if(diff < minDist) {
            minDist = diff;
            nearest = candidates[i];
        }
    }

    lastBottleReading = reading;  // 確定後にまとめてログへ出すために持っておく

    if(reading.hsv.s < kBottleMinSaturation || minDist > Config::COLOR_HUE_TOLERANCE) {
        return ColorJudge::Color::UNKNOWN;
    }
    return nearest;
}


// raiseArm/lowerArmはエンコーダ原点を動作ごとに作り直すので、その前に積算しておく
void DeliveryTask::commitArmMovement() {
    armAbsoluteBaseDeg += robot.getArmCount();
}

// 1回目の上げ直前を原点とした、アームの積算角度[度]（負が上げ側）
int DeliveryTask::armAbsoluteDeg() const {
    return armAbsoluteBaseDeg + robot.getArmCount();
}

// 診断: 車体の姿勢とアーム角度。静止していれば加速度は重力の分解なので、車体の傾きが読める
void DeliveryTask::logPosture(const char* label) {
    IMU::Acceleration accel = robot.getImuAcceleration();
    IMU::AngularVelocity ang = robot.getImuAngularVelocity();
    syslog(LOG_NOTICE, "[Posture] %s: accel %d/%d/%d mm/s2, stationary %s", label, (int)accel.x, (int)accel.y, (int)accel.z, robot.isImuStationary() ? "yes" : "no");
    syslog(LOG_NOTICE, "[Posture] %s: gyro %d/%d/%d deg/s, arm %d deg", label, (int)ang.x, (int)ang.y, (int)ang.z, std::abs(robot.getArmCount()));
    syslog(LOG_NOTICE, "[Posture] %s: arm power %d %%, stalled %s, abs %d deg", label, robot.getArmPower(), robot.isArmStalled() ? "YES" : "no", armAbsoluteDeg());
}

// 車体を止め切ってから、揺れが収まるのを待つ。アームを上げる前に使う
void DeliveryTask::stopAndSettleBeforeArm() {
    // アームの前は、時間切れでは止めない（元の挙動）。中断のときだけabortedが立つ。
    // 止まり切らないまま上げると読む向きがずれるが、ここで打ち切ると読めるはずの回まで落とすため
    const SettleOutcome outcome = brakeUntilStopped(kBottleStopSpeedDegPerSec, kBottleStopTimeoutMs, "before arm", false);
    if(outcome.result == SettleResult::TIMEOUT) {
        syslog(LOG_NOTICE, "[Settle] before arm: not settled, continuing anyway");
    }
    robot.stop();
    if(aborted) {
        return;
    }
    const int settleStartMs = nowMs();
    const int minLoops = (kBottleSettleMinMs * 1000) / Config::MOTION_POLL_INTERVAL_US;
    const int maxLoops = (kBottleSettleMaxMs * 1000) / Config::MOTION_POLL_INTERVAL_US;
    const char* reason = "timeout";
    int stableCount = 0;
    for(int i = 0; i < maxLoops; i++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            return;
        }
        IMU::AngularVelocity ang = robot.getImuAngularVelocity();
        const bool quiet = std::abs(ang.x) < kBottleSettleGyroDegPerSec
                           && std::abs(ang.y) < kBottleSettleGyroDegPerSec
                           && std::abs(ang.z) < kBottleSettleGyroDegPerSec;
        stableCount = quiet ? stableCount + 1 : 0;
        if(i >= minLoops && stableCount >= kBottleSettleStableCount) {
            reason = "gyro quiet";
            break;
        }
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }
    syslog(LOG_NOTICE, "Settled before arm raise: %d ms (%s), imu stationary %s", nowMs() - settleStartMs, reason, robot.isImuStationary() ? "yes" : "no");
}


// アームを上げた直後に色を読む一式。目標角まで待つ→整定待ち→角度と生値のログ→確定
ColorJudge::Color DeliveryTask::readBottleColorAfterRaise(int targetDeg) {
    // positionDeliveryArmが位置・速度の連続成立を確認済み。
    (void)targetDeg;
    dly_tsk(kArmSettleBeforeColorMs * 1000);
    logPosture("at color reading");
    // 診断: 色を読む瞬間のアーム角度。raiseArm()が止めた角度と比べ、保持中に動いていないかを見る
    syslog(LOG_NOTICE, "Arm position at color reading: count %d relative %d deg",
           robot.getArmCount(), armAbsoluteDeg());

    ColorJudge::Color color = confirmBottleColor();
    syslog(LOG_NOTICE, "Bottle reading: h=%d s=%d v=%d refl=%d", (int)lastBottleReading.hsv.h, (int)lastBottleReading.hsv.s, (int)lastBottleReading.hsv.v, lastBottleReading.reflection);
    return color;
}

// ボトル色を連続一致で確定させる。決まらなければUNKNOWNを返す
// 自分が鳴らした音だけを操作する。ここからsyslogは出さない（周期を乱さないため）
void DeliveryTask::applyNotifierAction(ColorNotifier::Action action) {
    if(action == ColorNotifier::Action::START_TONE) {
        robot.startBeepNonBlocking();
    } else if(action == ColorNotifier::Action::STOP_TONE) {
        robot.stopBeep();
    }
}

void DeliveryTask::logColorNotify(const ColorNotifier& notifier, int plannedCount, int startMs, const char* reason,
                                  int maxIntervalMs, int loopCount) {
    syslog(LOG_NOTICE, "[Notify] mode %d planned %d started %d reason %s (%d ms)",
           (int)kColorNotifyMode, plannedCount, notifier.getStartedCount(), reason, nowMs() - startMs);
    // 音あり／なしで比べるため、この区間の制御間隔の最大値を残す（狙いは約10ms）
    syslog(LOG_NOTICE, "[Notify] max control interval %d ms in %d loops", maxIntervalMs, loopCount);
}

ColorJudge::Color DeliveryTask::confirmBottleColor() {
    const int intervalUs = kBottleColorSampleIntervalMs * 1000;
    const int maxSamples = kBottleColorTimeoutMs / kBottleColorSampleIntervalMs;

    ColorJudge::Color stableColor = ColorJudge::Color::UNKNOWN;
    int matchedCount = 0;

    for(int i = 0; i < maxSamples; i++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            return ColorJudge::Color::UNKNOWN;
        }

        ColorJudge::Color sampled = judgeBottleColor();
        if(sampled != ColorJudge::Color::UNKNOWN && sampled == stableColor) {
            matchedCount++;
            if(matchedCount >= kBottleColorStableCount) {
                return stableColor;
            }
        } else {
            stableColor = sampled;
            matchedCount = 1;
        }
        dly_tsk(intervalUs);
    }
    return ColorJudge::Color::UNKNOWN;
}

// 青ラインかどうかの1サンプル判定。ColorJudge::judge()をそのまま使うと、ライントレースが追う
// 白黒の境界（グラデーション帯）を青と誤判定する。明度95〜100・反射率37〜58と明るいため、
// ColorJudgeの暗所ガード（黒を青と誤検知しない対策）には掛からない
bool DeliveryTask::isBlueReading(BlueStats& stats) const {
    ColorJudge::Reading reading = robot.getColorReading();

    int hueDiff = std::abs(static_cast<int>(reading.hsv.h) - Config::COLOR_BLUE_HUE);
    if(hueDiff > 180) {
        hueDiff = 360 - hueDiff;
    }
    bool inBlueHue = (hueDiff <= Config::COLOR_HUE_TOLERANCE);
    bool isBlue = inBlueHue && (reading.hsv.s >= kBlueLineMinSaturation);

    // 診断。彩度が惜しかったのか、そもそも青の色相すら見えていないのかを後から切り分ける
    if(inBlueHue && reading.hsv.s > stats.maxSaturationInHue) {
        stats.maxSaturationInHue = reading.hsv.s;
    }
    if(isBlue) {
        if(stats.currentRun==0) {
            if(stats.spanCount<24) {
                auto& span=stats.spans[stats.spanCount++];
                span.ms=nowMs();span.mm=wheelDistanceMm();
                span.armed=stats.entryArmed?1:0;span.required=stats.required;
                span.nextBlue=stats.nextBlue;span.pwm=stats.commandPwm;span.gapBefore=stats.nonBlueRun;
                span.speed=(robot.getLeftMotor().getSpeed()+robot.getRightMotor().getSpeed())/2;
            } else ++stats.dropped;
        }
        if(stats.dropped==0 && stats.spanCount>0) {
            auto& span=stats.spans[stats.spanCount-1];
            ++span.count;span.duration=nowMs()-span.ms;span.distance=wheelDistanceMm()-span.mm;
            span.endSpeed=(robot.getLeftMotor().getSpeed()+robot.getRightMotor().getSpeed())/2;
            if(reading.hsv.s>span.maxSat)span.maxSat=reading.hsv.s;
        }
        stats.satisfiedCount++;
        stats.nonBlueRun=0;
        stats.currentRun++;
        if(stats.currentRun > stats.maxConsecutive) {
            stats.maxConsecutive = stats.currentRun;
        }
    } else {
        stats.currentRun = 0;
        ++stats.nonBlueRun;
    }
    return isBlue;
}

// Robot::isOnColors()と同じ「連続一致回数」方式。カウンタは呼び出し側で持つ
bool DeliveryTask::isOnBlueLine(int& matchedCount, int stableCount, BlueStats& stats) const {
    if(isBlueReading(stats)) {
        matchedCount++;
    } else {
        matchedCount = 0;
    }
    return (matchedCount >= stableCount);
}

void DeliveryTask::logBlueStats(const char* label, const BlueStats& stats) const {
    stoppedDiagnosticLog(LOG_NOTICE,"[BlueRuns] %s stored %d dropped %d entryNeed %d finalNeed %d",label,stats.spanCount,stats.dropped,
           kBlueEntryConfirmMs*1000/Config::LINE_TRACE_POLL_INTERVAL_US,kBlueFinalEntryConfirmMs*1000/Config::LINE_TRACE_POLL_INTERVAL_US);
    for(int i=0;i<stats.spanCount;++i) {
        const auto& p=stats.spans[i];
        stoppedDiagnosticLog(LOG_NOTICE,"[BlueRun] %s #%d t=%d mm=%d n=%d",label,i,p.ms,p.mm,p.count);
        stoppedDiagnosticLog(LOG_NOTICE,"[BlueRun] spanMs %d spanMm %d maxSat %d",p.duration,p.distance,p.maxSat);
        stoppedDiagnosticLog(LOG_NOTICE,"[BlueGate] next %d armed %d need %d pwm %d speedDegSec %d",
               p.nextBlue,p.armed,p.required,p.pwm,p.speed);
        stoppedDiagnosticLog(LOG_NOTICE,"[BlueGate] nonBlueGapBefore %d (passNeed %d)",
               p.gapBefore,kBluePassedConfirmMs*1000/Config::LINE_TRACE_POLL_INTERVAL_US);
        stoppedDiagnosticLog(LOG_NOTICE,"[BluePosition] relativeStart %d relativeEnd %d speedStart %d speedEnd %d",
               p.mm-stats.distanceOriginMm,p.mm+p.distance-stats.distanceOriginMm,p.speed,p.endSpeed);
    }
    stoppedDiagnosticLog(LOG_NOTICE, "Blue stats[%s]: maxSat %d, satisfied %d, maxRun %d", label, stats.maxSaturationInHue, stats.satisfiedCount, stats.maxConsecutive);
}

// 左右のパワー差で弧を描きながら、startHeadingからturnDeg回頭したら停止する。
// 内輪は回頭の進み具合で段階的に上げる（時間ではなく角度基準なので、速度がばらついても弧の形が変わらない）
void DeliveryTask::diagonalMoveUntilImuTurn(bool isOuterLeft, int outerPwm, float startHeading, float turnDeg) {
    const int rampStageLoopCount = (kDiagonalRampStageMs * 1000) / Config::MOTION_POLL_INTERVAL_US;
    const int rampStageCount = static_cast<int>(sizeof(kDiagonalRampPwms) / sizeof(kDiagonalRampPwms[0]));
    const int innerStageCount = static_cast<int>(sizeof(kAreaDiagonalInnerPwms) / sizeof(kAreaDiagonalInnerPwms[0]));

    const int timeoutLoopCount = (kDiagonalTimeoutMs * 1000) / Config::MOTION_POLL_INTERVAL_US;

    int loopCount = 0;
    float turnedDeg = 0.0f;
    while(turnedDeg < turnDeg) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            break;
        }
        if(loopCount >= timeoutLoopCount) {
            syslog(LOG_NOTICE, "STOP[diagonalMoveUntilImuTurn]: TIMEOUT");
            aborted = true;
            break;
        }

        int innerStage = static_cast<int>(turnedDeg / turnDeg * innerStageCount);
        if(innerStage >= innerStageCount) {
            innerStage = innerStageCount - 1;
        }
        int appliedOuterPwm = outerPwm;
        int appliedInnerPwm = kAreaDiagonalInnerPwms[innerStage];

        // 出だしだけ出力を抑える。左右の比を保ったまま縮めるので、描く弧は変わらない
        int rampStage = loopCount / rampStageLoopCount;
        if(rampStage < rampStageCount) {
            appliedOuterPwm = appliedOuterPwm * kDiagonalRampPwms[rampStage] / kDiagonalPwmHigh;
            appliedInnerPwm = appliedInnerPwm * kDiagonalRampPwms[rampStage] / kDiagonalPwmHigh;
        }

        robot.setMotorPower(isOuterLeft ? appliedOuterPwm : appliedInnerPwm,
                            isOuterLeft ? appliedInnerPwm : appliedOuterPwm);
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        loopCount++;
        turnedDeg = std::fabs(robot.getImuHeading() - startHeading);
    }
    robot.stop();
    syslog(LOG_NOTICE, "Diagonal done: %d deg in %dms", (int)turnedDeg, loopCount * Config::MOTION_POLL_INTERVAL_US / 1000);
}

// 進行方向を変える前に、ブレーキで速度を落とし切る。惰性が残ったまま逆を指令すると
// 減速と加速が同じ制御に混ざり、初速が出ない（実測で後退に785ms掛かっていた）
DeliveryTask::SettleOutcome DeliveryTask::brakeUntilStopped(int speedThresholdDegPerSec, int timeoutMs, const char* label, bool abortOnTimeout) {
    SettleOutcome outcome;
    // 既に中断しているならブレーキループごと飛ばす。空振りの時間を積まない
    if(aborted) {
        outcome.result = SettleResult::CANCELLED;
        logSettleOutcome(label, outcome);
        return outcome;
    }
    const SettleGate gate{ speedThresholdDegPerSec,
                           (timeoutMs * 1000) / Config::MOTION_POLL_INTERVAL_US,
                           kSettleBrakeCycleLoops, kSettleBrakeOnLoops };
    for(int i = 0; i < gate.timeoutLoops; i++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            outcome.result = SettleResult::CANCELLED;
            outcome.elapsedMs = i * Config::MOTION_POLL_INTERVAL_US / 1000;
            releaseDriveAfterFailedSettle(outcome);
            logSettleOutcome(label, outcome);
            return outcome;
        }

        // 掛けっぱなしにせず間引くことで、制動力を平均で下げる。
        // Robot::brake()は「完全停止まで待つ」仕様に変わった（commit 242be65）ため使えない。
        // 1回目の呼び出しで止め切ってしまい、間引く意味がなくなって全掛けと同じになる
        if(gate.shouldBrake(i)) {
            robot.getLeftMotor().brake();
            robot.getRightMotor().brake();
        } else {
            robot.stop();
        }
        // Robotは速度を公開していないため、モーターのgetterから直接読む
        outcome.leftSpeed = robot.getLeftMotor().getSpeed();
        outcome.rightSpeed = robot.getRightMotor().getSpeed();
        if(gate.isStopped(outcome.leftSpeed, outcome.rightSpeed)) {
            // 成功。出力状態はこの周期のまま触らない（停止方式の変更を混ぜない）
            outcome.result = SettleResult::STOPPED;
            outcome.elapsedMs = i * Config::MOTION_POLL_INTERVAL_US / 1000;
            logSettleOutcome(label, outcome);
            return outcome;
        }
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }
    outcome.result = SettleResult::TIMEOUT;
    outcome.elapsedMs = timeoutMs;
    // 止まり切っていないまま次の動作へ進ませない。ただし呼び出し側が明示的に許す場所もある
    if(abortOnTimeout) aborted = true;
    releaseDriveAfterFailedSettle(outcome);
    logSettleOutcome(label, outcome);
    return outcome;
}

// 整定に失敗したときの後始末。駆動だけ解除し、止まるまで待つ処理は足さない
// （惰性ゼロまでは保証しない。再加速させないことが最低限の契約）
void DeliveryTask::releaseDriveAfterFailedSettle(SettleOutcome& outcome) {
    robot.getLeftMotor().brake();
    robot.getRightMotor().brake();
    outcome.leftSpeed = robot.getLeftMotor().getSpeed();
    outcome.rightSpeed = robot.getRightMotor().getSpeed();
}

void DeliveryTask::logSettleOutcome(const char* label, const SettleOutcome& outcome) {
    const char* name = outcome.result == SettleResult::STOPPED ? "STOPPED"
                       : outcome.result == SettleResult::TIMEOUT ? "TIMEOUT"
                                                                 : "CANCELLED";
    syslog(LOG_NOTICE, "[Settle] %s %s in %d ms (speed %d/%d)",
           label, name, outcome.elapsedMs, outcome.leftSpeed, outcome.rightSpeed);
}

// デューティ上限（＝トルク上限）を落として直進/後退する。
// 戻し忘れると以降のライントレースまで非力になるため、必ずこの関数内で復帰させる
int DeliveryTask::driveStraightWithDutyLimit(int distanceMm, int speedDegPerSec, int dutyLimit) {
    int oldLeftLimit = robot.getLeftMotor().setDutyLimit(dutyLimit);
    int oldRightLimit = robot.getRightMotor().setDutyLimit(dutyLimit);

    const int traveled = robot.driveStraight(distanceMm, speedDegPerSec);
    if(robot.isCenterButtonPressed() || std::abs(traveled) < std::abs(distanceMm)) aborted=true;

    robot.getLeftMotor().restoreDutyLimit(oldLeftLimit);
    robot.getRightMotor().restoreDutyLimit(oldRightLimit);
    return traveled;
}

// 両輪を逆向きに回してその場で旋回する。turnByImu(閉ループ)より速く回せる
void DeliveryTask::turnInPlaceByImu(int leftPwm, int rightPwm, float turnDeg) {
    const int timeoutLoopCount = (kAreaTurnTimeoutMs * 1000) / Config::MOTION_POLL_INTERVAL_US;

    float startHeading = robot.getImuHeading();
    const int startedMs = nowMs();
    float peakX=0,peakY=0,peakZ=0;
    int loopCount = 0;
    while(std::fabs(robot.getImuHeading() - startHeading) < turnDeg) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            break;
        }
        if(loopCount >= timeoutLoopCount) {
            syslog(LOG_NOTICE, "STOP[turnInPlaceByImu]: TIMEOUT");
            aborted = true;
            break;
        }
        // 発進80msを立ち上げ、終端20度で落とす。70度/最大PWM70は維持。
        const float turned = std::fabs(robot.getImuHeading()-startHeading);
        const float ramp = std::fmin(1.0f, 0.6f + 0.4f*(nowMs()-startedMs)/80.0f);
        const float taper = std::fmin(1.0f, 0.6f + 0.4f*(turnDeg-turned)/20.0f);
        const float ratio = std::fmin(ramp,taper);
        const auto gyro=robot.getImuAngularVelocity();
        peakX=std::fmax(peakX,std::fabs(gyro.x));
        peakY=std::fmax(peakY,std::fabs(gyro.y));
        peakZ=std::fmax(peakZ,std::fabs(gyro.z));
        robot.setMotorPower((int)(leftPwm*ratio), (int)(rightPwm*ratio));
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        loopCount++;
    }
    robot.stop();
    const float commandTurn=robot.getImuHeading()-startHeading;
    // 次の片輪探索に残留回転を持ち込まない。共有のブロッキングbrakeは使わない。
    // 成否はbrakeUntilStopped()が返すので、ここで速度を読み直す二重確認はしない
    const SettleOutcome settle=brakeUntilStopped(kSettleSpeedDegPerSec,kSettleTimeoutMs,"area turn");
    if(settle.ok()) robot.stop();
    syslog(LOG_NOTICE,"[AreaTurn] command10 %d settled10 %d ms %d",
           (int)(commandTurn*10),(int)((robot.getImuHeading()-startHeading)*10),nowMs()-startedMs);
    syslog(LOG_NOTICE,"[AreaTurn] peakGyro xyz %d/%d/%d",(int)peakX,(int)peakY,(int)peakZ);
}

bool DeliveryTask::acquireTraceEntry(Tracer& tracer, const char* label) {
    tracer.setConfig(0.30f,0.01f,0.02f,Config::TRACER_TARGET_REFLECTION,65);
    tracer.setCurveDecelGain(2.3f);
    tracer.resetPid();
    const int startMs=nowMs(), startMm=wheelDistanceMm();
    const float startHeading=robot.getImuHeading();
    int stable=0, firstReflection=-1, lastReflection=-1;
    float firstP=0.0f, firstI=0.0f, firstD=0.0f;
    bool sampled=false;
    SYSTIM lastControl=0;
    const char* result="timeout";
    bool ok=false;
    while(nowMs()-startMs < 1500) {
        if(robot.isCenterButtonPressed()) { result="button";break; }
        const int mm=wheelDistanceMm()-startMm;
        if(mm>=150) {result="distance limit";break;}
        if(std::fabs(robot.getImuHeading()-startHeading)>30) {result="heading limit";break;}
        tracer.run();
        get_tim(&lastControl);
        if(!sampled) {
            sampled=true;
            firstP=tracer.getLastP(); firstI=tracer.getLastI(); firstD=tracer.getLastD();
        }
        lastReflection=(int)std::lround(Config::TRACER_TARGET_REFLECTION-tracer.getLastP()/0.30f);
        if(firstReflection<0)firstReflection=lastReflection;
        stable=std::abs(lastReflection-Config::TRACER_TARGET_REFLECTION)<=15?stable+1:0;
        if(mm>=20 && stable>=8) {ok=true;result="stable";break;}
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }
    const int mm=wheelDistanceMm()-startMm, ms=nowMs()-startMs;
    const int heading10=(int)((robot.getImuHeading()-startHeading)*10);
    // 成否に関わらず、必ず止めてから返す。呼び出し側は止まった状態で高速設定へ進む
    robot.stop();
    // 診断は貯めるだけ。出すのは走り終えてから（ここで待つと接続のたびに走行が止まる）
    recordTraceEntry(label,result,mm,ms,heading10,firstReflection,lastReflection,stable,
                     firstP,firstI,firstD,true);
    traceEntryLastControl=lastControl;
    if(!ok) {
        aborted=true;
        printTraceEntryRecords();  // 中断するので、ここで出しておく
    }
    return ok;
}

// deferredなら貯めるだけ。そうでなければ従来どおりその場で出す
void DeliveryTask::recordTraceEntry(const char* label, const char* result, int mm, int ms, int heading10,
                                    int firstReflection, int lastReflection, int stable,
                                    float firstP, float firstI, float firstD, bool deferred) {
    TraceEntryRecord record;
    record.label=label; record.result=result;
    record.mm=mm; record.ms=ms; record.heading10=heading10;
    record.firstReflection=firstReflection; record.lastReflection=lastReflection; record.stable=stable;
    record.firstP100=(int)std::lround(firstP*100); record.firstI100=(int)std::lround(firstI*100);
    record.firstD100=(int)std::lround(firstD*100);
    if(!deferred) {
        syslog(LOG_NOTICE,"[Handoff] %s %s mm %d ms %d heading10 %d",label,result,mm,ms,heading10);
        syslog(LOG_NOTICE,"[Handoff] reflection %d -> %d stable %d",firstReflection,lastReflection,stable);
        syslog(LOG_NOTICE,"[Handoff] %s firstPID100 %d / %d / %d",label,record.firstP100,record.firstI100,record.firstD100);
        return;
    }
    traceEntryPendingIndex=traceEntryLog.add(record);
}

// 高速側の最初の制御の直後に呼ぶ。1回ぶんの時刻差を書き戻すだけで、制御には触らない
void DeliveryTask::noteFirstFastControl() {
    if(traceEntryPendingIndex<0) return;
    SYSTIM now; get_tim(&now);
    traceEntryLog.at(traceEntryPendingIndex).gapToFastUs=(int)(now-traceEntryLastControl);
    traceEntryPendingIndex=-1;
}

void DeliveryTask::printTraceEntryRecords() {
    if(traceEntryLog.count()==0 && traceEntryLog.dropped()==0) return;
    stoppedDiagnosticLog(LOG_NOTICE,"[Handoff] records %d dropped %d (max %d)",
           traceEntryLog.count(),traceEntryLog.dropped(),traceEntryLog.capacity());
    for(int i=0;i<traceEntryLog.count();i++) {
        const TraceEntryRecord& r=traceEntryLog.at(i);
        stoppedDiagnosticLog(LOG_NOTICE,"[Handoff] %s %s mm %d ms %d heading10 %d",r.label,r.result,r.mm,r.ms,r.heading10);
        stoppedDiagnosticLog(LOG_NOTICE,"[Handoff] %s reflection %d -> %d stable %d",r.label,r.firstReflection,r.lastReflection,r.stable);
        stoppedDiagnosticLog(LOG_NOTICE,"[Handoff] %s firstPID100 %d / %d / %d",r.label,r.firstP100,r.firstI100,r.firstD100);
        stoppedDiagnosticLog(LOG_NOTICE,"[Handoff] %s gapToFast %d us",r.label,r.gapToFastUs);
    }
    traceEntryLog.clear();
    traceEntryPendingIndex=-1;
}

// 帰りの線探し。片輪ピボットのまま反射率で線を見つけるまで回し続ける。
// 左右交互の蛇行だと、振り戻しで一度ライン上に来ても行き過ぎて見失うため一方向にした
bool DeliveryTask::pivotUntilReflectionBelow(int reflectionThreshold, int pwm, bool isRightTurn) {
    float startHeading = robot.getImuHeading();
    int loopCount = 0;

    while(std::fabs(robot.getImuHeading() - startHeading) < kSearchMaxTurnDeg) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            robot.stop();
            return false;
        }

        int reflection = robot.getReflection();
        if(reflection < reflectionThreshold) {
            robot.stop();
            syslog(LOG_NOTICE, "Line found by reflection: %d", reflection);

            syslog(LOG_NOTICE,"[Search] found heading10 %d turn10 %d",
                   (int)(robot.getImuHeading()*10),(int)((robot.getImuHeading()-startHeading)*10));
            return true;
        }

        if(loopCount >= kSearchTimeoutLoopCount) {
            syslog(LOG_NOTICE, "STOP[pivotUntilReflectionBelow]: TIMEOUT");
            break;
        }

        robot.setMotorPower(isRightTurn ? pwm : 0, isRightTurn ? 0 : pwm);
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        loopCount++;
    }
    robot.stop();
    syslog(LOG_NOTICE, "STOP[pivotUntilReflectionBelow]: line not found");
    return false;
}

// 直角コーナー用。内輪を落として旋回し、線を見つけた時点で止まる。
// 主の終了条件は反射率（線が正解）。どれだけ行き過ぎたかは毎回変わるので、IMUで固定角度を狙うのではなく
// 「ここまで回っても見つからなければ何かおかしい」の保険としてだけIMU角度とループ回数を使う
bool DeliveryTask::pivotUntilLineFound(bool isLeftTurn, int outerPwm, int innerPwm, float minTurnDeg, float maxTurnDeg, float& turnedDegOut) {
    float startHeading = robot.getImuHeading();
    int blackRun = 0;
    // ゲートが開いた後に「白を踏んでから黒に入った」ことを要求する。
    // ゲート中に線の近くへ居座ったまま開いた瞬間に抜けるのを防ぐ
    // （実測: ゲート40度に対し turned 40 ちょうどや、turned 47・反射率33=グラデーション帯で終了していた。
    //  正しく曲がれたときは白の上を通過してから反射率17〜18の真っ黒で終了している）
    bool seenWhiteAfterGate = false;
    turnedDegOut = 0.0f;

    // 以下は診断専用。制御には一切使わず、旋回が終わったときに一度だけまとめて出す
    // （周期ごとにログを出すと制御周期が伸びるうえ、以前カウンタを制御と共用して挙動を壊したことがある）
    int firstWhiteAfterGateDeg = -1;  // ゲート後に初めて白を読んだ角度
    int firstBlackAfterGateDeg = -1;  // ゲート後に初めて黒判定になった角度（線に触れた場所）
    int minReflectionAfterGate = 101;
    int minReflectionAtDeg = -1;
    int maxBlackRun = 0;  // 成功して抜けた分は含めない（途中で線に触れて連続条件に届かなかったかを見るため）

    for(int loopCount = 0; loopCount < kCornerPivotTimeoutLoopCount; loopCount++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            robot.stop();
            return false;
        }

        float turnedDeg = std::fabs(robot.getImuHeading() - startHeading);
        turnedDegOut = turnedDeg;

        int reflection = robot.getReflection();
        if(reflection <= kCornerBlackReflection) {
            blackRun++;
        } else {
            // 黒が途切れた時点で記録する。成功時はこの分岐を通らずに抜けるので、最大値に混ざらない
            if(blackRun > maxBlackRun) {
                maxBlackRun = blackRun;
            }
            blackRun = 0;
        }
        if(turnedDeg >= minTurnDeg && reflection >= kCornerWhiteReflection) {
            seenWhiteAfterGate = true;
        }

        if(turnedDeg >= minTurnDeg) {
            if(firstWhiteAfterGateDeg < 0 && reflection >= kCornerWhiteReflection) {
                firstWhiteAfterGateDeg = static_cast<int>(turnedDeg);
            }
            if(firstBlackAfterGateDeg < 0 && reflection <= kCornerBlackReflection) {
                firstBlackAfterGateDeg = static_cast<int>(turnedDeg);
            }
            if(reflection < minReflectionAfterGate) {
                minReflectionAfterGate = reflection;
                minReflectionAtDeg = static_cast<int>(turnedDeg);
            }
        }

        if(seenWhiteAfterGate && blackRun >= kCornerPivotBlackRunCount) {
            robot.stop();
            syslog(LOG_NOTICE, "Pivot done: line found (reflection %d, turned %d deg)", reflection, (int)turnedDeg);
            syslog(LOG_NOTICE, "Pivot stats: firstWhite %d deg, firstBlack %d deg, minRefl %d @ %d deg, maxBlackRun %d", firstWhiteAfterGateDeg, firstBlackAfterGateDeg, minReflectionAfterGate, minReflectionAtDeg, maxBlackRun);
            return true;
        }

        if(turnedDeg >= maxTurnDeg) {
            robot.stop();
            syslog(LOG_NOTICE, "STOP[pivotUntilLineFound]: MAX_TURN (reflection %d)", reflection);
            syslog(LOG_NOTICE, "Pivot stats: firstWhite %d deg, firstBlack %d deg, minRefl %d @ %d deg, maxBlackRun %d", firstWhiteAfterGateDeg, firstBlackAfterGateDeg, minReflectionAfterGate, minReflectionAtDeg, maxBlackRun);
            return false;
        }

        // 立ち上がりを緩やかにしてボトルを振らないようにする
        float powerRatio = 1.0f;
        if(loopCount < kCornerPivotRampLoopCount) {
            powerRatio = static_cast<float>(loopCount + 1) / kCornerPivotRampLoopCount;
        }

        // 線を探す区間は線形に落とす。ランプ中と重なる場合は小さい方を採用する。
        // 終端はmaxTurnDegではなく専用の定数にする（逆方向探索でmaxTurnが240度になっても傾きを変えないため）
        if(turnedDeg >= kCornerPivotTaperStartDeg) {
            float taperRatio = 1.0f - (1.0f - kCornerPivotTaperMinRatio) * (turnedDeg - kCornerPivotTaperStartDeg) / (kCornerPivotTaperEndDeg - kCornerPivotTaperStartDeg);
            if(taperRatio < kCornerPivotTaperMinRatio) {
                taperRatio = kCornerPivotTaperMinRatio;
            }
            if(taperRatio < powerRatio) {
                powerRatio = taperRatio;
            }
        }

        int scaledOuterPwm = static_cast<int>(outerPwm * powerRatio);
        int scaledInnerPwm = static_cast<int>(innerPwm * powerRatio);

        // 左旋回なら左が内輪、右旋回なら右が内輪
        robot.setMotorPower(isLeftTurn ? scaledInnerPwm : scaledOuterPwm, isLeftTurn ? scaledOuterPwm : scaledInnerPwm);
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }

    robot.stop();
    syslog(LOG_NOTICE, "STOP[pivotUntilLineFound]: TIMEOUT");
    syslog(LOG_NOTICE, "Pivot stats: firstWhite %d deg, firstBlack %d deg, minRefl %d @ %d deg, maxBlackRun %d", firstWhiteAfterGateDeg, firstBlackAfterGateDeg, minReflectionAfterGate, minReflectionAtDeg, maxBlackRun);
    return false;
}

int DeliveryTask::wheelDistanceMm() const {
    const int avgDeg = (robot.getLeftMotorCount() + robot.getRightMotorCount()) / 2;
    return static_cast<int>(avgDeg * M_PI * Config::WHEEL_RADIUS_MM / 180.0f);
}

// 曲がるべき向きを正とした、startHeadingからの回頭量。getImuHeading()は右回りが正なので、
// 左折なら符号を反転する。逆方向へ回った分はマイナスになる
float DeliveryTask::signedTurnFrom(float startHeading, bool isLeftTurn) const {
    float delta = robot.getImuHeading() - startHeading;
    return isLeftTurn ? -delta : delta;
}

// コーナーを検知した後の旋回。線を捕まえ直し、失敗したら逆へ振り戻す。行き（左折）と帰り（右折）で共用する。
// 完了判定は「正方向の旋回量 - 振り戻し量」ではなく、startHeadingからの実方位差で行う。
// 正方向と振り戻しでは軸にする車輪が変わって旋回中心が別物になるうえ、停止後の惰性も引き算には入らないため
DeliveryTask::CornerResult DeliveryTask::turnAtCorner(bool isLeftTurn, float startHeading, float minTurnDeg) {
    float pivotTurnedDeg = 0.0f;
    bool lineFound = pivotUntilLineFound(isLeftTurn, kCornerPivotOuterPwm, kCornerPivotInnerPwm, minTurnDeg, kCornerPivotMaxTurnDeg, pivotTurnedDeg);

    if(aborted) return CornerResult::FAILED;
    bool sweepBackFoundLine = false;
    if(!lineFound) {
        // 回りすぎて通り過ぎたか、そもそも線が無い方向だった。
        // 逆向きに、開始角度を跨いで反対側まで振り戻して探す（線の位置が不明なので最低旋回角はかけない）
        syslog(LOG_NOTICE, "Pivot failed. Sweeping back the other way.");
        float sweepBackTurnedDeg = 0.0f;
        sweepBackFoundLine = pivotUntilLineFound(!isLeftTurn, kCornerPivotOuterPwm, kCornerPivotInnerPwm, 0.0f, kCornerPivotMaxTurnDeg * 2.0f, sweepBackTurnedDeg);
        syslog(LOG_NOTICE, "Sweep back %s (turned %d deg)", sweepBackFoundLine ? "found line" : "FAILED", (int)sweepBackTurnedDeg);
    }

    if(!lineFound && !sweepBackFoundLine) {
        return CornerResult::FAILED;
    }

    // fabsで測ると、振り戻しが開始角度を越えて逆方向へ大きく回った場合も完了扱いになってしまう。
    // 曲がるべき向きを正とした符号付きの角度で判定する
    float turnedDeg = signedTurnFrom(startHeading, isLeftTurn);
    return (turnedDeg >= kCornerDoneTurnDeg) ? CornerResult::CLEARED : CornerResult::REACQUIRED;
}

// コーナー検知の1周期分。ライントレースのループから毎周期呼ぶ。
// 行き・帰りで状態を別に持つだけで、判定そのものは共通
// 往路の詳細ログ。走りながらではなく、止まっている場所で一度だけ出す。
// 早期returnの出し忘れを防ぐため、run()のガードのデストラクタからも呼ぶ（2回目は何もしない）
void DeliveryTask::printOutboundDiagnostics(BlueStats& beforeCorner, BlueStats& afterCorner) {
    if(!outboundSummary.pending) return;
    outboundSummary.pending = false;

    const OutboundSummary& summary = outboundSummary;
    stoppedDiagnosticLog(LOG_NOTICE,"[AreaBlue] expected %d slowPlan %d slowActual %d finalActual %d",
           summary.areaExpectedBlueMm,summary.areaSlowdownStartMm,summary.areaSlowActualMm,summary.finalBlueActualMm);
    stoppedDiagnosticLog(LOG_NOTICE,"[AreaBlue] searchPlan %d armedAt %d firstCandidate %d max %d",
           summary.areaSearchStartMm,summary.areaGateOpenedMm,summary.areaFirstRawBlueMm,summary.areaMaxMm);
    if(summary.slowToFinalMm>=0)
        stoppedDiagnosticLog(LOG_NOTICE,"[AreaBlue] slowToFinal %d mm %d ms",summary.slowToFinalMm,summary.slowToFinalMs);
    curveLog.print();outLog.print();areaLog.print();
    areaApproachLog.print();
    printTraceEntryRecords();
    stoppedDiagnosticLog(LOG_NOTICE,"[DTrace] curve slowdown at %d mm t=%d; outbound elapsed %d ms",
           summary.curveSlowMm,summary.curveSlowMs,summary.outboundElapsedMs);
    logBlueStats("before corner", beforeCorner);
    logBlueStats("after corner", afterCorner);
    stoppedDiagnosticLog(LOG_NOTICE, "Outbound end: preCornerCount %d, targetSlot %d, finalDetected %d, corner done %d, aborted %d",
           summary.detectedBlueCount, summary.targetBlueLineCount, summary.finalBlueActualMm>=0?1:0,
           summary.cornerDone, summary.aborted);
    stoppedDiagnosticLog(LOG_NOTICE, "Corner stats: maxWhiteRun(before trigger) %d / threshold %d",
           summary.maxWhiteRunBeforeTrigger, kCornerWhiteRunCount);
    // 1=ピボットから直接、2=Tracerの追従中
    stoppedDiagnosticLog(LOG_NOTICE, "[Corner] done reason %d turned %d deg, confirmedWhileSuppressed %d, confirmExpired %d",
           summary.cornerDoneReason, summary.cornerDoneTurnDeg,
           summary.cornerConfirmedWhileSuppressed, summary.cornerConfirmExpired);
}

// 配置の開始位置の比較。往路の詳細ログとは別に、設定に関わらず必ず出す
void DeliveryTask::printPlacementDiagnostics() {
    if(!placementSummary.pending) return;
    placementSummary.pending = false;
    const PlacementSummary& place = placementSummary;
    stoppedDiagnosticLog(LOG_NOTICE, "[Place] final blue t=%d mm=%d speed %d/%d heading10 %d",
           place.finalBlueMs, place.finalBlueMm, place.finalBlueLeftSpeed,
           place.finalBlueRightSpeed, place.finalBlueHeading10);
    if(place.diagonalStartMs < 0) {
        stoppedDiagnosticLog(LOG_NOTICE, "[Place] diagonal start: not reached");
        return;
    }
    stoppedDiagnosticLog(LOG_NOTICE, "[Place] diagonal start +%d ms +%d mm speed %d/%d heading10 %d",
           place.diagonalStartMs - place.finalBlueMs,
           place.diagonalStartMm - place.finalBlueMm,
           place.diagonalStartLeftSpeed, place.diagonalStartRightSpeed, place.diagonalStartHeading10);
}

// コーナー後の最初の制御の内訳。走行中は採るだけで、出すのは止まってから
void DeliveryTask::armPostCornerSample(const char* label, int reason, int pwm) {
    for(PostCornerSample& sample : postCornerSamples) {
        if(sample.armed || sample.captured) continue;
        sample.armed = true;
        sample.label = label;
        sample.reason = reason;
        sample.pwm = pwm;
        sample.doneUs = cornerDoneUs;
        cornerDoneUs = 0;
        return;
    }
}

void DeliveryTask::capturePostCornerSample(Tracer& tracer) {
    for(PostCornerSample& sample : postCornerSamples) {
        if(!sample.armed) continue;
        sample.armed = false;
        sample.captured = true;
        sample.p100 = (int)std::lround(tracer.getLastP() * 100);
        sample.i100 = (int)std::lround(tracer.getLastI() * 100);
        sample.d100 = (int)std::lround(tracer.getLastD() * 100);
        sample.leftPwm = tracer.getLastLeftPwm();
        sample.rightPwm = tracer.getLastRightPwm();
        sample.ms = nowMs();
        if(sample.doneUs != 0) {
            SYSTIM now;
            get_tim(&now);
            sample.gapToFirstControlUs = (int)(now - sample.doneUs);
        }
        return;
    }
}

void DeliveryTask::printPostCornerSamples() {
    if(postCornerSamplesPrinted) return;
    postCornerSamplesPrinted = true;
    for(const PostCornerSample& sample : postCornerSamples) {
        if(!sample.captured) continue;
        // reason 1=ピボットから直接、2=Tracerの追従中。方式はどちらも setConfig→resetPid
        stoppedDiagnosticLog(LOG_NOTICE, "[Corner] %s post-corner config: LEGACY reset pwm %d reason %d",
               sample.label, sample.pwm, sample.reason);
        stoppedDiagnosticLog(LOG_NOTICE, "[Corner] %s first control after switch: t=%d ms, gap from done %d us",
               sample.label, sample.ms, sample.gapToFirstControlUs);
        stoppedDiagnosticLog(LOG_NOTICE, "[Corner] %s firstPID100 %d / %d / %d",
               sample.label, sample.p100, sample.i100, sample.d100);
        stoppedDiagnosticLog(LOG_NOTICE, "[Corner] %s first pwm %d / %d",
               sample.label, sample.leftPwm, sample.rightPwm);
    }
}

// コーナーを曲がりきった後の次区間の設定。完了の仕方では分けず、行き・帰りとも同じにする。
// 経路別の引き継ぎ（ピボット完了はrestart／追従完了は履歴維持）は挙動を比較できる形へ戻すため撤回した。
// 旧方式ではsetConfig()の後にresetPid()を呼ぶので、再開直後の1回にD項が出ることがある。
// これは戻す対象そのものなので、ここで再びゼロ化しない。reasonは診断のためだけに受け取る
// 方式と完了理由はここではsyslogせず、armPostCornerSample()経由で固定容量の診断へ入れて停止後に出す。
// この呼び出しから最初のTracer出力までの間に同期処理を挟まないため
void DeliveryTask::applyPostCornerTracerConfig(Tracer& tracer, int pwm) {
    tracer.setConfig(0.30f, 0.01f, 0.02f, Config::TRACER_TARGET_REFLECTION, pwm);
    tracer.resetPid();
}

DeliveryTask::CornerUpdate DeliveryTask::updateCornerDetection(CornerState& state, bool isLeftTurn, float minTurnDeg, bool isOnBlue, const char* label) {
    // 1. 中断と完了を最優先。完了後は状態も進めない
    if(aborted) {
        return CornerUpdate::ABORTED;
    }
    if(state.done) {
        return CornerUpdate::IDLE;
    }

    // 2. 再検知の抑制と受付開始の更新。青ラインを跨ぐときは脇の白を踏んで誤検知するため抑制する
    state.loopCount++;
    if(state.suppressCount > 0) {
        state.suppressCount--;
    }
    if(isOnBlue) {
        state.suppressCount = kCornerSuppressAfterBlueCount;
    }

    if(state.pending && state.loopCount >= state.enableLoop) {
        state.pending = false;
        state.enabled = true;
        syslog(LOG_NOTICE, "%s detection ENABLED. t=%d ms (+%d ms / +%d mm since trace start)", label, nowMs(), nowMs() - state.armedMs, wheelDistanceMm() - state.armedMm);
    }

    // 完了の確認にも白の連続にも使わない周期では読まない（センサーを余分に読まない）
    const bool needsReflection = state.confirming || (state.enabled && state.suppressCount == 0);
    int reflection = needsReflection ? robot.getReflection() : 101;

    // 3. 線には復帰したが曲がりきってはいない状態。残りはTracerに任せ、方位が届いたら完了とする。
    // 再検知の抑制中でも完了は判定する（抑制は「新しいコーナーを検知しない」ためのもので、
    // 既に曲がり終わったことを認められない理由にはならない）。
    // ただし線を見失ったまま旋回しているだけでも方位は増えるので、暗い面にいることを条件にする
    // （実測: 反射率99のまま方位だけ70度回り、コーナー未通過なのに完了扱いになっていた）。
    // 青を読んでいる周期では完了させない。青はライン復帰ではないうえ、以前は青による抑制が
    // この判定ごと止めていたので、前倒しで青をラインと誤認する経路を新たに作らない
    if(state.confirming) {
        float turnedDeg = signedTurnFrom(state.startHeading, isLeftTurn);
        if(!isOnBlue && turnedDeg >= kCornerDoneTurnDeg && reflection < kCornerWhiteReflection) {
            state.confirming = false;
            state.enabled = false;
            state.done = true;
            state.doneReason = CornerDoneReason::TRACE_CONFIRMED;
            state.doneMs = nowMs();
            get_tim(&cornerDoneUs);
            state.doneTurnDeg = (int)turnedDeg;
            state.doneReflection = reflection;
            state.doneSuppressRemaining = state.suppressCount;
            if(state.suppressCount > 0) {
                state.confirmedWhileSuppressed++;
            }
            syslog(LOG_NOTICE, "%s cleared by tracer t=%d ms (%d deg, reflection %d, suppress %d). Detection DISABLED.", label, nowMs(), (int)turnedDeg, reflection, state.suppressCount);
            return CornerUpdate::DONE_TRACE;
        }
        if(nowMs() >= state.confirmDeadlineMs) {
            state.confirming = false;
            state.confirmExpiredCount++;
            syslog(LOG_NOTICE, "%s NOT cleared after wait (%d deg, reflection %d). Keep detecting.", label, (int)turnedDeg, reflection);
        }
    }

    // 4. ここから先は新しいコーナーの検知。受付前・抑制中は数えない
    if(!state.enabled || state.suppressCount > 0) {
        return state.confirming ? CornerUpdate::CONFIRMING : CornerUpdate::TRACKING;
    }

    // 5. 白の連続を数える
    if(reflection >= kCornerWhiteReflection) {
        state.whiteRun++;
        state.measureWhiteRun++;
        if(reflection < state.minReflectionInWhiteRun) {
            state.minReflectionInWhiteRun = reflection;
        }
    } else {
        // 白が途切れた時点で記録する。しきい値に達して検知に使われた分は下で0に戻すので混ざらない
        if(state.measureWhiteRun > state.maxWhiteRunBeforeTrigger) {
            state.maxWhiteRunBeforeTrigger = state.measureWhiteRun;
        }
        state.measureWhiteRun = 0;
        state.whiteRun = 0;
        state.minReflectionInWhiteRun = 101;
    }
    if(state.whiteRun < kCornerWhiteRunCount) {
        return state.confirming ? CornerUpdate::CONFIRMING : CornerUpdate::TRACKING;
    }
    state.measureWhiteRun = 0;

    syslog(LOG_NOTICE, "%s detected t=%d ms (reflection %d, darkest in run %d). Pivoting.", label, nowMs(), reflection, state.minReflectionInWhiteRun);
    // 検知は線の終わりから白kCornerWhiteRunCount回分だけ遅れる。減速位置はこの値から手前に取る
    syslog(LOG_NOTICE, "%s detected: +%d ms / +%d mm since trace start", label, nowMs() - state.armedMs, wheelDistanceMm() - state.armedMm);
    state.whiteRun = 0;
    state.minReflectionInWhiteRun = 101;
    state.startHeading = robot.getImuHeading();
    state.suppressCount = kCornerSuppressAfterBlueCount;
    clearCornerConfirm(state);  // 前回の確認を新しいピボットへ持ち込まない

    CornerResult result = turnAtCorner(isLeftTurn, state.startHeading, minTurnDeg);
    float turnedDeg = signedTurnFrom(state.startHeading, isLeftTurn);

    if(aborted) {
        clearCornerConfirm(state);
        return CornerUpdate::ABORTED;
    }

    if(result == CornerResult::CLEARED) {
        // カーブは1周に1つしかないので、通過後の検知は誤検知のリスクにしかならない
        state.enabled = false;
        state.done = true;
        state.doneReason = CornerDoneReason::PIVOT_CLEARED;
        state.doneMs = nowMs();
        get_tim(&cornerDoneUs);
        state.doneTurnDeg = (int)turnedDeg;
        state.doneReflection = robot.getReflection();
        state.doneSuppressRemaining = state.suppressCount;
        syslog(LOG_NOTICE, "%s cleared t=%d ms (%d deg). Detection DISABLED.", label, nowMs(), (int)turnedDeg);
        return CornerUpdate::DONE_PIVOT;
    }
    if(result == CornerResult::REACQUIRED) {
        // enabledは落とさない。Tracerが曲がりきるのを待ちつつ、再び線を見失ったら検知し直す。
        // 期限は「Tracerへ戻した時点」から測る
        state.confirming = true;
        state.confirmStartMs = nowMs();
        state.confirmDeadlineMs = state.confirmStartMs + kCornerConfirmTimeoutMs;
        // ここではPIDを触らない。旧方式にはreset()もrestart()も無く、ピボット直後の偏差・微分を
        // そのまま持ったまま追従へ戻る。代わりにreset()を入れると旧挙動の復元にならないので入れない
        syslog(LOG_NOTICE, "%s reacquired line (%d deg) t=%d ms. Waiting for tracer.", label, (int)turnedDeg, state.confirmStartMs);
        return CornerUpdate::REACQUIRED;
    }
    // 線を見つけられなかった。間隔を置いてから再挑戦させる
    state.suppressCount = kCornerRetryAfterFailCount;
    syslog(LOG_NOTICE, "%s pivot FAILED (%d deg). Retrying later.", label, (int)turnedDeg);
    return CornerUpdate::PIVOT_FAILED;
}

void DeliveryTask::clearCornerConfirm(CornerState& state) {
    state.confirming = false;
    state.confirmStartMs = -1;
    state.confirmDeadlineMs = -1;
    state.whiteRun = 0;
    state.measureWhiteRun = 0;
    state.minReflectionInWhiteRun = 101;
}

void DeliveryTask::run() {
    diagnosticOutputBlocked = false;
    diagnosticSkippedLines = 0;
    syslog(LOG_NOTICE, "--- DeliveryTask Started ---");
    // 今回の走行で何が有効か。早期returnでも残るよう先頭で出す
    syslog(LOG_NOTICE, "[Config] notifyMode %d, deferDiag %d, bluePassed %d ms",
           (int)kColorNotifyMode, kDeferOutboundDiagnostics ? 1 : 0, kBluePassedConfirmMs);
    // コーナーを曲がりきった後のPIDの引き継ぎ方式。LEGACY=setConfig→resetPidで、REACQUIREDでは何もしない
    syslog(LOG_NOTICE, "[Config] cornerPid=LEGACY");
    aborted = false;
    // 静的領域なので、前の走行の値を必ず捨てる（条件付きでしか埋めない項目が残る）
    outboundSummary = OutboundSummary{};
    placementSummary = PlacementSummary{};
    for(PostCornerSample& sample : postCornerSamples) sample = PostCornerSample{};
    postCornerSamplesPrinted = false;
    cornerDoneUs = 0;
    traceEntryLog.clear();
    traceEntryPendingIndex = -1;
    const int deliveryStartMs = nowMs();

    // このタスクの数値・エッジ・回頭方向はすべてLコースで実測調整したもの。Rコースはその鏡像になるので、
    // isLeftCourseがfalseのときは進行方向に関わる箇所（エッジ・旋回角度・斜め移動の左右パワー）を反転させる。
    // 前進・後退の距離やパワーは向きに依存しないため反転不要
    bool isLeftCourse = CourseConfig::isLeftCourse();
    float courseSign = isLeftCourse ? 1.0f : -1.0f;

    Tracer tracer(robot);
    tracer.setEdge(isLeftCourse ? Tracer::Edge::RIGHT : Tracer::Edge::LEFT);
    tracer.setPwm(kApproachPwm);

    // 1. ライントレースしながらボトルに近づく（この間、ラインに正対した基準角度をIMUで平均して求めておく）
    float headingSum = 0.0f;
    int headingSampleCount = 0;
    while(true) {
        if(robot.isCenterButtonPressed()) {  // センターボタンで安全停止
            aborted = true;
            tracer.terminate();
            return;
        }

        int currentDistance = robot.getUltrasonicDistance();

        if(currentDistance > 0 && currentDistance <= Config::DELIVERY_TARGET_DISTANCE_MM) {
            tracer.terminate();
            break;
        }

        headingSum += robot.getImuHeading();
        headingSampleCount++;

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }
    float baselineHeading = (headingSampleCount > 0) ? (headingSum / headingSampleCount) : robot.getImuHeading();
    // 診断: 以降の角度はすべてこの基準からのズレで出す。接近中に曲がっていると基準自体がずれるので、停止時点との差も見る
    syslog(LOG_NOTICE, "[Heading] baseline %d (0.1deg, %d samples), at bottle stop %d from baseline", (int)(baselineHeading * 10.0f), headingSampleCount, (int)((robot.getImuHeading() - baselineHeading) * 10.0f));

    // 2. ボトルの前でアームを上げる（Robotクラスに移譲）
    // 接近ループはcoastで抜けるだけなので、ここで止め切ってから上げる。
    // 惰性が残ったまま上げると車体がピッチ方向に傾き、アームの地面に対する向きがずれる
    logPosture("before stop");
    stopAndSettleBeforeArm();
    if(aborted) { robot.stop(); return; }
    logPosture("before raise 1");

    // 上げ角の上乗せ分。下げるときも同じだけ下げる。1回目から再試行と同じ角度で上げる
    int armExtraDeg = kBottleRetryExtraArmDeg;
    // raiseArm/lowerArmは動作の開始時にエンコーダ原点を作り直すので、診断の積算角度は動作の前に足す
    armAbsoluteBaseDeg = 0;
    commitArmMovement();
    robot.raiseArm(armExtraDeg);

    // 3. ボトルの色を判定。読めなければアームを下げ、少し前に出ながら上げ直して読み直す
    ColorJudge::Color bottleColor = readBottleColorAfterRaise(Config::ARM_RAISE_DEG + armExtraDeg);
    for(int retry = 1; bottleColor == ColorJudge::Color::UNKNOWN && !aborted && retry <= kBottleRetryMaxCount; retry++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            break;
        }
        commitArmMovement();
        robot.lowerArm(armExtraDeg);
        armExtraDeg += kBottleRetryExtraArmDeg;
        syslog(LOG_NOTICE, "Bottle color unknown. Retry %d/%d: advance %dmm while raising to %d deg", retry, kBottleRetryMaxCount, kBottleRetryAdvanceMm, Config::ARM_RAISE_DEG + armExtraDeg);
        commitArmMovement();
        robot.raiseArmWhileDriving(kBottleRetryAdvanceMm, kBottleRetryAdvanceSpeedDegPerSec, armExtraDeg);
        bottleColor = readBottleColorAfterRaise(Config::ARM_RAISE_DEG + armExtraDeg);
    }
    if(aborted) {
        robot.stop();
        syslog(LOG_NOTICE, "ABORTED during bottle color judgement. Stopping.");
        return;
    }
    int targetBlueLineCount = 0;  // 目標の青ライン通過回数
    int pendingNotifyToneCount = 0;      // 色の通知で鳴らす回数。鳴らすのはアームを下げた後
    const int colorDecidedMs = nowMs();  // 診断: 色が決まってから下降を始めるまでに何ms使ったか

    // 判定結果に応じて目標通過回数を設定する（通知は上のkNotifyBottleColorByBeep次第）
    switch(bottleColor) {
        case ColorJudge::Color::YELLOW:
            syslog(LOG_NOTICE, "Bottle Color: YELLOW");
            pendingNotifyToneCount = 1;
            targetBlueLineCount = 2;
            break;

        case ColorJudge::Color::BLUE:
            syslog(LOG_NOTICE, "Bottle Color: BLUE");
            pendingNotifyToneCount = 2;
            targetBlueLineCount = 3;
            break;

        case ColorJudge::Color::RED:
            syslog(LOG_NOTICE, "Bottle Color: RED");
            pendingNotifyToneCount = 3;
            targetBlueLineCount = 4;
            break;

        default:
            // 再試行しても読めなかった。目標本数が決まらないまま走ると、青1本目でエリアへ向かうなど
            // 的外れな動きになるので、走らずに止めて人が気づけるようにする
            syslog(LOG_NOTICE, "Bottle Color: UNKNOWN after %d retries. Stopping.", kBottleRetryMaxCount);
            robot.beep(500);
            robot.stop();
            return;
    }

    syslog(LOG_NOTICE, "[Notify] color decided -> arm lower start %d ms (mode %d, tones %d)",
           nowMs() - colorDecidedMs, (int)kColorNotifyMode, pendingNotifyToneCount);

    // 4. アームを下げる（Robotクラスに移譲）
    commitArmMovement();
    robot.lowerArm(armExtraDeg);
    if(robot.isCenterButtonPressed()) { robot.stop(); return; }

    // 5. 左35・右40のパワーでkAfterArmStraightSec秒直進してラインに復帰する（蛇行探索より速く、実機ではこれで十分だった）。
    // 色の通知はこの区間の中で鳴らす。既存の直進時間・出力は変えず、音のために待たない
    ColorNotifier notifier(kColorNotifyToneMs, kColorNotifyGapMs);
    const int notifyStartMs = nowMs();
    if(kColorNotifyMode == ColorNotifyMode::DEFERRED_NONBLOCKING) {
        notifier.start(pendingNotifyToneCount, notifyStartMs);
    }
    const char* notifyEndReason = "section end";
    int notifyMaxIntervalMs = 0, notifyLastLoopMs = notifyStartMs, notifyLoopCount = 0;
    int afterArmStraightLoopCount = static_cast<int>(kAfterArmStraightSec * 1000 * 1000 / Config::MOTION_POLL_INTERVAL_US);
    for(int i = 0; i < afterArmStraightLoopCount; i++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            robot.stop();
            notifyEndReason = "button";
            applyNotifierAction(notifier.cancel());
            logColorNotify(notifier, pendingNotifyToneCount, notifyStartMs, notifyEndReason,
                           notifyMaxIntervalMs, notifyLoopCount);
            return;
        }
        // 音のための処理が周期を乱していないか。出力より先に、前の周期からの間隔を測る
        const int loopMs = nowMs();
        const int intervalMs = loopMs - notifyLastLoopMs;
        if(intervalMs > notifyMaxIntervalMs) notifyMaxIntervalMs = intervalMs;
        notifyLastLoopMs = loopMs;
        notifyLoopCount++;
        // 安全確認とモーター出力が先。音は残りの時間で進めるだけで、待ちは足さない
        robot.setMotorPower(kAfterArmStraightLeftPwm, kAfterArmStraightRightPwm);
        applyNotifierAction(notifier.update(loopMs));
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }
    robot.stop();
    // 鳴らし終わっていなくても区間は延ばさない。次の走行へ音を持ち越さない
    if(!notifier.isFinished()) notifyEndReason = "section end (unfinished)";
    applyNotifierAction(notifier.cancel());
    logColorNotify(notifier, pendingNotifyToneCount, notifyStartMs, notifyEndReason,
                   notifyMaxIntervalMs, notifyLoopCount);

    // 6. 左エッジでライントレースを再開
    syslog(LOG_NOTICE, "Resuming line trace on LEFT edge (Slow Speed)");
    tracer.setEdge(isLeftCourse ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT);
    tracer.setPwm(kReacquireLinePwm);  // 蛇行直後はズレが大きくカーブ減速で止まりやすいため、kApproachPwmより高めに

    // 7. 0.5秒間、遅い速度でライントレース
    int slowTraceLoopCount = (1000 * 500) / Config::LINE_TRACE_POLL_INTERVAL_US;
    for(int i = 0; i < slowTraceLoopCount; i++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            tracer.terminate();
            return;
        }
        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    // 8. 通常速度に戻してライントレースを継続
    syslog(LOG_NOTICE, "Slow trace done. Switching to pwm %d.", kPostSlowTracePwm);
    tracer.setConfig(0.38f, 0.01f, 0.02f, Config::TRACER_TARGET_REFLECTION, kPostSlowTracePwm);
    tracer.setCurveDecelGain(2.3f);
    const int curveStartMm = wheelDistanceMm();
    const float curveStartHeading = robot.getImuHeading();
    // 早期持ち替えの診断サンプルの原点（角度は接近基準から、距離は曲線開始から）
    bool curveActive=true, curveSlowed=false;
    int curveSlowMm=-1, curveSlowMs=-1;
    float traceKp=0.38f;
    // タスクのスタックは4KiB。固定診断バッファは静的領域に置き毎回初期化する。
    // DeliveryTaskは単一タスクからのみ実行する（並列実行不可）。
    areaApproachLog=DeliveryTraceLog("area-slow");
    curveLog=DeliveryTraceLog("curve");
    outLog=DeliveryTraceLog("blue1-to-corner");
    areaLog=DeliveryTraceLog("corner-to-area");
    curveLog.begin(nowMs(),curveStartMm,curveStartHeading);
    auto sampleOutbound = [&]() {
        int ms=nowMs(), mm=wheelDistanceMm(); float h=robot.getImuHeading();
        curveLog.sample(tracer,traceKp,ms,mm,h);
        outLog.sample(tracer,traceKp,ms,mm,h);
        areaLog.sample(tracer,traceKp,ms,mm,h);
        areaApproachLog.sample(tracer,traceKp,ms,mm,h);
    };

    // 9. 指定回数青ラインを検知するまでライントレース
    syslog(LOG_NOTICE, "Tracing until blue line count: %d", targetBlueLineCount);

    int detectedBlueCount = 0;       // 青ラインを検知した回数
    bool isCurrentlyOnBlue = false;  // 現在青ライン上にいるかのフラグ

    int matchedBlueCount = 0;     // 青を連続で読んだ回数
    int matchedNonBlueCount = 0;  // 青以外を連続で読んだ回数

    // ms指定の確定時間を、現在のLINE_TRACE_POLL_INTERVAL_US(制御周期)でのサンプル回数に換算
    int blueEntryConfirmCount = (kBlueEntryConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    int blueFinalEntryConfirmCount = (kBlueFinalEntryConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    int bluePassedConfirmCount = (kBluePassedConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

    // 青判定の診断。コーナー前の高彩度がコーナー後の情報を隠さないよう、区間を分けて集計する
    static BlueStats blueStatsBeforeCorner, blueStatsAfterCorner;
    blueStatsBeforeCorner=BlueStats{};
    blueStatsAfterCorner=BlueStats{};

    // 往路の詳細ログは出口が多い。どの経路から戻っても一度だけ出るようにする
    struct OutboundDiagnosticsGuard {
        DeliveryTask* task;
        BlueStats* beforeCorner;
        BlueStats* afterCorner;
        ~OutboundDiagnosticsGuard() {
            task->printOutboundDiagnostics(*beforeCorner, *afterCorner);
            task->printPlacementDiagnostics();
            task->printPostCornerSamples();
            drainStoppedDiagnostics();
        }
    } outboundDiagnosticsGuard{ this, &blueStatsBeforeCorner, &blueStatsAfterCorner };

    // 診断: 向きは基準角度からのズレ[度]。Lコースの左カーブは負
    auto headingFromBaseline = [this, baselineHeading]() {
        return static_cast<int>(robot.getImuHeading() - baselineHeading);
    };

    // 直角コーナー対策の状態。手前での減速と同時に有効化する（下のisCornerSlowdownPendingの処理）
    CornerState outboundCorner;

    // 青1本目の上での踏み越え検知。診断用の集計とはカウンタを共用しない（共用して挙動を壊したことがあるため）
    int blue1OvershootWhiteRun = 0;
    bool isBlueIgnored = false;  // 踏み越え後、青1本目を二重に数えないための無視期間
    int blueIgnoreEndMs = 0;

    // コーナーを曲がりきった地点。エリアに入る青の手前での減速の起点。-1はまだ曲がりきっていない
    int outboundCornerClearedMs = 0;
    int outboundCornerClearedMm = -1;
    const int areaColorSteps = targetBlueLineCount - 2;  // 黄0・青1・赤2（エリアに入る青の本数から）
    const int areaExpectedBlueMm = kAreaExpectedBlueMm[areaColorSteps];
    const int areaSlowLeadMm = (areaColorSteps == 0) ? kAreaSlowLeadMmYellow : kAreaSlowLeadMm;  // 0は黄
    const int areaSlowdownStartMm = areaExpectedBlueMm - areaSlowLeadMm;
    const int areaSearchStartMm = areaExpectedBlueMm - kAreaSearchLeadMm;
    AreaBlueGate areaGate(areaSearchStartMm,areaExpectedBlueMm+kAreaMissingBlueLimitMm,
                          blueFinalEntryConfirmCount);
    bool areaSearchOpened=false;
    int areaGateOpenedMm=-1, areaFirstRawBlueMm=-1;
    int areaSlowActualMm=-1, areaSlowMs=-1, finalBlueActualMm=-1;
    bool isAreaSlowdownPending = true;

    // コーナー手前の減速。トレース開始からの距離で落とす
    bool isCornerSlowdownPending = false;
    int cornerSlowdownStartMm = kCornerSlowdownStartMm;

    // 右エッジのトレースに切り替え、手前での減速を予約する（通常時・踏み越え時で共通）。
    // コーナー判定は減速するまで始めない（姿勢の乱れによる誤検知を避けるため）
    auto startRightEdgeTrace = [&](int slowdownStartMm) {
        tracer.setEdge(isLeftCourse ? Tracer::Edge::RIGHT : Tracer::Edge::LEFT);
        curveLog.end(nowMs(),wheelDistanceMm());
        curveActive=false; traceKp=0.30f;
        isCornerSlowdownPending = true;
        cornerSlowdownStartMm = slowdownStartMm;
        outboundCorner.armedMs = nowMs();
        outboundCorner.armedMm = wheelDistanceMm();
        if(!acquireTraceEntry(tracer,"blue1")) return;
        tracer.setPwm(kCornerTracePwm); // 安定後はPID履歴を引き継ぐ
        outLog.begin(nowMs(),wheelDistanceMm(),robot.getImuHeading());
        syslog(LOG_NOTICE, "Corner trace start t=%d ms", outboundCorner.armedMs);
    };

    while(true) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            break;
        }

        if(curveActive && !curveSlowed &&
           ((robot.getImuHeading()-curveStartHeading)*courseSign <= -55.0f ||
            wheelDistanceMm()-curveStartMm >= 500)) {
            curveSlowed=true; traceKp=0.55f;
            curveSlowMm=wheelDistanceMm()-curveStartMm; curveSlowMs=nowMs();
            tracer.setConfig(0.55f,0.01f,0.02f,Config::TRACER_TARGET_REFLECTION,78);
        }
        const int reflection = robot.getReflection();

        // 青1本目の上で線を踏み越えたら、通過を待たずに基準角度から80度へ旋回して右エッジで進む
        if(!outboundCorner.done && detectedBlueCount == 1 && isCurrentlyOnBlue && !isBlueIgnored) {
            blue1OvershootWhiteRun = (reflection >= kCornerWhiteReflection) ? blue1OvershootWhiteRun + 1 : 0;
            if(blue1OvershootWhiteRun >= kBlue1OvershootWhiteRunCount) {
                curveLog.end(nowMs(),wheelDistanceMm());
                const int detectedMs = nowMs();
                syslog(LOG_NOTICE, "[Overshoot] blue1: white %d samples t=%d ms, heading %d. Turning to baseline-80.", blue1OvershootWhiteRun, detectedMs, headingFromBaseline());

                isCurrentlyOnBlue = false;
                matchedBlueCount = 0;
                matchedNonBlueCount = 0;
                isBlueIgnored = true;
                blueIgnoreEndMs = detectedMs + kBlue1OvershootBlueIgnoreMs;

                float targetHeading = baselineHeading - kBlue1OvershootTargetHeadingDeg * courseSign;
                const float turnedDeg = robot.turnByImu(targetHeading - robot.getImuHeading(), Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);
                syslog(LOG_NOTICE, "[Overshoot] blue1: turned %d deg in %d ms, heading %d, reflection %d", (int)turnedDeg, nowMs() - detectedMs, headingFromBaseline(), robot.getReflection());
                if(robot.isCenterButtonPressed()) {
                    aborted = true;
                    break;
                }
                // 弧を描く移動は線の左側から始める前提なので行わない
                startRightEdgeTrace(kCornerSlowdownStartMmAfterOvershoot);
                if(aborted) break;
            }
        }

        if(outboundCornerClearedMm>=0) {
            const int areaMm=wheelDistanceMm()-outboundCornerClearedMm;
            // 毎周期、同じ1回の色読みを観測と最終青判定に使う。途中の青は本数にしない。
            blueStatsAfterCorner.entryArmed=areaGate.armed();
            blueStatsAfterCorner.nextBlue=targetBlueLineCount;
            blueStatsAfterCorner.required=blueFinalEntryConfirmCount;
            blueStatsAfterCorner.commandPwm=tracer.getBasePwm();
            const bool rawBlue=isBlueReading(blueStatsAfterCorner);
            const auto decision=areaGate.update(areaMm,rawBlue,!isAreaSlowdownPending);
            if(areaGate.armed() && !areaSearchOpened) {
                areaSearchOpened=true;
                areaGateOpenedMm=areaMm;
            }
            if(areaSearchOpened && rawBlue && areaFirstRawBlueMm<0) areaFirstRawBlueMm=areaMm;
            if(decision==AreaBlueGate::Result::MISSING) {
                aborted=true;robot.stop();
                syslog(LOG_NOTICE,"[AreaBlue] target not found in window; stopping at %d mm",areaMm);
                break;
            }
            if(decision==AreaBlueGate::Result::FOUND) {
                finalBlueActualMm=areaMm;
                // 止める前に採る。惰性を測りたいので、最初の移動を落とさない
                placementSummary.pending=true;
                placementSummary.finalBlueMs=nowMs();
                placementSummary.finalBlueMm=wheelDistanceMm();
                placementSummary.finalBlueLeftSpeed=robot.getLeftMotor().getSpeed();
                placementSummary.finalBlueRightSpeed=robot.getRightMotor().getSpeed();
                placementSummary.finalBlueHeading10=(int)(robot.getImuHeading()*10);
                robot.stop();
                syslog(LOG_NOTICE,"[AreaBlue] target detected at %d mm (bottle target %d); placing",
                       areaMm,targetBlueLineCount);
                break;
            }
        }

        if(isBlueIgnored) {
            if(nowMs() >= blueIgnoreEndMs || outboundCorner.done) {
                isBlueIgnored = false;
                detectedBlueCount = 1;
                isCurrentlyOnBlue = false;
                matchedBlueCount = 0;
                matchedNonBlueCount = 0;
                syslog(LOG_NOTICE, "[Overshoot] blue ignore end t=%d ms (%s). Blue count set to 1.", nowMs(), outboundCorner.done ? "corner cleared" : "timeout");
            }
        }

        BlueStats& currentBlueStats=outboundCorner.done?blueStatsAfterCorner:blueStatsBeforeCorner;
        currentBlueStats.entryArmed=!isCurrentlyOnBlue && !isBlueIgnored;
        currentBlueStats.nextBlue=detectedBlueCount+1;
        currentBlueStats.required=(detectedBlueCount+1>=targetBlueLineCount)?blueFinalEntryConfirmCount:blueEntryConfirmCount;
        currentBlueStats.commandPwm=tracer.getBasePwm();
        // 無視期間中も、コーナー判定の抑制（青を跨ぐときの脇の白）には生の青判定を使う
        bool isOnBlueForCorner = isCurrentlyOnBlue;
        if(outboundCornerClearedMm>=0) {
            // 上で観測済み。コーナー後は旧本数カウンタも400ms通過待ちも使わない。
            isOnBlueForCorner=false;
        } else if(isBlueIgnored) {
            BlueStats& blueStats = outboundCorner.done ? blueStatsAfterCorner : blueStatsBeforeCorner;
            isOnBlueForCorner = isBlueReading(blueStats);
        } else if(!isCurrentlyOnBlue) {
            // まだ青ラインに乗っていない状態：青を探す。
            // エリアへ向かう最後の1本だけは、青の入口で抜けられるよう確定を早める
            bool isFinalBlue = (detectedBlueCount + 1 >= targetBlueLineCount);
            BlueStats& blueStats = outboundCorner.done ? blueStatsAfterCorner : blueStatsBeforeCorner;
            bool isOnBlueNow = isOnBlueLine(matchedBlueCount, isFinalBlue ? blueFinalEntryConfirmCount : blueEntryConfirmCount, blueStats);
            if(isOnBlueNow) {
                isCurrentlyOnBlue = true;
                matchedNonBlueCount = 0;  // 青以外カウントをリセット

                // 青を読んだ瞬間にカウントアップ！
                detectedBlueCount++;
                ColorJudge::Reading entryReading = robot.getColorReading();
                syslog(LOG_NOTICE, "Entered blue line. Count: %d / %d (h=%d s=%d v=%d)", detectedBlueCount, targetBlueLineCount, (int)entryReading.hsv.h, (int)entryReading.hsv.s, (int)entryReading.hsv.v);
                if(outboundCornerClearedMm >= 0) {
                    // 診断: エリアに入る青の手前で減速する位置を決めるため
                    syslog(LOG_NOTICE, "Blue %d entry: +%d ms / +%d mm since corner cleared", detectedBlueCount, nowMs() - outboundCornerClearedMs, wheelDistanceMm() - outboundCornerClearedMm);
                }
                if(detectedBlueCount == 1) {
                    // 青1本目の上に乗っている間（判定しなくなるまで）は速度を落とす
                    tracer.setPwm(kOnFirstBlueLinePwm);
                    blue1OvershootWhiteRun = 0;
                    syslog(LOG_NOTICE, "[Blue1] entry t=%d ms, heading %d", nowMs(), headingFromBaseline());
                }
                // 指定回数に達したら、その場ですぐに終了する
                if(detectedBlueCount >= targetBlueLineCount) {
                    // 配置はコーナー後の距離窓内の実検知だけで開始する。
                    aborted=true;
                    robot.stop();
                    syslog(LOG_NOTICE, "Unexpected blue count before corner. Stopping without placement.");
                    break;
                }
            }
            isOnBlueForCorner = isCurrentlyOnBlue;
        } else {
            // すでに青ラインに乗っている状態：青から降りる（青以外）のを探す
            BlueStats& blueStats = outboundCorner.done ? blueStatsAfterCorner : blueStatsBeforeCorner;
            if(!isBlueReading(blueStats)) {
                matchedNonBlueCount++;
            } else {
                matchedNonBlueCount = 0;  // もし途中で青を読んだらリセット
            }

            // 青以外を一定時間連続で読んだら「完全にラインを通り過ぎた」と判定して次のラインを探せるようにする
            if(matchedNonBlueCount >= bluePassedConfirmCount) {
                isCurrentlyOnBlue = false;
                matchedBlueCount = 0;  // 次の青ラインを探すためにリセット

                syslog(LOG_NOTICE, "Passed blue line!");

                if(detectedBlueCount == 1) {
                    syslog(LOG_NOTICE, "[Blue1] passed t=%d ms, heading %d", nowMs(), headingFromBaseline());
                    curveLog.end(nowMs(),wheelDistanceMm());

                    // 弧を描いて線を横切り、右エッジ側へ移る。黒を踏んだら止める
                    const float headingBefore = robot.getImuHeading();
                    const int leftPwm = isLeftCourse ? kAfterBlue1OuterPwm : kAfterBlue1InnerPwm;
                    const int rightPwm = isLeftCourse ? kAfterBlue1InnerPwm : kAfterBlue1OuterPwm;
                    const int moveLoopCount = (kAfterBlue1MoveMs * 1000) / Config::MOTION_POLL_INTERVAL_US;
                    const int moveStartMs = nowMs();
                    const int moveStartMm = wheelDistanceMm();
                    const int startReflection = robot.getReflection();

                    // 直前のtracer.run()が出した出力が残っているので、判定の前に明示的に切る。
                    // stop()はcoastなので惰性では進むが、少なくとも駆動はしていない状態で判定できる
                    robot.stop();

                    // 開始時点ですでに黒か。黒ならもう線の上なので動かさない
                    int startBlackRun = 0;
                    for(int k = 0; k < kAfterBlue1StartBlackCount; k++) {
                        if(robot.getReflection() > kAfterBlue1BlackReflection) {
                            startBlackRun = 0;
                            break;
                        }
                        startBlackRun++;
                        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
                    }
                    const bool alreadyOnLine = (startBlackRun >= kAfterBlue1StartBlackCount);

                    // 開始値が黒の近くなら、渡り切る前に止まらないよう短い下限だけ置く
                    const int minMoveLoopCount = (startReflection >= kAfterBlue1ClearOfBlackReflection)
                                                     ? 0
                                                     : (kAfterBlue1AmbiguousMinMoveMs * 1000) / Config::MOTION_POLL_INTERVAL_US;

                    int blackRun = 0;
                    const char* stopReason = alreadyOnLine ? "already on line (no move)" : "timeout";
                    if(!alreadyOnLine) {
                        for(int k = 0; k < moveLoopCount; k++) {
                            if(robot.isCenterButtonPressed()) {
                                aborted = true;
                                break;
                            }
                            blackRun = (robot.getReflection() <= kAfterBlue1BlackReflection) ? blackRun + 1 : 0;
                            if(k >= minMoveLoopCount && blackRun >= kAfterBlue1BlackRunCount) {
                                stopReason = "black";
                                break;
                            }
                            robot.setMotorPower(leftPwm, rightPwm);
                            dly_tsk(Config::MOTION_POLL_INTERVAL_US);
                        }
                    }
                    // ループを抜けた時点では弧の出力が残っている。ログを出す前に切る
                    robot.stop();
                    if(aborted) {
                        break;
                    }
                    if(!alreadyOnLine && blackRun < kAfterBlue1BlackRunCount) {
                        aborted=true;
                        syslog(LOG_NOTICE,"[Blue1] crossing failed; do not start fast trace");
                        break;
                    }
                    syslog(LOG_NOTICE, "[Blue1] move stop by %s at %d ms / %d mm, reflection start %d / end %d", stopReason, nowMs() - moveStartMs, wheelDistanceMm() - moveStartMm, startReflection, robot.getReflection());
                    syslog(LOG_NOTICE, "[Blue1] move heading change %d deg", (int)(robot.getImuHeading() - headingBefore));

                    startRightEdgeTrace(kCornerSlowdownStartMm);
                    if(aborted) break;
                }
            }
            isOnBlueForCorner = isCurrentlyOnBlue;
        }

        if(isCornerSlowdownPending) {
            if(outboundCorner.done) {
                isCornerSlowdownPending = false;  // 減速位置より手前で曲がりきった
            } else if(wheelDistanceMm() - outboundCorner.armedMm >= cornerSlowdownStartMm) {
                isCornerSlowdownPending = false;
                outLog.end(nowMs(),wheelDistanceMm());
                tracer.setConfig(0.38f,0.01f,0.02f,Config::TRACER_TARGET_REFLECTION,kCornerApproachPwm);
                traceKp=0.38f;
                // 次の区間の設定は、曲がりきった後に呼び出し側で一度だけ決める。
                // 減速したここから白の連続を数え始める
                outboundCorner.pending = true;
                outboundCorner.enableLoop = outboundCorner.loopCount;  // 次の周期から有効
                syslog(LOG_NOTICE, "Corner slowdown to pwm %d t=%d ms (+%d mm since trace start)", kCornerApproachPwm, nowMs(), wheelDistanceMm() - outboundCorner.armedMm);
            }
        }

        // Lコースでは左90度コーナーなので左へ回す
        const CornerUpdate cornerUpdate = updateCornerDetection(outboundCorner, isLeftCourse, kCornerPivotMinTurnDeg, isOnBlueForCorner, "Corner");
        if(aborted) break;
        (void)cornerUpdate;
        if(outboundCorner.done && outboundCornerClearedMm < 0) {
            traceKp=0.30f;
            armPostCornerSample("outbound", (int)outboundCorner.doneReason, kAreaFastPwm);
            applyPostCornerTracerConfig(tracer, kAreaFastPwm);
            tracer.setCurveDecelGain(2.3f);
            areaLog.begin(nowMs(),wheelDistanceMm(),robot.getImuHeading());
            outboundCornerClearedMs = nowMs();
            outboundCornerClearedMm = wheelDistanceMm();
            blueStatsAfterCorner.distanceOriginMm=outboundCornerClearedMm;
            isCurrentlyOnBlue=false;
            isBlueIgnored=false;
            matchedBlueCount=matchedNonBlueCount=0;
            syslog(LOG_NOTICE, "Corner cleared: distance origin for blue entries t=%d ms, area slowdown at %d mm", outboundCornerClearedMs, areaSlowdownStartMm);
        }
        if(outboundCornerClearedMm >= 0 && isAreaSlowdownPending && wheelDistanceMm() - outboundCornerClearedMm >= areaSlowdownStartMm) {
            isAreaSlowdownPending = false;
            areaSlowActualMm=wheelDistanceMm()-outboundCornerClearedMm;
            areaSlowMs=nowMs();
            areaLog.end(areaSlowMs,wheelDistanceMm());
            areaApproachLog.begin(areaSlowMs,wheelDistanceMm(),robot.getImuHeading());
            // 低速の最終青進入は従来設定へ戻し、速度以外の変更を分離する。
            traceKp=0.38f;
            tracer.setConfig(0.38f,0.01f,0.02f,Config::TRACER_TARGET_REFLECTION,kAreaApproachPwm);
            syslog(LOG_NOTICE, "Area slowdown to pwm %d t=%d ms (+%d mm since corner cleared)", kAreaApproachPwm, nowMs(), wheelDistanceMm() - outboundCornerClearedMm);
        }

        if(outboundCornerClearedMm>=0 &&
           wheelDistanceMm()-outboundCornerClearedMm>areaExpectedBlueMm+kAreaMissingBlueLimitMm) {
            aborted=true;
            robot.stop();
            syslog(LOG_NOTICE,"[AreaBlue] target missing beyond expected + %d mm; stopping",kAreaMissingBlueLimitMm);
            break;
        }
        // 青ライン上でも関係なく通常のライントレースを継続
        tracer.run();
        noteFirstFastControl();
        capturePostCornerSample(tracer);
        sampleOutbound();

        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    const int outboundDiagnosticStartMs=nowMs();
    // ここまでの値を確定させる。出力位置はkDeferOutboundDiagnosticsで決める
    areaApproachLog.end(outboundDiagnosticStartMs,wheelDistanceMm());
    curveLog.end(outboundDiagnosticStartMs,wheelDistanceMm());
    outLog.end(outboundDiagnosticStartMs,wheelDistanceMm());
    areaLog.end(outboundDiagnosticStartMs,wheelDistanceMm());
    outboundSummary.pending=true;
    outboundSummary.areaExpectedBlueMm=areaExpectedBlueMm;
    outboundSummary.areaSlowdownStartMm=areaSlowdownStartMm;
    outboundSummary.areaSlowActualMm=areaSlowActualMm;
    outboundSummary.finalBlueActualMm=finalBlueActualMm;
    outboundSummary.areaSearchStartMm=areaSearchStartMm;
    outboundSummary.areaGateOpenedMm=areaGateOpenedMm;
    outboundSummary.areaFirstRawBlueMm=areaFirstRawBlueMm;
    outboundSummary.areaMaxMm=areaExpectedBlueMm+kAreaMissingBlueLimitMm;
    if(areaSlowActualMm>=0 && finalBlueActualMm>=0) {
        outboundSummary.slowToFinalMm=finalBlueActualMm-areaSlowActualMm;
        outboundSummary.slowToFinalMs=outboundDiagnosticStartMs-areaSlowMs;
    }
    outboundSummary.curveSlowMm=curveSlowMm;
    outboundSummary.curveSlowMs=curveSlowMs;
    outboundSummary.outboundElapsedMs=outboundDiagnosticStartMs-deliveryStartMs;
    outboundSummary.detectedBlueCount=detectedBlueCount;
    outboundSummary.targetBlueLineCount=targetBlueLineCount;
    outboundSummary.cornerDone=outboundCorner.done?1:0;
    outboundSummary.aborted=aborted?1:0;
    outboundSummary.maxWhiteRunBeforeTrigger=outboundCorner.maxWhiteRunBeforeTrigger;
    outboundSummary.cornerDoneReason=(int)outboundCorner.doneReason;
    outboundSummary.cornerDoneTurnDeg=outboundCorner.doneTurnDeg;
    outboundSummary.cornerConfirmedWhileSuppressed=outboundCorner.confirmedWhileSuppressed;
    outboundSummary.cornerConfirmExpired=outboundCorner.confirmExpiredCount;
    // 旧位置で出す設定なら、ここで出し切ってから配置へ進む
    if(!kDeferOutboundDiagnostics) printOutboundDiagnostics(blueStatsBeforeCorner, blueStatsAfterCorner);
    if(aborted) {
        // 中断で抜けた場合にエリア配置へ進むと、各動作が即座に空振りして「到達した」ように見えてしまう
        robot.stop();
        syslog(LOG_NOTICE, "ABORTED during blue search. Stopping.");
        return;
    }
    const int outboundDiagnosticMs=nowMs()-outboundDiagnosticStartMs;
    const int placementStartMs=nowMs();
    syslog(LOG_NOTICE, "Reached target zone.");

    // 10. エリアへの配置（斜め移動 → 惰性を殺す → 後退 → その場旋回）。色に関わらず共通。
    // 前進は立ち上がりが遅くボトルネックだったため廃止し、斜め移動の弧だけでエリアまで運ぶ
    syslog(LOG_NOTICE, "Diagonal move into area");
    // 最終青の成立から、実際に動き出すまでに何が起きたか。ログ待ちの惰性の影響を見る
    placementSummary.pending=true;
    placementSummary.diagonalStartMs=nowMs();
    placementSummary.diagonalStartMm=wheelDistanceMm();
    placementSummary.diagonalStartLeftSpeed=robot.getLeftMotor().getSpeed();
    placementSummary.diagonalStartRightSpeed=robot.getRightMotor().getSpeed();
    placementSummary.diagonalStartHeading10=(int)(robot.getImuHeading()*10);
    diagonalMoveUntilImuTurn(isLeftCourse, kDiagonalPwmHigh, robot.getImuHeading(), kDiagonalTurnDeg);
    if(aborted) { robot.stop(); return; }

    // 時間切れでは止めない（ボタンのときだけabortedが立つ）。結果はログに残す
    brakeUntilStopped(kSettleSpeedDegPerSec, kSettleTimeoutMs, "after diagonal", false);
    if(aborted) { robot.stop(); return; }

    syslog(LOG_NOTICE, "Driving backward %dmm (duty limit %d)", kAreaBackwardMm, kAreaBackwardDutyLimit);
    // Robot::driveStraight()は先頭でresetMotorCounts()するので、外側で車輪距離の差を取ると原点が変わる。
    // 指令ぶんは戻り値を使い、惰性ぶんだけを指令終了後の差分で測る
    const int backwardDrivenMm = driveStraightWithDutyLimit(kAreaBackwardMm, kAreaBackwardSpeedDegPerSec, kAreaBackwardDutyLimit);
    const int backwardCommandEndMm = wheelDistanceMm();
    // driveStraight()の末尾はstop()（coast）なので、指令が終わった後も惰性で進む。行き過ぎの本体はここ
    brakeUntilStopped(kSettleSpeedDegPerSec, kSettleTimeoutMs, "after backward", false);
    const int backwardOverrunMm = wheelDistanceMm() - backwardCommandEndMm;
    syslog(LOG_NOTICE, "Backward: commanded %d mm, driven %d mm, overrun %d mm, total %d mm", kAreaBackwardMm, backwardDrivenMm, backwardOverrunMm, backwardDrivenMm + backwardOverrunMm);
    if(aborted) { robot.stop(); return; }

    syslog(LOG_NOTICE, "Turning %d degrees in place", (int)kAreaTurnDeg);
    turnInPlaceByImu(isLeftCourse ? kAreaTurnPwm : -kAreaTurnPwm, isLeftCourse ? -kAreaTurnPwm : kAreaTurnPwm, kAreaTurnDeg);

    if(aborted) {
        robot.stop();
        syslog(LOG_NOTICE, "ABORTED during area placement. Stopping.");
        return;
    }

    // 帰りの線探し（旋回方向はコース依存、反射率ベース）。見つけた瞬間の踏み込みもこの中で行う
    syslog(LOG_NOTICE, "Pivoting to find line by reflection");
    if(!pivotUntilReflectionBelow(kSearchReflectionThreshold, kSearchPwm, isLeftCourse) || aborted) {
        // 線が無い場所でTracerを起動すると白の上を暴走するだけなので、ここで打ち切る
        robot.stop();
        syslog(LOG_NOTICE, aborted ? "ABORTED during line search. Stopping." : "Line search FAILED. Aborting return trip.");
        return;
    }

    // 11. 左エッジでライントレースを再開
    syslog(LOG_NOTICE, "Resuming line trace on LEFT edge");
    tracer.setEdge(isLeftCourse ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT);
    const int returnEntryStartMs=nowMs(), returnEntryStartMm=wheelDistanceMm();
    if(!acquireTraceEntry(tracer,"area-return")) return;
    tracer.setPwm(kReturnTracePwm);
    const int placementElapsedMs=nowMs()-placementStartMs;
    static DeliveryTraceLog returnLog("area-to-corner"), finishLog("return-after-corner");
    returnLog=DeliveryTraceLog("area-to-corner");
    finishLog=DeliveryTraceLog("return-after-corner");
    returnLog.begin(nowMs(),wheelDistanceMm(),robot.getImuHeading());  // 暗黙の値継承に頼らず明示する

    // 帰りは青で止めない（青の本数はエリアの青の上から走り出すとずれるため）。
    // 90度コーナーを曲がりきってから一定距離を走った時点で終了し、ラリーへ引き渡す
    syslog(LOG_NOTICE, "Return: finish %d mm after the corner is cleared", kReturnFinishAfterCornerMm);

    static BlueStats returnBlueStats;
    returnBlueStats=BlueStats{};
    // 青の位置を、コーナーの距離と同じ原点（帰りのトレース開始）で出す。
    // 原点が違うと「青が判定窓の中に入ったか」をログから読めない
    returnBlueStats.distanceOriginMm=returnEntryStartMm;
    CornerState returnCorner;
    // 帰りにも直角コーナーが1つある。行きは左折だったが帰りは右折になる（Lコース基準）。
    // 判定の開始と減速は、トレース開始からの距離で行う（位置はボトル色で決まる）
    returnCorner.armedMs = returnEntryStartMs;
    returnCorner.armedMm = returnEntryStartMm;
    const int returnColorSteps = (bottleColor == ColorJudge::Color::RED) ? 2 : (bottleColor == ColorJudge::Color::BLUE) ? 1
                                                                                                                        : 0;
    const int returnCornerDetectStartMm = kReturnCornerDetectStartMmYellow + kReturnCornerColorStepMm * returnColorSteps;
    const int returnCornerSlowdownStartMm = kReturnCornerSlowdownStartMmYellow + kReturnCornerColorStepMm * returnColorSteps;
    bool isReturnCornerDetectStartPending = true;
    bool isReturnCornerSlowdownPending = true;
    bool isReturnFastTraceDone = false;  // 直線用の最速トレースが終わって減速に入ったか
    syslog(LOG_NOTICE, "Return trace start t=%d ms: slowdown at %d mm, corner detection at %d mm", returnCorner.armedMs, returnCornerSlowdownStartMm, returnCornerDetectStartMm);

    // 曲がりきった地点。終了までの距離の起点。-1はまだ曲がりきっていない
    int returnCornerClearedMm = -1;

    while(true) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            break;
        }

        const int returnTracedMm = wheelDistanceMm() - returnCorner.armedMm;
        if(isReturnCornerSlowdownPending && returnTracedMm >= returnCornerSlowdownStartMm) {
            isReturnCornerSlowdownPending = false;
            if(!returnCorner.done) {
                isReturnFastTraceDone = true;
                returnLog.end(nowMs(),wheelDistanceMm());
                tracer.setConfig(0.38f,0.01f,0.02f,Config::TRACER_TARGET_REFLECTION,kCornerApproachPwm);
                // 曲がりきると下でkReturnAfterCornerFastPwmに上書きする
                syslog(LOG_NOTICE, "Return corner slowdown to pwm %d t=%d ms (+%d mm since trace start)", kCornerApproachPwm, nowMs(), returnTracedMm);
            }
        }
        // 白の連続を数え始めるのは、直線用の最速トレースが終わって減速に入ってから。
        // 高速のまま揺れている区間で数え始めない。距離の下限も併せて残す
        // （時間で始めていた頃、エリア付近の揺れで白が9回続き黄の356mmで誤検知した）
        if(isReturnCornerDetectStartPending && isReturnFastTraceDone && returnTracedMm >= returnCornerDetectStartMm) {
            isReturnCornerDetectStartPending = false;
            returnCorner.pending = true;
            returnCorner.enableLoop = returnCorner.loopCount;  // 次の周期から有効
            syslog(LOG_NOTICE, "Return corner detection armed after slowdown (+%d mm since trace start)", returnTracedMm);
        }

        // エリア配置より後は、青を行動に一切使わない。
        // 以前はコーナー判定の抑制（青を跨ぐときの脇の白）に使っていたが、
        // 抑制が入るとコーナーの完了時刻＝ラリーへの引き渡し時刻まで青に左右される。
        // 読むのは診断のためだけで、制御へは渡さない
        if(!returnCorner.done) {
            isBlueReading(returnBlueStats);
        }

        // 帰りはLコースで右90度コーナーなので、行きとは逆の右へ回す。青は渡さない（常にfalse）
        const CornerUpdate returnCornerUpdate = updateCornerDetection(returnCorner, !isLeftCourse, kCornerPivotReturnMinTurnDeg, false, "Return corner");

        if(aborted) break;
        (void)returnCornerUpdate;
        // 曲がりきった後の設定は、検知側ではなくここで一度だけ決める
        if(returnCorner.done && returnCornerClearedMm < 0) {
            returnCornerClearedMm = wheelDistanceMm();
            armPostCornerSample("return", (int)returnCorner.doneReason, kReturnAfterCornerFastPwm);
            applyPostCornerTracerConfig(tracer, kReturnAfterCornerFastPwm);
            finishLog.begin(nowMs(),wheelDistanceMm(),robot.getImuHeading());
            syslog(LOG_NOTICE, "Return after corner: pwm %d for %d mm, then finish", kReturnAfterCornerFastPwm, kReturnFinishAfterCornerMm);
        }
        if(returnCornerClearedMm >= 0 && wheelDistanceMm() - returnCornerClearedMm >= kReturnFinishAfterCornerMm) {
            syslog(LOG_NOTICE, "Return finished: +%d mm since corner cleared, +%d mm since trace start. Finishing DeliveryTask.", wheelDistanceMm() - returnCornerClearedMm, wheelDistanceMm() - returnCorner.armedMm);
            break;
        }

        tracer.run();
        noteFirstFastControl();
        capturePostCornerSample(tracer);
        returnLog.sample(tracer,0.30f,nowMs(),wheelDistanceMm(),robot.getImuHeading());
        finishLog.sample(tracer,0.30f,nowMs(),wheelDistanceMm(),robot.getImuHeading());
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    robot.stop();
    // 走行そのものが終わった時刻。ここから先は診断の出力しかしない
    const int returnStoppedMs=nowMs();
    const int deliveryElapsedMs=returnStoppedMs-deliveryStartMs;
    returnLog.end(returnStoppedMs,wheelDistanceMm());finishLog.end(returnStoppedMs,wheelDistanceMm());
    printOutboundDiagnostics(blueStatsBeforeCorner, blueStatsAfterCorner);
    printPlacementDiagnostics();
    printPostCornerSamples();
    returnLog.print();finishLog.print();
    printTraceEntryRecords();
    logBlueStats("return", returnBlueStats);
    syslog(LOG_NOTICE, "Return end: corner done %d", returnCorner.done ? 1 : 0);
    syslog(LOG_NOTICE, "Corner stats: maxWhiteRun(before trigger) %d / threshold %d", returnCorner.maxWhiteRunBeforeTrigger, kCornerWhiteRunCount);
    // 走行だけを比べるための値。旧版は帰還後のログを総時間に含めていないので、
    // 旧版も新版も「total - outboundDiag」で比べる（runOnlyがその値）。
    // finalDiagは帰還して止まってからここまでに使った時間で、この行自身の出力は入らない
    syslog(LOG_NOTICE,"[Delivery] total %d ms outboundDiag %d ms placementAndSearch %d ms aborted %d",
           deliveryElapsedMs,outboundDiagnosticMs,placementElapsedMs,aborted?1:0);
    syslog(LOG_NOTICE,"[Delivery] timing: runOnly %d ms, outboundDiag %d ms, finalDiag %d ms, deferred %d",
           deliveryElapsedMs-outboundDiagnosticMs,outboundDiagnosticMs,nowMs()-returnStoppedMs,kDeferOutboundDiagnostics?1:0);
    syslog(LOG_NOTICE,"[Delivery] diagnostic skipped %d lines (output blocked %d)",
           diagnosticSkippedLines,diagnosticOutputBlocked?1:0);
    syslog(LOG_NOTICE, aborted ? "--- DeliveryTask ABORTED ---" : "--- DeliveryTask Finished ---");
}
