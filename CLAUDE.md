# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

ETロボコン2026向けの走行体プログラム。SPIKE-RT（TOPPERS/ASP3ベースのRTOS）上で動く C++ アプリケーションで、LEGO SPIKE Prime Hub をターゲットにクロスコンパイルする。

## ビルド・実行・デプロイ

このリポジトリ単体では完結せず、`~/etrobo`（spike-rt環境）のワークスペース（`~/etrobo/workspace/` = `~/etrobo/spike-rt/sdk/workspace/` へのシンボリックリンク）配下にクローンされている前提で動く。

```bash
# ~/etrobo から素のmakeで実行する場合（<リポジトリのディレクトリ名>はクローン先の名前）
cd ~/etrobo
make app="<リポジトリのディレクトリ名>/main"        # ビルドのみ
make app="<リポジトリのディレクトリ名>/main" up     # ビルド＋実機(DFUモード)への転送

# このリポジトリ直下から一括実行する場合（推奨）
./deploy.sh              # usbipd attach → ビルド → 転送 → BLE Monitorをブラウザで自動起動
./deploy.sh <ディレクトリ名>   # main以外のサブディレクトリをビルド対象にする場合（例: ./deploy.sh charge）
```

ビルド対象になるアプリのディレクトリは2つある。

- `main/`: 競技用の走行体プログラム（通常はこちら）
- `charge/`: バッテリー残量・充電状態をハブのディスプレイに表示するだけの単機能アプリ

**自動テストは存在しない。** コード品質に関わるCIワークフローは2つ：

- `format-check.yaml`: `main/**` 変更時、`clang-format -i` を実行し差分があれば自動コミット＆pushし直す（要`git pull`)
- `build-check.yaml`: Dockerイメージ上で `make img=main` を実行し、ビルドが通るかのみ確認する

いずれも変更が `main/**` 配下でないと発火しない（`Docs/`のみの変更等ではCIは動かない）。詳細は[Docs/CI.md](Docs/CI.md)を参照。

他に`auto-labeling.yaml`/`label-sync.yaml`（Issue/PRのラベル自動管理）・`notify-discord.yaml`（Discord通知）が`.github/workflows/`にあるが、コード品質には関わらない運用系ワークフローなのでここでは扱わない。

## アーキテクチャ

### エントリポイントと全体フロー

`main/app.cpp` には2つのタスクがある（定義は`main/app.cfg`・`main/app.h`）。

- `main_task`: 唯一の走行処理の起点。`Robot`（ハードウェア一式を所有）と `GameRunner` をグローバルに1個ずつ構築し、`runner.run()` を呼ぶだけ。復帰後はハブをシャットダウンする。
- `debug_log_task`: `DEBUG_LOG_PERIOD`（100ms）周期で起動される独立タスク。`main_task`が何をしていてもセンサー値をBLEへ流し続ける。

走行制御そのものに周期タスクは使っておらず、各ループが`dly_tsk()`で待つ方式になっている。

`GameRunner::run()`（[main/app/GameRunner.cpp](main/app/GameRunner.cpp)）が全体の流れを制御する：

```
Calibrator（L/Rコース選択・フォースセンサーでスタート）
→ フォースセンサーが離されるのを待つ
→ 開始モード選択（O/D/R/T、左右ボタンで選択しフォースセンサーで確定）
→ [O] lineTraceUntilLap → DeliveryTask → RallyTask → moveTowardGarageArea
→ [D] lineTraceUntilLap → DeliveryTask
→ [R] RallyTask
→ [T] Test（tasks/test.cpp。関数の試し打ち用）
```

各モードの分岐は `if(startMode <= N)` で書かれており、選んだモード以降のブロックが順に実行される（`O`を選ぶと本番ブロックの後にD/R/Tのブロックも続けて動く点に注意。本番運用前に見直しが必要な箇所）。

`lineTraceUntilLap()` は `Tracer` でライントレースしながら青（LAPゲート）を検出するまでループする。各Taskは、ゲートで離脱してから**元の位置・角度に戻ってくる**実装が前提になっている（そうしないと次の`lineTraceUntilLap()`が正しく機能しない）。

開始モード選択は「試走会用に途中の課題から単独で試したい」ための暫定機能。本番フローでは常にO（本番 = 全課題実行）を想定する。

`SumoTask`（ET相撲）と、その復路計算に使う`Odometry`はビルド対象には残っているが、現在の`GameRunner::run()`からは呼ばれていない。

### レイヤー構成

- **`Robot`**（[main/app/Robot.h](main/app/Robot.h)）: モーター・センサー・HMIの実体を保持する唯一のクラス。他クラスは必ずRobot経由でハードウェアを操作する。走行系は2系統ある。
  - エンコーダのみのデッドレコニング: `driveStraight` / `turn`
  - IMU（ジャイロ）を併用する系: `driveStraightByImu`（目標heading維持＋停止前減速）/ `turnByImu`（最短角で旋回し実測角を返す）/ `turnByImuUntilUltrasonic`（旋回しながら超音波で対象を探索）
  - 色で停止する走行: `runStraightUntilColor(s)`（直進）/ `runWavingUntilColor(s)`（蛇行探索）
  - 判定ヘルパ: `isOnColor(s)`（連続一致回数を呼び出し側で持つ方式）
- **`Tracer`**（[main/app/Tracer.h](main/app/Tracer.h)）: `Pid`を使った反射率ベースのライントレース。1個のカラーセンサーで黒/白の境界（エッジ）を追従する。`|turn|`のEMAに応じて基準PWMから減速するカーブ減速機能を内蔵。`setEdge()`で追従エッジ（LEFT/RIGHT）を切り替えられる（既定はRIGHT）。モーターへは`setPower()`（オープンループのPWM）で出力する。
- **`Pid`**（[main/app/Pid.h](main/app/Pid.h)）: 汎用PIDクラス。積分項クランプ・微分項ローパスフィルタ内蔵で、Tracer以外の制御にも使い回せる。`calculate()`には前回からの経過時間`deltaSec`を渡す。
- **`ColorJudge`**（[main/app/ColorJudge.h](main/app/ColorJudge.h)）: RGB/HSV/反射率から色（黒/白/赤/緑/青/黄）を判定する処理を集約。彩度が閾値未満なら無彩色として反射率で黒/白を分け、それ以外はHueが最も近い色を選ぶ。閾値は全て`Config`に定義。
- **`CourseConfig`**（[main/app/CourseConfig.h](main/app/CourseConfig.h)）: L/Rコース選択状態を保持するstaticクラス。`CourseConfig::sign()`で旋回方向をコースに応じて反転できる。
- **`Odometry`**（[main/app/Odometry.h](main/app/Odometry.h)）: 旋回・直進の実測値を積算し自己位置(x, y, 向き)を追跡するクラス。現在は`SumoTask`専用で、他からは使われていない。
- **`Calibrator`**（[main/app/Calibrator.h](main/app/Calibrator.h)）: 起動時の準備（ビープ→BLE接続待ち→L/Rコース選択→フォースセンサーでスタート）。
- **`Config`**（[main/app/Config.h](main/app/Config.h)）: チューニング用定数の一元管理。速度・タイムアウト・PIDゲイン・色判定閾値・コース寸法などをここに集約し、他クラスのコード中に定数を直書きしない方針（実機調整途中の値が各タスクの無名namespaceに残っている箇所があるが、確定したものはConfigへ移す）。
- **`main/app/tasks/`**: 各課題（`DeliveryTask`/`RallyTask`/`SumoTask`）と試験用の`Test`を実装。`Robot&`を受け取り`run()`を1回呼べば完結するインターフェースで統一する。
  - `main/app/tasks/RallyTask/`: ETラリー専用の補助クラス。`RallyTypes`（ゲート・ノード・方角の型）、`RallyRoute`（曲がる回数→歩数の順にコスト最小となる経路探索）、`HeadingCalibration`（ライントレース中のジャイロ角をリングバッファで平均し基準角を決める）。

### デバッグ・ログ

`main/debug_log.cpp` がカラーセンサー（RGB・HSV生値・反射率）・モーター・IMU・超音波センサー・フォースセンサー・ボタン・バッテリー電圧/電流の値を`syslog()`経由でBLE送信する。`syslog()`の引数上限（5個）に合わせてタグを分割し、値が変化したときだけ送るデルタ送信になっている。`debug_log.h`の`DEBUG_LOG`を0にすると全て無効化される。

受信側は`pybricks-ble-monitor/`（オフライン動作するブラウザ製ツール、MITライセンスで同梱・改変）。`deploy.sh`で転送後に自動起動し、ログは`log/`に自動保存される（`log/`はgit管理外）。詳細は[Docs/BLE_CONNECT.md](Docs/BLE_CONNECT.md)。

### Config Editor

`config-editor/`は、ブラウザから`Config.h`の数値定数を編集するローカルツール（Python製、`./config-editor/start.sh`で起動、http://localhost:8080 ）。`Config.h`が正本で、起動時に`config-editor/config.json`へ同期され、保存すると両方が更新される。詳細は[config-editor/README.md](config-editor/README.md)。

**`Config.h`を編集するときは、このツールの解析規則に沿った書き方を守ること。**

- `// ── 名前 ──`（罫線2本）がメインカテゴリ、`// ─ 名前 ─`（罫線1本）がサブカテゴリの区切りとして解釈される。
- 行末コメントの最初の`[...]`が単位として抽出され、UIの単位欄に表示される。
- 他の定数名を参照して計算している定数（例: `BLUE_LINE_WIDTH_MM = 0.2 * BLUE_LINE_LENGTH_MM`）は派生値として編集対象から自動的に除外される。基準値から算出できるものはこの形で書く。

## 新しいソースファイルを追加する場合

`main/Makefile.inc` の `APPL_CXXOBJS` にオブジェクト名を追記しないとビルド対象に含まれない。ソースを置くディレクトリが`APPL_DIRS`にも入っている必要がある（現在は`main/app`・`main/app/tasks`・`main/app/tasks/RallyTask`）。

インクルードパス（`INCLUDES`）に入っているのは`main/app`と`main/app/tasks`の2つだけで、`main/app/tasks/RallyTask`は入っていない。そのため`RallyTask/`配下のヘッダは`#include "RallyTask/RallyRoute.h"`のようにサブディレクトリ名を付けて書く。

## コーディング規約

- `.clang-format`（LLVMベース、4スペースインデント、タブ禁止、`ColumnLimit: 0`）に従う。保存時に自動整形される設定（`.vscode/settings.json`）。
- コメントは「なぜそのコードが必要か」を書く（何をするかは名前から自明なら書かない）。
- 詳細: [Docs/CODING_COMMENTS.md](Docs/CODING_COMMENTS.md)

## Git / PR運用

- GitHub Flow（`main` + feature branch）。`main`への直接pushは禁止、PR経由でマージ（Squash and merge推奨）。
- ブランチ名は `<type>/<description>`（`feat/`, `fix/`, `docs/`, `style/`, `refactor/`, `test/`, `chore/`）。
- コミットメッセージはConventional Commits（`feat: ...`, `fix: ...`など）。
- PRには関連Issueを`closes #番号`の形式で記載するとマージ時に自動クローズされる。
- `format-check`によって自動整形コミットがブランチに追加されることがあるため、push後に作業を再開する前は必ず`git pull`する。
- UMLモデル（astah\*）を変更した場合は`models/diagrams/`配下にPNG/SVGを添えてコミットする（詳細: [Docs/MODEL_WORKFLOW.md](Docs/MODEL_WORKFLOW.md)）。
- 詳細: [Docs/GIT_WORKFLOW.md](Docs/GIT_WORKFLOW.md)

## このファイル（CLAUDE.md）の保守

**コードの修正・機能追加を行ったら、その内容に合わせてこのCLAUDE.mdも同じPRの中で更新すること。** 実装とこのファイルの記述がずれると、以降の作業で誤った前提のまま進めることになる。特に次のような変更があった場合は必ず見直す。

- `GameRunner::run()`の全体フロー・モード構成が変わった
- クラスを追加/削除/改名した、責務を移動した（「レイヤー構成」の該当項目）
- `Robot`の公開メソッドを追加/改名した（メソッド名を挙げている箇所）
- ビルド手順・`Makefile.inc`・インクルードパスの構成が変わった
- ツール（`deploy.sh`・`config-editor`・BLE Monitor）の使い方や前提が変わった
- CIワークフロー・ブランチ運用のルールが変わった

記述量を増やすことが目的ではないので、詳細な手順は`Docs/`配下に書き、このファイルには「どこに何があるか」と「知らないと事故る前提」だけを残す。

## 関連ドキュメント

`Docs/`配下に、このファイルより詳しい説明がある。

- [Docs/CONTRIBUTING.md](Docs/CONTRIBUTING.md): クローンからmainマージまでの開発フロー全体（Windows VSCode想定）
- [Docs/CI.md](Docs/CI.md): CIワークフローの詳細
- [Docs/CODING_COMMENTS.md](Docs/CODING_COMMENTS.md): コーディング・コメント規約
- [Docs/GIT_WORKFLOW.md](Docs/GIT_WORKFLOW.md): ブランチ戦略・コミットフロー
- [Docs/BLE_CONNECT.md](Docs/BLE_CONNECT.md): BLE Monitorでの走行体内部状態の監視方法
- [Docs/MODEL_WORKFLOW.md](Docs/MODEL_WORKFLOW.md): UMLモデル(astah\*)の運用
- [config-editor/README.md](config-editor/README.md): Config Editorの使い方と`Config.h`の記法
