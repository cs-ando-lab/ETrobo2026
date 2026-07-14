# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

ETロボコン2026向けの走行体プログラム。SPIKE-RT（TOPPERS/ASP3ベースのRTOS）上で動く C++ アプリケーションで、LEGO SPIKE Prime Hub をターゲットにクロスコンパイルする。

## ビルド・実行・デプロイ

このリポジトリ単体では完結せず、`~/etrobo`（spike-rt環境）のワークスペース配下にクローンされている前提で動く。

```bash
# ~/etrobo から素のmakeで実行する場合
cd ~/etrobo
make app="ETrobo2026_code/main"        # ビルドのみ
make app="ETrobo2026_code/main" up     # ビルド＋実機(DFUモード)への転送

# このリポジトリ直下から一括実行する場合（推奨）
./deploy.sh              # usbipd attach → ビルド → 転送 → BLE Monitorをブラウザで自動起動
./deploy.sh <ディレクトリ名>   # main以外のサブディレクトリをビルド対象にする場合
```

CIやビルドエラーの再現にはDockerを使う（`Dockerfile`はCIと同じビルド環境を提供する）。

```bash
docker compose up -d --build
docker compose exec builder bash
# コンテナ内: make img=main -C /etrobo/spike-rt/sdk/workspace
```

**自動テストは存在しない。** 品質ゲートはCIの2つのワークフローのみ：
- `format-check.yaml`: `main/**` 変更時、`clang-format -i` を実行し差分があれば自動コミット＆pushし直す（要`git pull`)
- `build-check.yaml`: Dockerイメージ上で `make img=main` を実行し、ビルドが通るかのみ確認する

いずれも変更が `main/**` 配下でないと発火しない（`Docs/`のみの変更等ではCIは動かない）。

## アーキテクチャ

### エントリポイントと全体フロー

`main/app.cpp` の `main_task()` が唯一の起点。周期タスクは使っておらず、`Robot`（ハードウェア一式を所有）と `GameRunner` をグローバルに1個ずつ構築し、`runner.run()` を呼ぶだけの単純な構成。

`GameRunner::run()`（[main/app/GameRunner.cpp](main/app/GameRunner.cpp)）が全体の流れを制御する：

```
Calibrator（L/R選択・スタート待ち）
→ [lineTraceUntilLap → SumoTask]
→ [lineTraceUntilLap → DeliveryTask]
→ [lineTraceUntilLap → RallyTask]
→ lineTraceUntilLap → ゴール
```

`lineTraceUntilLap()` は `Tracer` でライントレースしながら青（LAPゲート）を検出するまでループする。各Taskは、ゲートで離脱してから**元の位置・角度に戻ってくる**実装が前提になっている（そうしないと次の`lineTraceUntilLap()`が正しく機能しない）。

### レイヤー構成

- **`Robot`**（[main/app/Robot.h](main/app/Robot.h)）: モーター・センサー・HMIの実体を保持する唯一のクラス。他クラスは必ずRobot経由でハードウェアを操作する。`driveStraight`/`turn`はホイールエンコーダのみに基づくデッドレコニングで、ジャイロ/IMUは使っていない。`runUntilColor`/`runUntilColors`で色検出停止の直進・蛇行ができる。
- **`Tracer`**（[main/app/Tracer.h](main/app/Tracer.h)）: `Pid`と`ColorJudge`を使った反射率ベースのライントレース。1個のカラーセンサーで黒/白の境界（エッジ）を追従する方式。
- **`Pid`**（[main/app/Pid.h](main/app/Pid.h)）: 汎用PIDクラス。積分項クランプ・微分項ローパスフィルタ内蔵で、Tracer以外の制御にも使い回せる。
- **`ColorJudge`**（[main/app/ColorJudge.h](main/app/ColorJudge.h)）: RGB/HSV/反射率から色（黒/白/赤/緑/青/黄）を判定する処理を集約。閾値は全て`Config`に定義。
- **`CourseConfig`**（[main/app/CourseConfig.h](main/app/CourseConfig.h)）: L/Rコース選択状態を保持するstaticクラス。`CourseConfig::sign()`で旋回方向をコースに応じて反転できる。
- **`Calibrator`**（[main/app/Calibrator.h](main/app/Calibrator.h)）: 起動時の準備（L/R選択、スタート合図待ち）。
- **`Config`**（[main/app/Config.h](main/app/Config.h)）: チューニング用定数の一元管理。速度・タイムアウト・PIDゲイン・色判定閾値などをここに集約し、他クラスのコード中に定数を直書きしない方針。
- **`main/app/tasks/`**: 各課題（`SumoTask`/`DeliveryTask`/`RallyTask`）を1クラス1ファイルで実装。`Robot&`を受け取り`run()`を1回呼べば完結するインターフェースで統一。

### デバッグ・ログ

`main/debug_log.cpp` がセンサー値を`syslog()`経由でBLE送信する。受信側は`pybricks-ble-monitor/`（オフライン動作するブラウザ製ツール、MITライセンスで同梱・改変）。`deploy.sh`で転送後に自動起動し、ログは`log/`に自動保存される。

## 新しいソースファイルを追加する場合

`main/Makefile.inc` の `APPL_CXXOBJS` にオブジェクト名を追記しないとビルド対象に含まれない。`main/app/`直下と`main/app/tasks/`はすでにインクルードパスに入っているので、この2つの配下に置く分にはパス追加は不要。

## コーディング規約

- `.clang-format`（LLVMベース、4スペースインデント、タブ禁止、`ColumnLimit: 0`）に従う。保存時に自動整形される設定（`.vscode/settings.json`）。
- コメントは「なぜそのコードが必要か」を書く（何をするかは名前から自明なら書かない）。

## Git / PR運用

- GitHub Flow（`main` + feature branch）。`main`への直接pushは禁止、PR経由でマージ（Squash and merge推奨）。
- ブランチ名は `<type>/<description>`（`feat/`, `fix/`, `docs/`, `style/`, `refactor/`, `test/`, `chore/`）。
- コミットメッセージはConventional Commits（`feat: ...`, `fix: ...`など）。
- PRには関連Issueを`closes #番号`の形式で記載するとマージ時に自動クローズされる。
- `format-check`によって自動整形コミットがブランチに追加されることがあるため、push後に作業を再開する前は必ず`git pull`する。
- UMLモデル（astah*）を変更した場合は`models/diagrams/`配下にPNG/SVGを添えてコミットする（詳細: [Docs/MODEL_WORKFLOW.md](Docs/MODEL_WORKFLOW.md)）。
