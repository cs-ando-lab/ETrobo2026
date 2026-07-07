# CI(継続的インテグレーション)

本リポジトリでは GitHub Actions を使い、`main/` ディレクトリ配下のコードに変更があるたびに「フォーマット」と「ビルド」を自動でチェックしています。ワークフロー定義は [`.github/workflows/`](../.github/workflows) にあります。

## 目次

- [ワークフロー一覧](#ワークフロー一覧)
- [フォーマットチェック (format-check.yaml)](#フォーマットチェック-format-checkyaml)
- [ビルドチェック (build-check.yaml)](#ビルドチェック-build-checkyaml)
- [Docker](#docker)
- [.vscode 設定](#vscode-設定)

## ワークフロー一覧

| ワークフロー | ファイル | トリガー | 内容 |
| --- | --- | --- | --- |
| Format and Fix | [`format-check.yaml`](../.github/workflows/format-check.yaml) | `main`/`master` への push、`main/**` を変更するPR、手動実行 | `clang-format` で自動整形し、差分があればコミットし直す |
| Build Check | [`build-check.yaml`](../.github/workflows/build-check.yaml) | `main` への push、`main/**` を変更するPR、手動実行 | Dockerコンテナ内でビルドし、エラーなく成功するか確認する |

いずれも対象パスは `main/**` のみで、`Docs/` などの変更だけでは実行されません。また、この2つ以外にIssue/PRのラベルを自動付与・同期する運用系ワークフロー（`auto-labeling.yaml`, `label-sync.yaml`）もありますが、コード品質には関わらないため本ドキュメントでは扱いません。

## フォーマットチェック (format-check.yaml)

このワークフローは、いわゆる「チェックのみで落とす」タイプのCIではなく、**その場で整形して自動コミットする**タイプです。

1. PRブランチ（`github.head_ref`。pushの場合は `github.ref_name`）をチェックアウトする。
2. `clang-format` をインストールする。
3. `main` 以下の `*.c` / `*.cpp` / `*.h` / `*.hpp` すべてに対して `clang-format -i` を実行する（整形ルールは [`.clang-format`](../.clang-format)、詳細は [`CODING_COMMENTS.md`](CODING_COMMENTS.md) を参照）。
4. 整形によって差分が生じていれば、`style: auto format with clang-format` というコミットとしてそのブランチに push し直す。

そのため、フォーマットが崩れた状態でPRを作成しても、ワークフロー実行後に自動整形コミットが追加されます。手元で `clang-format` を実行していなくても、pushやPR作成のタイミングで自動的に揃う仕組みです。ただし、その後 `git pull` してから作業を再開しないと、ローカルとリモートの内容がずれる点に注意してください。

## ビルドチェック (build-check.yaml)

このワークフローは、実際にETロボコンのビルド環境で `main/` がビルドできるかどうかを確認します。

1. リポジトリをチェックアウトする。
2. リポジトリ直下の [`Dockerfile`](../Dockerfile) から Dockerイメージ（`etrobo2026_builder`）をビルドする。
3. コンテナを起動し、`main/` ディレクトリをコンテナ内の `spike-rt` ワークスペース（`/etrobo/spike-rt/sdk/workspace/main`）にマウントした上で、`make img=main -C /etrobo/spike-rt/sdk/workspace` を実行してビルドする。

ビルドがエラーなく完了すればチェックは成功です。フォーマットチェックと異なり、失敗しても自動修正はされないため、ビルドエラーが出た場合はPR上でログを確認して修正する必要があります。

## Docker

[`Dockerfile`](../Dockerfile) は、CIと開発者のローカル環境の両方で同じビルド環境を再現するために使われています。

- ベースイメージ: `ubuntu:22.04`
- `build-essential` / `ruby` / `git` / `cmake` / `clang-format` などのビルドツール一式
- GNU Arm Embedded Toolchain (`gcc-arm-none-eabi-10.3-2021.10`)
- ETロボコンのプラットフォームである [`spike-rt`](https://github.com/ETrobocon/spike-rt-RasPike-ART) を `main` ブランチから浅いクローン(`--depth=1`)

CIの `build-check.yaml` はこのイメージをそのままビルド・実行しているだけなので、**CIでビルドが失敗する場合はローカルでも同じDockerイメージを使って再現・調査できます**。

ローカルで同様の環境を使いたい場合は、リポジトリ直下の [`compose.yaml`](../compose.yaml) を使ってコンテナを起動できます。

```bash
docker compose up -d --build
docker compose exec builder bash
```

`compose.yaml` はカレントディレクトリを `spike-rt/sdk/workspace/ETrobo2026` としてコンテナにマウントするため、コンテナ内に入ればホスト側の変更がそのまま反映された状態で `make` 等を実行できます（`command: tail -f /dev/null` によりコンテナはすぐには終了せず待機し続けます）。

## .vscode 設定

[`.vscode/`](../.vscode) 以下の設定は、エディタ上でもCI/Dockerと同じルールやビルド環境を意識できるようにするためのものです。

- [`settings.json`](../.vscode/settings.json)
  - `editor.formatOnSave: true` により、保存時に自動整形が走ります。
  - `C_Cpp.formatting: clangFormat` で、C/C++拡張機能に `.clang-format` のルールを使わせています。これにより、`format-check.yaml` が直すような差分をそもそもコミット前に防げます。
- [`extensions.json`](../.vscode/extensions.json)
  - `ms-vscode.cpptools`（C/C++のIntelliSense・デバッグ）、`twxs.cmake` / `ms-vscode.cmake-tools`（CMake連携）を推奨拡張機能として定義しています。
- [`c_cpp_properties.json`](../.vscode/c_cpp_properties.json)
  - コンパイラとして `arm-none-eabi-g++`（Dockerfileと同じ `gcc-arm-none-eabi-10.3-2021.10`）を指定し、`-mcpu=cortex-m4 -mthumb` を付与することで、実機と同じARMターゲット向けの解析をエディタに行わせています。
  - `includePath` には `spike-rt` (drivers, asp3, libpybricks) のヘッダ一式を指定しており、実際のビルドで解決されるインクルードパスとIntelliSenseのパスを一致させ、エディタ上での誤った赤波線（未解決インクルード等）を防いでいます。

これらの設定により、ローカルでの開発中からCIで検出されるような差分やビルドエラーに早い段階で気づけるようになっています。
