#!/usr/bin/env bash
#
# deploy.sh
# SPIKE Primeの認識(usbipd attach)、ビルド、転送を一括で行うスクリプト
#

# エラー時に中断
set -e

# ターゲットディレクトリの指定 (引数があればそれを使用、なければ main)
TARGET_DIR=${1:-main}

# プロジェクト名の取得 (実行ファイルの親ディレクトリ名。例: ETrobo2026)
PRJ_NAME=$(basename $(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P))

# etrobo環境のルートディレクトリを取得 (../../ に相当)
ETROBO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)

echo "[ deploy: 準備中... ]"

# 環境変数の読み込み (spikeスクリプトなどが依存するため)
if [ -f "$ETROBO_ROOT/scripts/etrobopath.sh" ]; then
    export ETROBO_ROOT
    . "$ETROBO_ROOT/scripts/etrobopath.sh" > /dev/null 2>&1
fi

# 1. usbipd attach を試行
echo "[ deploy: SPIKE PrimeをWSLに認識させます... ]"
if "$ETROBO_ROOT/scripts/spike" usbipd attach; then
    echo "[ deploy: 認識に成功しました。 ]"
else
    echo "[ deploy: 警告: 認識に失敗したか、既に接続されています。 ]"
    echo "[ deploy: 続行します... ]"
fi

# 2. ビルドと転送の実行
echo "[ deploy: '$PRJ_NAME/$TARGET_DIR' のビルドと転送を開始します... ]"
cd "$ETROBO_ROOT"
MAKE_LOG_FILE=$(mktemp)
trap 'rm -f "$MAKE_LOG_FILE"' EXIT

if ! make app="$PRJ_NAME/$TARGET_DIR" up 2>&1 | tee "$MAKE_LOG_FILE"; then
    echo "[ deploy: 転送に失敗しました。USBを一度抜いてから、再度DFUモードで接続してください。 ]"
    exit 1
fi

if grep -q "upload failed: SPIKE device not found in DFU mode" "$MAKE_LOG_FILE"; then
    echo "[ deploy: 転送に失敗しました。USBを一度抜いてから、再度DFUモードで接続してください。 ]"
    exit 1
fi

echo "[ deploy: 完了しました。 ]"
