#!/usr/bin/env bash
#
# deploy.sh
# SPIKE Primeの認識(usbipd attach)、ビルド、転送を一括で行うスクリプト
#

# エラー時に中断
# pipefail: パイプ内のいずれかのコマンドが失敗した場合もパイプ全体を失敗として扱う
# (これがないと `make ... | tee ...` はteeの終了コード(常に0)を返してしまい、makeの失敗を検知できない)
set -e
set -o pipefail

# ターゲットディレクトリの指定 (引数があればそれを使用、なければ main)
TARGET_DIR=${1:-main}

# スクリプト自身のディレクトリ (絶対パス。以降cdしても変わらないようにここで確定させる)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)

# プロジェクト名の取得 (実行ファイルの親ディレクトリ名。例: ETrobo2026_code)
PRJ_NAME=$(basename "$SCRIPT_DIR")

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

if grep -qE "upload failed: SPIKE device not found in DFU mode|No DFU device found" "$MAKE_LOG_FILE"; then
    echo "[ deploy: 転送に失敗しました。USBを一度抜いてから、再度DFUモードで接続してください。 ]"
    exit 1
fi

echo "[ deploy: 完了しました。 ]"

# 3. BLE Monitorの起動 (Windows側のChromeでhttp://localhost経由のindex.htmlを開く)
#
# file://wsl.localhost/... のようなホスト名付きURLで直接index.htmlを開くと、
# ChromeがそのoriginにFile System Access API(自動保存機能が使う)の権限を
# 正しく与えないことがあるため、ローカルサーバー経由でhttp://localhostとして開く。
echo "[ deploy: BLE Monitorを起動します... ]"
cd "$SCRIPT_DIR"

BLE_PORT=8765
BLE_MONITOR_DIR="$SCRIPT_DIR/pybricks-ble-monitor"

# 既にサーバーが起動していれば二重起動しない (/dev/tcpでポートの応答を確認)
if ! timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$BLE_PORT" 2>/dev/null; then
    echo "[ deploy: BLE Monitor用のローカルサーバーを起動します (port $BLE_PORT)... ]"
    (cd "$BLE_MONITOR_DIR" && nohup python3 -m http.server "$BLE_PORT" --bind 127.0.0.1 \
        > /tmp/pybricks-ble-monitor-server.log 2>&1 &)
    sleep 1
fi

BLE_MONITOR_URL="http://localhost:$BLE_PORT/index.html"
BLE_TAB_TITLE="Pybricks BLE Monitor"
CHROME_PATH='C:\Program Files\Google\Chrome\Application\chrome.exe'

# 実行ポリシーの制約を受けないよう、.ps1ファイルではなくコマンドテキストとして直接渡す
PS_SCRIPT=$(cat <<'PSEOF'
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public class NativeWindow {
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
    [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr hWnd);
}
"@

$root = [System.Windows.Automation.AutomationElement]::RootElement
$winCond = New-Object System.Windows.Automation.PropertyCondition([System.Windows.Automation.AutomationElement]::ClassNameProperty, 'Chrome_WidgetWin_1')
$allWindows = $root.FindAll([System.Windows.Automation.TreeScope]::Children, $winCond)
$chromeWindows = $allWindows | Where-Object { (Get-Process -Id $_.Current.ProcessId -ErrorAction SilentlyContinue).ProcessName -eq 'chrome' }

$tabCond = New-Object System.Windows.Automation.PropertyCondition([System.Windows.Automation.AutomationElement]::ControlTypeProperty, [System.Windows.Automation.ControlType]::TabItem)
$found = $null
foreach ($win in $chromeWindows) {
    $tabs = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $tabCond)
    foreach ($tab in $tabs) {
        if ($tab.Current.Name -like "__BLE_TAB_TITLE__*") {
            $found = New-Object PSObject -Property @{ Window = $win; Tab = $tab }
        }
    }
}

if ($null -ne $found) {
    $sel = $found.Tab.GetCurrentPattern([System.Windows.Automation.SelectionItemPattern]::Pattern)
    $sel.Select()
    $hwnd = [IntPtr]$found.Window.Current.NativeWindowHandle
    if ([NativeWindow]::IsIconic($hwnd)) { [NativeWindow]::ShowWindow($hwnd, 9) | Out-Null }
    [NativeWindow]::SetForegroundWindow($hwnd) | Out-Null
    Write-Host "既存のBLE Monitorタブを前面に表示しました"
} else {
    Start-Process -FilePath "__CHROME_PATH__" -ArgumentList "__BLE_MONITOR_URL__"
    Write-Host "BLE Monitorを新規に起動しました"
}
PSEOF
)
PS_SCRIPT=${PS_SCRIPT//__BLE_TAB_TITLE__/$BLE_TAB_TITLE}
PS_SCRIPT=${PS_SCRIPT//__CHROME_PATH__/$CHROME_PATH}
PS_SCRIPT=${PS_SCRIPT//__BLE_MONITOR_URL__/$BLE_MONITOR_URL}

powershell.exe -NoProfile -NonInteractive -Command "$PS_SCRIPT"
