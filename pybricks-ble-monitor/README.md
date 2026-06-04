# Pybricks BLE Monitor

Pybricks IDEのBluetooth通信と同じプロトコル（Nordic UART Service）を使用し、ブラウザからPybricks/SPIKE-RTハブに接続してデータを表示・保存するアプリです。

**Pybricks IDEとの違い:**
- Pybricks IDEはインターネット接続が必要ですが、本アプリは**インターネット接続不要**で動作します
- ローカルファイルとして実行可能で、オフライン環境でも使用できます

<img width="1858" height="924" alt="image" src="https://github.com/user-attachments/assets/96fe9ec3-9abf-4da9-b93b-572efe6eddfb" />

## 機能

### 基本機能
- **BLE接続** - PybricksハブへのBluetooth接続
- **リアルタイムデータ表示** - センサーデータの即時表示
- **フィルタ機能** - キーワードによるデータ絞り込み

### 表示オプション
- **自動スクロール** - 最新データを自動表示
- **タイムスタンプ表示** - 受信時刻の表示/非表示
- **CSVセル表示モード** - カンマ区切りデータをセル単位で表示
- **ログ追加停止** - バッファへの追加を停止（表示は維持）

### 保存機能
- **クリップボードにコピー** - 表示中のログをコピー
- **CSVで保存** - Excel対応形式で保存
- **テキストで保存** - プレーンテキストで保存
- **ストリーミング保存** - リアルタイムでファイルに追記（大容量対応）

## 使用方法

### 起動
```bash
start.bat
```

または `index.html` をChrome/Edgeで開く

### 接続
1. **「接続」ボタンをクリック**
2. **デバイス選択ダイアログでPybricksハブを選択**
3. **ペアリングを許可**

### データ保存

#### 通常保存（接続後に実行）
- 「CSVで保存」または「テキストで保存」をクリック

#### ストリーミング保存（大容量データ向け）
1. 「ストリーミング保存開始」をクリック
2. ファイル保存場所を選択
3. データ受信時に自動的にファイルに追記
4. 「ストリーミング保存停止」で終了

## 対応ブラウザ

| ブラウザ | 対応状況 |
|----------|----------|
| Chrome | ✅ 完全対応 |
| Edge | ✅ 完全対応 |
| Firefox | ❌ 未対応（Web Bluetooth API非対応） |
| Safari | ❌ 未対応（Web Bluetooth API非対応） |

## 技術仕様

| 項目 | 仕様 |
|------|------|
| 通信方式 | Bluetooth Low Energy (BLE) |
| API | Web Bluetooth API |
| サービス | Nordic UART Service (NUS) |
| Pybricks Service UUID | `c5f50001-8280-46da-89f4-6d8051e4aeef` |
| NUS Service UUID | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |

## ファイル構成

```
.
├── index.html          # メインHTML
├── css/
│   └── style.css       # スタイルシート
├── js/
│   ├── main.js         # 定数、変数、ユーティリティ
│   ├── bluetooth.js    # Bluetooth接続
│   ├── log.js          # ログ管理、表示
│   └── file.js         # ファイル保存
├── start.bat           # Windows起動スクリプト
└── README.md           # このファイル
```

## SPIKE-RT (Pybricks) 側の設定

### 方法1: SysLogを使用する場合

`app.cdl` を変更して、システムログタスクの出力先をBluetoothに変更します。

#### 設定手順

**ファイル:** `app.cdl` (134行目付近)

**変更前 (USBシリアル):**
```cdl
cell tLogTask LogTask {
  priority  = 4;
  stackSize = LogTaskStackSize;

  /* シリアルインタフェースドライバとの結合 */
  cSerialPort        = SerialPortUSB1.eSerialPort;
  cnSerialPortManage = SerialPortUSB1.enSerialPortManage;
  
  /* システムログ機能との結合 */
  cSysLog = SysLog.eSysLog;

  /* 低レベル出力との結合 */
  cPutLog = PutLogTarget.ePutLog;
};
```

**変更後 (Bluetoothシリアル):**
```cdl
cell tLogTask LogTask {
  priority  = 4;
  stackSize = LogTaskStackSize;

  /* シリアルインタフェースドライバとの結合 */
  cSerialPort        = SerialPortBluetooth1.eSerialPort;
  cnSerialPortManage = SerialPortBluetooth1.enSerialPortManage;
  
  /* システムログ機能との結合 */
  cSysLog = SysLog.eSysLog;

  /* 低レベル出力との結合 */
  cPutLog = PutLogTarget.ePutLog;
};
```

#### データ送信例

```c
#include <t_syslog.h>

void send_sensor_data(int sensor_id, int value1, int value2) {
    /* syslog経由でデータを送信 */
    syslog(LOG_NOTICE, "%d,%d,%d", sensor_id, value1, value2);
}
```

送信されたデータはPybricks BLE MonitorでCSV形式として解釈されます：
```
"14:32:10.123","1","39.00","15.00"
```

### 方法2: 直接送信する場合

`app.cdl` を変更せずに、アプリケーションから直接Bluetoothシリアルに送信します。

#### 設定手順

**注意:** ユーザープログラムでBluetoothデバイスをオープンしておく必要があります。これは接続時のデバイス一覧に表示させるために必要です。

```c
#include <serial.h>

void init_bluetooth() {
    /* Bluetoothシリアルポートをオープン (ポート番号2) */
    /* ポート番号2はBluetoothを指します（USBは1） */
    serial_opn_por(2);
}

void send_data_direct(int sensor_id, int value1, int value2) {
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "%d,%d,%d\n", sensor_id, value1, value2);
    
    /* Bluetoothシリアルに直接書き込み (ポート番号2) */
    serial_wri_dat(2, buffer, len);
}
```

## 注意事項

- **Pybricks IDEとの同時接続は不可** - 事前にIDEから切断してください
- **ストリーミング保存はChrome/Edgeのみ** - File System Access API使用
- **ログ追加停止中もストリーミング保存は動作** - ファイルには書き込まれる
- **1行の最大文字数は256文字** - それを超える場合は分割して表示されます

## ライセンス

MIT License
