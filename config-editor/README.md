# Config Editor

ブラウザから`main/app/Config.h`の数値定数を編集するローカルツールです。
`static constexpr`で宣言された編集可能な数値を自動検出するため、設定項目を追加しても
画面側の修正は必要ありません。

## 起動

リポジトリのルートで実行します。

```bash
./config-editor/start.sh
```

起動すると、`deploy.sh`と同じくPowerShell経由でWindows側のChromeを優先して
自動的に開きます。Chromeが見つからない場合は既定ブラウザを使用します。
自動的に開かない場合は、次のURLを開いてください。

```text
http://localhost:8080
```

終了するときは、起動したターミナルで`Ctrl+C`を押します。

## 仕組み

- 起動時は`Config.h`を正本として、編集可能な項目を`config.json`へ同期します。
- ブラウザで保存すると、`config.json`と`Config.h`が同時に更新されます。
- `Config.h`のセクションコメントを画面のカテゴリとして使用します。
- コメント内の`[...]`を単位として抽出し、数値の右側へ表示します。
- C++の型は数値の左側へ表示します。
- C++の型に合わせて、整数・浮動小数点数と整数型の範囲を検証します。
- ETラリーのゲート座標は1〜5かつ、左右の脚が隣接するように検証します。
- ETラリーのゲートは、色を選んでフィールドをクリックして配置できます。
- 赤・黄ゲートは横向き、青ゲートは縦向きに固定されます。
- ゲート配置ゾーンの灰色点は、左上を`row 1, col 1`、右下を`row 5, col 5`として扱います。

## パラメータの表示

各パラメータの入力欄は、左から`C++の型 | 数値 | 単位`の順で表示します。

例えば、次の定義は画面上で`int | 300 | °/秒`と表示されます。

```cpp
static constexpr int DRIVE_DEFAULT_SPEED_DEG_PER_SEC = 300;  // 直進の既定速度[°/秒]
```

- 型には`int`、`float`、`uint8_t`など、`Config.h`で宣言された型を表示します。
- 単位にはコメント内で最初に`[...]`で囲まれた文字列を表示します。
- `[...]`がないパラメータの単位欄は空欄になります。
- 単位として抽出した`[...]`は、説明文では重複表示しません。

## カテゴリ分け

カテゴリは、`Config.h`内にある次の形式の区切りコメントから自動的に判定します。

```cpp
// ── Tracer（ライントレース）──────────────────────────────
```

この例では、画面上のカテゴリ名は`Tracer（ライントレース）`になります。
区切りコメントの次にある`static constexpr`定数から、次の区切りコメントの直前にある
定数までが同じカテゴリに所属します。

```cpp
// ── Tracer（ライントレース）──────────────────────────────
static constexpr float TRACER_KP = 0.33f;
static constexpr float TRACER_KI = 0.01f;

// ── ColorJudge（色判定）──────────────────────────────────
static constexpr uint8_t COLOR_CHROMATIC_MIN_SATURATION = 30;
```

この場合、`TRACER_KP`と`TRACER_KI`は`Tracer（ライントレース）`、
`COLOR_CHROMATIC_MIN_SATURATION`は`ColorJudge（色判定）`として表示されます。

- 区切りコメントより前にある定数は`その他`に分類されます。
- `// 走行速度`のような通常のコメントは、カテゴリの区切りとして扱いません。
- カテゴリは折り返し可能なタグとしてすべて一覧表示します。
- タグを1つ選ぶと、選択したカテゴリの設定だけを表示します。
- タグ右側の数字は、そのカテゴリに含まれる設定数です。
- カテゴリ名は検索対象に含まれ、該当項目がないタグは薄く表示されます。
- `Config.h`へカテゴリを追加しても、Web UI側の修正は必要ありません。

### サブカテゴリー

メインカテゴリーの中をさらに分ける場合は、罫線を1本にした次のコメントを使用します。

```cpp
// ── Robot: 走行機能 ────────────────────────────────

// ─ driveStraight ─
static constexpr int DRIVE_DEFAULT_SPEED_DEG_PER_SEC = 300;
static constexpr int DRIVE_TIMEOUT_LOOP_COUNT = 2000;

// ─ turn ─
static constexpr int TURN_DEFAULT_SPEED_DEG_PER_SEC = 300;
static constexpr int TURN_TIMEOUT_LOOP_COUNT = 500;
```

この場合、`Robot: 走行機能`がメインカテゴリー、`driveStraight`と`turn`が
サブカテゴリーになります。Web UIではメインカテゴリーをタグで選択し、選択した
カテゴリー内の設定をサブカテゴリー見出しで区切って表示します。

- `// ── 名前 ──`のように先頭の罫線が2本ならメインカテゴリーです。
- `// ─ 名前 ─`のように先頭の罫線が1本ならサブカテゴリーです。
- メインカテゴリーが変わると、サブカテゴリーは`その他`へ戻ります。
- サブカテゴリーを指定していない定数は`その他`へ分類されます。
- `// driveStraight`のような通常コメントはサブカテゴリーとして扱いません。
- サブカテゴリー名も検索対象に含まれます。

## 編集対象外の派生定数

ほかの定数名を参照して計算される定数は、基準値から自動算出される派生値として
Config Editorの編集対象および`config.json`から除外します。

```cpp
static constexpr float BLUE_LINE_LENGTH_MM = 100.0f;  // 編集可能
static constexpr float BLUE_LINE_WIDTH_MM = 0.2f * BLUE_LINE_LENGTH_MM;  // 編集対象外
```

この例では`BLUE_LINE_LENGTH_MM`だけがWeb UIに表示されます。
`BLUE_LINE_WIDTH_MM`は`Config.h`内の式を維持し、基準値の変更に追従して
C++のコンパイル時に計算されます。

数値リテラルだけで構成された`10 * 1000`のような式は編集対象です。値を変更せず
保存した場合は元の式を維持し、Web UIで値を変更した場合は計算後の数値で更新します。

サーバーを起動せず同期だけ行う場合：

```bash
python3 config-editor/server.py --sync-only
```

ブラウザを開かずにサーバーだけ起動する場合：

```bash
python3 config-editor/server.py --no-browser
```
