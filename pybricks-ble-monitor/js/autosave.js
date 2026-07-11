// 接続時にログを自動でlogフォルダへ保存する機能
//
// File System Access API はセキュリティ上、フォルダへの書き込み許可を
// ユーザー操作（クリック）なしに取得できない。そのため、最初に1回だけ
// 「保存先フォルダを選択」してもらい、その許可(FileSystemDirectoryHandle)を
// IndexedDBに保存しておくことで、次回以降は自動で（ダイアログなしで）
// 同じフォルダ配下の`log`サブフォルダにCSVを書き込めるようにしている。

const AUTOSAVE_DB_NAME = 'pybricks-ble-monitor';
const AUTOSAVE_STORE_NAME = 'handles';
const AUTOSAVE_DB_KEY = 'logBaseDirHandle';
const AUTOSAVE_FIELD_COUNT = 4;  // timestamp,tag,count + 最大4個の値

let autoSaveBaseDirHandle = null;  // ユーザーが選択したフォルダ
let autoSaveLogDirHandle = null;   // その配下に作る"log"フォルダ
let autoSaveFileHandle = null;
let autoSaveWritable = null;
let isAutoSaving = false;

// ── IndexedDBへのハンドル保存/復元 ──────────────────────────
// FileSystemDirectoryHandle は構造化複製可能なため、そのままIndexedDBに保存できる

function openAutoSaveDB() {
    return new Promise((resolve, reject) => {
        const req = indexedDB.open(AUTOSAVE_DB_NAME, 1);
        req.onupgradeneeded = () => {
            req.result.createObjectStore(AUTOSAVE_STORE_NAME);
        };
        req.onsuccess = () => resolve(req.result);
        req.onerror = () => reject(req.error);
    });
}

async function idbSetHandle(handle) {
    const db = await openAutoSaveDB();
    return new Promise((resolve, reject) => {
        const tx = db.transaction(AUTOSAVE_STORE_NAME, 'readwrite');
        tx.objectStore(AUTOSAVE_STORE_NAME).put(handle, AUTOSAVE_DB_KEY);
        tx.oncomplete = () => resolve();
        tx.onerror = () => reject(tx.error);
    });
}

async function idbGetHandle() {
    const db = await openAutoSaveDB();
    return new Promise((resolve, reject) => {
        const tx = db.transaction(AUTOSAVE_STORE_NAME, 'readonly');
        const req = tx.objectStore(AUTOSAVE_STORE_NAME).get(AUTOSAVE_DB_KEY);
        req.onsuccess = () => resolve(req.result || null);
        req.onerror = () => reject(req.error);
    });
}

// ── フォルダ選択・権限確認 ──────────────────────────────────

function updateAutoSaveStatus(text) {
    const el = document.getElementById('autoSaveStatus');
    if (el) el.textContent = text;
}

// 選択済みのautoSaveBaseDirHandleに対して、書き込み許可があるか確認し、
// なければユーザー操作の流れの中で許可を求めたうえで"log"フォルダを準備する
async function prepareAutoSaveLogDir() {
    if (!autoSaveBaseDirHandle) {
        console.log('[autosave] prepareAutoSaveLogDir: no base directory handle');
        return false;
    }

    try {
        let permission = await autoSaveBaseDirHandle.queryPermission({ mode: 'readwrite' });
        console.log('[autosave] queryPermission result:', permission);
        if (permission !== 'granted') {
            permission = await autoSaveBaseDirHandle.requestPermission({ mode: 'readwrite' });
            console.log('[autosave] requestPermission result:', permission);
        }
        if (permission !== 'granted') {
            updateAutoSaveStatus(`保存先: ${autoSaveBaseDirHandle.name}（権限が許可されていません: ${permission}。もう一度「保存先フォルダを選択」を押してください）`);
            return false;
        }

        autoSaveLogDirHandle = await autoSaveBaseDirHandle.getDirectoryHandle('log', { create: true });
        console.log('[autosave] log directory ready:', autoSaveBaseDirHandle.name + '/log');
        updateAutoSaveStatus(`保存先: ${autoSaveBaseDirHandle.name}/log/`);
        return true;
    } catch (error) {
        console.error('[autosave] Auto-save directory preparation error:', error);
        updateAutoSaveStatus(`保存先フォルダの準備に失敗しました: ${error.name}: ${error.message}`);
        return false;
    }
}

// ページ読み込み時に、前回選択したフォルダをIndexedDBから復元する
// （許可が生きていれば無言で復元、切れていればボタンでの再選択を促す）
async function restoreAutoSaveDirectory() {
    try {
        const handle = await idbGetHandle();
        if (!handle) {
            updateAutoSaveStatus('保存先フォルダが未選択です');
            return;
        }
        autoSaveBaseDirHandle = handle;

        const permission = await handle.queryPermission({ mode: 'readwrite' });
        if (permission === 'granted') {
            await prepareAutoSaveLogDir();
        } else {
            updateAutoSaveStatus(`保存先: ${handle.name}（「保存先フォルダを選択」を押して権限を再許可してください）`);
        }
    } catch (error) {
        console.log('No saved auto-save directory yet:', error);
        updateAutoSaveStatus('保存先フォルダが未選択です');
    }
}

// 「保存先フォルダを選択」ボタンのハンドラ
// どの経路でも必ずautoSaveStatusに結果（成功/失敗の理由）を表示する。
// alert()はVSCode内蔵ブラウザ等の一部環境でブロックされ気づけないことがあるため、
// 判定の主目的には使わず、画面上のテキスト表示を必ず更新することを優先する。
async function chooseAutoSaveDirectory() {
    console.log('[autosave] chooseAutoSaveDirectory clicked');

    if (!window.showDirectoryPicker) {
        console.error('[autosave] showDirectoryPicker is not available in this browser/context');
        updateAutoSaveStatus('このブラウザ/画面では自動保存に対応していません（Chrome/Edgeでindex.htmlを直接開いてください）');
        return;
    }

    try {
        // ボタンが押されたら常に新しくダイアログを開く（別のフォルダに変更したい場合もあるため）。
        // 既存フォルダの権限再確認はrestoreAutoSaveDirectory()（ページ読み込み時）でのみ行う。
        console.log('[autosave] opening directory picker');
        // mode:'readwrite'を指定しないと読み取り専用の許可しか得られず、
        // 後段のqueryPermission/requestPermissionがユーザー操作なしに
        // 書き込み権限へ昇格できず静かに失敗するため、ここで直接要求する
        const handle = await window.showDirectoryPicker({ mode: 'readwrite' });
        console.log('[autosave] directory selected:', handle.name);

        autoSaveBaseDirHandle = handle;

        try {
            await idbSetHandle(handle);
        } catch (idbError) {
            // 次回起動時の自動復元ができなくなるだけなので、今回の保存自体は続行する
            console.error('[autosave] failed to persist directory handle:', idbError);
        }

        await prepareAutoSaveLogDir();
    } catch (error) {
        if (error.name === 'AbortError') {
            // ユーザーが明示的にキャンセルした場合だけでなく、Chromeが保護対象の
            // フォルダ（ホーム/デスクトップ/ドキュメント等のルートやネットワークパス）
            // の選択を内部的に拒否した場合もAbortErrorになることがあるため、
            // 画面上にもヒントを残す（コンソールログだけだと見落としやすい）
            console.log('[autosave] directory selection was aborted (user cancel, or a protected/unsupported folder was chosen)');
            updateAutoSaveStatus('フォルダ選択がキャンセル、または許可されませんでした。ホーム/デスクトップ/ドキュメント等の直下ではなく、その中の特定のフォルダ（例: このリポジトリのフォルダ）を選び直してみてください。');
            return;
        }
        console.error('[autosave] chooseAutoSaveDirectory failed:', error);
        updateAutoSaveStatus(`フォルダ選択でエラーが発生しました: ${error.name}: ${error.message}`);
    }
}

// ── 自動保存の開始・停止・書き込み ──────────────────────────

function autoSaveFileName(date) {
    const pad = (n, w = 2) => String(n).padStart(w, '0');
    return `${date.getFullYear()}${pad(date.getMonth() + 1)}${pad(date.getDate())}_` +
        `${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}.csv`;
}

function formatTimestampISO(date) {
    const pad = (n, w = 2) => String(n).padStart(w, '0');
    return `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())}` +
        `T${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}.${pad(date.getMilliseconds(), 3)}`;
}

// データ分析しやすいよう、タグ(COLOR/MOTOR等)ごとに列数が違っても
// 常に同じ列構成(timestamp,tag,count,field1〜field4)になるよう整形する
function buildAutoSaveRow(entry) {
    const prefix = extractPrefix(entry.text);
    let tag = '';
    let count = '';
    let values;

    if (prefix) {
        const rest = entry.text.substring(prefix.length + 1).split(',');
        tag = prefix;
        count = rest[0] || '';
        values = rest.slice(1);
    } else {
        values = [entry.text];
    }

    while (values.length < AUTOSAVE_FIELD_COUNT) values.push('');
    values = values.slice(0, AUTOSAVE_FIELD_COUNT);

    const cells = [formatTimestampISO(entry.date || new Date()), tag, count, ...values];
    const escaped = cells.map(c => `"${String(c).replace(/"/g, '""')}"`);
    return escaped.join(',') + '\n';
}

// BLE接続成功時に呼ばれる。フォルダが準備できていなければ何もしない
async function startAutoSave() {
    const enabled = document.getElementById('autoSaveEnabled')?.checked;
    if (!enabled) return;

    if (!autoSaveLogDirHandle) {
        addLogEntry('自動保存が有効ですが保存先フォルダが未選択のため、今回は自動保存されません。「保存先フォルダを選択」を押してください。', 'info');
        return;
    }

    try {
        const filename = autoSaveFileName(new Date());
        autoSaveFileHandle = await autoSaveLogDirHandle.getFileHandle(filename, { create: true });
        autoSaveWritable = await autoSaveFileHandle.createWritable();

        // Excelでも文字化けしないようUTF-8 BOM付きにする
        const utf8Bom = new Uint8Array([0xEF, 0xBB, 0xBF]);
        await autoSaveWritable.write(utf8Bom);
        await autoSaveWritable.write('timestamp,tag,count,field1,field2,field3,field4\n');

        isAutoSaving = true;
        addLogEntry(`自動保存を開始しました: log/${filename}`, 'info');
    } catch (error) {
        console.error('Auto-save start error:', error);
        addLogEntry('自動保存の開始に失敗しました: ' + error.message, 'error');
    }
}

// BLE切断時に呼ばれる
async function stopAutoSave() {
    if (!isAutoSaving || !autoSaveWritable) return;

    try {
        await autoSaveWritable.close();
        addLogEntry('自動保存を終了しました', 'info');
    } catch (error) {
        console.error('Auto-save stop error:', error);
    } finally {
        isAutoSaving = false;
        autoSaveWritable = null;
        autoSaveFileHandle = null;
    }
}

// 受信データ1件ごとに呼ばれる（log.jsのaddLogEntryから）
async function writeAutoSaveEntry(entry) {
    if (!isAutoSaving || !autoSaveWritable) return;

    try {
        await autoSaveWritable.write(buildAutoSaveRow(entry));
    } catch (error) {
        console.error('Auto-save write error:', error);
    }
}
