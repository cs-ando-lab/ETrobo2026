// Constants
const NUS_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_TX_CHAR_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_RX_CHAR_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
const PYBRICKS_SERVICE_UUID = 'c5f50001-8280-46da-89f4-6d8051e4aeef';

// タブごとの設定（列ヘッダー・閾値・カラープレビュー）
const TAB_CONFIGS = {
    'COLOR': {
        headers: ['count', 'R', 'G', 'B', '反射光(%)'],
        colorPreview: true,
        thresholds: []
    },
    'COLOR2': {
        headers: ['count', 'H', 'S', 'V'],
        thresholds: []
    },
    'MOTOR': {
        headers: ['count', '左速度(d/s)', '右速度(d/s)', '左cnt', '右cnt'],
        thresholds: []
    },
    'MOTOR2': {
        headers: ['count', '左パワー', '右パワー', '左失速', '右失速'],
        thresholds: [{ col: 3, op: 'eq', val: 1 }, { col: 4, op: 'eq', val: 1 }]  // 失速中
    },
    'IMU': {
        headers: ['count', '方位角(deg)', '加速度X', '加速度Y', '加速度Z'],
        thresholds: []
    },
    'IMU2': {
        headers: ['count', '角速度X(d/s)', '角速度Y(d/s)', '角速度Z(d/s)'],
        thresholds: []
    },
    'ULTRA': {
        headers: ['count', '距離(mm)'],
        thresholds: [{ col: 1, op: 'eq', val: -1 }]  // -1 = 測定不能
    },
    'FORCE': {
        headers: ['count', '接触', '力(N×100)', '押込距離(mm×100)'],
        thresholds: [{ col: 1, op: 'eq', val: 1 }]   // 接触中
    },
    'BUTTON': {
        headers: ['count', '左', '中央', '右', 'Bluetooth'],
        thresholds: [{ col: 1, op: 'eq', val: 1 }, { col: 2, op: 'eq', val: 1 }, { col: 3, op: 'eq', val: 1 }, { col: 4, op: 'eq', val: 1 }]
    }
};

// State variables
let device = null;
let server = null;
let txCharacteristic = null;
let rxCharacteristic = null;
let logEntries = [];
let filteredEntries = null;

// Tab state
let tabEntries = {};
let currentTab = 'ALL';

// 各センサーの最新値（センサーパネル表示用）
let latestBySensor = {};

// View state
let currentView = 'log';

// Streaming save variables
let streamingHandle = null;
let streamingWritable = null;
let streamingCount = 0;
let isStreaming = false;

// Utility functions
function formatTime(date) {
    return date.toTimeString().split(' ')[0] + '.' +
           String(date.getMilliseconds()).padStart(3, '0');
}

function updateStatus(status, message) {
    const indicator = document.getElementById('statusIndicator');
    const text = document.getElementById('statusText');
    indicator.className = 'status-indicator';
    if (status === 'connected') indicator.classList.add('connected');
    else if (status === 'connecting') indicator.classList.add('connecting');
    text.textContent = message;
}

function updateButtons() {
    const isConnected = device && device.gatt && device.gatt.connected;
    document.getElementById('connectBtn').disabled = isConnected;
    document.getElementById('disconnectBtn').disabled = !isConnected;
}

function updateLogStats() {
    const source = currentTab === 'ALL' ? logEntries : (tabEntries[currentTab] || []);
    const entries = filteredEntries || source;
    document.getElementById('logStats').textContent = `${entries.length} 行 / ${source.length} 行`;
}

function setView(view) {
    currentView = view;
    const logEl = document.getElementById('log');
    const graphEl = document.getElementById('graphContainer');
    const filterEl = document.getElementById('filterBarEl');
    const colEl = document.getElementById('colHeaders');

    if (view === 'log') {
        logEl.style.display = 'block';
        graphEl.style.display = 'none';
        filterEl.style.display = 'flex';
        document.getElementById('btnViewLog').classList.add('active');
        document.getElementById('btnViewGraph').classList.remove('active');
        renderColHeaders();
    } else {
        logEl.style.display = 'none';
        graphEl.style.display = 'block';
        filterEl.style.display = 'none';
        colEl.style.display = 'none';
        document.getElementById('btnViewLog').classList.remove('active');
        document.getElementById('btnViewGraph').classList.add('active');
        // DOM更新後にグラフを初期化してキャンバスサイズを正しく取得する
        setTimeout(() => initGraph(currentTab), 0);
    }
}

// センサーパネルとログ欄の高さをドラッグで調整できるようにする
function initResizeHandle() {
    const handle = document.getElementById('resizeHandle');
    const sensorsPanel = document.getElementById('sensorsPanel');
    const rightPanel = document.querySelector('.right-panel');
    if (!handle || !sensorsPanel || !rightPanel) return;

    const STORAGE_KEY = 'sensorsPanelHeight';
    const MIN_SENSORS_HEIGHT = 100;
    const MIN_LOG_HEIGHT = 80;

    const saved = parseInt(localStorage.getItem(STORAGE_KEY), 10);
    if (!isNaN(saved)) {
        sensorsPanel.style.height = saved + 'px';
    }

    let dragging = false;
    let startY = 0;
    let startHeight = 0;

    handle.addEventListener('mousedown', function (e) {
        dragging = true;
        startY = e.clientY;
        startHeight = sensorsPanel.getBoundingClientRect().height;
        handle.classList.add('dragging');
        document.body.style.cursor = 'row-resize';
        e.preventDefault();
    });

    window.addEventListener('mousemove', function (e) {
        if (!dragging) return;
        const maxHeight = rightPanel.getBoundingClientRect().height - MIN_LOG_HEIGHT - handle.offsetHeight;
        let newHeight = startHeight + (e.clientY - startY);
        newHeight = Math.max(MIN_SENSORS_HEIGHT, Math.min(newHeight, maxHeight));
        sensorsPanel.style.height = newHeight + 'px';
    });

    window.addEventListener('mouseup', function () {
        if (!dragging) return;
        dragging = false;
        handle.classList.remove('dragging');
        document.body.style.cursor = '';
        localStorage.setItem(STORAGE_KEY, Math.round(sensorsPanel.getBoundingClientRect().height));
    });
}

// Initialize
document.addEventListener('DOMContentLoaded', function () {
    updateButtons();
    initResizeHandle();

    document.getElementById('filterInput').addEventListener('keypress', function (e) {
        if (e.key === 'Enter') applyFilter();
    });

    document.getElementById('cellMode').addEventListener('change', function (e) {
        document.getElementById('cellWidthControl').style.display = e.target.checked ? 'block' : 'none';
        renderLog();
    });

    document.getElementById('cellWidth').addEventListener('change', function () {
        if (document.getElementById('cellMode').checked) renderLog();
    });

});
