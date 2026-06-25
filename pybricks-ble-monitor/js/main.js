// Constants
const NUS_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_TX_CHAR_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_RX_CHAR_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
const PYBRICKS_SERVICE_UUID = 'c5f50001-8280-46da-89f4-6d8051e4aeef';

// タブごとの設定（列ヘッダー・閾値・カラープレビュー）
const TAB_CONFIGS = {
    'COLOR': {
        headers: ['count', 'R', 'G', 'B'],
        colorPreview: true,
        thresholds: []
    },
    'MOTOR': {
        headers: ['count', '左速度(d/s)', '右速度(d/s)', '左cnt', '右cnt'],
        thresholds: []
    },
    'IMU': {
        headers: ['count', '方位角(deg)', '角速度Z(d/s)'],
        thresholds: []
    },
    'ULTRA': {
        headers: ['count', '距離(mm)'],
        thresholds: [{ col: 1, op: 'eq', val: -1 }]  // -1 = 測定不能
    },
    'FORCE': {
        headers: ['count', '接触', '力(N×100)'],
        thresholds: [{ col: 1, op: 'eq', val: 1 }]   // 接触中
    },
    'BAT': {
        headers: ['count', '電圧(mV)', '電流(mA)'],
        thresholds: []
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

// Initialize
document.addEventListener('DOMContentLoaded', function () {
    updateButtons();

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
