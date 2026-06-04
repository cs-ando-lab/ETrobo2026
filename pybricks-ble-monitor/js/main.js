// Constants
const NUS_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_TX_CHAR_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_RX_CHAR_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
const PYBRICKS_SERVICE_UUID = 'c5f50001-8280-46da-89f4-6d8051e4aeef';

// State variables
let device = null;
let server = null;
let txCharacteristic = null;
let rxCharacteristic = null;
let logEntries = [];
let filteredEntries = null;

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
    if (status === 'connected') {
        indicator.classList.add('connected');
    } else if (status === 'connecting') {
        indicator.classList.add('connecting');
    }
    
    text.textContent = message;
}

function updateButtons() {
    const isConnected = device && device.gatt && device.gatt.connected;
    document.getElementById('connectBtn').disabled = isConnected;
    document.getElementById('disconnectBtn').disabled = !isConnected;
}

function updateLogStats() {
    const entries = filteredEntries || logEntries;
    document.getElementById('logStats').textContent = `${entries.length} 行 / ${logEntries.length} 行`;
}

// Initialize
document.addEventListener('DOMContentLoaded', function() {
    updateButtons();
    
    // Event listeners
    document.getElementById('filterInput').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') {
            applyFilter();
        }
    });

    document.getElementById('cellMode').addEventListener('change', function(e) {
        document.getElementById('cellWidthControl').style.display = e.target.checked ? 'block' : 'none';
        renderLog();
    });

    document.getElementById('cellWidth').addEventListener('change', function() {
        if (document.getElementById('cellMode').checked) {
            renderLog();
        }
    });
});
