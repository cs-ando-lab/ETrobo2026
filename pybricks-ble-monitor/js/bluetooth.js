// Bluetooth connection functions

// Buffer for incomplete data
let receiveBuffer = '';
let textDecoder = null;

async function connect() {
    if (!navigator.bluetooth) {
        alert('Web Bluetooth APIがサポートされていません。Chrome/Edgeブラウザを使用してください。');
        return;
    }

    try {
        updateStatus('connecting', 'スキャン中...');
        
        const options = {
            filters: [{ services: [PYBRICKS_SERVICE_UUID] }],
            optionalServices: [PYBRICKS_SERVICE_UUID, NUS_SERVICE_UUID]
        };

        device = await navigator.bluetooth.requestDevice(options);
        
        const deviceName = device.name || 'Unknown';
        const deviceId = device.id;
        
        console.log('Selected device:', deviceName, deviceId);
        
        // Update device info
        document.getElementById('infoName').textContent = deviceName;
        document.getElementById('infoId').textContent = deviceId;
        document.getElementById('deviceInfoSection').style.display = 'block';
        
        addLogEntry('接続中: ' + deviceName, 'info');
        updateStatus('connecting', '接続中...');
        
        device.addEventListener('gattserverdisconnected', onDisconnected);
        
        server = await device.gatt.connect();
        
        const service = await server.getPrimaryService(NUS_SERVICE_UUID);
        txCharacteristic = await service.getCharacteristic(NUS_TX_CHAR_UUID);
        rxCharacteristic = await service.getCharacteristic(NUS_RX_CHAR_UUID);
        
        await txCharacteristic.startNotifications();
        txCharacteristic.addEventListener('characteristicvaluechanged', handleNotifications);
        
        // Reset buffer and create new decoder on new connection
        receiveBuffer = '';
        textDecoder = new TextDecoder('utf-8');
        
        window.currentDevice = device;
        window.currentTxCharacteristic = txCharacteristic;
        
        updateStatus('connected', '接続済み: ' + deviceName);
        updateButtons();
        addLogEntry('接続しました: ' + deviceName, 'info');
        
    } catch (error) {
        console.error('Connection error:', error);
        if (error.name === 'NotFoundError') {
            updateStatus('disconnected', 'デバイスが選択されませんでした');
        } else {
            updateStatus('disconnected', 'エラー: ' + error.message);
            addLogEntry('エラー: ' + error.message, 'error');
        }
        updateButtons();
    }
}

async function disconnect() {
    if (device && device.gatt.connected) {
        if (txCharacteristic) {
            try {
                await txCharacteristic.stopNotifications();
            } catch (e) {
                console.log('Stop notifications error:', e);
            }
        }
        await device.gatt.disconnect();
    }
}

function onDisconnected(event) {
    updateStatus('disconnected', '切断されました');
    updateButtons();
    addLogEntry('切断されました', 'info');
    
    // Clear buffer on disconnect
    receiveBuffer = '';
    
    device = null;
    server = null;
    txCharacteristic = null;
    rxCharacteristic = null;
}

function handleNotifications(event) {
    const value = event.target.value;
    
    // Use streaming decoder to handle split multi-byte characters
    const text = textDecoder.decode(value, { stream: true });
    
    // Append to buffer
    receiveBuffer += text;
    
    // Process complete lines
    let newlineIndex;
    while ((newlineIndex = receiveBuffer.indexOf('\n')) !== -1) {
        const line = receiveBuffer.substring(0, newlineIndex);
        if (line.trim()) {
            addLogEntry(line);
        }
        receiveBuffer = receiveBuffer.substring(newlineIndex + 1);
    }
    
    // Force display if buffer exceeds 256 characters without newline
    if (receiveBuffer.length > 256) {
        const line = receiveBuffer.substring(0, 256);
        if (line.trim()) {
            addLogEntry(line);
        }
        receiveBuffer = receiveBuffer.substring(256);
    }
}
