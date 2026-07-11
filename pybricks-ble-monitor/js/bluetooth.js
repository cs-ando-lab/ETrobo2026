// Bluetooth connection functions

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

        receiveBuffer = '';
        textDecoder = new TextDecoder('utf-8');

        window.currentDevice = device;
        window.currentTxCharacteristic = txCharacteristic;

        updateStatus('connected', '接続済み: ' + deviceName);
        updateButtons();
        addLogEntry('接続しました: ' + deviceName, 'info');
        await startAutoSave();

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

async function onDisconnected(event) {
    updateStatus('disconnected', '切断されました');
    updateButtons();
    addLogEntry('切断されました', 'info');
    await stopAutoSave();

    receiveBuffer = '';

    device = null;
    server = null;
    txCharacteristic = null;
    rxCharacteristic = null;
}

function handleNotifications(event) {
    const text = textDecoder.decode(event.target.value, { stream: true });
    receiveBuffer += text;

    let newlineIndex;
    while ((newlineIndex = receiveBuffer.indexOf('\n')) !== -1) {
        const line = receiveBuffer.substring(0, newlineIndex);
        if (line.trim()) addLogEntry(line);
        receiveBuffer = receiveBuffer.substring(newlineIndex + 1);
    }

    if (receiveBuffer.length > 256) {
        const line = receiveBuffer.substring(0, 256);
        if (line.trim()) addLogEntry(line);
        receiveBuffer = receiveBuffer.substring(256);
    }
}
