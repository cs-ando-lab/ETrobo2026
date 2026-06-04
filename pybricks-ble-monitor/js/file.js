// File operations
function copyLog() {
    const showTimestamp = document.getElementById('showTimestamp').checked;
    const entries = filteredEntries || logEntries;
    const text = entries.map(entry => {
        return showTimestamp ? `[${entry.time}] ${entry.text}` : entry.text;
    }).join('\n');
    
    navigator.clipboard.writeText(text).then(() => {
        alert('ログをクリップボードにコピーしました');
    }).catch(() => {
        alert('コピーに失敗しました');
    });
}

function saveLogText() {
    const showTimestamp = document.getElementById('showTimestamp').checked;
    const entries = filteredEntries || logEntries;
    const text = entries.map(entry => {
        return showTimestamp ? `[${entry.time}] ${entry.text}` : entry.text;
    }).join('\n');
    
    // UTF-8 with BOM for Excel compatibility
    const utf8Bom = new Uint8Array([0xEF, 0xBB, 0xBF]);
    const textEncoder = new TextEncoder();
    const textArray = textEncoder.encode(text);
    const combined = new Uint8Array(utf8Bom.length + textArray.length);
    combined.set(utf8Bom);
    combined.set(textArray, utf8Bom.length);
    const blob = new Blob([combined], { type: 'text/plain;charset=utf-8;' });
    
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    
    const now = new Date();
    const timestamp = now.toISOString().replace(/[:.]/g, '-').slice(0, 19);
    a.href = url;
    a.download = `pybricks-log-${timestamp}.txt`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    
    addLogEntry(`ログを保存しました: pybricks-log-${timestamp}.txt (UTF-8)`, 'info');
}

function saveLogCSV() {
    const entries = filteredEntries || logEntries;
    
    // CSV data (no trailing newline)
    let csv = '';
    
    entries.forEach((entry, index) => {
        // Remove newlines from text and split by comma
        const cleanText = entry.text.replace(/\r\n|\r|\n/g, '');
        const fields = cleanText.split(',');
        
        // Escape each field and wrap in quotes
        const escapedFields = fields.map(field => {
            const escaped = field.replace(/"/g, '""');
            return `"${escaped}"`;
        });
        
        // Add timestamp as first field
        csv += `"${entry.time}",${escapedFields.join(',')}`;
        
        // Add newline except for last entry
        if (index < entries.length - 1) {
            csv += '\n';
        }
    });
    
    // UTF-8 with BOM for Excel compatibility
    const utf8Bom = new Uint8Array([0xEF, 0xBB, 0xBF]);
    const textEncoder = new TextEncoder();
    const textArray = textEncoder.encode(csv);
    const combined = new Uint8Array(utf8Bom.length + textArray.length);
    combined.set(utf8Bom);
    combined.set(textArray, utf8Bom.length);
    const blob = new Blob([combined], { type: 'text/csv;charset=utf-8;' });
    
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    
    const now = new Date();
    const timestamp = now.toISOString().replace(/[:.]/g, '-').slice(0, 19);
    a.href = url;
    a.download = `pybricks-data-${timestamp}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    
    addLogEntry(`CSVを保存しました: pybricks-data-${timestamp}.csv (UTF-8 BOM)`, 'info');
}

// Streaming save functions
async function toggleStreaming() {
    if (isStreaming) {
        await stopStreaming();
    } else {
        await startStreaming();
    }
}

async function startStreaming() {
    if (!window.showSaveFilePicker) {
        alert('ストリーミング保存はこのブラウザではサポートされていません。ChromeまたはEdgeを使用してください。');
        return;
    }

    try {
        streamingHandle = await window.showSaveFilePicker({
            suggestedName: `pybricks-stream-${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.txt`,
            types: [{
                description: 'Text files',
                accept: { 'text/plain': ['.txt'] }
            }]
        });

        streamingWritable = await streamingHandle.createWritable();
        
        const utf8Bom = new Uint8Array([0xEF, 0xBB, 0xBF]);
        await streamingWritable.write(utf8Bom);
        
        isStreaming = true;
        streamingCount = 0;
        
        document.getElementById('streamBtn').textContent = 'ストリーミング保存停止';
        document.getElementById('streamBtn').classList.add('disconnect');
        document.getElementById('streamStatus').style.display = 'block';
        document.getElementById('streamCount').textContent = '0';
        
        addLogEntry('ストリーミング保存を開始しました', 'info');
        
    } catch (error) {
        if (error.name !== 'AbortError') {
            console.error('Streaming start error:', error);
            addLogEntry('ストリーミング保存の開始に失敗しました: ' + error.message, 'error');
        }
    }
}

async function stopStreaming() {
    if (!streamingWritable) return;
    
    try {
        await streamingWritable.close();
        
        isStreaming = false;
        
        document.getElementById('streamBtn').textContent = 'ストリーミング保存開始';
        document.getElementById('streamBtn').classList.remove('disconnect');
        document.getElementById('streamStatus').style.display = 'none';
        
        addLogEntry(`ストリーミング保存を終了しました (${streamingCount} 行)`, 'info');
        
        streamingHandle = null;
        streamingWritable = null;
        streamingCount = 0;
        
    } catch (error) {
        console.error('Streaming stop error:', error);
        addLogEntry('ストリーミング保存の終了に失敗しました: ' + error.message, 'error');
    }
}

async function writeToStream(entry) {
    if (!isStreaming || !streamingWritable) return;
    
    try {
        const line = entry.text + '\n';
        const encoder = new TextEncoder();
        const data = encoder.encode(line);
        
        await streamingWritable.write(data);
        streamingCount++;
        document.getElementById('streamCount').textContent = streamingCount;
        
    } catch (error) {
        console.error('Stream write error:', error);
    }
}
