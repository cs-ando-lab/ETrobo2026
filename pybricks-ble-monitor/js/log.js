// Log management functions
function addLogEntry(text, type = 'data') {
    const entry = {
        time: formatTime(new Date()),
        text: text,
        type: type
    };
    
    // Check if log addition is paused (only for data entries)
    const pauseLogElem = document.getElementById('pauseLog');
    const isPaused = pauseLogElem ? pauseLogElem.checked : false;
    
    // Only 'data' type entries are saved to file
    // 'info' and 'error' are system messages (display only)
    if (type === 'data') {
        // Write to streaming file if active (always, even when paused)
        writeToStream(entry);
        
        // Add to log entries only if not paused
        if (!isPaused) {
            logEntries.push(entry);
            
            // Apply current filter if exists
            const filterText = document.getElementById('filterInput').value.trim().toLowerCase();
            if (filterText) {
                if (text.toLowerCase().includes(filterText)) {
                    if (!filteredEntries) filteredEntries = [];
                    filteredEntries.push(entry);
                }
            }
        }
    }
    
    renderLog();
}

function renderLog() {
    const log = document.getElementById('log');
    const showTimestamp = document.getElementById('showTimestamp').checked;
    const cellMode = document.getElementById('cellMode').checked;
    const entries = filteredEntries || logEntries;
    
    if (entries.length === 0) {
        log.innerHTML = '<div class="empty-state">データがありません</div>';
        updateLogStats();
        return;
    }
    
    log.innerHTML = '';
    entries.forEach(entry => {
        const div = document.createElement('div');
        div.className = 'log-entry';
        
        const timestamp = document.createElement('span');
        timestamp.className = 'timestamp';
        if (showTimestamp) {
            timestamp.textContent = entry.time;
        }
        
        const data = document.createElement('span');
        data.className = 'data ' + (entry.type || '');
        
        // CSV cell mode: split by comma and display as cells
        if (cellMode && entry.text.includes(',')) {
            const cells = entry.text.split(',');
            const cellWidth = parseInt(document.getElementById('cellWidth').value) || 8;
            data.style.display = 'flex';
            data.style.gap = '4px';
            cells.forEach((cell, index) => {
                const cellSpan = document.createElement('span');
                cellSpan.style.background = '#3c3c3c';
                cellSpan.style.padding = '1px 6px';
                cellSpan.style.borderRadius = '2px';
                cellSpan.style.width = (cellWidth * 8) + 'px';
                cellSpan.style.minWidth = (cellWidth * 8) + 'px';
                cellSpan.style.textAlign = 'center';
                cellSpan.style.overflow = 'hidden';
                cellSpan.style.textOverflow = 'ellipsis';
                cellSpan.style.whiteSpace = 'nowrap';
                cellSpan.textContent = cell.trim();
                data.appendChild(cellSpan);
            });
        } else {
            data.textContent = entry.text;
        }
        
        div.appendChild(timestamp);
        div.appendChild(data);
        log.appendChild(div);
    });
    
    updateLogStats();
    
    if (document.getElementById('autoScroll').checked) {
        log.scrollTop = log.scrollHeight;
    }
}

function clearLog() {
    logEntries = [];
    filteredEntries = null;
    document.getElementById('filterInput').value = '';
    document.getElementById('filterStats').textContent = '';
    renderLog();
    addLogEntry('ログをクリアしました', 'info');
}

function applyFilter() {
    const filterText = document.getElementById('filterInput').value.trim().toLowerCase();
    
    if (!filterText) {
        clearFilter();
        return;
    }
    
    filteredEntries = logEntries.filter(entry => 
        entry.text.toLowerCase().includes(filterText)
    );
    
    document.getElementById('filterStats').textContent = 
        `${filteredEntries.length} / ${logEntries.length} 行`;
    
    renderLog();
}

function clearFilter() {
    document.getElementById('filterInput').value = '';
    document.getElementById('filterStats').textContent = '';
    filteredEntries = null;
    renderLog();
}
