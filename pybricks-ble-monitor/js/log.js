// Log management functions

const PREFIX_PATTERN = /^[A-Z][A-Z0-9_]*$/;

function extractPrefix(text) {
    const idx = text.indexOf(',');
    if (idx === -1) return null;
    const first = text.substring(0, idx);
    return PREFIX_PATTERN.test(first) ? first : null;
}

function createTab(name) {
    const tabBar = document.getElementById('tabBar');
    const btn = document.createElement('button');
    btn.id = 'tab-' + name;
    btn.className = 'tab';
    btn.textContent = name;
    btn.onclick = () => switchTab(name);
    tabBar.appendChild(btn);
}

function switchTab(name) {
    currentTab = name;
    filteredEntries = null;
    document.getElementById('filterInput').value = '';
    document.getElementById('filterStats').textContent = '';
    document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
    document.getElementById('tab-' + name).classList.add('active');
    renderColHeaders();
    if (currentView === 'graph') initGraph(name);
    renderLog();
}

// 列ヘッダーを描画
function renderColHeaders() {
    const el = document.getElementById('colHeaders');
    const config = TAB_CONFIGS[currentTab];
    if (!config || !config.headers || currentView === 'graph') {
        el.style.display = 'none';
        return;
    }
    el.style.display = 'flex';
    el.innerHTML = '<span class="col-header-time">時刻</span>' +
        config.headers.map(h => `<span class="col-header-item">${h}</span>`).join('');
}

// 閾値チェック
function checkThreshold(tabName, cells) {
    const config = TAB_CONFIGS[tabName];
    if (!config || !config.thresholds) return false;
    for (const rule of config.thresholds) {
        const val = parseInt(cells[rule.col]);
        if (rule.op === 'eq' && val === rule.val) return true;
        if (rule.op === 'lt' && val < rule.val) return true;
        if (rule.op === 'gt' && val > rule.val) return true;
    }
    return false;
}

// 全センサー現在値パネルを描画
function renderAllSensors() {
    const grid = document.getElementById('sensorsGrid');
    const sensorNames = Object.keys(TAB_CONFIGS);

    let html = '';
    sensorNames.forEach(name => {
        const entry = latestBySensor[name];
        const config = TAB_CONFIGS[name];

        html += `<div class="sensor-card">`;
        html += `<div class="sensor-card-title">${name}</div>`;

        if (!entry) {
            html += `<div class="sensor-no-data">—</div>`;
        } else {
            let text = entry.text;
            const prefix = extractPrefix(text);
            if (prefix) text = text.substring(prefix.length + 1);
            const cells = text.split(',').map(v => v.trim());

            // カラープレビュー（COLOR センサーのみ）
            if (config.colorPreview && cells.length >= 4) {
                const r = Math.round(parseInt(cells[1]) * 255 / 1023);
                const g = Math.round(parseInt(cells[2]) * 255 / 1023);
                const b = Math.round(parseInt(cells[3]) * 255 / 1023);
                html += `<div class="sensor-color-swatch" style="background:rgb(${r},${g},${b})"></div>`;
            }

            // 値と列名
            html += `<div class="sensor-values">`;
            cells.forEach((val, i) => {
                const label = (config.headers && config.headers[i]) ? config.headers[i] : `col${i}`;
                // 閾値チェックして値の色を変える
                const isHit = config.thresholds && config.thresholds.some(rule => {
                    const v = parseInt(val);
                    return (rule.col === i) && (
                        (rule.op === 'eq' && v === rule.val) ||
                        (rule.op === 'lt' && v < rule.val) ||
                        (rule.op === 'gt' && v > rule.val)
                    );
                });
                const cls = isHit ? 'sv-value highlight' : 'sv-value';
                html += `<div class="sensor-value-row"><span class="sv-label">${label}</span><span class="${cls}">${val}</span></div>`;
            });
            html += `</div>`;
        }

        html += `</div>`;
    });

    grid.innerHTML = html;
}

function addLogEntry(text, type = 'data') {
    const entry = { time: formatTime(new Date()), text, type };
    const isPaused = document.getElementById('pauseLog')?.checked || false;

    if (type === 'data') {
        writeToStream(entry);

        if (!isPaused) {
            logEntries.push(entry);

            const prefix = extractPrefix(text);
            if (prefix) {
                if (!tabEntries[prefix]) {
                    tabEntries[prefix] = [];
                    createTab(prefix);
                }
                tabEntries[prefix].push(entry);

                // 最新値を更新してセンサーパネルを再描画
                latestBySensor[prefix] = entry;
                renderAllSensors();
            }

            const filterText = document.getElementById('filterInput').value.trim().toLowerCase();
            if (filterText && text.toLowerCase().includes(filterText)) {
                if (!filteredEntries) filteredEntries = [];
                filteredEntries.push(entry);
            }

            if (currentView === 'graph') {
                const entryTab = prefix || 'ALL';
                if (currentTab === 'ALL' || currentTab === entryTab) {
                    updateGraph(entry, currentTab);
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

    let entries;
    if (filteredEntries) {
        entries = filteredEntries;
    } else if (currentTab === 'ALL') {
        entries = logEntries;
    } else {
        entries = tabEntries[currentTab] || [];
    }

    if (entries.length === 0) {
        log.innerHTML = '<div class="empty-state">データがありません</div>';
        updateLogStats();
        return;
    }

    log.innerHTML = '';
    entries.forEach(entry => {
        const div = document.createElement('div');
        div.className = 'log-entry';

        let displayText = entry.text;
        let tabName = currentTab;
        if (currentTab === 'ALL') {
            tabName = extractPrefix(entry.text) || 'ALL';
        } else if (entry.type === 'data') {
            const prefix = extractPrefix(entry.text);
            if (prefix) displayText = entry.text.substring(prefix.length + 1);
        }

        // 閾値ハイライト
        if (entry.type === 'data' && tabName !== 'ALL') {
            const cells = displayText.split(',');
            if (checkThreshold(tabName, cells)) div.classList.add('threshold-hit');
        }

        const timestamp = document.createElement('span');
        timestamp.className = 'timestamp';
        if (showTimestamp) timestamp.textContent = entry.time;

        const data = document.createElement('span');
        data.className = 'data ' + (entry.type || '');

        if (cellMode && displayText.includes(',')) {
            const cells = displayText.split(',');
            const cellWidth = parseInt(document.getElementById('cellWidth').value) || 8;
            data.style.display = 'flex';
            data.style.gap = '4px';
            data.style.alignItems = 'center';
            cells.forEach(cell => {
                const span = document.createElement('span');
                span.style.cssText = `background:#3c3c3c;padding:1px 6px;border-radius:2px;width:${cellWidth * 8}px;min-width:${cellWidth * 8}px;text-align:center;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;`;
                span.textContent = cell.trim();
                data.appendChild(span);
            });
        } else {
            data.textContent = displayText;
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
    tabEntries = {};
    latestBySensor = {};
    currentTab = 'ALL';

    const tabBar = document.getElementById('tabBar');
    tabBar.querySelectorAll('.tab:not(#tab-ALL)').forEach(t => t.remove());
    document.getElementById('tab-ALL').classList.add('active');

    document.getElementById('filterInput').value = '';
    document.getElementById('filterStats').textContent = '';
    document.getElementById('colHeaders').style.display = 'none';

    renderAllSensors();
    if (currentView === 'graph') initGraph('ALL');
    renderLog();
    addLogEntry('ログをクリアしました', 'info');
}

function applyFilter() {
    const filterText = document.getElementById('filterInput').value.trim().toLowerCase();
    if (!filterText) { clearFilter(); return; }

    const source = currentTab === 'ALL' ? logEntries : (tabEntries[currentTab] || []);
    filteredEntries = source.filter(e => e.text.toLowerCase().includes(filterText));

    document.getElementById('filterStats').textContent =
        `${filteredEntries.length} / ${source.length} 行`;
    renderLog();
}

function clearFilter() {
    document.getElementById('filterInput').value = '';
    document.getElementById('filterStats').textContent = '';
    filteredEntries = null;
    renderLog();
}
