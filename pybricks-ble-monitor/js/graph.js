// Chart.js time series graph

let chart = null;
const GRAPH_MAX_POINTS = 200;

const GRAPH_COLORS = [
    '#4fc1ff', '#4ec9b0', '#dcdcaa',
    '#f48771', '#ce9178', '#b5cea8', '#c586c0'
];

function initGraph(tabName) {
    const ctx = document.getElementById('graphCanvas').getContext('2d');
    if (chart) { chart.destroy(); chart = null; }

    const config = TAB_CONFIGS[tabName];
    if (!config || !config.headers || tabName === 'ALL') {
        ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
        ctx.fillStyle = '#6e6e6e';
        ctx.font = '14px Consolas';
        ctx.textAlign = 'center';
        ctx.fillText('グラフはセンサータブを選択してください', ctx.canvas.width / 2, 60);
        return;
    }

    // count列を除いた列名をデータセットに
    const labels = config.headers.slice(1);

    chart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: labels.map((label, i) => ({
                label,
                data: [],
                borderColor: GRAPH_COLORS[i % GRAPH_COLORS.length],
                backgroundColor: 'transparent',
                borderWidth: 1.5,
                pointRadius: 0,
                tension: 0.2
            }))
        },
        options: {
            animation: false,
            responsive: true,
            maintainAspectRatio: false,
            interaction: { mode: 'index', intersect: false },
            scales: {
                x: {
                    ticks: { color: '#808080', maxTicksLimit: 8, font: { size: 10 } },
                    grid: { color: '#3c3c3c' }
                },
                y: {
                    ticks: { color: '#808080', font: { size: 10 } },
                    grid: { color: '#3c3c3c' }
                }
            },
            plugins: {
                legend: {
                    labels: { color: '#d4d4d4', font: { size: 11 }, boxWidth: 12 }
                }
            }
        }
    });

    // 既存データをグラフに反映
    const source = tabEntries[tabName] || [];
    source.forEach(entry => updateGraph(entry, tabName));
}

function updateGraph(entry, tabName) {
    if (!chart || tabName === 'ALL') return;

    let text = entry.text;
    const prefix = extractPrefix(text);
    if (prefix) text = text.substring(prefix.length + 1);

    const cells = text.split(',').map(v => v.trim());
    const countVal = cells[0];
    const values = cells.slice(1).map(v => parseFloat(v));

    chart.data.labels.push(countVal);
    values.forEach((v, i) => {
        if (chart.data.datasets[i]) {
            chart.data.datasets[i].data.push(isNaN(v) ? null : v);
        }
    });

    if (chart.data.labels.length > GRAPH_MAX_POINTS) {
        chart.data.labels.shift();
        chart.data.datasets.forEach(ds => ds.data.shift());
    }

    chart.update('none');
}
