// pybricks-ble-monitor 用の簡易静的ファイルサーバー（Node.js標準ライブラリのみ、追加パッケージ不要）
//
// なぜ必要か:
// file://wsl.localhost/... のようなホスト名付きのfile:// URLでindex.htmlを開くと、
// Chromeがそのoriginに File System Access API (showDirectoryPicker等) の権限を
// 正しく与えず、「自動保存」機能が動作しない。
// http://localhost 経由で開けば通常のoriginとして扱われ問題が起きないため、
// このスクリプトでローカルサーバーを立ててそこ経由でアクセスできるようにする。
//
// 使い方: node serve.js [port]  (省略時は8765番ポート)

const http = require('http');
const fs = require('fs');
const path = require('path');

const PORT = parseInt(process.argv[2], 10) || 8765;
const ROOT = __dirname;

const MIME_TYPES = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon',
};

const server = http.createServer((req, res) => {
    let urlPath = decodeURIComponent((req.url || '/').split('?')[0]);
    if (urlPath === '/') urlPath = '/index.html';

    const filePath = path.normalize(path.join(ROOT, urlPath));

    // ディレクトリトラバーサル防止（ROOT配下のファイルのみ配信する）
    if (!filePath.startsWith(ROOT)) {
        res.writeHead(403);
        res.end('Forbidden');
        return;
    }

    fs.readFile(filePath, (err, data) => {
        if (err) {
            res.writeHead(404);
            res.end('Not found: ' + urlPath);
            return;
        }
        const ext = path.extname(filePath).toLowerCase();
        res.writeHead(200, { 'Content-Type': MIME_TYPES[ext] || 'application/octet-stream' });
        res.end(data);
    });
});

// 外部ネットワークに公開しないようlocalhostのみでlistenする
server.listen(PORT, '127.0.0.1', () => {
    console.log(`Pybricks BLE Monitor server: http://localhost:${PORT}/index.html`);
    console.log('このウィンドウを閉じるとサーバーが停止します。');
});
