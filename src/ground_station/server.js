// ╔══════════════════════════════════════════════════════════╗
// ║  ground_station/server.js                               ║
// ║  Nhận UDP từ ESP32 → ghi CSV → broadcast WebSocket      ║
// ║  Serve index.html cho browser                           ║
// ║                                                         ║
// ║  Cài đặt:  npm install                                  ║
// ║  Chạy:     node server.js                               ║
// ║  Browser:  http://localhost:3000                        ║
// ╚══════════════════════════════════════════════════════════╝

const dgram     = require('dgram');
const http      = require('http');
const fs        = require('fs');
const path      = require('path');
const { WebSocketServer } = require('ws');

// ── Cấu hình ────────────────────────────────────────────────
const UDP_PORT  = 1234;          // phải khớp với ESP32
const HTTP_PORT = 3000;          // mở browser vào localhost:3000
const LOG_DIR   = path.join(__dirname, 'logs');

// ── Tạo thư mục log nếu chưa có ─────────────────────────────
if (!fs.existsSync(LOG_DIR)) fs.mkdirSync(LOG_DIR);

// ── Xóa log cũ mỗi lần khởi động server ─────────────────────
for (const fileName of fs.readdirSync(LOG_DIR)) {
    if (!fileName.startsWith('flight_') || !fileName.endsWith('.csv')) continue;
    fs.unlinkSync(path.join(LOG_DIR, fileName));
}

// Tên file CSV theo timestamp lúc khởi động server
const csvFile = path.join(
    LOG_DIR,
    `flight_${new Date().toISOString().replace(/[:.]/g, '-')}.csv`
);

// Header CSV
const CSV_HEADER = [
    'ts_us',
    'pitch','roll','yaw_rate',
    'pitch_sp','roll_sp','yaw_sp',
    'kp_roll','ki_roll','kd_roll',
    'kp_pitch','ki_pitch','kd_pitch',
    'kp_yaw','ki_yaw','kd_yaw',
    'pid_out_pitch','pid_out_roll','pid_out_yaw',
    'err'
].join(',') + '\n';

fs.writeFileSync(csvFile, CSV_HEADER);
console.log(`[CSV] Ghi log vào: ${csvFile}`);

// ── Parse binary LogPacket từ ESP32 ─────────────────────────
// Phải khớp đúng thứ tự và kiểu dữ liệu trong wifi_log.h
//
// struct LogPacket {
//   uint32_t ts;                          offset 0,  4 bytes
//   float pitch, roll, yaw_rate;          offset 4,  12 bytes
//   float pitch_sp, roll_sp, yaw_sp;      offset 16, 12 bytes
//   float kp/ki/kd roll/pitch/yaw (9×);   offset 28, 36 bytes
//   float pid_out_pitch/roll/yaw;         offset 64, 12 bytes
//   uint8_t err;                          offset 76, 1 byte
// }  → tổng 77 bytes (có thể 80 sau padding — xem lưu ý bên dưới)

const PACKET_SIZE = 80; // 77 bytes + 3 padding (compiler align uint8_t → 4)

function parsePacket(buf) {
    if (buf.length < PACKET_SIZE) return null;
    let o = 0;
    const r = (fn, bytes) => { const v = fn.call(buf, o, true); o += bytes; return v; };
    return {
        ts:           r(buf.readUInt32LE, 4),
        pitch:        r(buf.readFloatLE,  4),
        roll:         r(buf.readFloatLE,  4),
        yaw_rate:     r(buf.readFloatLE,  4),
        pitch_sp:     r(buf.readFloatLE,  4),
        roll_sp:      r(buf.readFloatLE,  4),
        yaw_sp:       r(buf.readFloatLE,  4),
        kp_roll:      r(buf.readFloatLE,  4),
        ki_roll:      r(buf.readFloatLE,  4),
        kd_roll:      r(buf.readFloatLE,  4),
        kp_pitch:     r(buf.readFloatLE,  4),
        ki_pitch:     r(buf.readFloatLE,  4),
        kd_pitch:     r(buf.readFloatLE,  4),
        kp_yaw:       r(buf.readFloatLE,  4),
        ki_yaw:       r(buf.readFloatLE,  4),
        kd_yaw:       r(buf.readFloatLE,  4),
        pid_out_pitch:r(buf.readFloatLE,  4),
        pid_out_roll: r(buf.readFloatLE,  4),
        pid_out_yaw:  r(buf.readFloatLE,  4),
        err:          buf.readUInt8(o),
    };
}

// ── HTTP server (serve index.html) ──────────────────────────
const httpServer = http.createServer((req, res) => {
    const filePath = path.join(__dirname, 'public', 'index.html');
    fs.readFile(filePath, (err, data) => {
        if (err) {
            res.writeHead(404);
            res.end('index.html not found — đặt file vào ground_station/public/');
            return;
        }
        res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
        res.end(data);
    });
});

// ── WebSocket server (gắn vào cùng HTTP server) ──────────────
const wss = new WebSocketServer({ server: httpServer });

wss.on('connection', (ws) => {
    console.log('[WS] Browser kết nối');
    ws.on('close', () => console.log('[WS] Browser ngắt kết nối'));
});

function broadcastWS(data) {
    const json = JSON.stringify(data);
    wss.clients.forEach(client => {
        if (client.readyState === 1) client.send(json);
    });
}

// ── UDP receiver ─────────────────────────────────────────────
const udpSocket = dgram.createSocket('udp4');

udpSocket.on('listening', () => {
    const addr = udpSocket.address();
    // Cho phép nhận broadcast
    udpSocket.setBroadcast(true);
    console.log(`[UDP] Lắng nghe tại 0.0.0.0:${addr.port}`);
});

udpSocket.on('message', (msg, rinfo) => {
    const pkt = parsePacket(msg);
    if (!pkt) {
        console.warn(`[UDP] Packet lạ từ ${rinfo.address} — size: ${msg.length}`);
        return;
    }

    // 1. Ghi CSV (append, không block event loop)
    const row = [
        pkt.ts,
        pkt.pitch.toFixed(4),   pkt.roll.toFixed(4),    pkt.yaw_rate.toFixed(4),
        pkt.pitch_sp.toFixed(4),pkt.roll_sp.toFixed(4), pkt.yaw_sp.toFixed(4),
        pkt.kp_roll,  pkt.ki_roll,  pkt.kd_roll,
        pkt.kp_pitch, pkt.ki_pitch, pkt.kd_pitch,
        pkt.kp_yaw,   pkt.ki_yaw,   pkt.kd_yaw,
        pkt.pid_out_pitch.toFixed(4),
        pkt.pid_out_roll.toFixed(4),
        pkt.pid_out_yaw.toFixed(4),
        pkt.err
    ].join(',') + '\n';

    fs.appendFile(csvFile, row, err => {
        if (err) console.error('[CSV] Lỗi ghi:', err.message);
    });

    // 2. Broadcast qua WebSocket đến tất cả browser đang mở
    broadcastWS(pkt);
});

udpSocket.bind(UDP_PORT);

// ── Start ────────────────────────────────────────────────────
httpServer.listen(HTTP_PORT, () => {
    console.log(`[HTTP] Dashboard: http://localhost:${HTTP_PORT}`);
    console.log(`[INFO] Kết nối laptop vào WiFi "Drone_Log" trước khi bay`);
    console.log(`[INFO] Nhấn Ctrl+C để dừng`);
});

// ── Graceful shutdown ────────────────────────────────────────
process.on('SIGINT', () => {
    console.log('\n[INFO] Đang dừng server...');
    udpSocket.close();
    httpServer.close(() => process.exit(0));
});