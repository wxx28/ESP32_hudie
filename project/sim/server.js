// 仿生蝴蝶仿真服务：启动 C 仿真内核（真实飞控算法），把状态推给浏览器。
//   node server.js        然后打开 http://localhost:8090
// 依赖：只用 Node 自带模块。仿真内核不存在时会尝试用 gcc 自动编译。
'use strict';
const http = require('http');
const fs = require('fs');
const path = require('path');
const { spawn, execFileSync } = require('child_process');

const PORT = Number(process.env.PORT || 8090);
const DIR = __dirname;
const EXE = path.join(DIR, process.platform === 'win32' ? 'hudie_sim.exe' : 'hudie_sim');

function build() {
  const fp = path.join(DIR, '..', 'first', 'first_project', 'components');
  const algo = path.join(fp, 'Middlewares', 'algo');
  const srcs = fs.readdirSync(algo).filter(f => f.endsWith('.c')).map(f => path.join(algo, f));
  const args = ['-std=c11', '-O2', '-Wno-unused-variable',
    '-I' + algo, '-I' + path.join(fp, 'flight_control'), '-I' + path.join(fp, 'flight_control', 'include'),
    '-I' + path.join(fp, 'BSP', 'SERVO'), '-I' + path.join(DIR, 'stubs'),
    '-I' + path.join(DIR, '..', 'common', 'hudie_link', 'include'),
    path.join(DIR, 'sim.c'), ...srcs, path.join(fp, 'flight_control', 'fc_output.c'),
    path.join(fp, 'flight_control', 'fc_config.c'), path.join(DIR, '..', 'common', 'hudie_link', 'hudie_protocol.c'),
    '-lm', '-o', EXE];
  console.log('编译仿真内核 ...');
  execFileSync('gcc', args, { stdio: 'inherit' });
}

function needBuild() {
  if (!fs.existsSync(EXE)) return true;
  const t = fs.statSync(EXE).mtimeMs;
  const dirs = [DIR, path.join(DIR, '..', 'first', 'first_project', 'components', 'Middlewares', 'algo'),
    path.join(DIR, '..', 'first', 'first_project', 'components', 'flight_control')];
  return dirs.some(d => fs.readdirSync(d).some(f => /\.(c|h)$/.test(f) && fs.statSync(path.join(d, f)).mtimeMs > t));
}

try {
  if (needBuild()) build();
} catch (e) {
  if (!fs.existsSync(EXE)) {
    console.error('编译失败：请安装 gcc（Windows 用 MSYS2 mingw64）并加入 PATH，或先运行 build.sh');
    process.exit(1);
  }
  console.warn('重新编译失败，使用旧的仿真内核');
}

// ---------------- 仿真内核 ----------------
let sim = null;
let latest = '';
const clients = new Set();
let paused = false;
let speed = 1.0;
let acc = 0;

function startSim() {
  sim = spawn(EXE, [], { stdio: ['pipe', 'pipe', 'inherit'] });
  let buf = '';
  sim.stdout.on('data', chunk => {
    buf += chunk.toString('utf8');
    let i;
    while ((i = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, i).trim();
      buf = buf.slice(i + 1);
      if (!line.startsWith('{')) continue;
      latest = line;
      for (const res of clients) res.write('data: ' + line + '\n\n');
    }
  });
  sim.on('exit', code => {
    console.error('仿真内核退出 code=' + code + '，1 s 后重启');
    sim = null;
    setTimeout(startSim, 1000);
  });
}
startSim();

function send(cmd) {
  if (sim && sim.stdin.writable) sim.stdin.write(cmd.trim() + '\n');
}

// 每 10 ms 推进 4 个 2.5 ms 节拍（实时），按倍速累计
setInterval(() => {
  if (paused || !sim) return;
  acc += 4 * speed;
  const n = Math.floor(acc);
  if (n >= 1) {
    acc -= n;
    send('run ' + n);
  }
}, 10);

// ---------------- HTTP ----------------
const MIME = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css' };

http.createServer((req, res) => {
  const url = new URL(req.url, 'http://x');
  if (url.pathname === '/events') {
    res.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-cache', Connection: 'keep-alive' });
    res.write('retry: 1000\n\n');
    if (latest) res.write('data: ' + latest + '\n\n');
    clients.add(res);
    req.on('close', () => clients.delete(res));
    return;
  }
  if (url.pathname === '/cmd' && req.method === 'POST') {
    let body = '';
    req.on('data', d => (body += d));
    req.on('end', () => {
      for (const line of body.split('\n')) {
        const c = line.trim();
        if (!c) continue;
        if (c.startsWith('speed ')) speed = Math.max(0.05, Math.min(8, parseFloat(c.slice(6)) || 1));
        else if (c === 'pause') paused = true;
        else if (c === 'resume') paused = false;
        else if (c === 'step') send('run 4');
        else send(c);
      }
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ ok: true, paused, speed }));
    });
    return;
  }
  const file = path.join(DIR, 'web', url.pathname === '/' ? 'index.html' : path.normalize(url.pathname).replace(/^([/\\])+/, ''));
  if (!file.startsWith(path.join(DIR, 'web'))) { res.writeHead(403); res.end(); return; }
  fs.readFile(file, (err, data) => {
    if (err) { res.writeHead(404); res.end('not found'); return; }
    res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] || 'application/octet-stream' });
    res.end(data);
  });
}).listen(PORT, () => console.log('仿真界面：http://localhost:' + PORT));
