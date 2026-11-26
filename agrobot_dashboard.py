#!/usr/bin/env python3
"""AGROBOT single-motor bench dashboard. Run on Pi after sourcing ROS Jazzy."""
import collections
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parent
PORT = 8765
TOKEN = secrets.token_urlsafe(32)
ENC = '/agrobot/bench/encoder_ticks'
CMD = '/agrobot/bench/cmd_vel'
RESULT = '/plant_health/result'
INSPECT = '/plant_health/inspect'
LEASE = 10.0
SPECS = {
    'bridge': (Path('/usr/bin/python3'), ROOT/'raspberry_pi/single_motor_bridge.py'),
    'ai': (Path.home()/'agrobot_ai_env/bin/python', ROOT/'ai_vision/camera_ai_service.py'),
    'nav': (Path('/usr/bin/python3'), ROOT/'raspberry_pi/navigation_v1_auto.py'),
}

class Controller:
    def __init__(self):
        self.lock = threading.RLock()
        self.actions = threading.RLock()
        self.processes = {}
        self.logs = collections.deque(maxlen=500)
        self.seq = 0
        self.encoder = None
        self.encoder_at = 0.0
        self.command = None
        self.command_at = 0.0
        self.result = None
        self.result_at = 0.0
        self.phase = 'Not started'
        self.cycle = 0
        self.service_ready = False
        self.encoder_pubs = 0
        self.cmd_pubs = 0
        self.cmd_subs = 0
        self.graph_at = 0.0
        self.heartbeat = 0.0
        self.closed = False
        self.inspect_pending = False
        self.inspect_started = 0.0
        self.inspect_request = False

    def log(self, source, line):
        with self.lock:
            self.seq += 1
            self.logs.append({'id': self.seq, 'source': source, 'time': time.strftime('%H:%M:%S'), 'text': line[:2000]})
            if source == 'nav':
                if 'Requesting inspection' in line:
                    import re
                    m = re.search(r'CYCLE (\d+)', line)
                    if m: self.cycle = int(m.group(1))
                    self.phase = 'Inspecting (motor stop commanded)'
                elif 'AI RESULT:' in line:
                    self.phase = 'AI result received'
                elif 'MOVING FORWARD' in line:
                    self.phase = 'Forward motion commanded'
                elif 'Motor stop commanded' in line:
                    self.phase = 'Settling / motor stop commanded'
                elif 'ALL CYCLES COMPLETED' in line:
                    self.phase = 'All three cycles complete'
                elif '[ERROR]' in line or '[FATAL]' in line:
                    self.phase = 'Fault — inspect navigation logs'

    def running(self, key):
        p = self.processes.get(key)
        return p is not None and p.poll() is None

    def read_output(self, key, p):
        try:
            for line in p.stdout:
                self.log(key, line.rstrip())
        finally:
            p.stdout.close()
            code = p.wait()
            self.log('system', f'{key} exited (code {code})')
            with self.lock:
                if self.processes.get(key) is p and key == 'nav':
                    self.phase = f'Navigation stopped (exit {code})'

    def start(self, key):
        with self.actions, self.lock:
            if self.closed: raise ValueError('Dashboard shutting down')
            if self.running(key): return
            if time.monotonic() - self.graph_at > 3:
                raise ValueError('ROS graph unavailable — wait for connection')
            if key == 'bridge' and (self.encoder_pubs or self.cmd_subs > 1 or self.cmd_pubs):
                raise ValueError('Another motor bridge or command publisher is present. Stop old terminals first.')
            if key == 'ai' and self.service_ready:
                raise ValueError('Inspection service already exists. Stop the old AI node first.')
            if key == 'nav':
                if not self.running('bridge') or not self.running('ai'):
                    raise ValueError('Start motor bridge and AI service first.')
                if not self.service_ready or time.monotonic() - self.encoder_at > 1:
                    raise ValueError('Wait for fresh encoder feedback and AI service.')
                if self.encoder_pubs != 1 or self.cmd_pubs or self.cmd_subs != 2:
                    raise ValueError('Unexpected ROS publishers/subscribers. Stop external motor controllers first.')
                if self.inspect_pending: raise ValueError('Wait for manual inspection to finish.')
                self.heartbeat = time.monotonic()
                self.phase = 'Starting / settling'
                self.cycle = 0
            exe, script = SPECS[key]
            if not exe.is_file() or not script.is_file():
                raise ValueError(f'Missing interpreter or script: {exe} / {script}')
            env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0')
            p = subprocess.Popen([str(exe), '-u', str(script)], cwd=script.parent,
                                 env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, errors='replace', bufsize=1, start_new_session=True)
            self.processes[key] = p
            self.log('system', f'Started {key} (PID {p.pid})')
            threading.Thread(target=self.read_output, args=(key,p), daemon=True).start()

    def stop(self, key):
        with self.actions:
            if key != 'nav':
                self.stop('nav')
            with self.lock:
                p = self.processes.get(key)
                if p is None or p.poll() is not None: return
            self.log('system', f'Stopping {key}')
            for sig, wait in ((signal.SIGINT, 1.5), (signal.SIGTERM, 1.0), (signal.SIGKILL, 1.0)):
                try: os.killpg(p.pid, sig)
                except ProcessLookupError: break
                try: p.wait(timeout=wait); break
                except subprocess.TimeoutExpired: continue
            if key == 'nav':
                # Existing bridge command timeout is 250 ms; firmware watchdog 300 ms.
                time.sleep(0.4)
                with self.lock: self.phase = 'Navigation stopped — verify physical motor stop'

    def stop_all(self):
        with self.actions:
            for key in ('nav', 'bridge', 'ai'): self.stop(key)

    def monitor(self):
        while not self.closed:
            time.sleep(0.25)
            with self.actions:
                with self.lock:
                    active = self.running('nav')
                    now = time.monotonic()
                    reason = None
                    if active:
                        if now-self.heartbeat > LEASE: reason = 'Dashboard heartbeat expired'
                        elif not self.running('bridge') or not self.running('ai'): reason = 'Required node exited'
                        elif now-self.encoder_at > 1.5: reason = 'Encoder feedback stale'
                        elif now-self.graph_at > 3: reason = 'ROS monitor unavailable'
                        elif self.cmd_pubs > 1 or self.encoder_pubs > 1 or self.cmd_subs > 2: reason = 'Unexpected ROS graph endpoint'
                if reason:
                    self.log('system', reason + ' — stopping navigation')
                    self.stop('nav')

    def state(self):
        with self.lock:
            now = time.monotonic()
            return {'nodes': {k: {'running': self.running(k), 'exit': None if k not in self.processes else self.processes[k].poll()} for k in SPECS},
                    'encoder': self.encoder, 'encoder_age': round(now-self.encoder_at,2) if self.encoder_at else None,
                    'command': self.command if now-self.command_at < 0.5 else None,
                    'result': self.result, 'result_age': round(now-self.result_at,1) if self.result_at else None,
                    'phase': self.phase, 'cycle': self.cycle, 'service_ready': self.service_ready,
                    'inspect_pending': self.inspect_pending, 'graph_ready': now-self.graph_at < 3,
                    'logs': list(self.logs)}

C = Controller()

class Monitor(Node):
    def __init__(self):
        super().__init__('agrobot_dashboard_monitor')
        self.create_subscription(Int64, ENC, self.encoder, 10)
        self.create_subscription(Twist, CMD, self.command, 10)
        self.create_subscription(String, RESULT, self.result, 10)
        self.client = self.create_client(Trigger, INSPECT)
        self.future = None
        self.create_timer(0.2, self.poll)

    def encoder(self, msg):
        with C.lock: C.encoder, C.encoder_at = msg.data, time.monotonic()

    def command(self, msg):
        with C.lock: C.command, C.command_at = msg.linear.x, time.monotonic()

    def result(self, msg):
        try:
            data = json.loads(msg.data)
            json.dumps(data, allow_nan=False)
            if not isinstance(data, dict): raise ValueError('Expected JSON object')
            with C.lock: C.result, C.result_at = data, time.monotonic()
            C.log('ai-result', json.dumps(data, ensure_ascii=False))
        except (ValueError, TypeError) as e: C.log('system', f'Invalid AI result: {e}')

    def poll(self):
        with C.lock:
            C.service_ready = self.client.service_is_ready()
            C.encoder_pubs = self.count_publishers(ENC)
            C.cmd_pubs = self.count_publishers(CMD)
            C.cmd_subs = self.count_subscribers(CMD)
            C.graph_at = time.monotonic()
            if C.inspect_request:
                C.inspect_request = False
                try:
                    if not C.service_ready: raise RuntimeError('AI service unavailable')
                    self.future = self.client.call_async(Trigger.Request())
                    C.inspect_started = time.monotonic()
                    C.log('system', 'Manual inspection requested (no movement command)')
                except Exception as e:
                    C.inspect_pending = False
                    C.log('system', f'Inspection failed: {e}')
            if self.future is not None:
                if self.future.done():
                    try:
                        reply = self.future.result()
                        C.log('system', 'Manual inspection: ' + ('success' if reply.success else reply.message))
                    except Exception as e: C.log('system', f'Inspection failed: {e}')
                    self.future = None
                    C.inspect_pending = False
                elif not C.running('ai'):
                    self.future.cancel()
                    self.future = None
                    C.inspect_pending = False
                elif time.monotonic()-C.inspect_started > 15:
                    # Do not release the interlock until the server actually responds or is restarted.
                    if C.inspect_pending and C.inspect_started != float('inf'):
                        C.log('system', 'Manual inspection timed out; restart AI service before navigation.')
                        C.inspect_started = float('inf')

HTML = r'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>AGROBOT | Mission Control</title><style>
:root{color-scheme:dark;--bg:#0b1418;--card:#132229;--line:#293c42;--muted:#91a6a9;--green:#75e1ad;--yellow:#ffd17b;--red:#ff8b8b}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:#e6f2f0;font:15px system-ui,sans-serif}main{max-width:1200px;margin:auto;padding:28px}header{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:22px}h1{font-size:clamp(24px,4vw,38px);margin:0;letter-spacing:-1px}h2{font-size:16px;margin:0 0 13px}p{margin:7px 0;color:var(--muted);line-height:1.5}.eyebrow{font-size:11px;font-weight:700;letter-spacing:2px;color:var(--green)}.tag{border:1px solid var(--line);border-radius:20px;padding:7px 12px;white-space:nowrap}.good{color:var(--green)}.warn{color:var(--yellow)}.bad{color:var(--red)}.grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px;margin-bottom:14px}.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:17px;min-width:0}.wide{grid-column:span 2}.value{font-size:25px;font-weight:650;overflow-wrap:anywhere;line-height:1.25}.sub{color:var(--muted);font-size:13px;margin-top:7px}.buttons{display:flex;gap:8px;flex-wrap:wrap;margin-top:14px}button,select{font:inherit;color:#e6f2f0;background:#20343c;border:1px solid #3c555e;border-radius:8px;padding:9px 13px;cursor:pointer}button:disabled{opacity:.4;cursor:not-allowed}button.primary{background:#174d39;border-color:#388762;color:#a0f4c9}button.stop{background:#532930;border-color:#a05b61;color:#ffd0d0}button.big{font-weight:700}.node{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:10px 0;border-bottom:1px solid var(--line)}.node:last-child{border:0}.node .buttons{margin:0}.label{font-size:12px;text-transform:uppercase;letter-spacing:1px;color:var(--muted)}.notice{border-left:3px solid var(--yellow);padding:10px 13px;margin:0 0 14px;background:#302b20;color:#f0d7aa;border-radius:6px;font-size:13px;line-height:1.5}#logs{height:290px;overflow:auto;background:#091116;border-radius:8px;padding:12px;font:12px/1.6 ui-monospace,monospace;white-space:pre-wrap;overflow-wrap:anywhere}.loghead{display:flex;justify-content:space-between;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:10px}.loghead h2{margin:0}#feedback{min-height:22px;margin:8px 0;color:var(--yellow)}footer{color:var(--muted);font-size:12px;margin-top:16px}@media(max-width:800px){main{padding:14px}.grid{grid-template-columns:1fr}.wide{grid-column:auto}header{align-items:flex-start}.value{font-size:22px}}
</style></head><body><main><header><div><div class="eyebrow">AGROBOT / BENCH PROTOTYPE</div><h1>Mission Control</h1><p>Raspberry Pi · ROS 2 Jazzy · ESP32 UART</p></div><span id="connection" class="tag warn">Connecting…</span></header><div class="notice">Supervised bench testing only. Keep the wheel elevated and a physical motor-power disconnect within reach. Software Stop is not an emergency stop. Closing this dashboard normally stops navigation after the heartbeat timeout.</div>
<div class="grid"><section class="card wide"><div class="label">Current navigation state</div><div class="value" id="phase">—</div><p id="cycle">Cycle — / 3</p><div class="buttons"><button id="startnav" class="primary big" onclick="startNav()">▶ Start 3 cycles</button><button class="stop big" onclick="act('stop','nav')">■ Stop navigation</button><button class="stop" onclick="act('stop_all')">Stop all nodes</button></div></section><section class="card"><div class="label">Motor command</div><div class="value" id="motor">—</div><div class="sub">Command only; not proof of physical motion</div><p id="encoder">Encoder: —</p></section></div>
<div class="grid"><section class="card wide"><h2>Latest AI inspection</h2><div class="value" id="prediction">No result yet</div><p id="metrics">Confidence — · Inference —</p><div class="sub" id="resultage">Waiting for /plant_health/result</div><div class="buttons"><button id="inspect" onclick="act('inspect')">Inspect once (no movement)</button></div></section><section class="card"><h2>ROS connectivity</h2><p id="service">Inspection service: —</p><p id="feedbackstate">Encoder feedback: —</p><p id="graph">ROS graph: —</p></section></div>
<section class="card" style="margin-bottom:14px"><h2>Node controls</h2><div class="node"><div><strong>Single motor bridge</strong><div id="bridge-status" class="sub">—</div></div><div class="buttons"><button id="bridge-start" class="primary" onclick="act('start','bridge')">Start</button><button onclick="act('stop','bridge')">Stop</button></div></div><div class="node"><div><strong>Camera AI service</strong><div id="ai-status" class="sub">—</div></div><div class="buttons"><button id="ai-start" class="primary" onclick="act('start','ai')">Start</button><button onclick="act('stop','ai')">Stop</button></div></div><div class="node"><div><strong>Automatic navigation V1</strong><div id="nav-status" class="sub">—</div></div><div class="buttons"><button class="primary" onclick="startNav()" id="nav-start">Start</button><button onclick="act('stop','nav')">Stop</button></div></div><div class="buttons"><button class="primary" onclick="act('prepare')">Start bridge + AI</button></div><div id="feedback" role="status"></div></section>
<section class="card"><div class="loghead"><h2>Live node messages</h2><div><select id="filter" onchange="renderLogs()"><option value="all">All messages</option><option value="bridge">Motor bridge</option><option value="ai">AI service</option><option value="nav">Navigation</option><option value="ai-result">AI results</option><option value="system">Dashboard</option></select> <label><input id="follow" type="checkbox" checked> Follow</label></div></div><div id="logs" aria-label="Live log"></div></section><footer>Local-only SSH dashboard · 3-cycle timed bench navigation · No obstacle avoidance or automatic plant-location detection.</footer></main><script>
const token=__TOKEN__; let state=null,busy=false,connected=false;const $=id=>document.getElementById(id);const set=(id,v)=>$(id).textContent=v;
async function act(action,node){if(busy&&action!=='stop'&&action!=='stop_all')return;busy=true;try{let r=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json','X-Dashboard-Token':token},body:JSON.stringify({action,node})});let d=await r.json();set('feedback',d.message);await update();}catch(e){set('feedback','Request failed: '+e);}finally{busy=false;}}
function startNav(){if(confirm('Start three powered movement cycles? Confirm wheel is elevated, area clear, and motor power can be disconnected immediately.'))act('start','nav');}
function renderLogs(){if(!state)return;let f=$('filter').value, box=$('logs'), old=box.scrollTop;box.textContent=state.logs.filter(l=>f==='all'||l.source===f).map(l=>`[${l.time}] [${l.source}] ${l.text}`).join('\n');box.scrollTop=$('follow').checked?box.scrollHeight:old;}
function render(){let s=state;set('connection','● Dashboard connected');$('connection').className='tag good';set('phase',s.phase);set('cycle',`Cycle ${s.cycle} / 3`);set('motor',s.command===null?'No fresh command':s.command>.02?'FORWARD':s.command<-.02?'REVERSE':'STOP');$('motor').className='value '+(s.command===null?'warn':Math.abs(s.command)>.02?'warn':'good');set('encoder',`Encoder: ${s.encoder??'—'} ticks`);set('feedbackstate','Encoder feedback: '+(s.encoder_age!==null&&s.encoder_age<1?'LIVE':'STALE / unavailable'));set('service','Inspection service: '+(s.service_ready?'READY':'NOT READY'));set('graph','ROS graph: '+(s.graph_ready?'CONNECTED':'UNAVAILABLE'));let a=s.result;set('prediction',a?.prediction??'No result yet');let conf=typeof a?.confidence==='number'?(a.confidence*100).toFixed(1)+'%':'—';let ms=typeof a?.inference_ms==='number'?a.inference_ms.toFixed(1)+' ms':'—';set('metrics',`Tomato-class confidence ${conf} · Inference ${ms}`);set('resultage',a?`Last result ${s.result_age}s ago · ${a.status??'received'}`:'Waiting for /plant_health/result');for(let k of ['bridge','ai','nav']){let n=s.nodes[k];set(k+'-status',n.running?'● Running':n.exit===null?'○ Not running':`○ Exited (${n.exit})`);$(k+'-status').className='sub '+(n.running?'good':'');$(k+'-start').disabled=n.running;}let ready=s.nodes.bridge.running&&s.nodes.ai.running&&!s.nodes.nav.running&&s.service_ready&&s.encoder_age!==null&&s.encoder_age<1&&!s.inspect_pending&&s.graph_ready;$('startnav').disabled=$('nav-start').disabled=!ready;$('inspect').disabled=!s.nodes.ai.running||!s.service_ready||s.nodes.nav.running||s.inspect_pending;renderLogs();}
async function update(){try{let r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw Error(r.status);state=await r.json();connected=true;render();}catch(e){connected=false;set('connection','● Disconnected');$('connection').className='tag bad';for(let k of ['startnav','nav-start','bridge-start','ai-start','inspect'])$(k).disabled=true;}}
async function beat(){try{await fetch('/api/heartbeat',{method:'POST',headers:{'X-Dashboard-Token':token}});}catch(e){}}
update();beat();setInterval(update,650);setInterval(beat,2000);
</script></body></html>'''

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_): pass
    def reply(self, code, payload, html=False):
        body = payload.encode() if html else json.dumps(payload, allow_nan=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'text/html; charset=utf-8' if html else 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.end_headers()
        try: self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError): pass

    def host_ok(self): return self.headers.get('Host') in (f'127.0.0.1:{PORT}', f'localhost:{PORT}')
    def do_GET(self):
        if not self.host_ok(): return self.reply(403, {'message':'Invalid Host'})
        if self.path == '/': return self.reply(200, HTML.replace('__TOKEN__',json.dumps(TOKEN)), True)
        if self.path == '/api/state': return self.reply(200, C.state())
        return self.reply(404, {'message':'Not found'})

    def do_POST(self):
        if not self.host_ok() or not secrets.compare_digest(self.headers.get('X-Dashboard-Token',''),TOKEN):
            return self.reply(403, {'message':'Unauthorized'})
        if self.path == '/api/heartbeat':
            with C.lock: C.heartbeat = time.monotonic()
            return self.reply(200, {'message':'OK'})
        if self.path != '/api/action': return self.reply(404, {'message':'Not found'})
        try:
            length = int(self.headers.get('Content-Length',0))
            if not 0 < length <= 4096: raise ValueError('Invalid request size')
            data = json.loads(self.rfile.read(length))
            action, key = data.get('action'), data.get('node')
            if action == 'start' and key in SPECS: C.start(key)
            elif action == 'stop' and key in SPECS: C.stop(key)
            elif action == 'stop_all': C.stop_all()
            elif action == 'prepare':
                with C.actions:
                    C.start('bridge'); C.start('ai')
            elif action == 'inspect':
                with C.actions, C.lock:
                    if not C.running('ai') or not C.service_ready or C.running('nav') or C.cmd_pubs or C.inspect_pending:
                        raise ValueError('Manual inspection requires managed AI service, no navigation or external movement publisher, and no pending inspection.')
                    C.inspect_pending = C.inspect_request = True
            else: raise ValueError('Unknown action')
            return self.reply(200, {'message':'Command accepted. See node status and logs.'})
        except Exception as e:
            C.log('system', str(e))
            return self.reply(400, {'message':str(e)})

class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    server = Server(('127.0.0.1',PORT),Handler)
    server.timeout = 0.3
    quit_event = threading.Event()
    rclpy.init()
    node = Monitor()
    def spin():
        try: rclpy.spin(node)
        except Exception as e: C.log('system', f'ROS monitor stopped: {e}')
    t = threading.Thread(target=spin,daemon=True); t.start()
    threading.Thread(target=C.monitor,daemon=True).start()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        signal.signal(sig,lambda *_:quit_event.set())
    print(f'AGROBOT dashboard ready: http://127.0.0.1:{PORT}',flush=True)
    print('No robot nodes started. Open the browser through SSH forwarding.',flush=True)
    try:
        while not quit_event.is_set(): server.handle_request()
    finally:
        C.closed=True
        C.stop_all()
        server.server_close()
        if rclpy.ok(): rclpy.shutdown()
        t.join(timeout=2)
        node.destroy_node()
        print('Dashboard stopped. Verify motor power is disconnected.',flush=True)

if __name__ == '__main__': main()
