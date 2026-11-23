"""Local-only visual fixtures. Never sent to the robot or embedded in firmware."""
from pathlib import Path
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import json, threading, subprocess, re, sys

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'.pio/build/ui_redesign/preview'
OUT.mkdir(parents=True,exist_ok=True)
CHROME=Path('C:/Program Files/Google/Chrome/Application/chrome.exe')

def fixtures(mode):
    valid=mode not in ['empty','lost']
    state={'manual':2,'auto':1,'safety':3}.get(mode,0)
    faults=1 if mode=='safety' else 0
    status=dict(valid=valid,connected=valid,state=state if valid else None,faults=faults if valid else None,stop=(1 if faults else 0) if valid else None,age_ms=0 if valid else None)
    cells=['0']*6400
    for row in range(28,56):
        for col in range(23,57):cells[row*80+col]='1'
    for row in range(29,54):cells[row*80+56]='2'
    pose=dict(valid=valid,state='valid' if mode!='empty' else 'no_sample',stale=not valid,tick=0 if mode!='empty' else None,age_ms=900 if mode=='lost' else 0 if valid else None,left_counts='12345' if valid else None,right_counts='12300' if valid else None,x_mm=300 if mode!='empty' else None,y_mm=150 if mode!='empty' else None,theta_mrad=785 if mode!='empty' else None,increment_mm=0 if mode!='empty' else None,increment_mrad=0 if mode!='empty' else None)
    mapping=dict(connected=valid,calibrated=False,calibration='provisional' if mode!='empty' else 'incomplete',epoch=1,pose=pose,grid=dict(width=80,height=80,resolution_mm=50,origin_x_mm=-2000,origin_y_mm=-2000,revision=1,cells=''.join(cells) if mode!='empty' else '0'*6400),trajectory_mm=[[0,0],[0,50],[150,50],[300,150]] if mode!='empty' else [],rays=[dict(state=3,age_ms=50),dict(state=3,age_ms=120)])
    u=lambda distance:dict(state='valid',valid=True,failed=False,stale=False,distance_mm=distance,age_ms=120)
    sensors=dict(valid=True,connected=valid,stm32_connected=valid,paused=mode=='manual',snapshot_available=True,age_ms=0,snapshot_age_ms=0,limits_ms=dict(snapshot=1500,ultrasound=1500,ir=1500),front=u(682),rear=u(487),ir=dict(available=True,stale=False,floor=[True]*4,cliff=False,age_ms=40))
    imu=dict(received=True,connected=valid,stm32_connected=valid,paused=mode=='manual',valid=valid,stale=not valid,state='valid',hal_status=0,i2c_error=0,max_age_ms=1500,response_age_ms=0,sample_age_ms=0,accel_g=[0,0,1] if valid else [None]*3,gyro_dps=[0,0,0] if valid else [None]*3,gyro_raw=[0,0,0] if valid else [None]*3,gyro_corrected_dps=[0,0,0] if valid else [None]*3,calibration=dict(available=True,valid=valid,state='valid',reason='none',samples=250,target_samples=250,request=1,age_ms=1000,request_phase='idle',offset_dps=[0,0,0] if valid else [None]*3))
    exploration=dict(session_pending=False,session_active=mode=='auto',session_started=mode=='auto',session_cancelled=False,stop_received=False,link_connected=valid,link_confirmed=valid,link_monitoring=mode=='auto',renewals_required=False,control_ready=valid,received=True,connected=valid,available=mode=='auto',historical=False,sectors_available=False,age_ms=0,session_id=1,started_tick=0,state_code=6 if mode=='auto' else 0,action_code=1 if mode=='auto' else 0,challenge=0,segments=12,turns=8,retreats=1,selected_sector=0,candidates=0,repeated=0,lease_ms=0,sequence=0,critical_faults=faults,stabilization_state=2,stabilization_observations=4,stabilization_ms=60,authorization_mode='session',renewal_source='none',link_age_ms=0,link_watchdog_ms=3000,terminal_reason_code=0,terminal_reason='NONE',scan_deg=0,block_reason='none',sector_step_deg=10,sectors='',range_mm=[None]*36)
    return {'/health':dict(status='ok',service='esp32-web'),'/api/status':status,'/api/map':mapping,'/api/sensors':sensors,'/api/imu':imu,'/api/exploration':exploration,'/api/navigation':dict(received=False,available=False)}

CHECK=r'''window.addEventListener('error',e=>{window.previewErrors=(window.previewErrors||[]).concat(e.message);});
setTimeout(()=>{
 const results=[], assert=(v,n)=>results.push({name:n,pass:!!v});
 const scenario=document.body.dataset.previewScenario;
 const id=n=>document.getElementById(n),rect=n=>id(n).getBoundingClientRect();
 const ids=[...document.querySelectorAll('[id]')].map(n=>n.id);
 assert(new Set(ids).size===ids.length,'unique DOM identifiers');
 assert(document.documentElement.scrollWidth<=innerWidth,'no horizontal overflow');
 assert(rect('command-stop').top>=0&&rect('command-stop').bottom<=innerHeight,'STOP visible initially');
 assert(id('server-status').dataset.state==='ok','ESP32 health independent');
 assert(id('map-occupancy').checked,'possible occupancy visible by default');
 assert(id('robot-state').textContent===({idle:'En espera',manual:'Modo manual',auto:'Modo autónomo',safety:'Detenido por seguridad',empty:'Desconocido',lost:'Desconocido'}[scenario]),'real FSM labels and unknown states');
 assert(id('explore-start').disabled&&!id('explore-supervised').checked,'initial consent never bypassed');
 assert(id('manual-forward').disabled===(scenario!=='manual'),'manual controls require actual MANUAL');
 assert(id('imu-calibrate').disabled===(scenario!=='idle'&&scenario!=='safety'),'calibration mode restrictions');
 assert(!document.querySelector('.technical-panel').open&&!id('auto3-supervised').checked,'historic experiments secondary and not authorized');
 if(scenario==='lost')assert(id('map-state').textContent.includes('histórica'),'lost telemetry stays historical');
 if(scenario==='empty')assert(id('map-state').textContent==='Sin muestra','no sample is not a fictional pose');
 const before=id('map-canvas').toDataURL(); id('map-zoom-in').click();
 if(!id('map-zoom-in').disabled){assert(id('map-zoom').textContent==='2×','zoom click');assert(before!==id('map-canvas').toDataURL(),'zoom changes drawing');}
 id('map-center').click();id('map-fit').click();
 const visible=id('map-canvas').toDataURL();
 id('map-occupancy').checked=false;id('map-occupancy').dispatchEvent(new Event('change'));
 if(scenario!=='empty')assert(visible!==id('map-canvas').toDataURL(),'occupancy switch changes only the visual layer');
 id('map-occupancy').checked=true;id('map-occupancy').dispatchEvent(new Event('change'));
 assert(visible===id('map-canvas').toDataURL(),'show restores the same drawing');
 id('manual-panel').open=true;
 const f=rect('manual-forward'),l=rect('manual-left'),r=rect('manual-reverse'),d=rect('manual-right'),h=rect('manual-halt');
 assert(f.top<l.top&&Math.abs(l.top-r.top)<2&&Math.abs(r.top-d.top)<2&&h.top>l.top,'manual cross layout');
 assert(f.left>l.left&&r.left>l.left&&d.left>r.left&&h.width>r.width*2.8,'manual columns and full-width halt');
 assert(f.width>=44&&f.height>=44,'touch target');
 id('manual-panel').open=false;
 scrollTo(0,document.body.scrollHeight);assert(rect('command-stop').top>=0&&rect('command-stop').bottom<=innerHeight,'sticky STOP at page end');scrollTo(0,0);
 assert(!window.previewErrors?.length,'no uncaught application exceptions');
 const report={width:innerWidth,results,errors:window.previewErrors||[],mode:document.body.dataset.connection};
 const pre=document.createElement('pre');pre.id='preview-results';pre.hidden=true;pre.textContent=JSON.stringify(report);document.body.append(pre);
 if(parent!==window)parent.postMessage({previewReport:report},location.origin);
},1400);'''

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*a):pass
    def do_GET(self):
        url=urlparse(self.path);mode=parse_qs(url.query).get('scenario',[None])[0]
        if mode is None:mode=(self.headers.get('Cookie','').split('preview=')[-1].split(';')[0] or 'idle')
        if mode not in ['idle','manual','auto','safety','empty','lost']:mode='idle'
        frame=parse_qs(url.query).get('frame',[None])[0]
        fixture=fixtures(mode).get(url.path)
        if frame:
            width=int(frame)
            body=('''<!doctype html><meta charset="utf-8"><style>body{margin:0;background:#0b121a}iframe{border:0;display:block}</style><script>addEventListener('message',e=>{if(e.origin!==location.origin||!e.data.previewReport)return;const p=document.createElement('pre');p.id='preview-results';p.hidden=true;p.textContent=JSON.stringify(e.data.previewReport);document.body.append(p);});</script>'''+f'<iframe style="width:{width}px;height:1000px" src="/?scenario={mode}"></iframe>').encode();mime='text/html'
        elif fixture is not None:body=json.dumps(fixture).encode();mime='application/json'
        elif url.path=='/preview-check.js':body=CHECK.encode();mime='application/javascript'
        else:
            p=ROOT/'web'/('index.html' if url.path=='/' else url.path.lstrip('/'))
            if not p.is_file():self.send_error(404);return
            body=p.read_bytes();mime={'.html':'text/html','.js':'application/javascript','.css':'text/css'}.get(p.suffix,'text/plain')
            if p.suffix=='.html':body=body.replace(b'</body>',b'<div style="position:fixed;bottom:0;left:0;z-index:100;background:#f2c86e;color:#111;padding:4px 12px;font-size:12px">PRUEBA LOCAL - DATOS SINTETICOS</div><script src="/preview-check.js"></script></body>')
        if mime=='text/html' and not frame:body=body.replace(b'<body>',f'<body data-preview-scenario="{mode}">'.encode())
        self.send_response(200);self.send_header('Content-Type',mime+'; charset=utf-8');self.send_header('Set-Cookie','preview='+mode+'; SameSite=Strict');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def do_POST(self):
        self.server.preview_posts+=1
        self.send_error(403,'Preview never executes robot commands')

if __name__=='__main__':
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler);server.preview_posts=0
    threading.Thread(target=server.serve_forever,daemon=True).start()
    reports=[]
    targets=[(320,'idle'),(375,'idle'),(390,'manual'),(768,'auto'),(1280,'idle'),(390,'safety'),(390,'empty'),(390,'lost')]
    if len(sys.argv)>1:targets=[(int(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else 'idle')]
    for width,mode in targets:
        name=f'{width}_{mode}';profile=OUT/('chrome_'+name)
        args=[str(CHROME),'--headless=new','--no-first-run','--no-default-browser-check','--disable-background-networking','--disable-component-update','--disable-gpu','--hide-scrollbars',f'--user-data-dir={profile}',f'--window-size={max(500,width)},1150','--force-device-scale-factor=1','--virtual-time-budget=2200',f'--screenshot={OUT/(name+".png")}','--dump-dom',f'http://127.0.0.1:{server.server_port}/?frame={width}&scenario={mode}']
        r=subprocess.run(args,capture_output=True,timeout=45)
        (OUT/(name+'.html')).write_bytes(r.stdout);(OUT/(name+'.stderr')).write_bytes(r.stderr)
        s=r.stdout.decode('utf-8',errors='replace');m=re.search(r'<pre id="preview-results"[^>]*>(.*?)</pre>',s,re.S)
        if not m:raise RuntimeError('No browser results for '+name+': '+r.stderr.decode(errors='replace')[-1500:])
        import html
        report=json.loads(html.unescape(m[1]));reports.append(dict(requested_width=width,scenario=mode,**report));print(json.dumps(reports[-1]),flush=True)
        assert report['width']==width,'Viewport width mismatch'
    (OUT/'browser_results.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
    server.shutdown()
    assert server.preview_posts==0,'Unexpected POST during view checks'
    assert all(all(x['pass'] for x in r['results']) for r in reports),'Browser assertions failed'
