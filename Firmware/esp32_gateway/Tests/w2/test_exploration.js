/* No dependencies: run the real script with DOM/fetch/clock substitutes. */
async function runExplorationTests(source) {
  const results = [], assert = (v, m) => { if (!v) throw new Error(m); };
  async function fixture() {
    let now = 0, serial = 0, nonce = 0, failure = false, latency = 0, autoLag = 0, posts = 0, maximum = 0;
    const timers = new Map(), nodes = new Map(), events = {}, docEvents = {}, requests = [];
    const status = { valid: true, connected: true, state: 0, faults: 0, stop: 0, age_ms: 0 };
    let sample = {session_pending:false,session_active:false,renewal_source:"none",renewals_required:false,authorization_mode:"session",session_started:false,session_cancelled:false,stop_received:false,link_connected:false,link_confirmed:false,link_monitoring:false,link_age_ms:null,link_watchdog_ms:3000,terminal_reason_code:0,terminal_reason:"NONE",control_ready:false,stabilization_state:0,stabilization_observations:0,stabilization_ms:0,received:false,connected:false,available:false,historical:false,sectors_available:false,age_ms:0,
      session_id:0,started_tick:0,state_code:0,action_code:0,challenge:0,segments:0,turns:0,retreats:0,
      selected_sector:0,candidates:0,repeated:0,lease_ms:0,sequence:0,critical_faults:0,scan_deg:0,block_reason:"none",
      sector_step_deg:10,sectors:"",range_mm:Array(36).fill(null)};
    const node = id => { if (!nodes.has(id)) nodes.set(id, { disabled:false, checked:false, textContent:"", events:{}, addEventListener(e,f){this.events[e]=f;} }); return nodes.get(id); };
    const document = {hidden:false,getElementById:node,addEventListener(e,f){docEvents[e]=f;}};
    const set = (fn,dt) => { const id=++serial;timers.set(id,{fn,at:now+dt});return id; };
    const clear = id => timers.delete(id);
    class Controller { constructor(){this.signal={};}abort(){this.signal.abort?.();} }
    const crypto = {getRandomValues(a){a.fill(++nonce);return a;}};
    const global = {crypto,addEventListener(e,f){events[e]=f;}};
    const response = data => ({ok:true,json:async()=>data});
    async function fetchMock(url,options) {
      requests.push({url,options,at:now});
      if(failure)throw Error("offline");
      if(!options.method){assert(url==="/api/exploration","only local GET");return response(sample);}
      assert(options.method==="POST","POST only");
      assert(url==="/api/command","browser must never renew supervision");
      if(options.body==="AUTO_EXPLORE"){
        assert(/^[a-f0-9]{32}$/.test(options.headers["X-Explore-Owner"]),"private initial owner capability");
        sample={...sample,session_pending:true,session_active:false};
        set(()=>{status.state=1;sample={...sample,session_pending:false,session_active:true,session_started:true,link_connected:true,link_age_ms:0,control_ready:true,received:true,available:true,connected:true,state_code:6,action_code:1,session_id:1};},autoLag||300);
      }else{
        assert(options.body==="STOP","closed STOP only");status.state=0;
        sample={...sample,session_pending:false,session_active:false,session_cancelled:true,stop_received:true,terminal_reason_code:11,terminal_reason:"OPERATOR_STOP",control_ready:false,state_code:2,historical:true};
      }
      return response({accepted:true,confirmed:false});
    }

    const initialize = new Function("document","fetch","AbortController","setTimeout","clearTimeout","performance","globalThis","crypto","lastStatus",
      "let lastStatusAt=0;\n"+source+"\nreturn ()=>{lastStatusAt=performance.now();};");
    const refresh=initialize(document,fetchMock,Controller,set,clear,{now:()=>now},global,crypto,status);
    async function flush(){for(let i=0;i<15;++i)await Promise.resolve();}
    async function advance(dt){const end=now+dt;let guard=0;
      while(true){await flush();const next=[...timers].filter(([,v])=>v.at<=end).sort((a,b)=>a[1].at-b[1].at)[0];if(!next)break;
        assert(++guard<2000,"bounded scheduling");now=next[1].at;timers.delete(next[0]);refresh();next[1].fn();}
      now=end;refresh();await flush();
    }
    const postsTo = path => requests.filter(r=>r.url===path && r.options.method==="POST");
    await flush();return {node,status,requests,postsTo,events,docEvents,document,advance,flush,
      setFailure:v=>{failure=v;},setLatency:v=>{latency=v;},setAutoLag:v=>{autoLag=v;},setSample:v=>{sample={...sample,...v};},maximum:()=>maximum};
  }
  let f=await fixture();await f.advance(1000);
  assert(!f.postsTo("/api/command").length && !f.postsTo("/api/exploration").length,"no startup commands");
  await f.node("explore-start").events.click();assert(!f.postsTo("/api/command").length,"consent required");
  results.push("LOCAL load or reconnection sends GET only; explicit initial consent remains mandatory");
  f.node("explore-supervised").checked=true;
  const firstStart=f.node("explore-start").events.click();f.node("explore-start").events.click();await firstStart;await f.advance(1000);
  assert(f.postsTo("/api/command").length===1 && !f.postsTo("/api/exploration").length,"one start, no browser renewals");
  assert(!f.node("explore-supervised").checked,"consent consumed");
  const first=f.postsTo("/api/command")[0];assert(first.options.headers["X-Request-ID"]!==first.options.headers["X-Explore-Owner"],"separate initial capability");
  assert(f.node("explore-message").textContent.includes("Sesion confirmada"),"confirmation comes from telemetry");
  results.push("LOCAL one explicit initial POST, no duplicates or WATCH; STM32 confirmation is distinct from API acceptance");
  f=await fixture();f.setAutoLag(900);f.node("explore-supervised").checked=true;await f.node("explore-start").events.click();await f.advance(500);
  assert(!f.node("explore-message").textContent.includes("Sesion confirmada"),"pending is not confirmed");
  await f.advance(1000);assert(f.node("explore-message").textContent.includes("Sesion confirmada"),"later actual confirmation shown");
  results.push("LOCAL pending handshake remains visibly unconfirmed until the server reports an active STM32 session");
  for(const event of ["blur","pagehide","visibilitychange"]){
    f=await fixture();f.node("explore-supervised").checked=true;await f.node("explore-start").events.click();await f.advance(1000);
    if(event==="visibilitychange"){f.document.hidden=true;f.docEvents[event]?.();}else f.events[event]?.();
    await f.advance(3000);assert(f.postsTo("/api/command").length===1 && !f.postsTo("/api/exploration").length,"no browser cancel or renewal on "+event);
  }
  results.push("LOCAL locking the phone, changing tabs and closing the page neither renew nor cancel ESP32-managed continuity");
  f=await fixture();f.node("explore-supervised").checked=true;await f.node("explore-start").events.click();await f.advance(1000);
  f.setFailure(true);await f.advance(1500);
  assert(f.node("explore-diagnostics").textContent.includes("puede continuar"),"offline dashboard does not claim a stopped robot");
  f.setFailure(false);await f.advance(1500);assert(f.postsTo("/api/command").length===1 && !f.postsTo("/api/exploration").length,"reconnect does not start or take over supervision");
  assert(f.node("explore-message").textContent.includes("Sesion confirmada"),"real active session rediscovered");
  results.push("LOCAL Wi-Fi loss affects dashboard availability only; reconnect observes the continuing session without restarting it");
  f=await fixture();f.status.state=1;f.setSample({session_active:true,session_started:true,link_connected:true,link_age_ms:0,control_ready:true,received:true,available:true,connected:true,session_id:9,state_code:1});await f.advance(1000);
  assert(f.node("explore-start").disabled && !f.node("explore-end").disabled,"reconnected client can stop without starting");
  await f.node("explore-end").events.click();await f.advance(1000);
  assert(f.postsTo("/api/command").length===1 && f.postsTo("/api/command")[0].options.body==="STOP","reconnect STOP uses existing closed command path");
  await f.advance(3000);assert(f.postsTo("/api/command").length===1,"no restart after STOP");
  results.push("LOCAL new browser can observe and STOP an existing session; STOP never silently restarts exploration");
  f=await fixture();f.status.state=3;f.status.faults=1;f.status.stop=1;f.setSample({session_active:false,received:true,available:true,state_code:2,historical:true});await f.advance(1000);
  assert(f.node("explore-start").disabled && !f.postsTo("/api/command").length,"critical state cannot auto-start");
  f.status.state=0;f.status.faults=f.status.stop=0;await f.advance(2000);assert(!f.postsTo("/api/command").length,"rearm does not start");
  results.push("LOCAL Safety, STOP and mode changes are observations, never automatic restart or a new approval");
  f=await fixture();f.node("explore-supervised").checked=true;f.setFailure(true);await f.node("explore-start").events.click();f.setFailure(false);await f.advance(4000);
  assert(f.postsTo("/api/command").length===1 && !f.postsTo("/api/exploration").length,"ambiguous start not retried");
  results.push("LOCAL HTTP start failure is not retried; delayed or recovered network does not produce another start");
  f=await fixture();f.setFailure(true);await f.node("explore-end").events.click();
  assert(f.node("explore-message").textContent.includes("corte fisico"),"unconfirmed STOP warns about physical cut");
  results.push("LOCAL unavailable remote STOP reports no confirmation and retains the physical power-cut instruction");
  f=await fixture();f.setSample({scan_deg:NaN});await f.advance(500);assert(f.node("explore-diagnostics").textContent.includes("no disponible"),"invalid telemetry");
  assert(!source.includes("body: `WATCH ") && !source.includes("explore-approve"),"no heartbeat or per-maneuver approval implemented in JS");
  results.push("LOCAL malformed telemetry is rejected; dashboard has no heartbeat or per-maneuver approval path");
  f=await fixture();f.status.state=1;
  f.setSample({received:true,available:true,connected:true,session_started:true,session_active:false,session_cancelled:true,state_code:2,historical:true,
    link_connected:false,link_confirmed:false,link_monitoring:false,link_age_ms:3000,terminal_reason_code:16,terminal_reason:"INTERNAL_LINK_LOST",stop_received:true});
  await f.advance(1000);
  assert(f.node("explore-diagnostics").textContent.includes("INTERNAL_LINK_LOST") && f.node("explore-diagnostics").textContent.includes("STOP recibido"),"specific communication cause and retained STOP are visible");
  assert(!f.postsTo("/api/command").length && f.node("explore-start").disabled,"a recovered dashboard cannot restart a terminal AUTO session");
  results.push("LOCAL internal link loss is distinct from authorization, retains terminal STOP history and never resumes on reconnection");
  return results;
}
if(typeof module!=="undefined")module.exports={runExplorationTests};
