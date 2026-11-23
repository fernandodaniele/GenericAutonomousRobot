async function runManualTests(appSource, manualSource, report = () => {}, sensorsSource = "", mapSource = "") {
  const assert = (value, message) => { if (!value) throw new Error(message); };
  async function fixture({ latency = 0, openLatency = 0 } = {}) {
    let now = 0, serial = 1, nonce = 1, mode = "ok", openResolve;
    let pendingRenewals = 0, maxPendingRenewals = 0, serverExpires = Infinity;
    const nodes = new Map(), timers = new Map(), requests = [], docEvents = {}, winEvents = {};
    let status = { valid: true, connected: true, state: 2, faults: 0, stop: 0, age_ms: 0 };
    function node(id) {
      if (!nodes.has(id)) nodes.set(id, { dataset: {}, textContent: "", disabled: false, events: {},
        addEventListener(event, fn) { this.events[event] = fn; }, setPointerCapture() {}, getContext() { return null; } });
      return nodes.get(id);
    }
    const document = { hidden: false, getElementById: node, addEventListener(event, fn) { docEvents[event] = fn; } };
    const window = { addEventListener(event, fn) { winEvents[event] = fn; } };
    class Controller { constructor() { this.signal = {}; } abort() { this.signal.aborted = true; this.signal.abort?.(); } }
    const response = body => ({ ok: true, json: async () => body });
    function delayedRenew(options) {
      ++pendingRenewals; maxPendingRenewals = Math.max(maxPendingRenewals, pendingRenewals);
      return new Promise((resolve, reject) => {
        let done = false, replyTimer;
        const finish = (value, error) => {
          if (done) return; done = true; --pendingRenewals;
          timers.delete(arrivalTimer); timers.delete(replyTimer);
          error ? reject(error) : resolve(value);
        };
        options.signal.abort = () => finish(null, new Error("abort"));
        const arrivalTimer = serial++;
        timers.set(arrivalTimer, { at: now + latency / 2, fn() {
          const accepted = now < serverExpires;
          if (accepted) serverExpires = now + 300;
          replyTimer = serial++;
          timers.set(replyTimer, { at: now + latency / 2, fn() {
            finish(accepted ? response({ accepted: true, lease_ms: 300 }) :
              { ok: false, json: async () => ({ error: "expired" }) });
          } });
        } });
      });
    }
    async function fetchMock(url, options) {
      requests.push({ url, options, now });
      if (url === "/health") return response({ status: "ok", service: "esp32-web" });
      if (url === "/api/status") return response(status);
      if (url === "/api/sensors") throw new Error("telemetry unavailable");
      if (url === "/api/map") throw new Error("mapping unavailable");
      assert(options.method === "POST", "operations require POST");
      if (url === "/api/command") return response({ accepted: true, confirmed: false,
        id: options.headers["X-Request-ID"], command: options.body, phase: "queued" });
      assert(/^\/api\/manual\/(start|renew|end|halt)$/.test(url), "closed manual paths");
      assert(options.headers["Content-Type"] === "text/plain" && /^[a-f0-9]{32}$/.test(options.headers["X-Request-ID"]), "headers");
      if (url.endsWith("/start")) {
        if (["busy", "unsafe", "expired"].includes(mode))
          return { ok: false, json: async () => ({ error: mode }) };
        serverExpires = now + 300;
        const value = { accepted: true, token: (nonce++).toString(16).padStart(32, "0"), lease_ms: 300 };
        if (mode === "late-open") return new Promise(resolve => { openResolve = () => resolve(response(value)); });
        if (openLatency > 0) {
          serverExpires = now + openLatency / 2 + 300;
          return new Promise((resolve, reject) => {
            const timer = serial++;
            timers.set(timer, { at: now + openLatency, fn: () => resolve(response(value)) });
            options.signal.abort = () => { timers.delete(timer); reject(new Error("abort")); };
          });
        }
        return response(value);
      }
      if (url.endsWith("/renew")) {
        assert(["AVANZAR", "RETROCEDER", "GIRAR_IZQUIERDA", "GIRAR_DERECHA"].includes(options.body), "closed directions");
        assert(/^[a-f0-9]{32}$/.test(options.headers["X-Session-Token"]) && /^[1-9][0-9]*$/.test(options.headers["X-Sequence"]), "owner sequence");
        if (mode === "offline") throw new Error("WiFi");
        if (mode === "pending-renew") return new Promise((resolve, reject) => { options.signal.abort = () => reject(new Error("abort")); });
        if (mode === "rejected") return { ok: false, json: async () => ({ error: "expired" }) };
        if (latency > 0) return delayedRenew(options);
        serverExpires = now + 300;
        return response({ accepted: true, lease_ms: 300 });
      }
      return response({ accepted: true, phase: "halt_requested" });
    }
    const run = new Function("document", "window", "fetch", "AbortController", "setTimeout", "clearTimeout", "performance", "crypto",
      appSource + "\n" + manualSource + "\n" + sensorsSource + "\n" + mapSource + "\nreturn { snapshot(s) {renderStatus(s);manualControl.onStatus(s);}, cancel:manualControl.cancel }; ");
    const api = run(document, window, fetchMock, Controller,
      (fn, delay) => { const id = serial++; timers.set(id, { fn, at: now + delay }); return id; },
      id => timers.delete(id), { now: () => now }, { getRandomValues(bytes) { bytes.fill(nonce++ % 256); return bytes; } });
    async function flush() { for (let i = 0; i < 18; i++) await Promise.resolve(); }
    async function advance(ms) {
      const target = now + ms;
      for (let n = 0; n < 200; n++) {
        const first = [...timers].filter(([, t]) => t.at <= target).sort((a, b) => a[1].at - b[1].at)[0];
        if (!first) { now = target; await flush(); return; }
        const [id, timer] = first; timers.delete(id); now = Math.max(now, timer.at); timer.fn(); await flush();
      }
      throw new Error("timer burst");
    }
    const event = (pointerId = 7) => ({ pointerId, isPrimary: true, button: 0, preventDefault() {} });
    async function press(name = "forward") { node("manual-" + name).events.pointerdown(event()); await flush(); }
    async function release() { docEvents.pointerup(event()); await flush(); }
    const count = suffix => requests.filter(r => r.url === "/api/manual/" + suffix).length;
    await flush();
    return { node, requests, press, release, advance, flush, count, docEvents, winEvents, document,
      maxPending() { return maxPendingRenewals; },
      latency(value) { latency = value; }, stall(ms) { now += ms; },
      mode(value) { mode = value; }, resolveOpen() { openResolve(); },
      snapshot(value) { status = { ...status, ...value }; api.snapshot(status); } };
  }
  const results = [];
  let f = await fixture();
  assert(f.count("start") === 0 && !f.node("manual-forward").disabled, "safe UI ready without emission");
  await f.press(); assert(f.count("start") === 1 && f.count("renew") === 1, "open before direction");
  await f.advance(600); assert(f.count("renew") === 4, "held 200-ms cadence");
  const sequences = f.requests.filter(r => r.url.endsWith("/renew")).map(r => r.options.headers["X-Sequence"]);
  assert(sequences.join() === "1,2,3,4", "monotonic sequences");
  results.push("held press opens once and renews every 200 ms with strictly increasing sequences");
  await f.release(); f.docEvents.pointercancel({ pointerId: 7 }); await f.advance(600);
  assert(f.count("end") === 1 && f.count("renew") === 4, "release once no later renew");
  assert(f.node("manual-session").textContent === "Sesión manual inactiva", "inactive");
  results.push("release/cancel duplicates close once, no residual directions or click action");
  f = await fixture(); f.mode("late-open"); await f.press(); await f.release();
  f.resolveOpen(); await f.flush(); assert(f.count("renew") === 0 && f.count("end") === 1, "late grant after release");
  results.push("open response after release is closed without a single direction");
  for (const event of ["blur", "pagehide", "visibilitychange", "pointercancel", "lostpointercapture"]) {
    f = await fixture(); await f.press();
    if (event === "visibilitychange") { f.document.hidden = true; f.docEvents[event](); }
    else if (event === "pointercancel") f.docEvents[event]({ pointerId: 7 });
    else if (event === "lostpointercapture") f.node("manual-forward").events[event]({ pointerId: 7 });
    else f.winEvents[event]();
    await f.flush(); const count = f.count("renew"); await f.advance(600);
    assert(f.count("end") === 1 && f.count("renew") === count, event);
    const reason = { blur: "foco", pagehide: "cerró la página", visibilitychange: "Página oculta",
      pointercancel: "canceló el puntero", lostpointercapture: "captura del puntero" }[event];
    assert(f.node("manual-message").textContent.includes(reason), "distinct cancellation cause: " + event);
  }
  results.push("blur, pagehide, hidden document, pointercancel and lost capture release authorization");
  for (const mode of ["offline", "pending-renew", "rejected"]) {
    f = await fixture(); await f.press(); f.mode(mode); await f.advance(700);
    const count = f.count("renew"); f.mode("ok"); f.snapshot({ connected: true }); await f.advance(700);
    assert(f.count("renew") === count && f.count("start") === 1 && f.node("manual-session").textContent === "Sesión manual inactiva", mode);
  }
  results.push("Wi-Fi failure, hung request and expired/rejected grant close lease and never reopen on recovery");
  for (const unsafe of [{ state: 0 }, { faults: 4 }, { stop: 1 }, { connected: false }, { age_ms: 2000 }, { valid: false }]) {
    f = await fixture(); await f.press(); f.snapshot(unsafe); await f.flush();
    assert(f.node("manual-forward").disabled && f.count("end") === 1, "unsafe cache");
  }
  results.push("mode change, Safety, STOP, old or absent STATUS disable directions and close active press");
  f = await fixture(); await f.press(); await f.press("left");
  f.docEvents.pointerup({ pointerId: 99 }); await f.flush(); assert(f.count("start") === 1 && f.count("end") === 0, "single pointer");
  f.node("command-stop").events.click(); await f.flush();
  assert(f.count("end") === 1 && f.requests.filter(r => r.url === "/api/command").length === 1, "W3 STOP cancels manual");
  results.push("second pointer/direction ignored; actual W3 STOP callback cancels manual before POST");
  f = await fixture(); await f.press(); f.node("manual-halt").events.click(); f.node("manual-halt").events.click(); await f.flush();
  assert(f.count("halt") === 1 && f.count("end") === 0, "explicit halt coalesces double click");
  results.push("DETENER button issues one independent halt without duplicate owner end");
  f = await fixture(); f.mode("late-open"); await f.press(); await f.release();
  f.mode("ok"); await f.press("reverse"); const renewCount = f.count("renew"); f.resolveOpen(); await f.flush();
  assert(f.count("renew") === renewCount && f.count("end") === 1 && f.node("manual-session").textContent.includes("activa"), "old open cannot replace newer gesture");
  results.push("late older session response cannot replace a newer press or resume old direction");
  const latencyResults = [];
  for (const latency of [0, 50, 120, 180, 230, 260]) {
    f = await fixture({ latency }); await f.press(); await f.advance(2000);
    const renewals = f.requests.filter(r => r.url.endsWith("/renew"));
    const firstEnd = f.requests.find(r => r.url.endsWith("/end"));
    const active = f.node("manual-session").textContent === "Autorización manual activa";
    latencyResults.push({ latency, active, renewals: renewals.length,
      cancelledAt: firstEnd?.now ?? null, message: f.node("manual-message").textContent });
    assert(f.maxPending() <= 1, "only one pending renewal at latency " + latency);
    assert(renewals.every((r, i) => !i || r.now > renewals[i - 1].now), "no catch-up burst");
    const count = f.count("renew"); await f.release(); await f.advance(1000);
    assert(f.count("renew") === count && f.count("start") === 1, "release/failure never reopens");
  }
  report(latencyResults);
  for (const row of latencyResults) {
    assert(row.active === (row.latency < 250), "two-second hold at latency " + row.latency + " ms: " + JSON.stringify(row));
    results.push("two-second hold, single pending renewal and no retry at " + row.latency + " ms HTTP latency");
  }
  assert(latencyResults.at(-1).cancelledAt === 250 && latencyResults.at(-1).message.includes("Timeout de red"), "HTTP timeout distinguished from lease expiry");
  for (const latency of [249, 250]) {
    f = await fixture({ latency }); await f.press(); await f.advance(2000);
    assert((f.count("end") === 0) === (latency < 250), "HTTP deadline exact boundary " + latency);
    assert(f.maxPending() === 1, "one pending at timeout boundary");
  }
  results.push("HTTP succeeds at 249 ms and cancels at exactly 250 ms, without increasing timeout");
  for (const code of ["busy", "unsafe", "expired"]) {
    f = await fixture(); f.mode(code); await f.press(); await f.advance(2000);
    const reason = { busy: "Sesión ocupada", unsafe: "Estado del robot inseguro", expired: "Autorización vencida" }[code];
    assert(f.count("renew") === 0 && f.count("start") === 1 && f.node("manual-message").textContent.includes(reason), "explicit ESP32 rejection " + code);
  }
  f = await fixture(); await f.press(); f.mode("rejected"); await f.advance(200);
  assert(f.count("end") === 1 && f.node("manual-message").textContent.includes("Autorización vencida"), "renewal rejection differs from HTTP timeout");
  results.push("ESP32 busy, unsafe and expired rejections have distinct messages and never retry");
  f = await fixture(); await f.press(); f.stall(350); await f.advance(0);
  assert(f.count("renew") === 1 && f.count("end") === 1 && f.node("manual-message").textContent.includes("Autorización vencida"), "late timer cannot revive expired authorization");
  results.push("delayed browser timers past lease cannot send a catch-up direction or reactivate session");
  f = await fixture({ latency: 180 }); await f.press(); await f.advance(200); await f.release();
  await f.advance(1000); assert(f.count("renew") === 2 && f.count("end") === 1, "pending response after release never schedules another direction");
  results.push("release while delayed renewal is pending prevents all subsequent direction emissions");
  f = await fixture(); await f.press();
  for (const latency of [120, 230, 50, 180]) { f.latency(latency); await f.advance(400); }
  assert(f.count("end") === 0 && f.maxPending() === 1, "variable RTT stays single-flight without premature expiry");
  results.push("variable HTTP latency remains single-flight with no accumulated renewals");
  for (const latency of [0, 50, 120, 180, 230, 260]) {
    f = await fixture({ latency, openLatency: latency }); await f.press(); await f.advance(2000);
    assert((f.node("manual-session").textContent === "Autorización manual activa") === (latency < 250), "latency also on session opening " + latency);
    if (latency > 250) assert(f.count("renew") === 0 && f.node("manual-message").textContent.includes("Timeout de red"), "slow opening never moves");
    assert(f.maxPending() <= 1, "single-flight after delayed opening");
  }
  results.push("same latency matrix on both opening and renewals; slow opening never emits a direction");
  f = await fixture(); await f.press();
  f.node("manual-panel").open = false;
  f.node("manual-panel").events.toggle(); await f.flush(); await f.advance(1000);
  assert(f.count("end") === 1 && f.count("renew") === 1, "collapse cancels the captured gesture and future renewals");
  f.node("manual-panel").open = true;
  f.node("manual-panel").events.toggle(); await f.advance(500);
  assert(f.count("start") === 1 && f.count("renew") === 1, "reopening panel cannot resume movement");
  results.push("closing the secondary manual panel stops renewal; reopening never restarts movement");
  return results;
}
if (typeof module !== "undefined" && require.main === module) {
  const fs = require("node:fs"), path = require("node:path");
  runManualTests(fs.readFileSync(path.join(__dirname, "../../web/app.js"), "utf8"), fs.readFileSync(path.join(__dirname, "../../web/manual.js"), "utf8"))
    .then(results => { results.forEach((name, i) => console.log(`PASS ${i + 1} ${name}`)); console.log(`RESULT ${results.length} manual JavaScript groups passed`); })
    .catch(error => { console.error(error); process.exitCode = 1; });
}
