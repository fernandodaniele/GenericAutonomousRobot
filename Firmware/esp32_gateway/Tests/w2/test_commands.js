/* Arnes sin dependencias para los controles W3 del app.js real. */
async function runCommandTests(source) {
  const assert = (ok, message) => { if (!ok) throw new Error(message); };
  const results = [], timers = new Map(), nodes = new Map(), requests = [];
  let nextTimer = 1, nextId = 1, now = 0, mode = "ok", pendingResolve, pendingReject;
  let currentId = "", currentName = "";
  let statusFailure = false;
  const snapshot = { valid: true, connected: true, state: 3, faults: 4, stop: 1, age_ms: 0 };
  function node(id) {
    if (!nodes.has(id)) nodes.set(id, { dataset: {}, textContent: "", disabled: false,
      addEventListener(event, callback) { assert(event === "click", "click only"); this.click = callback; } });
    return nodes.get(id);
  }
  class Controller {
    constructor() { this.signal = {}; }
    abort() { if (this.signal.reject) this.signal.reject(new Error("timeout")); }
  }
  const cryptoMock = { getRandomValues(array) { array.fill(nextId++); return array; } };
  async function fetchMock(url, options) {
    requests.push({ url, options });
    if (url === "/health") return { ok: true, json: async () => ({ status: "ok", service: "esp32-web" }) };
    if (url === "/api/status") {
      if (statusFailure) throw new Error("offline");
      return { ok: true, json: async () => snapshot };
    }
    assert(url === "/api/command" && options.method === "POST", "only command POST");
    assert(["STOP", "RESET", "MANUAL", "AUTO"].includes(options.body), "closed body list");
    currentId = options.headers["X-Request-ID"]; currentName = options.body;
    assert(/^[a-f0-9]{32}$/.test(currentId) && options.headers["Content-Type"] === "text/plain", "ID/body headers");
    if (mode === "pending") return new Promise((resolve, reject) => {
      pendingResolve = resolve; pendingReject = reject; options.signal.reject = reject;
    });
    if (mode === "network") throw new Error("offline");
    return { ok: mode !== "rejected", json: async () => mode === "malformed" ? {} : mode === "rejected" ?
      { accepted: false, error: "command_busy" } :
      { accepted: true, confirmed: false, id: currentId, command: currentName, phase: "queued" } };
  }
  const create = new Function("document", "fetch", "AbortController", "setTimeout", "clearTimeout", "performance", "crypto",
    source + "\nreturn {sendCommand, observeCommand, renderStatus, pollStatus};");
  const api = create({ getElementById: node }, fetchMock, Controller,
    (fn, delay) => { const id = nextTimer++; timers.set(id, { fn, delay }); return id; },
    id => timers.delete(id), { now: () => now }, cryptoMock);
  function click(name) { now += 1001; return node("command-" + name).click(); }
  async function flush() { for (let i = 0; i < 12; i++) await Promise.resolve(); }
  function fire(delay) {
    const found = [...timers].filter(([, t]) => t.delay === delay);
    assert(found.length === 1, `one timer ${delay}`); const [id, timer] = found[0]; timers.delete(id); return timer.fn();
  }
  const commandCount = () => requests.filter(r => r.url === "/api/command").length;
  function observe(phase, extra = {}, id = currentId, name = currentName) {
    api.observeCommand({ ...snapshot, ...extra, command: { id, name, phase, status_after_send: true, ...(extra.command || {}) } });
  }
  await flush();
  assert(commandCount() === 0 && !node("command-stop").disabled && !node("command-reset").disabled, "no startup command");
  await api.sendCommand("AUTO"); await api.sendCommand("AVANZAR"); assert(commandCount() === 0, "unsupported UI call rejected");
  results.push("no command on load; AUTO rejected with unsafe STATUS; movement and arbitrary UI strings rejected");
  await click("reset"); const resetId = currentId;
  assert(commandCount() === 1 && node("command-message").textContent.includes("aceptada por la API"), "API accepted only");
  await node("command-reset").click(); await node("command-manual").click(); assert(commandCount() === 1, "double-click gated");
  assert(node("command-reset").disabled && !node("command-stop").disabled, "STOP remains available");
  results.push("repeated clicks/mode requests gated while waiting; STOP available");
  observe("transmitted", { command: { status_after_send: false } });
  assert(node("command-message").textContent.includes("trama entregada a UART") && node("command-reset").disabled, "TX is not reset success");
  observe("transmitted"); assert(node("command-reset").disabled, "blocked Safety does not confirm RESET");
  observe("transmitted", { state: 0, faults: 0, stop: 0, connected: false }); assert(node("command-reset").disabled, "stale STATUS not confirmation");
  results.push("UART delivery, Safety still blocked and stale response never declared reset success");
  await click("stop"); assert(commandCount() === 2 && currentId !== resetId && node("command-stop").disabled, "explicit new STOP");
  await node("command-stop").click(); assert(commandCount() === 2, "duplicate STOP click blocked");
  observe("transmitted", { state: 0, faults: 0, stop: 0 }, resetId, "RESET"); assert(node("command-stop").disabled, "old receipt cannot finish STOP");
  observe("transmitted"); assert(!node("command-stop").disabled && node("command-message").textContent.includes("Detenido por seguridad"), "STOP observed latched stop");
  await node("command-stop").click(); assert(commandCount() === 2, "rapid second click after early state observation still ignored");
  results.push("STOP supersedes UI action, ignores old receipts and accepts observed Safety-stop state without clearing it");
  await click("manual"); observe("transmitted", { state: 2, faults: 0, stop: 0 });
  assert(node("command-message").textContent.includes("Modo manual") && !node("command-manual").disabled, "manual observed");
  await click("reset"); observe("transmitted", { state: 0, faults: 0, stop: 0 });
  assert(node("command-message").textContent.includes("En espera"), "reset observed state");
  results.push("friendly posterior state observations for MANUAL/RESET; no movement requests");
  for (const phase of ["failed", "cancelled"]) {
    await click("reset"); const count = commandCount(); observe(phase);
    assert(!node("command-reset").disabled && commandCount() === count && node("command-message").textContent.includes("reintentará"), phase);
  }
  results.push("failed/cancelled transmissions unlock controls without retry");
  await click("reset"); const beforeTimeout = commandCount(); fire(6000);
  assert(commandCount() === beforeTimeout && node("command-message").textContent.includes("No fue posible confirmar"), "missing STM32 response");
  results.push("six-second observation deadline reports unconfirmed, without resend");
  for (mode of ["rejected", "network", "malformed"]) {
    const count = commandCount(); await click("reset");
    assert(commandCount() === count + 1 && !node("command-reset").disabled, "one attempt only " + mode);
    assert(!node("command-message").textContent.includes("Estado posterior observado"), "no false success");
  }
  results.push("HTTP rejection, network loss and malformed acceptance remain rejected/uncertain, never retried");
  mode = "pending"; const request = click("reset"); await flush(); const beforeAbort = commandCount();
  fire(3500); await request; assert(commandCount() === beforeAbort && !node("command-reset").disabled, "POST abort no retry");
  results.push("3.5-second POST timeout preserves uncertainty about execution");
  mode = "pending"; const older = click("reset"); await flush(); const oldResolve = pendingResolve;
  const oldId = currentId; mode = "ok"; await click("stop"); const stopId = currentId;
  oldResolve({ ok: true, json: async () => ({ accepted: true, confirmed: false, id: oldId, command: "RESET", phase: "queued" }) });
  await older; assert(node("command-stop").disabled && currentId === stopId, "old POST completion cannot unlock newer STOP");
  observe("transmitted"); results.push("late older POST response cannot replace or unlock STOP tracking");
  const beforePoll = commandCount(); fire(1000); await flush();
  assert(commandCount() === beforePoll && requests.filter(r => r.url === "/api/status").length === 2, "W2 polling independent");
  assert(typeof pendingReject === "function", "timeout exercise");
  results.push("periodic STATUS remains read-only and independent of command execution");
  for (const unsafe of [
    { valid: false, connected: false, state: null, faults: null, stop: null, age_ms: null },
    { state: 2, faults: 0, stop: 0 }, { state: 1, faults: 0, stop: 0 },
    { state: 0, faults: 1, stop: 0 }, { state: 0, faults: 0, stop: 1 },
    { state: 0, faults: 0, stop: 0, connected: false },
    { state: 0, faults: 0, stop: 0, age_ms: 2000 }
  ]) {
    api.renderStatus({ ...snapshot, ...unsafe });
    const before = commandCount(); await api.sendCommand("AUTO");
    assert(node("command-auto").disabled && commandCount() === before && !node("command-stop").disabled, "AUTO unsafe state gated");
  }
  results.push("W6 AUTO disabled for unknown, MANUAL, AUTO, fault, STOP, disconnected or stale STATUS; STOP stays available");
  const safeIdle = { valid: true, connected: true, state: 0, faults: 0, stop: 0, age_ms: 0 };
  mode = "ok"; api.renderStatus(safeIdle);
  assert(!node("command-auto").disabled, "safe IDLE allows explicit AUTO");
  const beforeAuto = commandCount(); await node("command-auto").click();
  assert(commandCount() === beforeAuto + 1 && currentName === "AUTO", "one explicit AUTO POST");
  await node("command-auto").click(); assert(commandCount() === beforeAuto + 1, "no double AUTO");
  observe("transmitted", { ...safeIdle, state: 1, command: { status_after_send: false } });
  assert(node("command-auto").disabled && !node("command-stop").disabled, "TX alone not confirmation");
  observe("transmitted", safeIdle); assert(node("command-auto").disabled, "IDLE does not confirm AUTO");
  api.renderStatus({ ...safeIdle, state: 1 }); observe("transmitted", { ...safeIdle, state: 1 });
  assert(node("command-message").textContent.includes("Modo autónomo") && node("command-auto").disabled, "later actual AUTO observed");
  results.push("W6 explicit AUTO accepted once and confirmed only by a recent posterior AUTO STATUS");
  api.renderStatus(safeIdle); now += 2000;
  const beforeAged = commandCount(); await api.sendCommand("AUTO");
  assert(commandCount() === beforeAged && node("command-auto").disabled, "cached STATUS age continues advancing");
  results.push("W6 browser elapsed time cannot make old IDLE actionable");
  now += 1001; api.renderStatus(safeIdle); mode = "rejected";
  await node("command-auto").click();
  assert(!node("command-message").textContent.includes("Estado posterior observado"), "AUTO rejection not success");
  const rejectedCount = commandCount(); now += 1001;
  api.renderStatus({ ...safeIdle, connected: false }); await api.sendCommand("AUTO");
  api.renderStatus(safeIdle); await flush(); assert(commandCount() === rejectedCount, "recovery has no AUTO replay");
  results.push("W6 AUTO rejection and telemetry recovery do not replay requests or claim success");
  statusFailure = true; await api.pollStatus();
  const beforeLost = commandCount(); await api.sendCommand("AUTO");
  assert(node("command-auto").disabled && !node("command-stop").disabled && commandCount() === beforeLost, "HTTP loss disables cached AUTO");
  results.push("W6 HTTP monitoring failure disables AUTO even if the cached IDLE was safe");
  now += 1001; api.renderStatus(safeIdle); mode = "pending";
  const lateAuto = node("command-auto").click(); await flush();
  const autoResolve = pendingResolve, lateId = currentId;
  mode = "ok"; await click("stop"); const newerStop = currentId;
  autoResolve({ ok: true, json: async () => ({ accepted: true, confirmed: false, id: lateId, command: "AUTO", phase: "queued" }) });
  await lateAuto;
  assert(currentId === newerStop && node("command-stop").disabled, "late AUTO cannot replace STOP observation");
  observe("transmitted", { ...safeIdle });
  results.push("W6 STOP preempts pending AUTO and ignores its late HTTP completion without restarting it");
  return results;
}
if (typeof module !== "undefined" && require.main === module) {
  const fs = require("node:fs"), path = require("node:path");
  runCommandTests(fs.readFileSync(path.join(__dirname, "../../web/app.js"), "utf8"))
    .then(results => { results.forEach((name, i) => console.log(`PASS ${i + 1} ${name}`)); console.log(`RESULT ${results.length} W3 JavaScript groups passed`); })
    .catch(error => { console.error(error); process.exitCode = 1; });
}
