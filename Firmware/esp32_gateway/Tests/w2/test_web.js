/* Pruebas del app.js real con DOM/fetch/reloj sustituidos. Sin dependencias. */
async function runWebTests(source) {
  function assert(condition, message) { if (!condition) throw new Error(message); }
  const results = [];
  const pass = message => results.push(message);
  let writes = 0, now = 0, nextTimer = 1, mode = "ok", healthMode = "ok";
  let payload = { valid: false, connected: false, state: null, faults: null, stop: null, age_ms: null };
  const timers = new Map(), nodes = new Map(), requests = [];
  function node(id) {
    if (!nodes.has(id)) {
      let text = "";
      nodes.set(id, {
        dataset: {}, disabled: false,
        get textContent() { return text; },
        set textContent(value) { text = value; writes++; },
        addEventListener(event, callback) { assert(event === "click", "click listener"); this.click = callback; }
      });
    }
    return nodes.get(id);
  }
  class Controller {
    constructor() { this.signal = {}; }
    abort() { if (this.signal.reject) this.signal.reject(new Error("timeout")); }
  }
  async function fetchMock(url, options) {
    requests.push(url);
    assert(["/health", "/api/status"].includes(url), "only read-only local routes");
    assert(options.cache === "no-store" && options.signal && !options.body && !options.method, "GET without command/body");
    const currentMode = url === "/health" ? healthMode : mode;
    if (currentMode === "pending") return new Promise((resolve, reject) => { options.signal.reject = reject; });
    if (currentMode === "network") throw new Error("offline");
    return { ok: currentMode !== "http", json: async () => {
      if (currentMode === "json") throw new Error("JSON");
      return url === "/health" ? { status: "ok", service: "esp32-web" } : payload;
    } };
  }
  const create = new Function("document", "fetch", "AbortController", "setTimeout", "clearTimeout", "performance",
    source + "\nreturn { validStatus, describeFaults }; ");
  const api = create({ getElementById: node }, fetchMock, Controller,
    (fn, delay) => { const id = nextTimer++; timers.set(id, { fn, delay }); return id; },
    id => timers.delete(id), { now: () => now });
  async function flush() { for (let n = 0; n < 12; n++) await Promise.resolve(); }
  function fire(delay) {
    const matches = [...timers].filter(([, timer]) => timer.delay === delay);
    assert(matches.length === 1, `exactly one timer ${delay}, found ${matches.length}`);
    const [id, timer] = matches[0]; timers.delete(id); return timer.fn();
  }
  async function poll(value, nextMode = "ok") { payload = value; mode = nextMode; fire(1000); await flush(); }
  await flush();
  assert(node("server-status").dataset.state === "ok", "W1 health success");
  assert(node("stm32-connection").textContent === "Sin datos desde el arranque" && node("robot-state").textContent === "Desconocido", "unknown boot state");
  assert(requests.length === 2 && timers.size === 1, "initial one health and one status request");
  pass("initial health isolated from unknown STM32; null data not rendered as safe");
  const names = ["ROBOT_STATE_IDLE", "ROBOT_STATE_AUTONOMOUS", "ROBOT_STATE_MANUAL", "ROBOT_STATE_SAFETY_STOP"];
  for (let state = 0; state < 4; state++) {
    await poll({ valid: true, connected: true, state, faults: state, stop: state % 2, age_ms: 100 });
    assert(node("robot-state").textContent.includes(names[state]), "real FSM label");
    assert(node("robot-values").dataset.stale === "false", "fresh values");
  }
  pass("all four firmware FSM names and recent communication");
  for (const [bit, label] of [[1, "CLIFF"], [2, "OBSTACLE"], [4, "SENSOR"], [8, "EMERGENCY"]]) {
    assert(api.describeFaults(bit).includes("SAFETY_FAULT_" + label), "firmware fault label");
  }
  assert(api.describeFaults(15).includes("CLIFF") && api.describeFaults(15).includes("EMERGENCY"), "combined bits");
  assert(api.describeFaults(0).includes("Sin fallas informadas"), "zero mask");
  assert(api.describeFaults(2147483664).includes("2147483664"), "unknown bits not signed or dropped");
  pass("all four real Safety bits, combined masks, zero and unknown unsigned bits");
  const safe = { valid: true, connected: true, state: 2, faults: 0, stop: 0, age_ms: 0 };
  await poll(safe); assert(node("robot-stop").textContent === "0 · Sin bloqueo informado", "STOP zero");
  await poll({ ...safe, stop: 1 }); assert(node("robot-stop").textContent === "1 · Bloqueo activo", "STOP one");
  pass("STOP active/inactive rendered without issuing commands");
  await poll({ ...safe, connected: false, faults: 4, age_ms: 3000 });
  assert(node("robot-values").dataset.stale === "true" && node("stm32-connection").textContent === "Comunicación desactualizada", "stale visible");
  assert(node("robot-faults").textContent.includes("SENSOR") && node("robot-age").textContent === "3000 ms", "last fault retained");
  pass("stale data explicitly historical and last faults retained");
  await poll(safe); const previousWrites = writes; await poll(safe);
  assert(writes === previousWrites, "unchanged text not rewritten");
  pass("unchanged values avoid redundant text updates");
  now = 600; await poll(safe, "network");
  assert(node("robot-values").dataset.stale === "true" && node("robot-age").textContent.includes("600 ms"), "network error ages cached values");
  assert(node("stm32-connection").textContent === "Monitoreo sin respuesta", "web failure not mistaken for fresh STM32");
  pass("HTTP/network loss never presents last values as current");
  for (const failure of ["http", "json"]) {
    await poll(safe, failure); assert(node("stm32-connection").textContent === "Monitoreo sin respuesta", failure);
  }
  const malformed = [null, {}, { ...safe, state: 4 }, { ...safe, faults: -1 }, { ...safe, faults: 4294967296 },
    { ...safe, stop: true }, { ...safe, age_ms: -1 }, { ...safe, connected: "true" }, { valid: false, connected: true }];
  for (const value of malformed) {
    assert(!api.validStatus(value), "invalid schema"); await poll(value);
    assert(node("stm32-connection").textContent === "Monitoreo sin respuesta", "invalid data not rendered");
  }
  pass("HTTP, JSON and nine malformed schema cases rejected without replacing display");
  mode = "pending"; fire(1000); await flush();
  assert(![...timers.values()].some(t => t.delay === 1000), "no overlapping polling during pending fetch");
  const prior = requests.length; fire(2500); await flush();
  assert(requests.length === prior && timers.size === 1, "timeout does not create request burst");
  await poll(safe); assert(node("robot-values").dataset.stale === "false", "recovery");
  pass("2.5s timeout; one pending query; one delayed retry; recovery without catch-up");
  const unknown = { valid: false, connected: false, state: null, faults: null, stop: null, age_ms: null };
  await poll(unknown); assert(node("robot-faults").textContent === "Desconocidas" && node("robot-age").textContent === "Sin datos", "ESP32 restart clears old display");
  pass("new ESP32 boot without STATUS clears previous cached display");
  for (const failure of ["network", "http", "json"]) {
    healthMode = failure; await node("check-health").click();
    assert(node("server-status").dataset.state === "error" && !node("check-health").disabled, "W1 health error");
  }
  healthMode = "ok"; await node("check-health").click();
  assert(node("server-status").dataset.state === "ok", "W1 health retry");
  assert(requests.filter(url => url === "/health").length === 5, "no periodic health requests");
  pass("W1 health button and error recovery preserved; no UART/operational endpoint");
  return results;
}

if (typeof module !== "undefined" && require.main === module) {
  const fs = require("node:fs"), path = require("node:path");
  runWebTests(fs.readFileSync(path.join(__dirname, "../../web/app.js"), "utf8"))
    .then(results => { results.forEach((name, i) => console.log(`PASS ${i + 1} ${name}`)); console.log(`RESULT ${results.length} JavaScript groups passed`); })
    .catch(error => { console.error(error); process.exitCode = 1; });
}
