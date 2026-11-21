/* No packages required. Run with Node, or supply sources to the same V8 runner. */
async function runJavaScriptTests(sources) {
  const app = sources["web/app.js"], manual = sources["web/manual.js"], sensors = sources["web/sensors.js"];
  const specs = [
    ["W2", "test_web", "runWebTests", [app]],
    ["W3", "test_commands", "runCommandTests", [app]],
    ["W4", "test_manual", "runManualTests", [app, manual]],
    ["W5", "test_sensors", "runSensorTests", [sensors]],
    ["W7", "test_map", "runMapTests", [sources["web/map.js"]]],
    ["IMU-1", "test_imu", "runImuTests", [sources["web/imu.js"]]],
    ["IMU-2", "test_imu", "runImuCalibrationTests", [sources["web/imu.js"]]],
    ["IMU-3", "test_imu", "runImuOrientationTests", [sources["web/imu.js"]]],
    ["W4+W5 endpoint unavailable", "test_manual", "runManualTests", [app, manual, () => {}, sensors]],
    ["W4+W5+W7 endpoints unavailable", "test_manual", "runManualTests", [app, manual, () => {}, sensors, sources["web/map.js"]]],
    ["W4+W5+W7+IMU endpoints unavailable", "test_manual", "runManualTests", [app, manual, () => {}, sensors + "\n" + sources["web/imu.js"], sources["web/map.js"]]]
  ];
  const results = [];
  for (const [suite, file, entry, args] of specs) {
    const fn = new Function("module", sources["Tests/w2/" + file + ".js"] + "\nreturn " + entry + ";")(undefined);
    const cases = await fn(...args); results.push({ suite, passed: cases.length, cases });
  }
  return results;
}
if (typeof module !== "undefined" && require.main === module) {
  const fs = require("node:fs"), path = require("node:path"), root = path.resolve(__dirname, "../..");
  const sources = {};
  for (const file of ["web/app.js", "web/manual.js", "web/sensors.js", "web/map.js", "web/imu.js", "Tests/w2/test_web.js", "Tests/w2/test_commands.js", "Tests/w2/test_manual.js", "Tests/w2/test_sensors.js", "Tests/w2/test_map.js", "Tests/w2/test_imu.js"])
    sources[file] = fs.readFileSync(path.join(root, file), "utf8");
  runJavaScriptTests(sources).then(results => {
    results.forEach(s => console.log(`PASS ${s.suite}: ${s.passed} groups`));
    console.log(`RESULT ${results.reduce((sum, s) => sum + s.passed, 0)} JavaScript groups passed; no hardware validation`);
  }).catch(error => { console.error(error); process.exitCode = 1; });
}
