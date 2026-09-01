const canvas = document.querySelector("#routeCanvas");
const ctx = canvas.getContext("2d");

const ui = {
  start: document.querySelector("#startButton"),
  pause: document.querySelector("#pauseButton"),
  stop: document.querySelector("#stopButton"),
  reset: document.querySelector("#resetButton"),
  state: document.querySelector("#missionState"),
  heading: document.querySelector("#headingValue"),
  target: document.querySelector("#targetValue"),
  error: document.querySelector("#errorValue"),
  distance: document.querySelector("#distanceValue"),
  waypoint: document.querySelector("#waypointValue"),
  mode: document.querySelector("#modeValue"),
  log: document.querySelector("#telemetryLog"),
  speed: document.querySelector("#speedInput"),
  speedOutput: document.querySelector("#speedOutput"),
  turn: document.querySelector("#turnInput"),
  turnOutput: document.querySelector("#turnOutput"),
  radius: document.querySelector("#radiusInput"),
  radiusOutput: document.querySelector("#radiusOutput")
};

const stages = {
  sensors: { eyebrow: "Input stage", title: "Read sensors", body: "TinyGPSPlus decodes incoming position fixes while the QMC5883P provides magnetic heading. Bluetooth is checked for an emergency stop command every loop.", code: "while (GPS_Serial.available()) gps.encode(GPS_Serial.read());" },
  smooth: { eyebrow: "Filter stage", title: "Smooth the GPS position", body: "An exponential moving average gives 30% weight to the newest fix. This reduces GPS jitter before it can create rapid steering changes.", code: "smoothLat += GPS_ALPHA * (rawLat - smoothLat);" },
  calculate: { eyebrow: "Navigation stage", title: "Calculate distance and bearing", body: "The Haversine formula measures great-circle distance to the active waypoint. An initial-bearing calculation produces the target heading from the current position.", code: "bearingToTarget = calculateBearing(position, target);" },
  steer: { eyebrow: "Control stage", title: "Choose turn or proportional steering", body: "A large heading error turns the robot in place. Once aligned, proportional correction changes left and right motor power while the robot drives forward.", code: "correction = headingError * Kp;" },
  advance: { eyebrow: "Mission stage", title: "Advance to the next waypoint", body: "Inside the arrival radius, both motors stop for two seconds. The index advances, or navigation is disabled after the final waypoint.", code: "if (distToTarget < 5.0) currentWaypoint++;" }
};

const route = [
  { x: 0.13, y: 0.76, name: "START" },
  { x: 0.36, y: 0.32, name: "WP 2" },
  { x: 0.68, y: 0.47, name: "WP 3" },
  { x: 0.84, y: 0.19, name: "FINISH" }
];

const sim = {
  running: false, paused: false, stopped: false, complete: false, waypoint: 1,
  x: route[0].x, y: route[0].y, heading: -18, targetHeading: 0, error: 0,
  distance: 0, mode: "IDLE", lastTime: 0, trail: [{ x: route[0].x, y: route[0].y }], logTick: 0
};

function shortestRotation(target, current) {
  let diff = target - current;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

function normalizedHeading(value) { return ((value % 360) + 360) % 360; }

function addLog(label, message) {
  const line = document.createElement("p");
  const prefix = document.createElement("span");
  prefix.textContent = label;
  line.append(prefix, ` ${message}`);
  ui.log.prepend(line);
  while (ui.log.children.length > 8) ui.log.lastElementChild.remove();
}

function setControls() {
  ui.start.disabled = sim.running && !sim.paused;
  ui.start.textContent = sim.paused ? "Resume mission" : sim.complete || sim.stopped ? "Start again" : "Start mission";
  ui.pause.disabled = !sim.running || sim.paused || sim.complete || sim.stopped;
  ui.stop.disabled = !sim.running || sim.complete || sim.stopped;
}

function resetSimulation(clearLog = true) {
  Object.assign(sim, {
    running: false, paused: false, stopped: false, complete: false, waypoint: 1,
    x: route[0].x, y: route[0].y, heading: -18, targetHeading: 0, error: 0,
    distance: 0, mode: "IDLE", lastTime: 0, trail: [{ x: route[0].x, y: route[0].y }], logTick: 0
  });
  if (clearLog) ui.log.innerHTML = "<p><span>SYS</span> Mission reset. Awaiting start.</p>";
  ui.state.textContent = "Ready";
  setControls();
  updateTelemetry();
  drawMap();
}

function startMission() {
  if (sim.complete || sim.stopped) resetSimulation(false);
  sim.running = true;
  sim.paused = false;
  sim.lastTime = performance.now();
  ui.state.textContent = "Navigating";
  addLog("NAV", `Heading to waypoint ${sim.waypoint + 1}.`);
  setControls();
}

function pauseMission() {
  sim.paused = true;
  sim.running = false;
  sim.mode = "PAUSED";
  ui.state.textContent = "Paused";
  addLog("SYS", "Mission paused by operator.");
  setControls();
  updateTelemetry();
}

function emergencyStop() {
  sim.stopped = true;
  sim.running = false;
  sim.mode = "STOPPED";
  ui.state.textContent = "Emergency stop";
  addLog("STOP", "Bluetooth S command received. Motors disabled.");
  setControls();
  updateTelemetry();
}

function updateSimulation(delta) {
  if (!sim.running) return;
  const target = route[sim.waypoint];
  const dx = target.x - sim.x;
  const dy = target.y - sim.y;
  sim.distance = Math.hypot(dx * canvas.width, dy * canvas.height) * 0.12;
  sim.targetHeading = normalizedHeading(Math.atan2(dx, -dy) * 180 / Math.PI);
  sim.error = shortestRotation(sim.targetHeading, normalizedHeading(sim.heading));

  if (Math.abs(sim.error) > Number(ui.turn.value)) {
    sim.mode = "TURN";
    sim.heading += Math.sign(sim.error) * Math.min(Math.abs(sim.error), delta * 96);
  } else {
    sim.mode = "DRIVE";
    sim.heading += sim.error * Math.min(1, delta * 3.2);
    const step = delta * (Number(ui.speed.value) / 100) * 0.095;
    const radians = sim.heading * Math.PI / 180;
    sim.x += Math.sin(radians) * step;
    sim.y -= Math.cos(radians) * step;
    const lastTrail = sim.trail[sim.trail.length - 1];
    if (Math.hypot(sim.x - lastTrail.x, sim.y - lastTrail.y) > .008) sim.trail.push({ x: sim.x, y: sim.y });
  }

  if (sim.distance <= Number(ui.radius.value)) {
    addLog("WP", `Waypoint ${sim.waypoint + 1} reached at ${sim.distance.toFixed(1)} m.`);
    sim.x = target.x;
    sim.y = target.y;
    sim.trail.push({ x: sim.x, y: sim.y });
    if (sim.waypoint === route.length - 1) {
      sim.complete = true;
      sim.running = false;
      sim.mode = "DONE";
      sim.distance = 0;
      ui.state.textContent = "Mission complete";
      addLog("SYS", "Final waypoint reached. Motors stopped.");
      setControls();
    } else {
      sim.waypoint += 1;
      addLog("NAV", `Advancing to waypoint ${sim.waypoint + 1}.`);
    }
  }

  sim.logTick += delta;
  if (sim.logTick > 1.35 && sim.running) {
    addLog("TEL", `Hdg:${String(Math.round(normalizedHeading(sim.heading))).padStart(3, "0")} Tgt:${String(Math.round(sim.targetHeading)).padStart(3, "0")} Err:${Math.round(sim.error)} Dist:${sim.distance.toFixed(1)}`);
    sim.logTick = 0;
  }
}

function updateTelemetry() {
  ui.heading.textContent = String(Math.round(normalizedHeading(sim.heading))).padStart(3, "0");
  ui.target.textContent = String(Math.round(sim.targetHeading)).padStart(3, "0");
  ui.error.textContent = `${sim.error >= 0 ? "+" : ""}${Math.round(sim.error)}`;
  ui.distance.textContent = sim.distance.toFixed(1);
  ui.waypoint.textContent = `${Math.min(sim.waypoint + 1, route.length)}/${route.length}`;
  ui.mode.textContent = sim.mode;
  ui.mode.style.color = sim.mode === "STOPPED" ? "#ff6b6b" : sim.mode === "TURN" ? "#ff9f43" : "#a9f28c";
}

function drawGrid(width, height) {
  ctx.strokeStyle = "rgba(125, 155, 137, .09)";
  ctx.lineWidth = 1;
  for (let x = 0; x < width; x += 42) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke(); }
  for (let y = 0; y < height; y += 42) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }
}

function drawMap() {
  const { width, height } = canvas;
  ctx.clearRect(0, 0, width, height);
  drawGrid(width, height);
  ctx.strokeStyle = "#52675a";
  ctx.lineWidth = 3;
  ctx.setLineDash([10, 12]);
  ctx.beginPath();
  route.forEach((point, index) => index ? ctx.lineTo(point.x * width, point.y * height) : ctx.moveTo(point.x * width, point.y * height));
  ctx.stroke();
  ctx.setLineDash([]);

  if (sim.trail.length > 1) {
    ctx.strokeStyle = "#54d66f";
    ctx.lineWidth = 4;
    ctx.beginPath();
    sim.trail.forEach((point, index) => index ? ctx.lineTo(point.x * width, point.y * height) : ctx.moveTo(point.x * width, point.y * height));
    ctx.stroke();
  }

  route.forEach((point, index) => {
    const x = point.x * width;
    const y = point.y * height;
    const active = index === sim.waypoint && !sim.complete;
    const reached = index < sim.waypoint || sim.complete;
    ctx.beginPath();
    ctx.arc(x, y, active ? 13 : 10, 0, Math.PI * 2);
    ctx.fillStyle = active ? "#ff9f43" : reached ? "#54d66f" : "#34473c";
    ctx.fill();
    ctx.lineWidth = 4;
    ctx.strokeStyle = "#0c1510";
    ctx.stroke();
    ctx.fillStyle = active ? "#ffc887" : "#91a398";
    ctx.font = "700 13px ui-monospace, monospace";
    ctx.fillText(point.name, x + 17, y - 13);
  });

  ctx.save();
  ctx.translate(sim.x * width, sim.y * height);
  ctx.rotate(sim.heading * Math.PI / 180);
  ctx.fillStyle = sim.stopped ? "#ff6b6b" : "#eaf4ee";
  ctx.strokeStyle = "#0c1510";
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(0, -18);
  ctx.lineTo(13, 13);
  ctx.lineTo(0, 8);
  ctx.lineTo(-13, 13);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function animate(time) {
  const delta = Math.min((time - (sim.lastTime || time)) / 1000, .04);
  sim.lastTime = time;
  updateSimulation(delta);
  updateTelemetry();
  drawMap();
  requestAnimationFrame(animate);
}

ui.start.addEventListener("click", startMission);
ui.pause.addEventListener("click", pauseMission);
ui.stop.addEventListener("click", emergencyStop);
ui.reset.addEventListener("click", () => resetSimulation());
ui.speed.addEventListener("input", () => { ui.speedOutput.textContent = ui.speed.value; });
ui.turn.addEventListener("input", () => { ui.turnOutput.textContent = `${ui.turn.value}°`; });
ui.radius.addEventListener("input", () => { ui.radiusOutput.textContent = `${Number(ui.radius.value).toFixed(1)} m`; });

document.querySelectorAll(".stage").forEach(button => {
  button.addEventListener("click", () => {
    document.querySelectorAll(".stage").forEach(stage => stage.classList.toggle("active", stage === button));
    const stage = stages[button.dataset.stage];
    document.querySelector("#detailEyebrow").textContent = stage.eyebrow;
    document.querySelector("#detailTitle").textContent = stage.title;
    document.querySelector("#detailBody").textContent = stage.body;
    document.querySelector("#detailCode").textContent = stage.code;
  });
});

resetSimulation(false);
requestAnimationFrame(animate);
