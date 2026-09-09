/* AMR Fleet Coordination — dashboard client.
   Polls /api/state, draws the warehouse on a canvas, and posts control
   actions. No frameworks, no build step. */

const $ = (id) => document.getElementById(id);

const COLORS = {
  floor: "#0d1320", floorAlt: "#101728", rack: "#38455f", rackTop: "#4c5b78",
  wall: "#5b6784", grid: "#161e2e", corridor: "#1d2c52", corridorEdge: "#2f4478",
  path: "rgba(76,194,255,.32)", res: "rgba(167,139,250,.20)",
  block: "#ff6b6b", obstacle: "#7a3540",
  pickup: "#ffb020", dropoff: "#a78bfa", text: "#e7edf8", dim: "#8b96ad",
};
const ROBOT_HUES = [200, 152, 42, 280, 12, 96, 320, 176, 250, 68, 0, 220];
const robotColor = (id, l = 58) => `hsl(${ROBOT_HUES[(id - 1) % ROBOT_HUES.length]} 72% ${l}%)`;

const state = {
  layout: null, data: null, prev: new Map(), stamp: 0, period: 1000 / 6,
  scenarios: [], algorithms: {}, cell: 22, hoverCell: null,
};

/* ------------------------------------------------------------------ api */
async function post(action, value) {
  const r = await fetch("/api/control", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action, value }),
  });
  ingest(await r.json());
}

async function poll() {
  try { ingest(await (await fetch("/api/state")).json()); }
  catch (e) { /* server restarting — try again next tick */ }
}

function ingest(d) {
  if (state.data && d.tick !== state.data.tick) {
    state.prev = new Map(state.data.robots.map((r) => [r.id, { x: r.x, y: r.y }]));
    state.stamp = performance.now();
  }
  state.data = d;
  state.period = Math.max(60, 1000 / (d.speed || 6));
  syncControls(d);
  renderRail(d);
}

/* ------------------------------------------------------------- controls */
function syncControls(d) {
  $("playBtn").textContent = d.playing ? "❚❚ Pause" : "▶ Play";
  document.querySelector(".dot").classList.toggle("paused", !d.playing);
  for (const b of document.querySelectorAll("#modeSeg button"))
    b.classList.toggle("on", b.dataset.mode === d.mode);
  const sc = state.scenarios.find((s) => s.name === d.scenario);
  $("scenarioLabel").textContent = sc
    ? `${sc.label} · tick ${d.tick} · seed ${d.seed}` : `tick ${d.tick}`;
  if ($("scenario").value !== d.scenario) $("scenario").value = d.scenario;
  if (+$("robots").value !== d.robot_count) {
    $("robots").value = d.robot_count; $("robotsOut").textContent = d.robot_count;
  }
}

function wireControls() {
  $("playBtn").onclick = () => post(state.data?.playing ? "pause" : "play");
  $("stepBtn").onclick = () => post("step");
  $("resetBtn").onclick = () => post("reset");
  $("speed").oninput = (e) => { $("speedOut").textContent = e.target.value + "×"; };
  $("speed").onchange = (e) => post("speed", +e.target.value);
  $("robots").oninput = (e) => { $("robotsOut").textContent = e.target.value; };
  $("robots").onchange = (e) => post("robots", +e.target.value);
  $("scenario").onchange = (e) => post("scenario", e.target.value);
  for (const b of document.querySelectorAll("#modeSeg button"))
    b.onclick = () => { if (b.dataset.mode !== state.data.mode) post("mode"); };
  for (const id of ["showHeat", "showPaths"]) $(id).onchange = draw;

  const cv = $("grid");
  cv.addEventListener("mousemove", (e) => {
    const c = cellAt(e); state.hoverCell = c; });
  cv.addEventListener("mouseleave", () => { state.hoverCell = null; });
  cv.addEventListener("click", (e) => {
    const c = cellAt(e); if (!c) return;
    const hit = state.data.robots.find((r) => r.x === c[0] && r.y === c[1]);
    if (hit) post(hit.state === "failed" ? "revive" : "fail", hit.id);
    else post("obstacle", c);
  });
  addEventListener("keydown", (e) => {
    if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
    if (e.code === "Space") { e.preventDefault(); $("playBtn").click(); }
    if (e.key === "s") post("step");
    if (e.key === "r") post("reset");
    if (e.key === "m") post("mode");
  });
  addEventListener("resize", resize);
}

function cellAt(e) {
  const cv = $("grid"), rect = cv.getBoundingClientRect();
  const scale = cv.width / window.devicePixelRatio / rect.width;
  const x = Math.floor((e.clientX - rect.left) * scale / state.cell);
  const y = Math.floor((e.clientY - rect.top) * scale / state.cell);
  const L = state.layout;
  return L && x >= 0 && y >= 0 && x < L.w && y < L.h ? [x, y] : null;
}

/* --------------------------------------------------------------- canvas */
function resize() {
  const L = state.layout; if (!L) return;
  const cv = $("grid"), dpr = window.devicePixelRatio || 1;
  const avail = cv.parentElement.clientWidth - 24;
  state.cell = Math.max(12, Math.min(26, Math.floor(avail / L.w)));
  const w = state.cell * L.w, h = state.cell * L.h;
  cv.width = w * dpr; cv.height = h * dpr;
  cv.style.width = w + "px"; cv.style.height = h + "px";
  cv.getContext("2d").setTransform(dpr, 0, 0, dpr, 0, 0);
  drawStatic();
}

let staticLayer = null;
function drawStatic() {
  const L = state.layout, C = state.cell;
  staticLayer = document.createElement("canvas");
  staticLayer.width = L.w * C; staticLayer.height = L.h * C;
  const g = staticLayer.getContext("2d");

  for (let y = 0; y < L.h; y++) for (let x = 0; x < L.w; x++) {
    const t = L.tiles[y][x];
    if (t === 0) { g.fillStyle = (x + y) % 2 ? COLORS.floor : COLORS.floorAlt; }
    else if (t === 1) { g.fillStyle = COLORS.rack; }
    else { g.fillStyle = COLORS.wall; }
    g.fillRect(x * C, y * C, C, C);
    if (t === 1) {                              // rack face highlight
      g.fillStyle = COLORS.rackTop;
      g.fillRect(x * C, y * C, C, Math.max(1, C * 0.14));
    }
  }
  g.strokeStyle = COLORS.grid; g.lineWidth = 1;
  for (let x = 0; x <= L.w; x++) { g.beginPath(); g.moveTo(x * C + .5, 0); g.lineTo(x * C + .5, L.h * C); g.stroke(); }
  for (let y = 0; y <= L.h; y++) { g.beginPath(); g.moveTo(0, y * C + .5); g.lineTo(L.w * C, y * C + .5); g.stroke(); }

  for (const c of L.corridors) {
    for (const [x, y] of c.cells) {
      g.fillStyle = COLORS.corridor;
      g.fillRect(x * C, y * C, C, C);
      g.strokeStyle = COLORS.corridorEdge; g.lineWidth = 1;
      g.strokeRect(x * C + .5, y * C + .5, C - 1, C - 1);
    }
  }
  for (const [x, y] of L.parking) {
    g.strokeStyle = "rgba(139,150,173,.35)"; g.lineWidth = 1;
    g.strokeRect(x * C + 3.5, y * C + 3.5, C - 7, C - 7);
  }
  const zoneTint = { inbound: "#4c8fd4", storage: "#7d879e", outbound: "#d48f4c", service: "#3ddc97" };
  g.font = "600 9px ui-sans-serif,system-ui";
  for (const z of L.zones) {
    const col = zoneTint[z.kind] || "#7d879e";
    g.strokeStyle = col + "44"; g.lineWidth = 1.5;
    roundRect(g, z.x0 * C + 2, z.y0 * C + 2, (z.x1 - z.x0 + 1) * C - 4, (z.y1 - z.y0 + 1) * C - 4, 6);
    g.stroke();
    g.fillStyle = col + "cc";
    g.fillText(z.label.toUpperCase(), z.x0 * C + 6, z.y0 * C + 13);
  }
}

function roundRect(g, x, y, w, h, r) {
  g.beginPath();
  g.moveTo(x + r, y); g.arcTo(x + w, y, x + w, y + h, r);
  g.arcTo(x + w, y + h, x, y + h, r); g.arcTo(x, y + h, x, y, r);
  g.arcTo(x, y, x + w, y, r); g.closePath();
}

function draw() {
  const d = state.data, L = state.layout;
  if (!d || !L || !staticLayer) return;
  const g = $("grid").getContext("2d"), C = state.cell;
  g.clearRect(0, 0, L.w * C, L.h * C);
  g.drawImage(staticLayer, 0, 0);

  if ($("showHeat").checked && d.heat.length) {
    const max = Math.max(...d.heat.map((h) => h[2]));
    for (const [x, y, v] of d.heat) {
      g.fillStyle = `hsla(${28 - 28 * (v / max)} 90% 55% / ${0.10 + 0.45 * (v / max)})`;
      g.fillRect(x * C, y * C, C, C);
    }
  }

  for (const c of d.corridors) {                   // chokepoint flow direction
    if (!c.direction || !c.cells.length) continue;
    const a = c.cells[0], b = c.cells[c.cells.length - 1];
    const [from, to] = c.direction > 0 ? [a, b] : [b, a];
    g.strokeStyle = "rgba(76,194,255,.55)"; g.lineWidth = 2; g.setLineDash([3, 3]);
    g.beginPath();
    g.moveTo(from[0] * C + C / 2, from[1] * C + C / 2);
    g.lineTo(to[0] * C + C / 2, to[1] * C + C / 2);
    g.stroke(); g.setLineDash([]);
  }

  for (const [x, y] of d.obstacles) {              // physical, not yet detected
    g.fillStyle = COLORS.obstacle;
    g.fillRect(x * C + 3, y * C + 3, C - 6, C - 6);
  }
  for (const b of d.blocks) {                      // gossiped block events
    const [x, y] = b.cell, cx = x * C + C / 2, cy = y * C + C / 2;
    g.strokeStyle = b.confidence === "CONFIRMED" ? COLORS.block : "#9b6b6b";
    g.lineWidth = 2; g.beginPath();
    g.moveTo(cx - C / 3, cy - C / 3); g.lineTo(cx + C / 3, cy + C / 3);
    g.moveTo(cx + C / 3, cy - C / 3); g.lineTo(cx - C / 3, cy + C / 3);
    g.stroke();
  }

  for (const t of d.tasks) {                       // open task endpoints
    if (!t.in_transit) marker(g, t.pickup, COLORS.pickup, C, true);
    marker(g, t.dropoff, COLORS.dropoff, C, false);
  }

  const alpha = Math.min(1, (performance.now() - state.stamp) / state.period);
  if ($("showPaths").checked) {
    for (const r of d.robots) {
      if (r.state === "failed" || r.path.length < 2) continue;
      g.strokeStyle = robotColor(r.id, 60).replace(")", " / .38)").replace("hsl", "hsla");
      g.lineWidth = 2; g.lineJoin = "round"; g.beginPath();
      r.path.forEach(([x, y], i) => {
        const px = x * C + C / 2, py = y * C + C / 2;
        i ? g.lineTo(px, py) : g.moveTo(px, py);
      });
      g.stroke();
      const last = r.path[r.path.length - 1];
      g.fillStyle = robotColor(r.id, 62);
      g.fillRect(last[0] * C + C / 2 - 2, last[1] * C + C / 2 - 2, 4, 4);
    }
  }

  for (const r of d.robots) drawRobot(g, r, C, alpha);

  if (state.hoverCell) {
    g.strokeStyle = "rgba(231,237,248,.35)"; g.lineWidth = 1.5;
    g.strokeRect(state.hoverCell[0] * C + 1, state.hoverCell[1] * C + 1, C - 2, C - 2);
  }
}

function marker(g, [x, y], color, C, filled) {
  const cx = x * C + C / 2, cy = y * C + C / 2, r = Math.max(2.5, C * 0.17);
  g.beginPath(); g.arc(cx, cy, r, 0, 6.283);
  if (filled) { g.fillStyle = color + "dd"; g.fill(); }
  else { g.strokeStyle = color + "cc"; g.lineWidth = 1.5; g.stroke(); }
}

function drawRobot(g, r, C, alpha) {
  const p = state.prev.get(r.id) || r;
  const x = (p.x + (r.x - p.x) * alpha) * C, y = (p.y + (r.y - p.y) * alpha) * C;
  const pad = Math.max(1.5, C * 0.11), s = C - pad * 2;
  const dead = r.state === "failed";
  const fill = dead ? "#4a2027" : robotColor(r.id, 55);

  g.save();
  g.shadowColor = "rgba(0,0,0,.45)"; g.shadowBlur = 5; g.shadowOffsetY = 1;
  g.fillStyle = fill;
  roundRect(g, x + pad, y + pad, s, s, Math.max(2, C * 0.22)); g.fill();
  g.restore();

  g.lineWidth = 2;
  const ring = { waiting: "#ffb020", queued: "#ffb020", in_resolution: "#a78bfa",
                 shuffling: "#a78bfa", failed: "#ff6b6b", recovering: "#4cc2ff" }[r.state];
  if (ring) {
    g.strokeStyle = ring;
    roundRect(g, x + pad - 1.5, y + pad - 1.5, s + 3, s + 3, Math.max(3, C * 0.26));
    g.stroke();
  }
  if (r.battery < 100) {                    // battery arc along the top edge
    g.strokeStyle = r.battery > 30 ? "#3ddc97" : "#ff6b6b";
    g.lineWidth = 1.6; g.beginPath();
    g.moveTo(x + pad + 1, y + pad + 1.2);
    g.lineTo(x + pad + 1 + (s - 2) * (r.battery / 100), y + pad + 1.2);
    g.stroke();
  }
  if (C >= 15) {
    g.fillStyle = dead ? "#ff9aa2" : "#08101c";
    g.font = `700 ${Math.round(C * 0.44)}px ui-sans-serif,system-ui`;
    g.textAlign = "center"; g.textBaseline = "middle";
    g.fillText(r.id, x + C / 2, y + C / 2 + 0.5);
  }
  if (r.phase === "to_dropoff") {           // carrying an item
    g.fillStyle = "#ffd97a";
    g.fillRect(x + C - pad - 4, y + pad + 1, 3, 3);
  }
}

/* ----------------------------------------------------------------- rail */
function renderRail(d) {
  const m = d.metrics;
  const ts = d.tasks_stats, batt = d.battery;

  // task pipeline: the four buckets always sum back to "received"
  const seg = [["delivered", ts.delivered, "#4cc2ff"], ["in_progress", ts.in_progress, "#3ddc97"],
               ["auction", ts.auction, "#a78bfa"], ["pending", ts.pending, "#8b96ad"],
               ["recovery", ts.recovery, "#ff6b6b"]];
  const total = Math.max(1, ts.received);
  $("pipe").innerHTML = seg.map(([k, v, c]) =>
    `<span title="${k}: ${v}" style="width:${(100 * v / total).toFixed(2)}%;background:${c}"></span>`).join("");
  $("pipeSub").textContent = `${ts.outstanding} outstanding of ${ts.received} received`;
  $("taskTiles").innerHTML = [
    ["Received", ts.received, "accent"],
    ["Delivered", ts.delivered, "good"],
    ["In progress", ts.in_progress, ""],
    ["Pending", ts.pending + ts.auction, ts.pending + ts.auction > 40 ? "warn" : ""],
  ].map(([k, v, cls]) => `<div class="tile ${cls}"><b>${v}</b><span>${k}</span></div>`).join("");

  $("fleetSub").textContent =
    `battery avg ${batt.avg}% · min ${batt.min}%` + (batt.charging ? ` · ${batt.charging} charging` : "");

  const tiles = [
    ["Per 100 ticks", m.throughput_per_100.toFixed(1), "accent"],
    ["Avg / p90 time", `${m.avg_task_time.toFixed(0)}/${m.p90_task_time.toFixed(0)}`, ""],
    ["Fleet battery", `${batt.avg.toFixed(0)}%`,
      batt.min < 25 ? "warn" : batt.avg > 60 ? "good" : ""],
    ["Collisions", m.collisions, m.collisions ? "bad" : "good"],
    ["Hard stops", m.hard_stops, m.hard_stops ? "warn" : "good"],
    ["Conflicts", m.conflict_events, ""],
    ["Deadlocks", m.deadlock_events, m.gridlocks ? "warn" : ""],
    ["Re-routes", m.reroute_events, ""],
    ["Reassign lag", m.avg_reassignment_latency.toFixed(0), "warn"],
  ];
  $("tiles").innerHTML = tiles.map(([k, v, cls]) =>
    `<div class="tile ${cls}"><b>${v}</b><span>${k}</span></div>`).join("");

  $("algos").innerHTML = Object.entries(state.algorithms).map(([k, label]) =>
    `<div class="algo ${d.activity[k] ? "hot" : ""}"><code>${k}</code>${label}</div>`).join("");

  $("fleet").innerHTML = d.robots.map((r) => `
    <div class="rob" data-id="${r.id}" title="${r.state === "failed" ? "click to revive" : "click to kill"}">
      <span class="chip" style="background:${robotColor(r.id)}">${r.id}</span>
      <span>${r.capability}${r.task ? ` · T${r.task}` : ""}
        <span class="meta">${r.battery}% · ${r.distance} cells${r.waiting_for ? ` · waits on ${r.waiting_for}` : ""}</span></span>
      <span class="st ${r.state}">${r.state}</span>
    </div>`).join("");
  for (const el of document.querySelectorAll(".rob"))
    el.onclick = () => {
      const r = d.robots.find((x) => x.id === +el.dataset.id);
      post(r.state === "failed" ? "revive" : "fail", r.id);
    };

  $("chokes").innerHTML = d.corridors.map((c) => {
    const arrow = c.direction > 0 ? "▶" : c.direction < 0 ? "◀" : "·";
    return `<div class="choke"><span class="id">#${c.id}</span>
      <span class="flow">${arrow}</span>
      <span>${c.inside.length ? "in: " + c.inside.join(",") : "clear"}</span>
      <span class="q">${c.queue.length ? "queue " + c.queue.join(",") : ""} · ${c.served} through</span></div>`;
  }).join("");

  $("log").innerHTML = d.log.slice().reverse().map((e) =>
    `<li class="${e.level}"><span class="t">${String(e.t).padStart(4)}</span><span>${escapeHtml(e.text)}</span></li>`).join("");

  const H = d.history;
  spark("c_done", H, [{ pick: (h) => h.received, color: "#8b96ad" },
                      { pick: (h) => h.done, color: "#4cc2ff", fill: true }]);
  spark("c_queue", H, [{ pick: (h) => h.pending + h.in_progress, color: "#a78bfa", fill: true }],
        { min: 0 });
  spark("c_wait", H, [{ pick: (h) => h.waiting, color: "#ffb020", fill: true }],
        { min: 0, max: Math.max(2, d.robots.length), step: 1 });
  spark("c_avg", H, [{ pick: (h) => h.avg, color: "#3ddc97", fill: true }], { min: 0 });
  spark("c_batt", H, [{ pick: (h) => h.battery, color: "#3ddc97" },
                      { pick: (h) => h.battery_min, color: "#ffb020" }],
        { min: 0, max: 100, suffix: "%" });
}

const escapeHtml = (s) => s.replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]));

function spark(id, history, series, opts = {}) {
  const cv = $(id), dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (!w || !h) return;
  cv.width = w * dpr; cv.height = h * dpr;
  const g = cv.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  if (history.length < 2) return;

  const cols = series.map((s) => history.map(s.pick));
  const flat = cols.flat();
  // A fixed range where one exists (robot counts, percentages) — auto-scaling a
  // series that is almost always zero turns a single blip into a full-height
  // spike, which is what made these charts unreadable.
  let lo = opts.min !== undefined ? opts.min : Math.min(...flat);
  let hi = opts.max !== undefined ? opts.max : Math.max(...flat);
  if (opts.step) hi = Math.max(hi, Math.ceil(Math.max(...flat) / opts.step) * opts.step);
  if (hi - lo < 1e-6) hi = lo + 1;

  const padR = 40, padT = 12, padB = 3;          // room for the value readout
  const X = (i) => (i / (history.length - 1)) * (w - padR - 2) + 1;
  const Y = (v) => h - padB - ((Math.min(hi, Math.max(lo, v)) - lo) / (hi - lo)) * (h - padT - padB);

  g.strokeStyle = "rgba(255,255,255,.055)"; g.lineWidth = 1;
  for (let i = 0; i <= 2; i++) {
    const y = Math.round(Y(lo + (hi - lo) * (i / 2))) + .5;
    g.beginPath(); g.moveTo(0, y); g.lineTo(w - padR + 4, y); g.stroke();
  }

  const drawn = [];
  series.forEach((s, si) => {
    const vals = cols[si];
    if (s.fill) {
      g.beginPath();
      g.moveTo(X(0), Y(lo));
      vals.forEach((v, i) => g.lineTo(X(i), Y(v)));
      g.lineTo(X(vals.length - 1), Y(lo)); g.closePath();
      const grad = g.createLinearGradient(0, 0, 0, h);
      grad.addColorStop(0, s.color + "3a"); grad.addColorStop(1, s.color + "00");
      g.fillStyle = grad; g.fill();
    }
    g.beginPath();
    vals.forEach((v, i) => (i ? g.lineTo(X(i), Y(v)) : g.moveTo(X(i), Y(v))));
    g.strokeStyle = s.color; g.lineWidth = si ? 1.3 : 1.6;
    g.lineJoin = "round"; g.stroke();

    const last = vals[vals.length - 1];
    g.fillStyle = s.color;
    g.beginPath(); g.arc(X(vals.length - 1), Y(last), 2.2, 0, 6.283); g.fill();
    g.font = "600 10.5px ui-monospace,monospace";
    g.textAlign = "left"; g.textBaseline = "middle";
    let ly = Y(last);                       // nudge apart when two series coincide
    while (drawn.some((y) => Math.abs(y - ly) < 11)) ly += 11;
    ly = Math.min(h - 7, Math.max(7, ly));
    drawn.push(ly);
    g.fillText(fmt(last) + (opts.suffix || ""), w - padR + 7, ly);
  });
}

const fmt = (v) => (Number.isInteger(v) ? String(v)
  : Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1));

/* ----------------------------------------------------------------- boot */
async function boot() {
  const init = await (await fetch("/api/init")).json();
  state.layout = init.layout;
  state.scenarios = init.scenarios;
  state.algorithms = init.algorithms;
  $("scenario").innerHTML = init.scenarios.map((s) =>
    `<option value="${s.name}" title="${s.description}">${s.label}</option>`).join("");
  $("legend").innerHTML = [
    ["#26314a", "Racks"], ["#1a2440", "Single-file aisle"], ["#ffb020", "Pickup"],
    ["#a78bfa", "Drop-off"], ["#ff6b6b", "Reported block"], ["#7a3540", "Undetected obstacle"],
  ].map(([c, l]) => `<span><i style="background:${c}"></i>${l}</span>`).join("");

  wireControls();
  ingest(init);
  resize();
  setInterval(poll, 90);
  (function frame() { draw(); requestAnimationFrame(frame); })();
}
boot();
