/*
**        The newt viewer page: a three.js scene animated by forward
**        kinematics over recorded trajectory frames, with two scrubbers
**        -- animation time within an iterate, and optimization iteration
**        -- so the motion can be watched refining as the solver works.
*/

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { buildTree, fkPose } from "./fk.js";

// ------------------------------------------------------------------ api

async function getJSON(url) {
   const r = await fetch(url);
   if (!r.ok) throw new Error(`${url}: ${r.status}`);
   return r.json();
}

async function getFrames(runId, k) {
   const r = await fetch(`/api/run/${runId}/iteration/${k}/frames`);
   if (!r.ok) throw new Error(`frames ${runId}/${k}: ${r.status}`);
   const n = +r.headers.get("X-Newt-Frames");
   const d = +r.headers.get("X-Newt-Dofs");
   const buf = await r.arrayBuffer();
   return {
      n,
      times: new Float64Array(buf, 0, n),
      frames: new Float64Array(buf, n * 8, n * d * 2),
      d,
   };
}

// ---------------------------------------------------------------- state

const state = {
   runs: [],
   run: null,            // /api/run/:id payload
   tree: null,           // fk tree for run.creature
   iterations: [],       // /api/run/:id/iterations payload
   selectedK: -1,
   cur: null,            // frames of the selected iterate
   frameCache: new Map(),
   t: 0,
   playing: true,
   speed: 1,
};

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------- scene

const glCanvas = $("gl");
const renderer = new THREE.WebGLRenderer({ canvas: glCanvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14161a);
scene.environment = new THREE.PMREMGenerator(renderer)
   .fromScene(new RoomEnvironment()).texture;

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
camera.position.set(0, 1, 6);

const controls = new OrbitControls(camera, glCanvas);
controls.enableDamping = true;

scene.add(new THREE.AmbientLight(0xffffff, 0.25));
const key = new THREE.DirectionalLight(0xfff0d8, 2.2);
key.position.set(3, 6, 8);
scene.add(key);
const fill = new THREE.DirectionalLight(0x9db8ff, 0.5);
fill.position.set(-5, 2, 5);
scene.add(fill);

const grid = new THREE.GridHelper(40, 80, 0x3a414e, 0x262b33);
scene.add(grid);

// the dirty gold of the original RenderMan snapshots
const bodyMaterial = new THREE.MeshStandardMaterial({
   color: 0xc08a3e,
   metalness: 0.75,
   roughness: 0.35,
});

let creatureGroup = null;    // parent of all body groups
let bodyGroups = {};         // body name -> THREE.Group

function bodyMesh(body) {
   let geo;
   switch (body.type) {
   case "sphere":
      geo = new THREE.SphereGeometry(body.radius, 32, 16);
      break;
   case "rod":
      // capsule reads chonkier than a bare cylinder at the joints
      geo = new THREE.CapsuleGeometry(body.radius, body.length, 8, 24);
      geo.rotateZ(Math.PI / 2);             // axis y -> x
      break;
   case "disk":
   case "cylinder":
      geo = new THREE.CylinderGeometry(body.radius, body.radius,
                                       body.height, 32);
      geo.rotateX(Math.PI / 2);             // axis y -> z
      break;
   default:
      throw new Error(`unknown body type ${body.type}`);
   }
   return new THREE.Mesh(geo, bodyMaterial);
}

function rebuildCreature() {
   if (creatureGroup) {
      scene.remove(creatureGroup);
      creatureGroup.traverse((o) => o.geometry?.dispose());
   }
   creatureGroup = new THREE.Group();
   bodyGroups = {};
   for (const body of state.run.creature.bodies) {
      const g = new THREE.Group();
      g.add(bodyMesh(body));
      bodyGroups[body.name] = g;
      creatureGroup.add(g);
   }
   scene.add(creatureGroup);
}

function applyPose(pose) {
   for (const [name, p] of Object.entries(pose)) {
      const g = bodyGroups[name];
      g.position.set(p.x, p.y, 0);
      g.rotation.z = p.angle;
   }
}

// largest body half-extent, as camera-framing padding
function creatureExtent() {
   let e = 0.1;
   for (const b of state.run.creature.bodies) {
      e = Math.max(e, b.radius ?? 0, (b.length ?? 0) / 2, (b.height ?? 0) / 2);
   }
   return e;
}

function frameCamera() {
   const cur = state.cur;
   if (!cur || cur.n === 0) return;
   const d = cur.d;
   let minX = 1e30, maxX = -1e30, minY = 1e30, maxY = -1e30;
   const step = Math.max(1, Math.floor(cur.n / 60));
   const q = {};
   for (let f = 0; f < cur.n; f += step) {
      state.run.dof_names.forEach((name, i) => {
         q[name] = cur.frames[f * d * 2 + 2 * i];
      });
      for (const p of Object.values(fkPose(state.tree, q))) {
         minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
         minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
      }
   }
   const pad = creatureExtent() * 1.5;
   const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
   const halfW = (maxX - minX) / 2 + pad, halfH = (maxY - minY) / 2 + pad;
   const tan = Math.tan((camera.fov * Math.PI) / 360);
   const dist = 1.1 * Math.max(halfH / tan, halfW / (tan * camera.aspect), 1);
   camera.position.set(cx, cy, dist);
   controls.target.set(cx, cy, 0);
   controls.update();
}

glCanvas.addEventListener("dblclick", frameCamera);

// ------------------------------------------------------- run navigation

function fmt(v, digits = 4) {
   if (v === null || v === undefined) return "—";
   if (v === 0) return "0";
   const a = Math.abs(v);
   return a >= 0.01 && a < 100000 ? v.toFixed(digits) : v.toExponential(2);
}

function renderRunList() {
   const ul = $("run-list");
   ul.replaceChildren(...state.runs.map((r) => {
      const li = document.createElement("li");
      if (state.run && r.id === state.run.id) li.classList.add("selected");
      const left = document.createElement("span");
      const dot = document.createElement("span");
      dot.className = `status ${r.status}`;
      const name = document.createElement("span");
      name.className = "name";
      name.textContent = `${r.scenario_name} #${r.id}`;
      left.append(dot, name);
      const meta = document.createElement("span");
      meta.className = "meta";
      meta.textContent = `${r.n_iterations} it`;
      li.append(left, meta);
      li.addEventListener("click", () => loadRun(r.id));
      return li;
   }));
}

async function loadRun(id) {
   state.run = await getJSON(`/api/run/${id}`);
   location.hash = `run=${id}`;
   state.tree = buildTree(state.run.creature);
   state.frameCache = new Map();
   state.iterations = await getJSON(`/api/run/${id}/iterations`);
   rebuildCreature();
   renderRunList();
   const last = state.iterations.findLast((i) => i.n_frames > 0);
   await selectIteration(last ? last.k : -1);
   frameCamera();
}

async function selectIteration(k) {
   state.selectedK = k;
   const it = state.iterations.find((i) => i.k === k);
   if (!it || it.n_frames === 0) {
      state.cur = null;
      showOverlay(it ? "no frames for this iterate (nonpositive stage duration)"
                     : "no iterations recorded");
      return;
   }
   showOverlay(null);
   if (!state.frameCache.has(k)) {
      state.frameCache.set(k, await getFrames(state.run.id, k));
   }
   state.cur = state.frameCache.get(k);
   // keep t across iterates: scrubbing k then morphs the same moment
   state.t = Math.min(state.t, duration());
}

function duration() {
   return state.cur && state.cur.n > 0 ? state.cur.times[state.cur.n - 1] : 0;
}

function showOverlay(msg) {
   const o = $("overlay");
   o.hidden = !msg;
   o.textContent = msg ?? "";
}

// ------------------------------------------------------- scrub widgets

function scrubber(canvas, onPick) {
   let dragging = false;
   const pick = (e) => {
      const rect = canvas.getBoundingClientRect();
      onPick(Math.min(1, Math.max(0, (e.clientX - rect.left) / rect.width)));
   };
   canvas.addEventListener("pointerdown", (e) => {
      dragging = true;
      canvas.setPointerCapture(e.pointerId);
      pick(e);
   });
   canvas.addEventListener("pointermove", (e) => dragging && pick(e));
   canvas.addEventListener("pointerup", () => { dragging = false; });
}

function sizeCanvas(canvas) {
   const dpr = window.devicePixelRatio;
   const w = canvas.clientWidth, h = canvas.clientHeight;
   if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
   }
   const ctx = canvas.getContext("2d");
   ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
   return [ctx, w, h];
}

const GOLD = "#d4a23f", TEAL = "#4fb8a8", DIM = "#7d838d";

// objective (gold) and infeasibility (teal), each log-scaled to its own
// range; nonpositive values pin to the bottom edge
function drawIterCanvas() {
   const [ctx, w, h] = sizeCanvas($("iter-canvas"));
   ctx.clearRect(0, 0, w, h);
   const its = state.iterations;
   if (!its.length) return;
   const maxK = Math.max(1, its[its.length - 1].k);
   const X = (k) => 4 + (k / maxK) * (w - 8);

   for (const [series, color] of [["objective", GOLD],
                                  ["infeasibility", TEAL]]) {
      const logs = its.map((i) => (i[series] > 0 ? Math.log10(i[series]) : null));
      const present = logs.filter((v) => v !== null);
      if (!present.length) continue;
      const lo = Math.min(...present), hi = Math.max(...present);
      const Y = (v) => v === null ? h - 4
         : h - 4 - ((v - lo) / Math.max(hi - lo, 1e-12)) * (h - 12);
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      its.forEach((it, j) => {
         const x = X(it.k), y = Y(logs[j]);
         j === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      });
      ctx.stroke();
   }

   // frameless iterates get a warning tick
   ctx.fillStyle = "#b04a4a";
   for (const it of its) {
      if (it.n_frames === 0) ctx.fillRect(X(it.k) - 1, h - 6, 2, 6);
   }

   const sel = X(Math.max(0, state.selectedK));
   ctx.strokeStyle = "#e8e8e8";
   ctx.lineWidth = 1;
   ctx.beginPath();
   ctx.moveTo(sel, 0);
   ctx.lineTo(sel, h);
   ctx.stroke();
}

function drawTimeCanvas() {
   const [ctx, w, h] = sizeCanvas($("time-canvas"));
   ctx.clearRect(0, 0, w, h);
   const T = duration();
   if (T <= 0) return;
   const X = (t) => (t / T) * w;

   ctx.fillStyle = "#262b33";
   ctx.fillRect(0, 0, X(state.t), h);

   const it = state.iterations.find((i) => i.k === state.selectedK);
   ctx.strokeStyle = DIM;
   ctx.lineWidth = 1;
   for (const b of it?.stage_bounds ?? []) {
      if (b >= T) continue;
      ctx.beginPath();
      ctx.moveTo(X(b), 0);
      ctx.lineTo(X(b), h);
      ctx.stroke();
   }

   ctx.fillStyle = GOLD;
   ctx.fillRect(X(state.t) - 1, 0, 2, h);
}

scrubber($("iter-canvas"), (frac) => {
   const its = state.iterations;
   if (!its.length) return;
   const k = Math.round(frac * its[its.length - 1].k);
   // snap to the nearest iterate that actually exists
   const nearest = its.reduce((best, i) =>
      Math.abs(i.k - k) < Math.abs(best.k - k) ? i : best);
   if (nearest.k !== state.selectedK) selectIteration(nearest.k);
});

scrubber($("time-canvas"), (frac) => {
   state.t = frac * duration();
});

// ------------------------------------------------------------ controls

$("play").addEventListener("click", () => {
   state.playing = !state.playing;
   $("play").innerHTML = state.playing ? "&#10074;&#10074;" : "&#9654;";
});

$("speed").addEventListener("change", (e) => {
   state.speed = +e.target.value;
});

// --------------------------------------------------------------- loop

function updateReadouts() {
   const it = state.iterations.find((i) => i.k === state.selectedK);
   $("iter-readout").textContent = it
      ? `k ${it.k} · f ${fmt(it.objective)} · inf ${fmt(it.infeasibility)}`
      : "";
   $("time-readout").textContent =
      `t ${state.t.toFixed(3)} / ${duration().toFixed(3)} s`;

   const r = state.run;
   $("hud").innerHTML = r
      ? `<span class="big">${r.scenario_name} #${r.id}</span>\n` +
        `${r.status} · ${r.n_dofs} dofs · ` +
        `∇L ${fmt(it?.norm_grd_l)}`
      : "";
}

function displayedPose() {
   const cur = state.cur;
   if (!cur || cur.n === 0) return null;
   const d = cur.d, dt = state.run.frame_dt;
   const i = Math.min(Math.floor(state.t / dt), cur.n - 1);
   const j = Math.min(i + 1, cur.n - 1);
   const frac = Math.min(state.t / dt - i, 1);
   const q = {};
   state.run.dof_names.forEach((name, di) => {
      const a = cur.frames[i * d * 2 + 2 * di];
      const b = cur.frames[j * d * 2 + 2 * di];
      q[name] = a + (b - a) * frac;
   });
   return fkPose(state.tree, q);
}

let lastTick = performance.now();

function tick(now) {
   requestAnimationFrame(tick);
   const dt = Math.min((now - lastTick) / 1000, 0.1);
   lastTick = now;

   const view = $("view");
   const w = view.clientWidth, h = view.clientHeight;
   if (glCanvas.width !== w * devicePixelRatio ||
       glCanvas.height !== h * devicePixelRatio) {
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
   }

   if (state.playing && duration() > 0) {
      state.t = (state.t + dt * state.speed) % duration();
   }
   const pose = displayedPose();
   if (pose) applyPose(pose);

   controls.update();
   renderer.render(scene, camera);
   drawIterCanvas();
   drawTimeCanvas();
   updateReadouts();
}

// ---------------------------------------------------------------- boot

state.runs = await getJSON("/api/runs");
renderRunList();
const hash = location.hash.match(/run=(\d+)/);
const initial = hash && state.runs.some((r) => r.id === +hash[1])
   ? +hash[1] : state.runs[0]?.id;
if (initial !== undefined) await loadRun(initial);
requestAnimationFrame(tick);
