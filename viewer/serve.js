/*
**        Read-only viewer server for the NewT run database.
**
**        Serves the static three.js page plus a small JSON/binary API
**        over a newt.db produced by demo/run. The database is opened
**        read-only; WAL mode means a live optimizer run can keep
**        appending while we read.
**
**        Usage:  bun serve.js [path/to/newt.db] [--port N]
**                bun serve.js [path/to/newt.db] --export <run-id> [-o out.html]
**
**        --export writes one run as a self-contained HTML file (data
**        inlined; three.js still loaded from its CDN pin).
*/

import { Database } from "bun:sqlite";

const STATIC_DIR = import.meta.dir + "/static";
const SCHEMA_VERSION = 1;

let dbPath = "newt.db";
let port = 8717;
let exportId = null;
let outPath = null;

const argv = process.argv.slice(2);
for (let i = 0; i < argv.length; i++) {
   if (argv[i] === "--port") {
      port = parseInt(argv[++i], 10);
      if (!Number.isFinite(port)) fail("--port needs a number");
   } else if (argv[i] === "--export") {
      exportId = parseInt(argv[++i], 10);
      if (!Number.isFinite(exportId)) fail("--export needs a run id");
   } else if (argv[i] === "-o") {
      outPath = argv[++i];
   } else if (argv[i].startsWith("-")) {
      fail(`unknown option ${argv[i]}`);
   } else {
      dbPath = argv[i];
   }
}

function fail(msg) {
   console.error(`newt-view: ${msg}`);
   process.exit(1);
}

if (!(await Bun.file(dbPath).exists())) {
   fail(`no database at ${dbPath}`);
}

const db = new Database(dbPath, { readonly: true });
db.exec("PRAGMA busy_timeout=2000;");

const version = db.query("PRAGMA user_version;").get().user_version;
if (version !== SCHEMA_VERSION) {
   fail(`${dbPath} has schema version ${version}, expected ${SCHEMA_VERSION}`);
}

const qRuns = db.query(`
   SELECT r.id, r.scenario_name, r.status, r.started_at, r.finished_at,
          r.n_dofs, r.frame_dt, r.final_iter,
          (SELECT COUNT(*) FROM iterations i WHERE i.run_id = r.id)
             AS n_iterations,
          (SELECT objective FROM iterations i WHERE i.run_id = r.id
              ORDER BY k DESC LIMIT 1) AS last_objective
   FROM runs r ORDER BY r.id DESC;`);

const qRun = db.query("SELECT * FROM runs WHERE id = ?;");

const qIterations = db.query(`
   SELECT k, objective, infeasibility, norm_grd_l, n_frames, stage_bounds
   FROM iterations WHERE run_id = ? ORDER BY k;`);

const qFrames = db.query(`
   SELECT i.n_frames, r.n_dofs, i.times, i.frames
   FROM iterations i JOIN runs r ON r.id = i.run_id
   WHERE i.run_id = ? AND i.k = ?;`);

function json(body, status = 200) {
   return new Response(JSON.stringify(body), {
      status,
      headers: { "Content-Type": "application/json" },
   });
}

function notFound(why) {
   return new Response(why + "\n", { status: 404 });
}

function apiRuns() {
   return json(qRuns.all());
}

function runDetail(id) {
   const run = qRun.get(id);
   if (!run) return null;
   run.dof_names = JSON.parse(run.dof_names);
   run.creature = run.creature_yaml ? Bun.YAML.parse(run.creature_yaml) : null;
   delete run.creature_yaml;
   return run;
}

function apiRun(id) {
   const run = runDetail(id);
   return run ? json(run) : notFound(`no run ${id}`);
}

function apiIterations(id) {
   if (!qRun.get(id)) return notFound(`no run ${id}`);
   const rows = qIterations.all(id).map((r) => ({
      ...r,
      stage_bounds: r.stage_bounds ? JSON.parse(r.stage_bounds) : [],
   }));
   return json(rows);
}

// times blob followed by frames blob, both raw little-endian float64;
// the client recovers the split from the X-Newt-Frames/-Dofs headers
function apiFrames(id, k) {
   const row = qFrames.get(id, k);
   if (!row) return notFound(`no iteration ${k} in run ${id}`);
   const times = row.times ?? new Uint8Array(0);
   const frames = row.frames ?? new Uint8Array(0);
   return new Response(new Blob([times, frames]), {
      headers: {
         "Content-Type": "application/octet-stream",
         "X-Newt-Frames": String(row.n_frames),
         "X-Newt-Dofs": String(row.n_dofs),
      },
   });
}

const MIME = {
   ".html": "text/html; charset=utf-8",
   ".js": "text/javascript; charset=utf-8",
   ".css": "text/css; charset=utf-8",
   ".svg": "image/svg+xml",
};

async function serveStatic(pathname) {
   if (pathname === "/") pathname = "/index.html";
   if (pathname.includes("..")) return notFound("no");
   const file = Bun.file(STATIC_DIR + pathname);
   if (!(await file.exists())) return notFound(`no such file ${pathname}`);
   const ext = pathname.slice(pathname.lastIndexOf("."));
   return new Response(file, {
      headers: {
         "Content-Type": MIME[ext] ?? "application/octet-stream",
         "Cache-Control": "no-cache",
      },
   });
}

// ------------------------------------------------------- static export

async function exportRun(id, out) {
   const run = runDetail(id);
   if (!run) fail(`no run ${id}`);
   const summary = qRuns.all().find((r) => r.id === id);
   const iterations = qIterations.all(id).map((r) => ({
      ...r,
      stage_bounds: r.stage_bounds ? JSON.parse(r.stage_bounds) : [],
   }));

   const frames = {};
   for (const it of iterations) {
      if (it.n_frames === 0) continue;
      const row = qFrames.get(id, it.k);
      const buf = new Uint8Array(row.times.length + row.frames.length);
      buf.set(row.times, 0);
      buf.set(row.frames, row.times.length);
      frames[it.k] = { n: it.n_frames, b64: Buffer.from(buf).toString("base64") };
   }

   const bundle = await Bun.build({
      entrypoints: [STATIC_DIR + "/app.js"],
      target: "browser",
      external: ["three", "three/addons/*"],
   });
   if (!bundle.success) fail("bundling app.js failed");
   const js = await bundle.outputs[0].text();
   const css = await Bun.file(STATIC_DIR + "/style.css").text();

   const embed = JSON.stringify({ summary, run, iterations, frames })
      .replace(/</g, "\\u003c");   // keep </script> inert inside the tag
   const html = (await Bun.file(STATIC_DIR + "/index.html").text())
      .replace('<link rel="stylesheet" href="/style.css">',
               `<style>\n${css}</style>`)
      .replace('<script type="module" src="/app.js"></script>',
               `<script>window.NEWT_EMBED = ${embed};</script>\n` +
               `<script type="module">\n${js}</script>`);

   await Bun.write(out, html);
   console.log(`newt-view: wrote ${out} (${(html.length / 1024).toFixed(0)} KiB,`
      + ` ${Object.keys(frames).length} iterates; three.js loads from CDN)`);
}

if (exportId !== null) {
   await exportRun(exportId,
      outPath ?? `newt-run-${exportId}.html`);
   process.exit(0);
}

const server = Bun.serve({
   port,
   fetch(req) {
      const { pathname } = new URL(req.url);
      let m;

      if (pathname === "/api/runs") return apiRuns();
      if ((m = pathname.match(/^\/api\/run\/(\d+)$/)))
         return apiRun(+m[1]);
      if ((m = pathname.match(/^\/api\/run\/(\d+)\/iterations$/)))
         return apiIterations(+m[1]);
      if ((m = pathname.match(/^\/api\/run\/(\d+)\/iteration\/(\d+)\/frames$/)))
         return apiFrames(+m[1], +m[2]);
      if (pathname.startsWith("/api/")) return notFound("no such endpoint");

      return serveStatic(pathname);
   },
});

console.log(`newt-view: ${dbPath} -> http://localhost:${server.port}/`);
