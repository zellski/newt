/*
**        Tests for the viewer's forward kinematics against hand-computed
**        poses, plus a smoke test over a real run in ../newt.db when one
**        is available. Run with: bun test
*/

import { describe, expect, test } from "bun:test";
import { existsSync } from "node:fs";
import { Database } from "bun:sqlite";
import { buildTree, fkPose } from "./static/fk.js";

const close = (got, want) => expect(got).toBeCloseTo(want, 12);

describe("fkPose", () => {
   test("single pendulum rod", () => {
      const tree = buildTree({
         root: { x: "X", y: "Y", attach: "Rod.Top" },
         bodies: [{
            name: "Rod", type: "rod", dof: "Theta",
            points: [{ name: "Top", at: [0.5, 0] }],
         }],
      });
      const pose = fkPose(tree, { X: 2, Y: 3, Theta: Math.PI / 6 });
      // center = root - R(30deg) * (0.5, 0)
      close(pose.Rod.x, 2 - 0.5 * Math.cos(Math.PI / 6));
      close(pose.Rod.y, 3 - 0.25);
      close(pose.Rod.angle, Math.PI / 6);
   });

   test("two-link chain accumulates angles", () => {
      const tree = buildTree({
         root: { x: "X", y: "Y", attach: "A.ABot" },
         bodies: [
            {
               name: "A", type: "rod", dof: "ThA",
               points: [
                  { name: "ABot", at: [-0.5, 0] },
                  { name: "ATop", at: [0.5, 0] },
               ],
            },
            {
               name: "B", type: "rod", dof: "ThB",
               points: [{ name: "BBot", at: [-0.5, 0] }],
            },
         ],
         attachments: [{ parent: "A.ATop", child: "B.BBot" }],
      });
      // A straight up from the origin, B folded back to horizontal:
      // B spans (0,1) -> (-1,1)
      const pose = fkPose(tree, {
         X: 0, Y: 0, ThA: Math.PI / 2, ThB: Math.PI / 2,
      });
      close(pose.A.x, 0);
      close(pose.A.y, 0.5);
      close(pose.B.x, -0.5);
      close(pose.B.y, 1);
      close(pose.B.angle, Math.PI);
   });

   test("siblings on a branched body stay independent", () => {
      const tree = buildTree({
         root: { x: "X", y: "Y", attach: "Torso.TBot" },
         bodies: [
            {
               name: "Torso", type: "rod", dof: "ThT",
               points: [
                  { name: "TBot", at: [-0.5, 0] },
                  { name: "TMid", at: [0, 0] },
                  { name: "TTop", at: [0.5, 0] },
               ],
            },
            {
               name: "Arm", type: "rod", dof: "ThArm",
               points: [{ name: "ArmIn", at: [-0.25, 0] }],
            },
            {
               name: "Leg", type: "rod", dof: "ThLeg",
               points: [{ name: "LegIn", at: [0.25, 0] }],
            },
         ],
         attachments: [
            { parent: "Torso.TTop", child: "Arm.ArmIn" },
            { parent: "Torso.TMid", child: "Leg.LegIn" },
         ],
      });
      const pose = fkPose(tree, {
         X: 1, Y: 2, ThT: 0, ThArm: Math.PI / 2, ThLeg: -Math.PI / 2,
      });
      close(pose.Torso.x, 1.5);
      close(pose.Torso.y, 2);
      close(pose.Arm.x, 2);
      close(pose.Arm.y, 2.25);
      close(pose.Arm.angle, Math.PI / 2);
      close(pose.Leg.x, 1.5);
      close(pose.Leg.y, 2.25);
      close(pose.Leg.angle, -Math.PI / 2);
   });

   test("dangling point reference throws", () => {
      expect(() => buildTree({
         root: { x: "X", y: "Y", attach: "Rod.Nope" },
         bodies: [{ name: "Rod", type: "rod", dof: "T", points: [] }],
      })).toThrow(/cannot resolve/);
   });
});

const dbPath = import.meta.dir + "/../newt.db";

describe.skipIf(!existsSync(dbPath))("against a real run database", () => {
   test("luxo blobs and FK are coherent", () => {
      const db = new Database(dbPath, { readonly: true });
      const run = db.query(`
         SELECT id, n_dofs, dof_names, creature_yaml, final_iter FROM runs
         WHERE scenario_name LIKE 'luxo' AND status = 'optimal'
         ORDER BY id DESC LIMIT 1;`).get();
      if (!run) return;   // db exists but holds no finished luxo run

      const it = db.query(`
         SELECT n_frames, times, frames FROM iterations
         WHERE run_id = ? AND k = ?;`).get(run.id, run.final_iter);
      const n = it.n_frames, d = run.n_dofs;
      expect(it.times.byteLength).toBe(n * 8);
      expect(it.frames.byteLength).toBe(n * d * 2 * 8);

      const times = new Float64Array(it.times.buffer, it.times.byteOffset, n);
      for (let i = 1; i < n; i++) expect(times[i]).toBeGreaterThan(times[i - 1]);

      const dofNames = JSON.parse(run.dof_names);
      const tree = buildTree(Bun.YAML.parse(run.creature_yaml));
      const frames = new Float64Array(
         it.frames.buffer, it.frames.byteOffset, n * d * 2);
      for (const f of [0, n - 1]) {
         const q = {};
         dofNames.forEach((name, i) => { q[name] = frames[f * d * 2 + 2 * i]; });
         const pose = fkPose(tree, q);
         for (const body of ["Base", "L1", "L2", "Head"]) {
            expect(Number.isFinite(pose[body].x)).toBe(true);
            expect(Number.isFinite(pose[body].y)).toBe(true);
            expect(Number.isFinite(pose[body].angle)).toBe(true);
         }
      }
      db.close();
   });
});
