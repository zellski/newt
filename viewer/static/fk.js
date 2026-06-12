/*
**        Forward kinematics for NewT creatures, mirroring the sweep in
**        src/RigidBody.C (thesis eq. 4.21): body angles are relative,
**        accumulated down the attachment chain; a body entered through
**        point E sits with its center at  entryWorld - R(acc)*E  and
**        carries its other points at  center + R(acc)*P.
**
**        Plain ES module, shared by the browser page and bun test.
*/

// creature: the parsed creature YAML (root, bodies, attachments)
export function buildTree(creature) {
   const bodies = new Map();
   for (const b of creature.bodies) {
      bodies.set(b.name, b);
   }

   const resolve = (ref) => {
      const dot = ref.indexOf(".");
      const body = bodies.get(ref.slice(0, dot));
      const point = body?.points?.find((p) => p.name === ref.slice(dot + 1));
      if (!point) throw new Error(`fk: cannot resolve point ${ref}`);
      return { body, at: point.at };
   };

   // children grouped by parent body, each carrying the parent-local
   // anchor position and the child's entry-point position
   const childrenOf = new Map();
   for (const a of creature.attachments ?? []) {
      const parent = resolve(a.parent);
      const child = resolve(a.child);
      if (!childrenOf.has(parent.body.name)) {
         childrenOf.set(parent.body.name, []);
      }
      childrenOf.get(parent.body.name).push({ parent, child });
   }

   const makeNode = (body, entryAt) => ({
      body,
      entry: entryAt,
      children: (childrenOf.get(body.name) ?? []).map((a) => ({
         at: a.parent.at,
         node: makeNode(a.child.body, a.child.at),
      })),
   });

   const rootEntry = resolve(creature.root.attach);
   return {
      rootX: creature.root.x,
      rootY: creature.root.y,
      root: makeNode(rootEntry.body, rootEntry.at),
   };
}

// q: dof name -> value. Returns body name -> {x, y, angle} with angle
// the accumulated world angle (radians, CCW).
export function fkPose(tree, q) {
   const out = {};
   const walk = (node, px, py, acc) => {
      const a = acc + q[node.body.dof];
      const c = Math.cos(a), s = Math.sin(a);
      const rx = (v) => c * v[0] - s * v[1];
      const ry = (v) => s * v[0] + c * v[1];
      const cx = px - rx(node.entry);
      const cy = py - ry(node.entry);
      out[node.body.name] = { x: cx, y: cy, angle: a };
      for (const ch of node.children) {
         walk(ch.node, cx + rx(ch.at), cy + ry(ch.at), a);
      }
   };
   walk(tree.root, q[tree.rootX], q[tree.rootY], 0);
   return out;
}
