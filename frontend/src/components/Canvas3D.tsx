/**
 * Canvas3D — Three.js (react-three-fiber) 3D viewport, a sibling to Canvas2D.
 *
 * Coordinate mapping (sim world → three.js):
 *   Sim world is right-handed X-forward, Y-left, Z-up. Three.js is right-handed
 *   Y-up. We map  (x, y, z_up) → (x, z_up, -y).  This preserves handedness, so
 *   a CCW yaw ψ about sim +Z becomes a rotation of +ψ about three's +Y axis.
 *   Helper `w2t(x, y, h)` does the mapping (h = height above ground = sim z).
 *
 * Performance model:
 *   The component subscribes only to *stable* primitives (geometry params, a
 *   JSON signature of the scene). Per-frame live state (pose, wheels, ICR) is
 *   read imperatively inside useFrame via useSimStore.getState() and applied to
 *   mesh refs — so the React tree does not re-render at 60 Hz.
 */

import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import {
  BufferAttribute,
  BufferGeometry,
  CylinderGeometry,
  DoubleSide,
  ExtrudeGeometry,
  Float32BufferAttribute,
  Group,
  Line,
  LineBasicMaterial,
  Mesh,
  MeshStandardMaterial,
  Object3D,
  Shape,
  ShapeGeometry,
  SphereGeometry,
  Vector2,
  Vector3,
} from "three";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";

import { useSimStore } from "@/store/sim";
import type { DisturbanceMsg } from "@/types/sim";
import { fmtKmh } from "@/ui/units";

// sim world (x fwd, y left, h up) → three.js (x, up, -y)
function w2t(x: number, y: number, h = 0): [number, number, number] {
  return [x, h, -y];
}

const TRAJ_MAX = 4000;

// ---------- Vehicle ----------

interface Geom {
  L: number; tF: number; tR: number; tireR: number;
}

// Wheel tint by effective μ (matches Canvas2D palette).
function muHex(mu: number): number {
  if (mu >= 0.85) return 0x0f172a;
  if (mu >= 0.5) return 0x78350f;
  if (mu >= 0.3) return 0x7f1d1d;
  return 0x4c1d95;
}

function Vehicle({ geom }: { geom: Geom }) {
  const root = useRef<Group>(null);
  const sprung = useRef<Group>(null);  // body shell — heaves + rolls + pitches
  // Per-wheel steer groups + spin meshes.
  const steerRefs = useRef<(Group | null)[]>([null, null, null, null]);
  const spinRefs = useRef<(Group | null)[]>([null, null, null, null]);
  const tireMats = useRef<(MeshStandardMaterial | null)[]>([null, null, null, null]);
  const spin = useRef<number[]>([0, 0, 0, 0]);

  const { L, tF, tR, tireR } = geom;
  const wheelLocal: [number, number][] = [
    [+L / 2, +tF / 2],   // FL
    [+L / 2, -tF / 2],   // FR
    [-L / 2, +tR / 2],   // RL
    [-L / 2, -tR / 2],   // RR
  ];
  const bodyLen = L + 0.55;
  const bodyWidth = Math.max(tF, tR) + 0.3;
  const bodyHeight = 0.55;
  const wheelWidth = 0.22;
  const bodyHeightBase = tireR;  // sprung-group pivot height (≈ roll/pitch axis)

  useFrame((_, dt) => {
    const st = useSimStore.getState().state;
    const g = root.current;
    if (!st || !g) return;
    const [px, py, pz] = w2t(st.pose.x, st.pose.y, 0);
    g.position.set(px, py, pz);
    g.rotation.y = st.pose.psi;
    // Sprung-body attitude (multibody): heave z, roll about forward (x),
    // pitch about lateral (z). Exaggerate heave ×4 so cm-scale motion reads.
    const sg = sprung.current;
    if (sg) {
      const att = st.attitude;
      sg.position.y = bodyHeightBase + (att ? att.z * 4 : 0);
      sg.rotation.x = att ? att.roll : 0;
      sg.rotation.z = att ? att.pitch : 0;
    }
    for (let i = 0; i < 4; i++) {
      const sg = steerRefs.current[i];
      const sp = spinRefs.current[i];
      if (sg) sg.rotation.y = st.wheels[i]?.delta ?? 0;
      if (sp) {
        spin.current[i] += (st.wheels[i]?.omega ?? 0) * dt;
        // Roll about the lateral axis (three +Z = sim right). For forward motion
        // (omega > 0) the wheel top must move +X, whose angular velocity points
        // in −Z — hence the negation, otherwise the wheels appear to spin backward.
        sp.rotation.z = -spin.current[i];
      }
      const mat = tireMats.current[i];
      if (mat) mat.color.setHex(muHex(st.wheels[i]?.mu ?? 1));
    }
  });

  return (
    <group ref={root}>
      {/* Sprung body — heaves / rolls / pitches relative to the wheels */}
      <group ref={sprung} position={[0, bodyHeightBase, 0]}>
        {/* Body shell */}
        <mesh position={[0, bodyHeight / 2, 0]} castShadow>
          <boxGeometry args={[bodyLen, bodyHeight, bodyWidth]} />
          <meshStandardMaterial color="#3b82f6" transparent opacity={0.55} />
        </mesh>
        {/* Roof / cabin hint */}
        <mesh position={[-L * 0.05, bodyHeight + 0.12, 0]}>
          <boxGeometry args={[bodyLen * 0.45, 0.25, bodyWidth * 0.82]} />
          <meshStandardMaterial color="#1e3a8a" transparent opacity={0.5} />
        </mesh>
        {/* Forward nose marker */}
        <mesh position={[bodyLen / 2 - 0.05, bodyHeight / 2, 0]}>
          <boxGeometry args={[0.12, 0.18, bodyWidth * 0.5]} />
          <meshStandardMaterial color="#93c5fd" emissive="#1d4ed8" emissiveIntensity={0.4} />
        </mesh>
      </group>

      {/* Wheels */}
      {wheelLocal.map(([bx, by], i) => {
        const [lx, ly, lz] = w2t(bx, by, tireR);
        return (
          <group key={i} position={[lx, ly, lz]}>
            {/* steer group rotates about up (three Y) */}
            <group ref={(r) => (steerRefs.current[i] = r)}>
              {/* spin group rotates about the wheel's lateral axis.
                  A cylinder's default axis is Y; rotate it onto Z (lateral),
                  then spin about local X to roll forward. */}
              <group ref={(r) => (spinRefs.current[i] = r)}>
                {/* Cylinder axis along the lateral (Z) axis → wheel rolls forward */}
                <mesh rotation={[Math.PI / 2, 0, 0]}>
                  <cylinderGeometry args={[tireR, tireR, wheelWidth, 20]} />
                  <meshStandardMaterial ref={(m) => (tireMats.current[i] = m)} color="#0f172a" />
                </mesh>
                {/* radial spoke (in the forward-up plane) so spin is visible */}
                <mesh position={[tireR * 0.55, 0, 0]}>
                  <boxGeometry args={[tireR * 0.7, 0.05, wheelWidth * 1.02]} />
                  <meshStandardMaterial color="#e2e8f0" />
                </mesh>
              </group>
            </group>
          </group>
        );
      })}
    </group>
  );
}

// ---------- ICR markers ----------

function bodyToWorld(px: number, py: number, psi: number, bx: number, by: number): [number, number] {
  const c = Math.cos(psi), s = Math.sin(psi);
  return [px + c * bx - s * by, py + s * bx + c * by];
}

function IcrMarkers() {
  const actual = useRef<Mesh>(null);
  const target = useRef<Mesh>(null);

  useFrame(() => {
    const st = useSimStore.getState().state;
    if (!st) return;
    const [ax, ay] = st.icr_vehicle_body;
    if (actual.current) {
      if (ax != null && ay != null) {
        const [wx, wy] = bodyToWorld(st.pose.x, st.pose.y, st.pose.psi, ax, ay);
        const [tx, ty, tz] = w2t(wx, wy, 0.05);
        actual.current.position.set(tx, ty, tz);
        actual.current.visible = true;
      } else {
        actual.current.visible = false;
      }
    }
    const [bx, by] = st.icr_target_body;
    if (target.current) {
      if (bx != null && by != null) {
        const [wx, wy] = bodyToWorld(st.pose.x, st.pose.y, st.pose.psi, bx, by);
        const [tx, ty, tz] = w2t(wx, wy, 0.05);
        target.current.position.set(tx, ty, tz);
        target.current.visible = true;
      } else {
        target.current.visible = false;
      }
    }
  });

  return (
    <>
      <mesh ref={actual} visible={false}>
        <sphereGeometry args={[0.22, 16, 16]} />
        <meshStandardMaterial color="#ef4444" emissive="#ef4444" emissiveIntensity={0.5} />
      </mesh>
      <mesh ref={target} visible={false} rotation={[Math.PI / 2, 0, 0]}>
        <torusGeometry args={[0.3, 0.05, 12, 24]} />
        <meshStandardMaterial color="#60a5fa" emissive="#3b82f6" emissiveIntensity={0.4} />
      </mesh>
    </>
  );
}

// ---------- Trajectory trail ----------

function Trajectory() {
  const ref = useRef<Line>(null);
  const geom = useMemo(() => {
    const g = new BufferGeometry();
    const positions = new Float32Array(TRAJ_MAX * 3);
    g.setAttribute("position", new BufferAttribute(positions, 3));
    g.setDrawRange(0, 0);
    return g;
  }, []);
  const mat = useMemo(
    () => new LineBasicMaterial({ color: "#22d3ee", transparent: true, opacity: 0.8 }),
    [],
  );

  useFrame(() => {
    const traj = useSimStore.getState().trajectory;
    const n = Math.min(traj.length / 2, TRAJ_MAX);
    const pos = geom.getAttribute("position") as BufferAttribute;
    const arr = pos.array as Float32Array;
    const start = Math.max(0, traj.length / 2 - TRAJ_MAX) * 2;
    for (let i = 0; i < n; i++) {
      const x = traj[start + i * 2];
      const y = traj[start + i * 2 + 1];
      arr[i * 3] = x;
      arr[i * 3 + 1] = 0.03;
      arr[i * 3 + 2] = -y;
    }
    pos.needsUpdate = true;
    geom.setDrawRange(0, n);
  });

  return <primitive object={new Line(geom, mat)} ref={ref} />;
}

// ---------- A/B comparison overlay ----------

function RunOverlay({ sig }: { sig: string }) {
  const lines = useMemo(() => {
    const { A, B } = useSimStore.getState().savedRuns;
    const make = (traj: number[] | undefined, color: number) => {
      if (!traj || traj.length < 4) return null;
      const pts: Vector3[] = [];
      for (let i = 0; i < traj.length; i += 2) pts.push(new Vector3(traj[i], 0.04, -traj[i + 1]));
      const g = new BufferGeometry().setFromPoints(pts);
      return new Line(g, new LineBasicMaterial({ color }));
    };
    return [make(A?.trajectory, 0x34d399), make(B?.trajectory, 0xfb923c)].filter(Boolean) as Line[];
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sig]);
  return <>{lines.map((l, i) => <primitive key={i} object={l} />)}</>;
}

// ---------- Reference path + cones ----------

function ReferencePath({ pathSig }: { pathSig: string }) {
  const data = useMemo<{ points: [number, number][]; cones: [number, number][]; closed: boolean }>(() => {
    try {
      return JSON.parse(pathSig);
    } catch {
      return { points: [], cones: [], closed: false };
    }
  }, [pathSig]);

  const line = useMemo(() => {
    if (data.points.length < 2) return null;
    const pts = data.points.map(([x, y]) => new Vector3(x, 0.05, -y));
    if (data.closed && pts.length > 2) pts.push(pts[0].clone());
    const g = new BufferGeometry().setFromPoints(pts);
    const m = new LineBasicMaterial({ color: "#f472b6" });
    return new Line(g, m);
  }, [data]);

  return (
    <>
      {line && <primitive object={line} />}
      {data.cones.map(([x, y], i) => (
        <mesh key={i} position={[x, 0.3, -y]}>
          <coneGeometry args={[0.22, 0.6, 12]} />
          <meshStandardMaterial color="#f59e0b" emissive="#b45309" emissiveIntensity={0.3} />
        </mesh>
      ))}
    </>
  );
}

// ---------- Static scenario (roads / markings / lights) ----------

function ribbonGeometry(points: [number, number][], width: number, h: number): BufferGeometry {
  const hw = width / 2;
  const pos: number[] = [];
  for (let i = 0; i < points.length - 1; i++) {
    const [x1, y1] = points[i], [x2, y2] = points[i + 1];
    let dx = x2 - x1, dy = y2 - y1;
    const len = Math.hypot(dx, dy) || 1; dx /= len; dy /= len;
    const nx = -dy * hw, ny = dx * hw;            // left normal · half-width (world)
    const w = (px: number, py: number) => [px, h, -py];
    const aL = w(x1 + nx, y1 + ny), aR = w(x1 - nx, y1 - ny);
    const bL = w(x2 + nx, y2 + ny), bR = w(x2 - nx, y2 - ny);
    pos.push(...aL, ...aR, ...bL, ...aR, ...bR, ...bL);
  }
  const g = new BufferGeometry();
  g.setAttribute("position", new Float32BufferAttribute(pos, 3));
  g.computeVertexNormals();
  return g;
}

function surfaceMesh(points: [number, number][], color: string, h: number): Mesh {
  const shape = new Shape(points.map(([x, y]) => new Vector2(x, y)));
  const geom = new ShapeGeometry(shape);
  geom.rotateX(-Math.PI / 2);   // lay flat: shape (x,y)→three (x,0,-y)
  const mat = new MeshStandardMaterial({ color, roughness: 0.95, metalness: 0.0 });
  const mesh = new Mesh(geom, mat);
  mesh.position.y = h;
  mesh.receiveShadow = true;
  return mesh;
}

// Extruded building/landmark — footprint rises along +y. Height is a
// deterministic pseudo-random function of the centroid so a town gets a varied,
// crowded roofline without any per-building data.
function buildingMesh(points: [number, number][], color: string, landmark: boolean): Mesh {
  const shape = new Shape(points.map(([x, y]) => new Vector2(x, y)));
  let cx = 0, cy = 0;
  for (const [x, y] of points) { cx += x; cy += y; }
  cx /= points.length; cy /= points.length;
  const r = Math.abs(Math.sin(cx * 12.9898 + cy * 78.233) * 43758.5453) % 1;
  const height = landmark ? 24 + r * 10 : 7 + r * 9;
  const geom = new ExtrudeGeometry(shape, { depth: height, bevelEnabled: false });
  geom.rotateX(-Math.PI / 2);   // shape (x,y,z)→three (x,z,-y): extrude rises +y from ground
  const mat = new MeshStandardMaterial({ color, roughness: 0.9, metalness: 0.0 });
  const mesh = new Mesh(geom, mat);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

const TL3D: Record<string, number> = { red: 0xef4444, yellow: 0xfacc15, green: 0x22c55e };

function Scenario3D({ sig }: { sig: string }) {
  const objects = useMemo<Object3D[]>(() => {
    const sc = useSimStore.getState().scenario;
    if (!sc) return [];
    const out: Object3D[] = [];
    const kindH: Record<string, number> = {
      grass: 0.0, water: -0.06, road: 0.01, plaza: 0.012, sidewalk: 0.02, paint: 0.05,
    };
    sc.surfaces.forEach((s) => {
      if (s.points.length < 3) return;
      if (s.kind === "building" || s.kind === "landmark") {
        out.push(buildingMesh(s.points, s.color, s.kind === "landmark"));
      } else {
        out.push(surfaceMesh(s.points, s.color, kindH[s.kind] ?? 0.01));
      }
    });
    sc.lines.forEach((ln) => {
      if (ln.points.length >= 2) {
        const geom = ribbonGeometry(ln.points, Math.max(0.12, ln.width), 0.06);
        const mat = new MeshStandardMaterial({ color: ln.color, roughness: 0.8 });
        out.push(new Mesh(geom, mat));
      }
    });
    sc.markers.forEach((m) => {
      if (m.type === "traffic_light") {
        const g = new Group();
        const [px, , pz] = w2t(m.x, m.y, 0);
        const pole = new Mesh(
          new CylinderGeometry(0.08, 0.08, 3.0, 8),
          new MeshStandardMaterial({ color: "#2b2f36" }),
        );
        pole.position.set(px, 1.5, pz);
        g.add(pole);
        const state = (m.meta?.state as string) || "red";
        const lamp = new Mesh(
          new SphereGeometry(0.32, 12, 12),
          new MeshStandardMaterial({
            color: TL3D[state] ?? 0xef4444,
            emissive: TL3D[state] ?? 0xef4444,
            emissiveIntensity: 0.7,
          }),
        );
        lamp.position.set(px, 3.1, pz);
        g.add(lamp);
        out.push(g);
      }
    });
    return out;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sig]);
  return <>{objects.map((o, i) => <primitive key={i} object={o} />)}</>;
}

// ---------- Disturbance regions ----------

function muColor(mu: number): string {
  if (mu >= 0.9) return "#60a5fa";
  if (mu >= 0.6) return "#fbbf24";
  if (mu >= 0.3) return "#ef4444";
  return "#6366f1";
}

function DisturbanceMesh({ d }: { d: DisturbanceMsg }) {
  const [cx, cy, cz] = w2t(d.x, d.y, 0);
  // length is along the region's heading (local x), width is cross (local y→three z)
  const len = d.length;
  const wid = d.width;

  if (d.type === "speed_bump") {
    return (
      <group position={[cx, cy, cz]} rotation={[0, d.heading, 0]}>
        <mesh position={[0, d.height / 2, 0]}>
          <boxGeometry args={[len, Math.max(d.height, 0.02), wid]} />
          <meshStandardMaterial color="#fbbf24" transparent opacity={0.85} />
        </mesh>
      </group>
    );
  }
  if (d.type === "slope") {
    // Tilt a thin slab about the cross axis (three Z) by the grade angle.
    return (
      <group position={[cx, cy, cz]} rotation={[0, d.heading, 0]}>
        <mesh rotation={[0, 0, -d.angle]} position={[0, (len / 2) * Math.tan(d.angle) / 2, 0]}>
          <boxGeometry args={[len / Math.cos(d.angle), 0.04, wid]} />
          <meshStandardMaterial color="#a855f7" transparent opacity={0.45} />
        </mesh>
      </group>
    );
  }
  if (d.type === "split_mu") {
    return (
      <group position={[cx, cy, cz]} rotation={[0, d.heading, 0]}>
        {/* left half = +Y world = -Z three */}
        <mesh position={[0, 0.015, -wid / 4]} rotation={[-Math.PI / 2, 0, 0]}>
          <planeGeometry args={[len, wid / 2]} />
          <meshStandardMaterial color={muColor(d.mu_left)} transparent opacity={0.4} side={DoubleSide} />
        </mesh>
        <mesh position={[0, 0.015, wid / 4]} rotation={[-Math.PI / 2, 0, 0]}>
          <planeGeometry args={[len, wid / 2]} />
          <meshStandardMaterial color={muColor(d.mu_right)} transparent opacity={0.4} side={DoubleSide} />
        </mesh>
      </group>
    );
  }
  // ice_patch
  return (
    <group position={[cx, cy, cz]} rotation={[0, d.heading, 0]}>
      <mesh position={[0, 0.015, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <planeGeometry args={[len, wid]} />
        <meshStandardMaterial color={muColor(d.mu)} transparent opacity={0.45} side={DoubleSide} />
      </mesh>
    </group>
  );
}

function Disturbances({ sceneSig }: { sceneSig: string }) {
  const list = useMemo<DisturbanceMsg[]>(() => {
    try {
      return JSON.parse(sceneSig) as DisturbanceMsg[];
    } catch {
      return [];
    }
  }, [sceneSig]);
  return (
    <>
      {list.map((d) => (
        <DisturbanceMesh key={d.id} d={d} />
      ))}
    </>
  );
}

// ---------- Follow camera ----------

function FollowCamera({ follow }: { follow: boolean }) {
  const { camera } = useThree();
  const controls = useRef<OrbitControlsImpl>(null);
  const lastPos = useRef<[number, number, number] | null>(null);

  // Restore the camera pose saved before the last 2D switch; persist on unmount
  // so toggling 2D ↔ 3D keeps the user's viewpoint.
  useEffect(() => {
    const saved = useSimStore.getState().camera3d;
    if (saved) {
      camera.position.set(...saved.position);
      controls.current?.target.set(...saved.target);
      controls.current?.update();
    }
    return () => {
      const c = controls.current;
      useSimStore.getState().setCamera3d({
        position: [camera.position.x, camera.position.y, camera.position.z],
        target: c ? [c.target.x, c.target.y, c.target.z] : [0, 0, 0],
      });
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useFrame(() => {
    const c = controls.current;
    if (!c) return;
    const st = useSimStore.getState().state;
    if (follow && st) {
      const [tx, ty, tz] = w2t(st.pose.x, st.pose.y, 0);
      const last = lastPos.current;
      if (last) {
        // Shift camera by the same delta so the orbit offset is preserved.
        camera.position.x += tx - last[0];
        camera.position.y += ty - last[1];
        camera.position.z += tz - last[2];
      }
      c.target.set(tx, ty, tz);
      lastPos.current = [tx, ty, tz];
    } else {
      lastPos.current = null;
    }
    c.update();
  });

  return <OrbitControls ref={controls} makeDefault enableDamping dampingFactor={0.1} />;
}

// ---------- Top-level Canvas3D ----------

export default function Canvas3D() {
  const ready = useSimStore((s) => !!s.state);
  const L = useSimStore((s) => s.state?.params.wheelbase ?? 2.8);
  const tF = useSimStore((s) => s.state?.params.track_front ?? 1.56);
  const tR = useSimStore((s) => s.state?.params.track_rear ?? 1.56);
  const tireR = useSimStore((s) => s.state?.params.tire_radius ?? 0.33);
  // Stable signature of the scene so disturbance meshes only rebuild on change.
  const sceneSig = useSimStore((s) => JSON.stringify(s.state?.scene?.disturbances ?? []));
  // Scenario rebuilds only when name/version changes (geometry built once).
  const scenarioSig = useSimStore((s) => (s.scenario ? `${s.scenario.name}:${s.scenarioVersion}` : ""));
  // Path rebuilds only when the fetched path object changes (not every frame).
  const pathSig = useSimStore((s) =>
    JSON.stringify({
      points: s.path?.points ?? [],
      cones: s.path?.cones ?? [],
      closed: s.path?.closed ?? false,
    }),
  );
  // A/B overlay rebuilds only when a run is saved/cleared or visibility toggles.
  const runSig = useSimStore((s) =>
    s.showOverlay ? `${s.savedRuns.A?.trajectory.length ?? 0}:${s.savedRuns.B?.trajectory.length ?? 0}` : "",
  );

  const [follow, setFollow] = useState(true);
  const theme = useSimStore((s) => s.theme);
  const dark = theme === "dark";

  if (!ready) {
    return (
      <div className="canvas-container">
        <div className="canvas-loading">等待仿真数据…</div>
      </div>
    );
  }

  return (
    <div className="canvas-container">
      <Canvas
        shadows
        camera={{ position: [-8, 7, 8], fov: 50, near: 0.1, far: 2000, up: [0, 1, 0] }}
        gl={{ antialias: true }}
      >
        <color attach="background" args={[dark ? "#0a0f1c" : "#eef2f7"]} />
        <hemisphereLight args={[dark ? "#cbd5e1" : "#ffffff", dark ? "#0a0f1c" : "#c7d2dd", dark ? 0.9 : 1.1]} />
        <directionalLight position={[20, 30, 10]} intensity={1.1} castShadow />

        <Grid
          args={[400, 400]}
          cellSize={1}
          cellThickness={0.5}
          cellColor={dark ? "#152033" : "#c2cdda"}
          sectionSize={5}
          sectionThickness={1}
          sectionColor={dark ? "#1e293b" : "#94a3b8"}
          infiniteGrid
          fadeDistance={120}
          fadeStrength={1.5}
          followCamera
        />

        {scenarioSig && <Scenario3D sig={scenarioSig} />}
        <Disturbances sceneSig={sceneSig} />
        <ReferencePath pathSig={pathSig} />
        {runSig && <RunOverlay sig={runSig} />}
        <Trajectory />
        <Vehicle geom={{ L, tF, tR, tireR }} />
        <IcrMarkers />
        <FollowCamera follow={follow} />
      </Canvas>

      <HUD3D follow={follow} onToggleFollow={() => setFollow((f) => !f)} />
    </div>
  );
}

// ---------- HUD overlay (DOM) ----------

function mu3dTextColor(mu?: number): string {
  if (mu == null || mu >= 0.85) return "#e2e8f0";
  if (mu >= 0.5) return "#fbbf24";
  if (mu >= 0.3) return "#f87171";
  return "#a78bfa";
}

function HUD3D({ follow, onToggleFollow }: { follow: boolean; onToggleFollow: () => void }) {
  const strategy = useSimStore((s) => s.state?.strategy ?? "—");
  const vx = useSimStore((s) => s.state?.velocity.vx ?? 0);
  const yaw = useSimStore((s) => s.state?.velocity.yaw_rate ?? 0);
  const wheels = useSimStore((s) => s.state?.wheels);
  const att = useSimStore((s) => s.state?.attitude);
  const modelType = useSimStore((s) => s.state?.model_type);

  return (
    <div className="canvas-hud">
      <div className="hud-row">
        <span className="hud-label">策略</span>
        <span className="hud-value strong">{strategy}</span>
      </div>
      <div className="hud-row">
        <span className="hud-label">vₓ</span>
        <span className="hud-value mono">{fmtKmh(vx)} km/h</span>
      </div>
      <div className="hud-row">
        <span className="hud-label">ψ̇</span>
        <span className="hud-value mono">{((yaw * 180) / Math.PI).toFixed(1)} °/s</span>
      </div>
      {modelType === "multibody" && att && (
        <div className="hud-row">
          <span className="hud-label">侧倾/俯仰</span>
          <span className="hud-value mono small">
            φ{((att.roll * 180) / Math.PI).toFixed(1)}° / θ{((att.pitch * 180) / Math.PI).toFixed(1)}°
          </span>
        </div>
      )}
      {wheels && (
        <div className="hud-row">
          <span className="hud-label">μ</span>
          <span className="hud-value mono small">
            {wheels.map((w, i) => (
              <span key={i} style={{ color: mu3dTextColor(w.mu) }}>
                {(w.mu ?? 1).toFixed(2)}{i < 3 ? " / " : ""}
              </span>
            ))}
          </span>
        </div>
      )}
      <div className="hud-zoom">
        <span className="mono small">相机</span>
        <button onClick={onToggleFollow}>{follow ? "跟随中" : "自由"}</button>
      </div>
    </div>
  );
}
