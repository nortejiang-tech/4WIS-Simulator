/**
 * RoofCamera — the driving view: a rig bolted to the vehicle body, above the
 * roof, aimed forward.
 *
 * It replaces the orbit controls entirely while active (OrbitControls is
 * unmounted by the parent, which also saves/restores the orbit pose), and
 * drives the shared perspective camera imperatively inside useFrame so the
 * React tree still doesn't re-render at 60 Hz.
 *
 * Two details that matter for driving rather than sightseeing:
 *
 *   * The rig tracks the **body heading**, not the velocity vector. On a 4WIS
 *     vehicle those differ — in a crab manoeuvre the car travels sideways while
 *     pointing straight ahead — and what a driver sees is where the car points.
 *   * Roll/pitch feed through at a user-set fraction (`attitude`). Full body
 *     attitude tilts the horizon so much that judging a cone gate gets harder,
 *     but zero makes a loaded-up chassis feel inert. The default is a third.
 *
 * See view/roofCamera.ts for the rig geometry and its persisted config.
 */

import { useEffect, useRef } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import { PerspectiveCamera, Vector3 } from "three";

import { useSimStore } from "@/store/sim";
import { egoPose } from "@/view/renderPose";

const DEG = Math.PI / 180;

export default function RoofCamera() {
  const { camera } = useThree();
  // Only the *heading* is filtered — the rig stays bolted to the body, so the
  // camera can never trail the car it is mounted on. Heading is carried as a
  // unit vector so it filters without ±π wrap-around artefacts.
  const head = useRef<{ cx: number; sy: number } | null>(null);
  const att = useRef({ roll: 0, pitch: 0, z: 0 });
  const aim = useRef(new Vector3());
  const restoreFov = useRef(50);

  useEffect(() => {
    const cam = camera as PerspectiveCamera;
    restoreFov.current = cam.fov;
    return () => {
      cam.fov = restoreFov.current;
      cam.up.set(0, 1, 0);
      cam.updateProjectionMatrix();
    };
  }, [camera]);

  useFrame((rs, dt) => {
    const store = useSimStore.getState();
    const st = store.state;
    if (!st) return;
    const cfg = store.roofCam;
    const cam = camera as PerspectiveCamera;

    // The rig is welded to the body, so it inherits every artefact of the pose
    // it is given. Reading the raw 66.7 Hz sample made each packet boundary a
    // whole-screen jump; the reconstruction in view/renderPose puts the rig on
    // the render clock instead.
    const { x, y, psi } = egoPose.sample(st, rs.clock.elapsedTime, dt);

    // --- heading filter ----------------------------------------------------
    const cx = Math.cos(psi), sy = Math.sin(psi);
    const h0 = head.current;
    if (!h0) {
      head.current = { cx, sy };
    } else {
      const alpha = cfg.smooth > 1e-3
        ? 1 - Math.exp(-Math.min(dt, 0.1) / cfg.smooth)
        : 1;
      h0.cx += (cx - h0.cx) * alpha;
      h0.sy += (sy - h0.sy) * alpha;
      const n = Math.hypot(h0.cx, h0.sy) || 1;
      h0.cx /= n;
      h0.sy /= n;
    }
    const q = { x, y, ...head.current! };

    // Body attitude reaches the camera only at the configured share, and is
    // filtered on the same time constant so it can't add jitter of its own.
    const a = st.attitude;
    const aAlpha = 1 - Math.exp(-Math.min(dt, 0.1) / 0.08);
    att.current.roll += ((a?.roll ?? 0) - att.current.roll) * aAlpha;
    att.current.pitch += ((a?.pitch ?? 0) - att.current.pitch) * aAlpha;
    att.current.z += ((a?.z ?? 0) - att.current.z) * aAlpha;
    const g = cfg.attitude;

    // --- place the rig -----------------------------------------------------
    // Body offset (−back along heading), then world → three: (x, h, −y).
    const wx = q.x - cfg.back * q.cx;
    const wy = q.y - cfg.back * q.sy;
    const h = cfg.height + att.current.z * g;
    cam.position.set(wx, h, -wy);

    // Aim: forward, pitched down; a nose-up body attitude lifts the aim.
    const pitch = cfg.pitch * DEG - att.current.pitch * g;
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    aim.current.set(wx + q.cx * cp, h - sp, -(wy + q.sy * cp));
    cam.up.set(0, 1, 0);
    cam.lookAt(aim.current);
    // Body roll tilts the horizon. The camera looks along its local −Z, so a
    // rotation of +roll about the forward axis is −roll about local Z.
    if (g > 0 && att.current.roll !== 0) cam.rotateZ(-att.current.roll * g);

    if (Math.abs(cam.fov - cfg.fov) > 1e-3) {
      cam.fov = cfg.fov;
      cam.updateProjectionMatrix();
    }
  });

  return null;
}
