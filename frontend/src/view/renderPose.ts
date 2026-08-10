/**
 * renderPose — a continuous vehicle pose for the 3D renderer.
 *
 * ## Why this exists
 *
 * The backend integrates at 200 Hz and pushes a state snapshot every
 * `round(dt_push / dt_sim) = 3` steps — 15 ms, i.e. **66.7 Hz**, not the 60 Hz
 * the constant reads as. The browser draws at its own refresh rate on its own
 * clock. Measured on the dev machine, arrivals were mean 15.0 ms with p90 16.7,
 * p99 19.6, min 9.1, max 20.7.
 *
 * The old renderer read `store.state.pose` straight into the mesh and camera
 * transforms inside `useFrame`. That quantises world motion to whenever a
 * packet happened to land:
 *
 *   * 66.7 Hz samples against a 60 Hz display beat at ~6.7 Hz — several times a
 *     second one frame repeats the previous pose and the next double-steps.
 *   * The ±5 ms arrival jitter adds an irregular component on top.
 *
 * In the orbit view that is a small wobble of a distant car. In the **roof
 * camera the whole world is rigidly welded to the pose**, so the same error
 * becomes the entire screen hitching — which is the "闪烁 / 不流畅" report.
 *
 * ## What it does
 *
 * Per rendered frame it rebuilds the pose on the *render* clock:
 *
 *   1. **Dead reckoning.** The newest sample carries body velocities
 *      (vx, vy, ψ̇), so the pose is integrated forward by the time elapsed since
 *      that sample landed. Between packets the world keeps moving.
 *   2. **Residual smoothing.** Each new sample corrects the extrapolation by a
 *      centimetre or so; approaching the target with time constant τ turns that
 *      step into a ramp.
 *   3. **Lag cancellation.** The target is extrapolated by the smoother's own
 *      steady-state lag, so the two cancel and the drawn car sits on the true
 *      pose rather than trailing it. That lag is `dt·(1−α)/α`, not τ: at a
 *      frame time comparable to τ the discrete filter is quicker than the
 *      continuous one it approximates, and compensating by τ would leave the
 *      render *ahead* of the simulation. Under transients the residual is
 *      ½·a·τ² ≈ 6 mm at 1 g — far below what the smoothing buys.
 *
 * The result is independent of the push cadence: a stream that degrades to
 * 20 Hz (a slow host, a coarse OS timer) still renders smoothly instead of
 * turning into visible steps.
 *
 * ## What it is not
 *
 * Render-only. Charts, the recorder, the trajectory trail and every exported
 * number keep using the raw samples — nothing here touches simulation output.
 * Extrapolation is capped and a teleport (reset, or a script jumping the pose)
 * snaps rather than sliding across the map.
 */

import type { SimStateMessage } from "@/types/sim";

/** Never dead-reckon further than this past the newest sample [s]. */
const EXTRAP_MAX = 0.08;
/**
 * The steer angle gets a tighter one. Coasting the *pose* through a stall reads
 * as the car continuing to roll, which it is; coasting δ reads as the wheels
 * continuing to turn on their own, which is a lie the driver would notice.
 */
const DELTA_EXTRAP_MAX = 0.04;
/** Residual-smoothing time constant for position/heading [s]. */
const TAU_POSE = 0.035;
/** Steer angles get a shorter one — they are small and want to feel immediate. */
const TAU_DELTA = 0.02;
/** Sanity cap on the δ rate estimated from consecutive samples [rad/s]. */
const DELTA_RATE_MAX = 25;
/** Per-sample pull of the clock offset toward a *later* observation. */
const OFFSET_CATCHUP = 0.01;
/** Beyond these the change is a teleport, not motion — snap instead of sliding. */
const JUMP_M = 3.0;
const JUMP_RAD = 1.0;

export interface RenderPose {
  x: number;
  y: number;
  psi: number;
  /** Per-wheel steer angle [rad], smoothed onto the render clock. */
  delta: number[];
}

/**
 * One vehicle's reconstructed pose.
 *
 * `sample()` is idempotent within a frame: react-three-fiber advances
 * `state.clock` once per frame *before* running any `useFrame` subscriber, so
 * every consumer in a frame passes the same `elapsed` and gets the same pose
 * back regardless of subscription order.
 */
class PoseTrack {
  // Newest sample from the backend.
  private simT = Number.NaN;
  private sx = 0; private sy = 0; private spsi = 0;
  private svx = 0; private svy = 0; private sr = 0;
  private sdelta = [0, 0, 0, 0];
  private sdeltaRate = [0, 0, 0, 0];

  /**
   * Render clock minus sim clock, for the least-delayed packet seen.
   *
   * The age of the newest sample is what the extrapolation horizon is made of,
   * and it must be measured, not assumed. Treating every sample as brand-new
   * on the frame it is first observed re-creates the very sawtooth this module
   * exists to remove: a 15 ms sample seen by a 16.7 ms display is on average
   * already 7.5 ms old, varying frame to frame. Packets are only ever *late*,
   * so the smallest observed offset is the best estimate of the true one — with
   * a slow pull toward later observations so a sim clock that falls behind wall
   * time is still tracked (see the update site).
   */
  private offset = Number.NaN;

  // Reconstructed output.
  private out: RenderPose = { x: 0, y: 0, psi: 0, delta: [0, 0, 0, 0] };
  private primed = false;

  // Frame memo.
  private frameKey = Number.NaN;

  reset(): void {
    this.simT = Number.NaN;
    this.offset = Number.NaN;
    this.primed = false;
    this.frameKey = Number.NaN;
  }

  sample(st: SimStateMessage, elapsed: number, dt: number): RenderPose {
    if (elapsed === this.frameKey) return this.out;
    this.frameKey = elapsed;

    // The renderer's first frame reports dt = 0, and a long tab-switch reports
    // a huge one. Both have to be fenced off before they reach a division:
    // this feeds the *camera* transform, so a single NaN does not degrade the
    // picture, it blanks the viewport for the rest of the session.
    const step = Math.min(Math.max(dt, 1e-4), 0.1);

    // --- latch a new backend sample -----------------------------------------
    if (st.t !== this.simT) {
      const rewound = !(st.t > this.simT);        // reset / replay rewind
      const dtSample = st.t - this.simT;
      for (let i = 0; i < 4; i++) {
        const d = st.wheels[i]?.delta ?? 0;
        // δ arrives without a rate, so estimate one across the sample interval
        // — otherwise the wheels can only be smoothed, which trades the steps
        // for a visible lag exactly when δ̇ is largest (turn-in).
        this.sdeltaRate[i] = rewound || !(dtSample > 1e-6)
          ? 0
          : Math.max(-DELTA_RATE_MAX,
                     Math.min(DELTA_RATE_MAX, (d - this.sdelta[i]) / dtSample));
        this.sdelta[i] = d;
      }
      this.simT = st.t;
      this.sx = st.pose.x;
      this.sy = st.pose.y;
      this.spsi = st.pose.psi;
      this.svx = st.velocity.vx;
      this.svy = st.velocity.vy;
      this.sr = st.velocity.yaw_rate;

      // Snap down to any earlier observation (that packet was less delayed, so
      // it is the better estimate); creep up slowly otherwise, which is what
      // recovers if the sim clock itself falls behind wall time — a backend
      // that can't hold 200 Hz advances sim time slower than the render clock,
      // and a pure minimum would leave us extrapolating ahead of it forever.
      const obs = elapsed - st.t;
      this.offset = rewound || !Number.isFinite(this.offset)
        ? obs
        : obs < this.offset
          ? obs
          : this.offset + (obs - this.offset) * OFFSET_CATCHUP;
      if (rewound) this.primed = false;
    }

    if (!Number.isFinite(this.offset)) {
      // Nothing to reckon from — hand back the raw sample.
      const o0 = this.out;
      o0.x = this.sx; o0.y = this.sy; o0.psi = this.spsi;
      for (let i = 0; i < 4; i++) o0.delta[i] = this.sdelta[i];
      this.primed = true;
      return o0;
    }

    // Age of the newest sample on the render clock, capped so a stalled stream
    // coasts to a stop rather than flying off the map.
    const age = Math.max(0, Math.min(elapsed - this.offset - this.simT, EXTRAP_MAX));

    // Smoothing gains, and the lag each one costs at this frame time. Feeding
    // the lag back into the extrapolation horizon is what makes the pair
    // net out to zero.
    const a = 1 - Math.exp(-step / TAU_POSE);
    const ad = 1 - Math.exp(-step / TAU_DELTA);

    // --- dead-reckon the target far enough ahead to cancel the smoother's lag -
    const h = age + step * (1 - a) / a;
    const psiT = this.spsi + this.sr * h;
    // Integrate the body velocity about the mid-heading over the horizon: at
    // yaw rates that matter this is visibly straighter than freezing ψ.
    const psiMid = this.spsi + this.sr * h * 0.5;
    const c = Math.cos(psiMid);
    const s = Math.sin(psiMid);
    const xT = this.sx + (this.svx * c - this.svy * s) * h;
    const yT = this.sy + (this.svx * s + this.svy * c) * h;
    const hd = Math.min(age, DELTA_EXTRAP_MAX) + step * (1 - ad) / ad;

    const o = this.out;
    // A non-finite reckoning (a diverged sim, a malformed packet) must never
    // reach a transform — fall back to the raw sample instead.
    if (!Number.isFinite(xT) || !Number.isFinite(yT) || !Number.isFinite(psiT)) {
      o.x = this.sx; o.y = this.sy; o.psi = this.spsi;
      for (let i = 0; i < 4; i++) o.delta[i] = this.sdelta[i];
      this.primed = true;
      return o;
    }

    const teleported =
      Math.abs(xT - o.x) > JUMP_M ||
      Math.abs(yT - o.y) > JUMP_M ||
      Math.abs(psiT - o.psi) > JUMP_RAD;

    if (!this.primed || teleported) {
      this.primed = true;
      o.x = xT; o.y = yT; o.psi = psiT;
      for (let i = 0; i < 4; i++) o.delta[i] = this.sdelta[i];
      return o;
    }

    o.x += (xT - o.x) * a;
    o.y += (yT - o.y) * a;
    o.psi += (psiT - o.psi) * a;

    for (let i = 0; i < 4; i++) {
      o.delta[i] += (this.sdelta[i] + this.sdeltaRate[i] * hd - o.delta[i]) * ad;
    }
    return o;
  }
}

export type { PoseTrack };

/** A fresh tracker — one per vehicle. Exposed so the behaviour can be tested. */
export function createPoseTrack(): PoseTrack {
  return new PoseTrack();
}

/** The ego vehicle — shared by the body, the wheels, the ICR markers and the cameras. */
export const egoPose = createPoseTrack();
