// Shared vehicle silhouette + styling, consumed by BOTH Canvas2D and Canvas3D.
//
// Keeping the outline in one place means the top-down view and the extruded
// 3D shell are literally the same shape, so switching viewports doesn't feel
// like switching vehicles.
//
// All helpers return points in BODY FRAME metres (x forward, y left).
// Renderers handle their own frame flips.

// ---------------------------------------------------------------------------
// Outline
// ---------------------------------------------------------------------------

/**
 * Half-outline of a generic 3-box vehicle seen from above, normalised to
 * [-0.5, +0.5] on both axes. Traced from the nose centre along the LEFT
 * side (+y) to the tail centre; the full outline mirrors it.
 */
const HALF: [number, number][] = [
  [0.500, 0.000],
  [0.494, 0.168],
  [0.473, 0.306],
  [0.436, 0.404],
  [0.375, 0.458],
  [0.280, 0.489],
  [0.095, 0.500],
  [-0.158, 0.500],
  [-0.332, 0.490],
  [-0.423, 0.463],
  [-0.471, 0.412],
  [-0.493, 0.295],
  [-0.500, 0.104],
  [-0.500, 0.000],
];

/** Cabin / greenhouse footprint, same normalisation as HALF. */
const CABIN_HALF: [number, number][] = [
  [0.190, 0.000],
  [0.176, 0.132],
  [0.136, 0.252],
  [0.062, 0.320],
  [-0.062, 0.350],
  [-0.194, 0.346],
  [-0.270, 0.310],
  [-0.302, 0.228],
  [-0.312, 0.098],
  [-0.312, 0.000],
];

function mirror(half: [number, number][], length: number, width: number): [number, number][] {
  const left = half.map(([u, v]) => [u * length, v * width] as [number, number]);
  const right = [...half]
    .reverse()
    .slice(1, -1)                       // don't duplicate nose / tail centre
    .map(([u, v]) => [u * length, -v * width] as [number, number]);
  return [...left, ...right];
}

/** Closed body outline. */
export function bodyOutline(length: number, width: number): [number, number][] {
  return mirror(HALF, length, width);
}

/** Closed greenhouse outline (sits inside the body). */
export function cabinOutline(length: number, width: number): [number, number][] {
  return mirror(CABIN_HALF, length, width);
}

/** Windshield quad at the front edge of the greenhouse. */
export function windshieldQuad(length: number, width: number): [number, number][] {
  return [
    [0.176 * length, 0.128 * width],
    [0.136 * length, 0.248 * width],
    [0.136 * length, -0.248 * width],
    [0.176 * length, -0.128 * width],
  ];
}

/** Lamp centres in body frame: two head lamps up front, two tail lamps aft. */
export function lampPositions(length: number, width: number) {
  return {
    head: [
      [0.464 * length, +0.328 * width],
      [0.464 * length, -0.328 * width],
    ] as [number, number][],
    tail: [
      [-0.476 * length, +0.350 * width],
      [-0.476 * length, -0.350 * width],
    ] as [number, number][],
  };
}

/**
 * Drawable body dimensions from wire parameters. Overhangs scale with the
 * wheelbase so short-wheelbase profiles (delivery_robot) stay proportionate.
 */
export function bodyDimensions(wheelbase: number, trackMax: number) {
  return {
    length: wheelbase * 1.20,
    width: trackMax * 1.14,
  };
}

// ---------------------------------------------------------------------------
// Styling
// ---------------------------------------------------------------------------

/** Wheel proportions shared by 2D rects and 3D cylinders. */
export const WHEEL = {
  widthRatio: 0.33,   // tyre width / tyre diameter
  rimRatio: 0.56,     // rim diameter / tyre diameter
  hubRatio: 0.19,
  spokes: 5,
} as const;

export interface VehiclePalette {
  shellLight: string;
  shellMid: string;
  shellDark: string;
  edge: string;
  cabinFill: string;
  cabinEdge: string;
  glass: string;
  crease: string;
  shadow: string;
  headlight: string;
  headlightGlow: string;
  taillight: string;
  rim: string;
  hub: string;
  tyreEdge: string;
}

/** Body palettes per UI theme — dark is the primary, light is a daylight variant. */
export const PALETTE: Record<"dark" | "light", VehiclePalette> = {
  dark: {
    shellLight: "#3d7fc4",
    shellMid: "#245484",
    shellDark: "#15304e",
    edge: "#60a5fa",
    cabinFill: "rgba(147,197,253,0.20)",
    cabinEdge: "rgba(191,219,254,0.42)",
    glass: "rgba(191,219,254,0.34)",
    crease: "rgba(191,219,254,0.20)",
    shadow: "rgba(0,0,0,0.45)",
    headlight: "#fef3c7",
    headlightGlow: "rgba(254,243,199,0.30)",
    taillight: "#dc2626",
    rim: "#94a3b8",
    hub: "#e2e8f0",
    tyreEdge: "#cbd5e1",
  },
  light: {
    shellLight: "#7fb3ea",
    shellMid: "#4a8fd4",
    shellDark: "#2c6cb0",
    edge: "#1d4ed8",
    cabinFill: "rgba(30,64,175,0.16)",
    cabinEdge: "rgba(30,64,175,0.40)",
    glass: "rgba(96,165,250,0.30)",
    crease: "rgba(30,58,138,0.22)",
    shadow: "rgba(15,23,42,0.28)",
    headlight: "#fde68a",
    headlightGlow: "rgba(252,211,77,0.35)",
    taillight: "#b91c1c",
    rim: "#475569",
    hub: "#1e293b",
    tyreEdge: "#334155",
  },
};
