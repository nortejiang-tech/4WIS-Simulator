// Speed unit helpers. Internally everything is m/s (matches the backend);
// convert only at display / input boundaries.

export const MS_TO_KMH = 3.6;

/** metres-per-second → kilometres-per-hour */
export const toKmh = (ms: number): number => ms * MS_TO_KMH;

/** kilometres-per-hour → metres-per-second */
export const fromKmh = (kmh: number): number => kmh / MS_TO_KMH;

/** Format a m/s value as a km/h string with the given decimals. */
export const fmtKmh = (ms: number, digits = 1): string => toKmh(ms).toFixed(digits);
