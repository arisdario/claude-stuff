/**
 * CurveCue Engine -- JavaScript Port
 *
 * Complete curve detection and classification pipeline for route analysis.
 * Ported from the Kotlin engine module of the CurveCue Android app.
 *
 * Pipeline stages:
 *   1. Interpolation -- resample to uniform spacing
 *   2. Curvature -- Menger radius + rolling average smoothing + outlier rejection
 *   3. Segmentation -- curve vs straight, merge short gaps
 *   4. Classification -- severity, direction, modifiers, angle change
 *   5. Speed advisory -- physics-based recommended speeds
 *   6. Lean angle -- motorcycle cornering angle
 *   7. Compound detection -- S-bends, chicanes, series, tightening sequences
 *   8. Narration -- human-readable TTS text generation
 *
 * @module engine
 */

// ============================================================================
// Constants
// ============================================================================

/** Earth mean radius in meters (WGS-84). */
export const EARTH_RADIUS_M = 6_371_000.0;

/** Standard gravity in m/s squared. */
export const GRAVITY = 9.81;

/** Conversion factor: m/s to km/h. */
export const MS_TO_KMH = 3.6;

// ============================================================================
// Enums (as frozen string-constant objects)
// ============================================================================

/**
 * Curve direction.
 * @enum {string}
 */
export const Direction = Object.freeze({
  LEFT: 'LEFT',
  RIGHT: 'RIGHT',
});

/**
 * Curve severity based on minimum radius.
 * Ordered from least severe (GENTLE) to most severe (HAIRPIN).
 * @enum {string}
 */
export const Severity = Object.freeze({
  GENTLE:   'GENTLE',    // R > 200 m
  MODERATE: 'MODERATE',  // 100 < R <= 200 m
  FIRM:     'FIRM',      //  50 < R <= 100 m
  SHARP:    'SHARP',     //  25 < R <=  50 m
  HAIRPIN:  'HAIRPIN',   //       R <=  25 m
});

/**
 * Curve modifiers describing how the curve geometry changes.
 * @enum {string}
 */
export const CurveModifier = Object.freeze({
  TIGHTENING: 'TIGHTENING',
  OPENING:    'OPENING',
  HOLDS:      'HOLDS',
  LONG:       'LONG',
});

/**
 * Compound curve pattern types.
 * @enum {string}
 */
export const CompoundType = Object.freeze({
  S_BEND:              'S_BEND',
  CHICANE:             'CHICANE',
  SERIES:              'SERIES',
  TIGHTENING_SEQUENCE: 'TIGHTENING_SEQUENCE',
});

// ============================================================================
// Core Math Utilities
// ============================================================================

/**
 * Converts degrees to radians.
 * @param {number} deg - Angle in degrees.
 * @returns {number} Angle in radians.
 */
export function toRadians(deg) {
  return deg * (Math.PI / 180);
}

/**
 * Converts radians to degrees.
 * @param {number} rad - Angle in radians.
 * @returns {number} Angle in degrees.
 */
export function toDegrees(rad) {
  return rad * (180 / Math.PI);
}

/**
 * Computes the great-circle distance between two points using the Haversine formula.
 * @param {{lat: number, lon: number}} p1 - First point.
 * @param {{lat: number, lon: number}} p2 - Second point.
 * @returns {number} Distance in meters.
 */
export function haversineDistance(p1, p2) {
  const lat1 = toRadians(p1.lat);
  const lat2 = toRadians(p2.lat);
  const dLat = toRadians(p2.lat - p1.lat);
  const dLon = toRadians(p2.lon - p1.lon);

  const a =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return EARTH_RADIUS_M * c;
}

/**
 * Computes the initial bearing (forward azimuth) from one point to another.
 * @param {{lat: number, lon: number}} from - Origin point.
 * @param {{lat: number, lon: number}} to - Destination point.
 * @returns {number} Bearing in degrees, normalized to [0, 360).
 */
export function bearing(from, to) {
  const lat1 = toRadians(from.lat);
  const lat2 = toRadians(to.lat);
  const dLon = toRadians(to.lon - from.lon);

  const x = Math.sin(dLon) * Math.cos(lat2);
  const y =
    Math.cos(lat1) * Math.sin(lat2) -
    Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);

  return (toDegrees(Math.atan2(x, y)) + 360) % 360;
}

/**
 * Computes the signed angular difference between two bearings.
 * Positive = clockwise (right), Negative = counter-clockwise (left).
 * @param {number} b1 - First bearing in degrees.
 * @param {number} b2 - Second bearing in degrees.
 * @returns {number} Angle in degrees, in the range (-180, 180].
 */
export function bearingDifference(b1, b2) {
  let diff = b2 - b1;
  while (diff > 180) diff -= 360;
  while (diff <= -180) diff += 360;
  return diff;
}

/**
 * Spherical interpolation along the great-circle path from p1 to p2.
 * @param {{lat: number, lon: number}} p1 - Start point.
 * @param {{lat: number, lon: number}} p2 - End point.
 * @param {number} fraction - 0.0 returns p1, 1.0 returns p2.
 * @returns {{lat: number, lon: number}} Interpolated point.
 * @private
 */
function interpolatePoint(p1, p2, fraction) {
  if (fraction <= 0) return { lat: p1.lat, lon: p1.lon };
  if (fraction >= 1) return { lat: p2.lat, lon: p2.lon };

  const lat1 = toRadians(p1.lat);
  const lon1 = toRadians(p1.lon);
  const lat2 = toRadians(p2.lat);
  const lon2 = toRadians(p2.lon);

  // Angular distance in radians
  const d = haversineDistance(p1, p2) / EARTH_RADIUS_M;
  if (d < 1e-12) return { lat: p1.lat, lon: p1.lon };

  const a = Math.sin((1 - fraction) * d) / Math.sin(d);
  const b = Math.sin(fraction * d) / Math.sin(d);

  const x = a * Math.cos(lat1) * Math.cos(lon1) + b * Math.cos(lat2) * Math.cos(lon2);
  const y = a * Math.cos(lat1) * Math.sin(lon1) + b * Math.cos(lat2) * Math.sin(lon2);
  const z = a * Math.sin(lat1) + b * Math.sin(lat2);

  return {
    lat: toDegrees(Math.atan2(z, Math.sqrt(x * x + y * y))),
    lon: toDegrees(Math.atan2(y, x)),
  };
}

/**
 * Projects a point onto the closest position on a line segment, returning
 * the fraction along the segment and the perpendicular distance.
 * Uses a flat-earth approximation for the projection since segments are short.
 * @param {{lat: number, lon: number}} point - The point to project.
 * @param {{lat: number, lon: number}} segStart - Segment start.
 * @param {{lat: number, lon: number}} segEnd - Segment end.
 * @returns {{fraction: number, distance: number}} Fraction along segment [0,1] and distance in meters.
 * @private
 */
function projectOntoSegment(point, segStart, segEnd) {
  const dTotal = haversineDistance(segStart, segEnd);
  if (dTotal < 1e-6) {
    return { fraction: 0, distance: haversineDistance(point, segStart) };
  }

  const cosLat = Math.cos(toRadians(segStart.lat));
  const dx1 = (segEnd.lon - segStart.lon) * cosLat;
  const dy1 = segEnd.lat - segStart.lat;
  const dx2 = (point.lon - segStart.lon) * cosLat;
  const dy2 = point.lat - segStart.lat;

  const dot = dx1 * dx2 + dy1 * dy2;
  const lenSq = dx1 * dx1 + dy1 * dy1;
  const t = lenSq < 1e-18 ? 0 : Math.max(0, Math.min(1, dot / lenSq));

  const projected = interpolatePoint(segStart, segEnd, t);
  return { fraction: t, distance: haversineDistance(point, projected) };
}

// ============================================================================
// Menger Curvature
// ============================================================================

/**
 * Computes the Menger radius -- the circumscribed circle radius through three points.
 * Uses Heron's formula for the triangle area: R = abc / (4 * area).
 * @param {{lat: number, lon: number}} p1 - First point.
 * @param {{lat: number, lon: number}} p2 - Second (center) point.
 * @param {{lat: number, lon: number}} p3 - Third point.
 * @returns {number} Radius in meters. Infinity if points are collinear.
 */
export function mengerRadius(p1, p2, p3) {
  const a = haversineDistance(p1, p2);
  const b = haversineDistance(p2, p3);
  const c = haversineDistance(p1, p3);

  const s = (a + b + c) / 2;
  const areaSquared = s * (s - a) * (s - b) * (s - c);

  // Guard against negative values from floating-point imprecision
  if (areaSquared <= 0) return Infinity;

  const area = Math.sqrt(areaSquared);
  return area < 1e-10 ? Infinity : (a * b * c) / (4 * area);
}

/**
 * Determines the curve direction from the cross product of vectors p1->p2 and p2->p3
 * on a local tangent plane centered at p2.
 * Positive cross product (counter-clockwise turn) = LEFT.
 * Negative cross product (clockwise turn) = RIGHT.
 * @param {{lat: number, lon: number}} p1 - First point.
 * @param {{lat: number, lon: number}} p2 - Second (center) point.
 * @param {{lat: number, lon: number}} p3 - Third point.
 * @returns {string|null} 'LEFT', 'RIGHT', or null if collinear.
 */
export function mengerDirection(p1, p2, p3) {
  const cosLat = Math.cos(toRadians(p2.lat));
  const degToM = EARTH_RADIUS_M * Math.PI / 180;

  // Vector from p1 to p2 in local coordinates (approximate meters)
  const v1x = (p2.lon - p1.lon) * cosLat * degToM;
  const v1y = (p2.lat - p1.lat) * degToM;

  // Vector from p2 to p3 in local coordinates
  const v2x = (p3.lon - p2.lon) * cosLat * degToM;
  const v2y = (p3.lat - p2.lat) * degToM;

  // Cross product z-component
  const cross = v1x * v2y - v1y * v2x;

  if (cross > 1e-6) return Direction.LEFT;
  if (cross < -1e-6) return Direction.RIGHT;
  return null;
}

// ============================================================================
// Stage 1: Interpolation
// ============================================================================

/**
 * Resamples a polyline of geographic points to uniform spacing along the
 * great-circle path. The first point is always included. The last point is
 * included if the remaining distance from the last emitted point is greater
 * than half the spacing.
 * @param {{lat: number, lon: number}[]} points - Ordered route coordinates (>= 2 points).
 * @param {number} [spacing=5.0] - Target distance between consecutive output points in meters.
 * @returns {{lat: number, lon: number}[]} Uniformly spaced points.
 * @throws {Error} If fewer than 2 points are provided or spacing is not positive.
 */
export function interpolateRoute(points, spacing = 5.0) {
  if (points.length < 2) {
    throw new Error(`Need at least 2 points to interpolate, got ${points.length}`);
  }
  if (spacing <= 0) {
    throw new Error(`Spacing must be positive, got ${spacing}`);
  }

  const result = [{ lat: points[0].lat, lon: points[0].lon }];

  let segmentIndex = 0;
  let distanceAlongCurrentSegment = 0;
  let remaining = spacing;

  while (segmentIndex < points.length - 1) {
    const segStart = points[segmentIndex];
    const segEnd = points[segmentIndex + 1];
    const segLength = haversineDistance(segStart, segEnd);

    let positionInSegment = distanceAlongCurrentSegment;

    while (positionInSegment + remaining <= segLength) {
      positionInSegment += remaining;
      const fraction = Math.max(0, Math.min(1, positionInSegment / segLength));
      result.push(interpolatePoint(segStart, segEnd, fraction));
      remaining = spacing;
    }

    // Consume part of the spacing budget in this segment
    remaining -= (segLength - positionInSegment);
    distanceAlongCurrentSegment = 0;
    segmentIndex++;
  }

  // Include the last point if not too close to the last emitted point
  const lastEmitted = result[result.length - 1];
  const lastOriginal = points[points.length - 1];
  if (haversineDistance(lastEmitted, lastOriginal) > spacing * 0.5) {
    result.push({ lat: lastOriginal.lat, lon: lastOriginal.lon });
  }

  return result;
}

// ============================================================================
// Stage 2: Curvature Computation
// ============================================================================

/**
 * Outlier rejection constants.
 * @private
 */
const SPIKE_RATIO = 0.2;
const SPIKE_NEIGHBOR_MIN_RADIUS = 100;
const MAX_LATERAL_DEVIATION = 15;
const RADIUS_CAP = 10_000;

/**
 * Applies a rolling average (centered window) to a numeric array.
 * Edge elements use a smaller window automatically.
 * @param {number[]} values - Input values.
 * @param {number} windowSize - Window size for averaging.
 * @returns {number[]} Smoothed values.
 * @private
 */
function rollingAverage(values, windowSize) {
  if (windowSize <= 1) return values.slice();

  const halfWindow = Math.floor(windowSize / 2);
  const result = new Array(values.length);

  for (let i = 0; i < values.length; i++) {
    const from = Math.max(0, i - halfWindow);
    const to = Math.min(values.length - 1, i + halfWindow);
    let sum = 0;
    for (let j = from; j <= to; j++) {
      sum += values[j];
    }
    result[i] = sum / (to - from + 1);
  }

  return result;
}

/**
 * Detects and repairs single-point GPS spikes in the raw radius array.
 *
 * A spike is identified when either:
 * 1. The point's radius is < SPIKE_RATIO of the median of its 4 nearest
 *    neighbors, AND the neighbor median is above SPIKE_NEIGHBOR_MIN_RADIUS
 *    (so we don't accidentally smooth genuine hairpins), AND both immediate
 *    neighbors are normal (isolated spike, not curve entry).
 * 2. The point deviates > MAX_LATERAL_DEVIATION from the line between its
 *    neighbors, indicating a GPS position jump.
 *
 * Repaired points get the neighbor median radius. Modifies rawRadii in place.
 * @param {number[]} rawRadii - Raw radius values (mutated in place).
 * @param {{lat: number, lon: number}[]} points - Route points.
 * @private
 */
function rejectOutliers(rawRadii, points) {
  // Need at least 2 neighbors on each side
  if (rawRadii.length < 5) return;

  for (let i = 2; i < rawRadii.length - 2; i++) {
    const neighbors = [
      rawRadii[i - 2], rawRadii[i - 1], rawRadii[i + 1], rawRadii[i + 2],
    ];
    neighbors.sort((a, b) => a - b);
    const neighborMedian = (neighbors[1] + neighbors[2]) / 2;

    // Check 1: radius spike -- isolated tight radius surrounded by normal radii
    const isIsolated =
      rawRadii[i - 1] > neighborMedian * 0.5 &&
      rawRadii[i + 1] > neighborMedian * 0.5;
    const isRadiusSpike =
      rawRadii[i] < neighborMedian * SPIKE_RATIO &&
      neighborMedian > SPIKE_NEIGHBOR_MIN_RADIUS &&
      isIsolated;

    // Check 2: lateral deviation -- point far from line between neighbors
    const { distance: lateralDist } = projectOntoSegment(
      points[i], points[i - 1], points[i + 1]
    );
    const isPositionSpike = lateralDist > MAX_LATERAL_DEVIATION;

    if (isRadiusSpike || isPositionSpike) {
      rawRadii[i] = neighborMedian;
    }
  }
}

/**
 * Computes curvature at each point along a uniformly-spaced route.
 *
 * For each triplet (i-1, i, i+1), the Menger curvature radius and direction
 * are computed. Endpoints inherit values from their adjacent interior points.
 * GPS outliers are rejected, large radii are capped, and a rolling average
 * is applied for smoothing.
 *
 * @param {{lat: number, lon: number}[]} points - Uniformly spaced route points (>= 3).
 * @param {number} [window=5] - Rolling average smoothing window size.
 * @returns {{radius: number, rawRadius: number, direction: string|null, point: {lat: number, lon: number}}[]}
 *   Array of curvature data, same length as input points.
 * @throws {Error} If fewer than 3 points are provided.
 */
export function computeCurvature(points, window = 5) {
  if (points.length < 3) {
    throw new Error(`Need at least 3 points for curvature, got ${points.length}`);
  }

  const n = points.length;
  const rawRadii = new Array(n);
  const directions = new Array(n);

  // Compute raw Menger curvature for interior points
  for (let i = 1; i < n - 1; i++) {
    rawRadii[i] = mengerRadius(points[i - 1], points[i], points[i + 1]);
    directions[i] = mengerDirection(points[i - 1], points[i], points[i + 1]);
  }

  // Endpoints inherit from their neighbors
  rawRadii[0] = rawRadii[1];
  rawRadii[n - 1] = rawRadii[n - 2];
  directions[0] = directions[1];
  directions[n - 1] = directions[n - 2];

  // Reject GPS outliers before smoothing
  rejectOutliers(rawRadii, points);

  // Cap large radii to avoid distortion, then smooth
  const capped = rawRadii.map((r) => Math.min(r, RADIUS_CAP));
  const smoothed = rollingAverage(capped, window);

  return points.map((point, i) => ({
    radius: smoothed[i],
    rawRadius: rawRadii[i],
    direction: directions[i],
    point,
  }));
}

// ============================================================================
// Stage 3: Segmentation
// ============================================================================

/**
 * Computes the polyline length between two point indices.
 * @param {{lat: number, lon: number}[]} points - Route points.
 * @param {number} start - Start index (inclusive).
 * @param {number} end - End index (inclusive).
 * @returns {number} Length in meters.
 * @private
 */
function segmentLength(points, start, end) {
  let len = 0;
  for (let i = start; i < end; i++) {
    len += haversineDistance(points[i], points[i + 1]);
  }
  return len;
}

/**
 * Segments the route into alternating curve and straight sections.
 *
 * A point is considered "in a curve" if its smoothed radius is below the
 * curvature threshold. Consecutive curve points form a curve segment. Short
 * straight gaps between curves (below gapMerge meters) are merged into a
 * single compound curve segment.
 *
 * @param {{radius: number, direction: string|null}[]} curvaturePoints - Output from computeCurvature.
 * @param {{lat: number, lon: number}[]} points - Interpolated route points (same indices).
 * @param {number} [threshold=500] - Radius below which a point is "in a curve" (meters).
 * @param {number} [gapMerge=50] - Straight gaps shorter than this are merged into adjacent curves (meters).
 * @returns {{startIndex: number, endIndex: number, isCurve: boolean}[]} Ordered raw segments.
 */
export function segmentRoute(curvaturePoints, points, threshold = 500, gapMerge = 50) {
  if (curvaturePoints.length === 0) return [];

  // Step 1: Mark each point as curve or straight
  const isCurve = curvaturePoints.map((cp) => cp.radius < threshold);

  // Step 2: Build initial segments from consecutive same-type runs
  const segments = [];
  let currentStart = 0;
  let currentType = isCurve[0];

  for (let i = 1; i < isCurve.length; i++) {
    if (isCurve[i] !== currentType) {
      segments.push({ startIndex: currentStart, endIndex: i - 1, isCurve: currentType });
      currentStart = i;
      currentType = isCurve[i];
    }
  }
  segments.push({ startIndex: currentStart, endIndex: isCurve.length - 1, isCurve: currentType });

  // Step 3: Merge short straight gaps between curves
  if (segments.length <= 1) return segments;

  const merged = [];
  let idx = 0;

  while (idx < segments.length) {
    const current = segments[idx];

    if (!current.isCurve) {
      const prev = merged.length > 0 ? merged[merged.length - 1] : null;
      const next = idx + 1 < segments.length ? segments[idx + 1] : null;

      if (prev && prev.isCurve && next && next.isCurve) {
        const gapLen = segmentLength(points, current.startIndex, current.endIndex);
        if (gapLen < gapMerge) {
          // Merge: remove the previous curve, skip this straight,
          // and merge with the next curve
          merged.pop();
          merged.push({ startIndex: prev.startIndex, endIndex: next.endIndex, isCurve: true });
          idx += 2; // skip both the straight and the next curve
          continue;
        }
      }
    }

    merged.push(current);
    idx++;
  }

  return merged;
}

// ============================================================================
// Stage 4: Classification
// ============================================================================

/**
 * Maps a minimum radius to a severity level using configurable thresholds.
 * @param {number} minRadius - Minimum smoothed radius in meters.
 * @param {{gentle: number, moderate: number, firm: number, sharp: number}} [thresholds]
 *   Radius thresholds for each severity level.
 * @returns {string} One of the Severity enum values.
 */
export function classifySeverity(
  minRadius,
  thresholds = { gentle: 200, moderate: 100, firm: 50, sharp: 25 }
) {
  if (minRadius > thresholds.gentle) return Severity.GENTLE;
  if (minRadius > thresholds.moderate) return Severity.MODERATE;
  if (minRadius > thresholds.firm) return Severity.FIRM;
  if (minRadius > thresholds.sharp) return Severity.SHARP;
  return Severity.HAIRPIN;
}

/**
 * Determines the dominant curve direction across a range of curvature points
 * by majority vote of LEFT vs RIGHT.
 * @param {{direction: string|null}[]} curvaturePoints
 * @param {number} startIdx - Start index (inclusive).
 * @param {number} endIdx - End index (inclusive).
 * @returns {string} 'LEFT' or 'RIGHT'.
 * @private
 */
function computeDirection(curvaturePoints, startIdx, endIdx) {
  let leftCount = 0;
  let rightCount = 0;

  for (let i = startIdx; i <= endIdx; i++) {
    if (curvaturePoints[i].direction === Direction.LEFT) leftCount++;
    else if (curvaturePoints[i].direction === Direction.RIGHT) rightCount++;
  }

  return leftCount >= rightCount ? Direction.LEFT : Direction.RIGHT;
}

/**
 * Finds the minimum smoothed radius within a segment range.
 * @param {{radius: number}[]} curvaturePoints
 * @param {number} startIdx - Start index (inclusive).
 * @param {number} endIdx - End index (inclusive).
 * @returns {number} Minimum radius in meters.
 * @private
 */
function computeMinRadius(curvaturePoints, startIdx, endIdx) {
  let min = Infinity;
  for (let i = startIdx; i <= endIdx; i++) {
    if (curvaturePoints[i].radius < min) {
      min = curvaturePoints[i].radius;
    }
  }
  return min;
}

/**
 * Computes the total arc length of a segment by summing point-to-point distances.
 * @param {{lat: number, lon: number}[]} points
 * @param {number} startIdx - Start index (inclusive).
 * @param {number} endIdx - End index (inclusive).
 * @returns {number} Arc length in meters.
 * @private
 */
function computeArcLength(points, startIdx, endIdx) {
  let len = 0;
  for (let i = startIdx; i < endIdx; i++) {
    len += haversineDistance(points[i], points[i + 1]);
  }
  return len;
}

/**
 * Computes the average smoothed radius over a range, capping large values
 * at RADIUS_CAP to avoid skewing the average in straight sections.
 * @param {{radius: number}[]} curvaturePoints
 * @param {number} from - Start index (inclusive).
 * @param {number} to - End index (inclusive).
 * @returns {number} Average radius in meters.
 * @private
 */
function averageRadius(curvaturePoints, from, to) {
  let sum = 0;
  let count = 0;
  for (let i = from; i <= to; i++) {
    sum += Math.min(curvaturePoints[i].radius, RADIUS_CAP);
    count++;
  }
  return count > 0 ? sum / count : Infinity;
}

/**
 * Determines curve modifiers: TIGHTENING, OPENING, HOLDS, LONG.
 *
 * - TIGHTENING: avg radius of last third < avg radius of first third * 0.8
 * - OPENING: avg radius of last third > avg radius of first third * 1.2
 * - LONG: arc length > 200m
 * - HOLDS: LONG + neither TIGHTENING nor OPENING (constant radius)
 *
 * @param {{radius: number}[]} curvaturePoints
 * @param {number} startIdx
 * @param {number} endIdx
 * @param {number} arcLength
 * @returns {Set<string>} Set of CurveModifier values.
 * @private
 */
function computeModifiers(curvaturePoints, startIdx, endIdx, arcLength) {
  const mods = new Set();
  const count = endIdx - startIdx + 1;
  if (count < 3) return mods;

  const thirdSize = Math.floor(count / 3);
  if (thirdSize < 1) return mods;

  const firstThirdAvg = averageRadius(curvaturePoints, startIdx, startIdx + thirdSize - 1);
  const lastThirdAvg = averageRadius(curvaturePoints, endIdx - thirdSize + 1, endIdx);

  // Tightening: last third has significantly smaller radius (tighter)
  if (lastThirdAvg < firstThirdAvg * 0.8) {
    mods.add(CurveModifier.TIGHTENING);
  } else if (lastThirdAvg > firstThirdAvg * 1.2) {
    mods.add(CurveModifier.OPENING);
  }

  if (arcLength > 200) {
    mods.add(CurveModifier.LONG);

    // HOLDS: long arc with no significant tightening or opening
    if (!mods.has(CurveModifier.TIGHTENING) && !mods.has(CurveModifier.OPENING)) {
      mods.add(CurveModifier.HOLDS);
    }
  }

  return mods;
}

/**
 * Computes the total angle change through a curve segment.
 * This is the absolute difference between the entry bearing and exit bearing.
 * @param {{lat: number, lon: number}[]} points
 * @param {number} startIdx
 * @param {number} endIdx
 * @returns {number} Absolute angle change in degrees.
 * @private
 */
function computeTotalAngleChange(points, startIdx, endIdx) {
  if (endIdx - startIdx < 1) return 0;

  const entryBearing = bearing(points[startIdx], points[Math.min(startIdx + 1, endIdx)]);
  const exitBearing = bearing(points[Math.max(endIdx - 1, startIdx)], points[endIdx]);

  return Math.abs(bearingDifference(entryBearing, exitBearing));
}

/**
 * Fully classifies a raw curve segment, computing all properties needed
 * for narration and display.
 *
 * @param {{startIndex: number, endIndex: number, isCurve: boolean}} rawSegment
 *   The raw segment (must have isCurve = true).
 * @param {{radius: number, rawRadius: number, direction: string|null, point: {lat: number, lon: number}}[]} curvaturePoints
 *   Per-point curvature data from computeCurvature.
 * @param {{lat: number, lon: number}[]} points - Interpolated route points.
 * @param {number} distanceFromStart - Distance from route start to this segment's start.
 * @param {{gentle: number, moderate: number, firm: number, sharp: number}} [severityThresholds]
 *   Optional severity threshold overrides.
 * @returns {object} A fully classified curve segment object.
 */
export function classifyCurve(
  rawSegment,
  curvaturePoints,
  points,
  distanceFromStart,
  severityThresholds
) {
  const { startIndex, endIndex } = rawSegment;

  const direction = computeDirection(curvaturePoints, startIndex, endIndex);
  const minRadius = computeMinRadius(curvaturePoints, startIndex, endIndex);
  const severity = classifySeverity(minRadius, severityThresholds);
  const arcLength = computeArcLength(points, startIndex, endIndex);
  const modifiers = computeModifiers(curvaturePoints, startIndex, endIndex, arcLength);
  const totalAngleChange = computeTotalAngleChange(points, startIndex, endIndex);
  const is90Degree = totalAngleChange >= 85 && totalAngleChange <= 95 && arcLength < 50;

  return {
    type: 'curve',
    direction,
    severity,
    minRadius,
    arcLength,
    modifiers,
    totalAngleChange,
    is90Degree,
    advisorySpeedKmh: null,  // set later by speed advisor
    leanAngleDeg: null,      // set later by lean angle calculator
    compoundType: null,      // set later by compound detector
    compoundSize: null,      // set later by compound detector
    narration: null,         // set later by narration generator
    startPoint: points[startIndex],
    endPoint: points[endIndex],
    startIndex,
    endIndex,
    confidence: 1.0,
    distanceFromStart,
  };
}

// ============================================================================
// Stage 5: Speed Advisory
// ============================================================================

/**
 * Computes the physics-based advisory speed for a curve.
 *
 * Formula: speed_ms = sqrt(radius * lateralG * GRAVITY), then convert to km/h
 * and round down (floor) to the nearest 5 km/h.
 *
 * @param {number} radius - Curve minimum radius in meters.
 * @param {number} [lateralG=0.35] - Lateral g-force limit (car=0.35, motorcycle=0.25).
 * @returns {number} Advisory speed in km/h, floored to nearest 5. Returns 0 if radius <= 0.
 */
export function computeAdvisorySpeed(radius, lateralG = 0.35) {
  if (radius <= 0) return 0;
  const speedMs = Math.sqrt(radius * lateralG * GRAVITY);
  const kmh = speedMs * MS_TO_KMH;
  return Math.floor(kmh / 5) * 5;
}

/**
 * Determines whether a curve needs a speed advisory based on its severity
 * and the computed advisory speed.
 *
 * - GENTLE: never
 * - MODERATE: only if advisory < 70 km/h
 * - FIRM, SHARP, HAIRPIN: always
 *
 * @param {string} severity - Severity enum value.
 * @param {number} advisoryKmh - Computed advisory speed in km/h.
 * @returns {boolean} True if a speed advisory should be included.
 * @private
 */
function needsAdvisory(severity, advisoryKmh) {
  if (severity === Severity.GENTLE) return false;
  if (severity === Severity.MODERATE) return advisoryKmh < 70;
  return true; // FIRM, SHARP, HAIRPIN
}

/**
 * Applies advisory speed to a curve object. Mutates the curve in place.
 * @param {object} curve - Classified curve segment.
 * @param {number} [lateralG=0.35] - Lateral g-force limit.
 * @private
 */
function applyAdvisorySpeed(curve, lateralG = 0.35) {
  const advisory = computeAdvisorySpeed(curve.minRadius, lateralG);
  if (needsAdvisory(curve.severity, advisory)) {
    curve.advisorySpeedKmh = advisory;
  }
}

// ============================================================================
// Stage 6: Lean Angle
// ============================================================================

/**
 * Calculates the motorcycle lean angle from speed and curve radius.
 *
 * Formula: lean_angle = atan(v^2 / (r * g)) converted to degrees,
 * rounded to the nearest 5 degrees, and capped at 45 degrees.
 *
 * @param {number} speedKmh - Advisory speed in km/h.
 * @param {number} radiusM - Curve minimum radius in meters.
 * @returns {number|null} Lean angle in degrees (rounded to nearest 5, max 45),
 *   or null if speed is zero/invalid.
 */
export function computeLeanAngle(speedKmh, radiusM) {
  if (!speedKmh || speedKmh <= 0 || radiusM <= 0) return null;

  const speedMs = speedKmh / MS_TO_KMH;
  const tanTheta = (speedMs * speedMs) / (radiusM * GRAVITY);
  const exact = toDegrees(Math.atan(tanTheta));
  const rounded = Math.round(exact / 5) * 5;
  return Math.min(rounded, 45);
}

/**
 * Applies lean angle calculation to a curve object. Mutates the curve in place.
 * @param {object} curve - Classified curve segment with advisorySpeedKmh set.
 * @private
 */
function applyLeanAngle(curve) {
  curve.leanAngleDeg = computeLeanAngle(curve.advisorySpeedKmh, curve.minRadius);
}

// ============================================================================
// Stage 7: Compound Detection
// ============================================================================

/**
 * Checks if a severity is SHARP or tighter (HAIRPIN).
 * @param {string} severity - Severity enum value.
 * @returns {boolean}
 * @private
 */
function isSeveritySharpOrTighter(severity) {
  return severity === Severity.SHARP || severity === Severity.HAIRPIN;
}

/**
 * Detects compound curve patterns and annotates curve segments in place.
 *
 * Detection rules:
 * - S-bend: 2 curves, opposite directions, gap < threshold
 * - Chicane: S-bend where both curves are SHARP or HAIRPIN
 * - Series: 3+ curves linked with < threshold gaps
 * - Tightening sequence: 2+ same-direction curves, each tighter, gaps < threshold
 *
 * @param {object[]} segments - Mixed array of curve and straight segment objects.
 *   Curve objects must have type='curve', direction, severity, minRadius,
 *   distanceFromStart, arcLength, and startIndex properties.
 * @param {{lat: number, lon: number}[]} points - Interpolated route points.
 * @param {number} [gapThreshold=50] - Maximum gap between curves to consider them
 *   part of a compound (meters).
 */
export function detectCompounds(segments, points, gapThreshold = 50) {
  const curves = segments.filter((s) => s.type === 'curve');
  if (curves.length < 2) return;

  /** Track startIndex of already-annotated curves. */
  const updated = new Set();

  // --- S-bends and chicanes (most safety-critical -- detect first) ---
  for (let i = 0; i < curves.length - 1; i++) {
    const a = curves[i];
    const b = curves[i + 1];

    if (updated.has(a.startIndex) || updated.has(b.startIndex)) continue;

    const gap = b.distanceFromStart - (a.distanceFromStart + a.arcLength);
    if (gap >= gapThreshold) continue;

    // Must be opposite direction for S-bend
    if (a.direction === b.direction) continue;

    const isChicane =
      isSeveritySharpOrTighter(a.severity) && isSeveritySharpOrTighter(b.severity);
    const type = isChicane ? CompoundType.CHICANE : CompoundType.S_BEND;

    a.compoundType = type;
    a.compoundSize = 2;
    b.compoundType = type;
    b.compoundSize = 2;
    updated.add(a.startIndex);
    updated.add(b.startIndex);
  }

  // --- Series: 3+ curves linked with < gapThreshold gaps ---
  if (curves.length >= 3) {
    let runStart = 0;
    while (runStart < curves.length) {
      let runEnd = runStart;

      while (runEnd < curves.length - 1) {
        const curr = curves[runEnd];
        const next = curves[runEnd + 1];
        let gap = next.distanceFromStart - (curr.distanceFromStart + curr.arcLength);
        if (gap < 0) gap = 0;

        if (gap < gapThreshold) {
          runEnd++;
        } else {
          break;
        }
      }

      const runLength = runEnd - runStart + 1;
      if (runLength >= 3) {
        for (let j = runStart; j <= runEnd; j++) {
          // Don't overwrite existing compound annotations
          if (!updated.has(curves[j].startIndex)) {
            curves[j].compoundType = CompoundType.SERIES;
            curves[j].compoundSize = runLength;
            updated.add(curves[j].startIndex);
          }
        }
      }

      runStart = runEnd + 1;
    }
  }

  // --- Tightening sequences: same direction, each tighter, < gapThreshold ---
  if (curves.length >= 2) {
    let seqStart = 0;
    while (seqStart < curves.length - 1) {
      if (updated.has(curves[seqStart].startIndex)) {
        seqStart++;
        continue;
      }

      let seqEnd = seqStart;
      while (seqEnd < curves.length - 1) {
        const curr = curves[seqEnd];
        const next = curves[seqEnd + 1];

        // Must be same direction
        if (curr.direction !== next.direction) break;

        // Must be linked (< gapThreshold)
        const gap = next.distanceFromStart - (curr.distanceFromStart + curr.arcLength);
        if (gap >= gapThreshold) break;

        // Next must be tighter (smaller radius)
        if (next.minRadius >= curr.minRadius) break;

        // Skip if next already has a compound annotation
        if (updated.has(next.startIndex)) break;

        seqEnd++;
      }

      const seqLength = seqEnd - seqStart + 1;
      if (seqLength >= 2) {
        for (let j = seqStart; j <= seqEnd; j++) {
          curves[j].compoundType = CompoundType.TIGHTENING_SEQUENCE;
          curves[j].compoundSize = seqLength;
          updated.add(curves[j].startIndex);
        }
      }

      seqStart = seqEnd + 1;
    }
  }
}

// ============================================================================
// Stage 8: Narration
// ============================================================================

/**
 * Capitalizes the first character of a string.
 * @param {string} str
 * @returns {string}
 * @private
 */
function capitalize(str) {
  return str.charAt(0).toUpperCase() + str.slice(1);
}

/**
 * Generates spoken narration text for a classified curve segment.
 *
 * Templates:
 * - Compound S-bend: "{dir} into {oppositeDir}, S-bend, {severity}"
 * - Compound Chicane: "Chicane, {dir}-{oppositeDir}"
 * - Compound Series: "Series of N curves, {severity}"
 * - Tightening sequence: "{Dir}, tightening through N curves"
 * - HAIRPIN: "Hairpin {dir} ahead{, tightening}{, slow to X}"
 * - 90-degree: "90 degree {dir} ahead"
 * - SHARP: "Sharp {dir} ahead{, tightening}{, slow to X}"
 * - Standard: "{Dir} curve ahead, {severity}{, tightening}{, opening}{, holds for Xm}{, slow to X}"
 * - Long variant: "Long {severity} {dir}{, slow to X}"
 *
 * All templates append ", slow to X" if advisorySpeedKmh is set.
 *
 * @param {object} curve - Classified curve from the pipeline. Must have direction,
 *   severity, modifiers (Set), advisorySpeedKmh, compoundType, compoundSize,
 *   arcLength, and is90Degree properties.
 * @param {number} [verbosity=2] - Narration detail level: 1=Minimal, 2=Standard, 3=Detailed.
 * @returns {string} Narration text suitable for TTS output.
 */
export function generateNarration(curve, verbosity = 2) {
  const dir = curve.direction.toLowerCase();
  const oppositeDir = curve.direction === Direction.LEFT ? 'right' : 'left';
  const sev = curve.severity.toLowerCase();
  const hasTightening = curve.modifiers.has(CurveModifier.TIGHTENING);
  const hasOpening = curve.modifiers.has(CurveModifier.OPENING);
  const hasHolds = curve.modifiers.has(CurveModifier.HOLDS);
  const hasLong = curve.modifiers.has(CurveModifier.LONG);
  const speed = curve.advisorySpeedKmh;

  // --- Compound narrations ---

  if (curve.compoundType === CompoundType.S_BEND) {
    const parts = [];
    parts.push(`${capitalize(dir)} into ${oppositeDir}`);
    parts.push('S-bend');
    parts.push(sev);
    if (speed !== null) parts.push(`slow to ${speed}`);
    return parts.join(', ');
  }

  if (curve.compoundType === CompoundType.CHICANE) {
    const parts = ['Chicane', `${dir}-${oppositeDir}`];
    if (speed !== null) parts.push(`slow to ${speed}`);
    return parts.join(', ');
  }

  if (curve.compoundType === CompoundType.SERIES) {
    const count = curve.compoundSize || 3;
    const parts = [`Series of ${count} curves`, sev];
    if (speed !== null) parts.push(`slow to ${speed}`);
    return parts.join(', ');
  }

  if (curve.compoundType === CompoundType.TIGHTENING_SEQUENCE) {
    const count = curve.compoundSize || 2;
    const parts = [`${capitalize(dir)}, tightening through ${count} curves`];
    if (speed !== null) parts.push(`slow to ${speed}`);
    return parts.join(', ');
  }

  // --- Single-curve narrations ---

  if (curve.severity === Severity.HAIRPIN) {
    const parts = [`Hairpin ${dir} ahead`];
    if (hasTightening) parts.push('tightening');
    if (speed !== null) parts.push(`slow to ${speed}`);
    return parts.join(', ');
  }

  if (curve.is90Degree) {
    const parts = [`90 degree ${dir} ahead`];
    if (speed !== null) parts.push(`slow to ${speed}`);
    return parts.join(', ');
  }

  if (curve.severity === Severity.SHARP) {
    const parts = [`Sharp ${dir} ahead`];
    if (hasTightening) parts.push('tightening');
    if (speed !== null) parts.push(`slow to ${speed}`);
    return parts.join(', ');
  }

  // Standard curves (GENTLE, MODERATE, FIRM)
  const parts = [];

  if (hasLong && verbosity >= 2) {
    parts.push(`Long ${sev} ${dir}`);
  } else {
    parts.push(`${capitalize(dir)} curve ahead`);
    parts.push(sev);
  }

  if (hasTightening) parts.push('tightening');
  if (hasOpening && verbosity >= 2) parts.push('opening');
  if (hasHolds && verbosity >= 2) {
    const holdDist = Math.round(curve.arcLength / 10) * 10;
    parts.push(`holds for ${holdDist} meters`);
  }
  if (speed !== null) parts.push(`slow to ${speed}`);

  return parts.join(', ');
}

// ============================================================================
// Cumulative Distance Helper
// ============================================================================

/**
 * Computes cumulative distance from the route start for each point.
 * @param {{lat: number, lon: number}[]} points - Route points.
 * @returns {number[]} Array of cumulative distances (same length as points).
 * @private
 */
function computeCumulativeDistances(points) {
  const distances = new Array(points.length);
  distances[0] = 0;
  for (let i = 1; i < points.length; i++) {
    distances[i] = distances[i - 1] + haversineDistance(points[i - 1], points[i]);
  }
  return distances;
}

// ============================================================================
// Main API: analyzeRoute
// ============================================================================

/**
 * Full CurveCue analysis pipeline. Transforms raw GPS coordinates into a
 * fully classified list of route segments (curves and straights) with
 * narration text.
 *
 * Pipeline stages:
 *   1. Interpolation -- resample to uniform spacing
 *   2. Curvature -- Menger radius + smoothing + outlier rejection
 *   3. Segmentation -- split into curve vs straight segments
 *   4. Classification -- severity, direction, modifiers, angle change
 *   5. Speed advisory -- physics-based recommended speeds
 *   6. Lean angle -- motorcycle cornering angle
 *   7. Compound detection -- S-bends, chicanes, series, tightening sequences
 *   8. Narration -- human-readable TTS text
 *
 * @param {{lat: number, lon: number}[]} points - Route coordinates (>= 3 for meaningful analysis).
 * @param {object} [options={}] - Pipeline configuration options.
 * @param {number} [options.spacingMeters=5] - Interpolation spacing in meters.
 * @param {number} [options.smoothingWindow=5] - Curvature smoothing window size.
 * @param {number} [options.curvatureThreshold=500] - Max radius for curve detection (meters).
 * @param {number} [options.gapMerge=50] - Max straight gap to merge between curves (meters).
 * @param {number} [options.lateralG=0.35] - Lateral g-force limit for speed advisory.
 * @param {{gentle: number, moderate: number, firm: number, sharp: number}} [options.severityThresholds]
 *   Radius thresholds for severity classification.
 * @returns {{
 *   segments: object[],
 *   curves: object[],
 *   stats: {
 *     totalCurves: number,
 *     totalDistance: number,
 *     curvesByType: Object<string, number>,
 *     interpolatedPointCount: number
 *   }
 * }}
 *   - segments: ordered array of all route segments (curves and straights).
 *     Each curve has: type, direction, severity, minRadius, arcLength,
 *     modifiers (Set), totalAngleChange, is90Degree, advisorySpeedKmh,
 *     leanAngleDeg, compoundType, compoundSize, narration, startPoint,
 *     endPoint, startIndex, endIndex, confidence, distanceFromStart.
 *   - curves: filtered array of only curve segments.
 *   - stats: summary statistics.
 */
export function analyzeRoute(points, options = {}) {
  const {
    spacingMeters = 5,
    smoothingWindow = 5,
    curvatureThreshold = 500,
    gapMerge = 50,
    lateralG = 0.35,
    severityThresholds = { gentle: 200, moderate: 100, firm: 50, sharp: 25 },
  } = options;

  // Edge case: fewer than 3 points cannot produce meaningful analysis
  if (!points || points.length < 3) {
    return {
      segments: [],
      curves: [],
      stats: {
        totalCurves: 0,
        totalDistance: 0,
        curvesByType: {},
        interpolatedPointCount: 0,
      },
    };
  }

  // Stage 1: Interpolation -- resample to uniform spacing
  const interpolated = interpolateRoute(points, spacingMeters);

  // Early exit if interpolation produced too few points
  if (interpolated.length < 3) {
    return {
      segments: [],
      curves: [],
      stats: {
        totalCurves: 0,
        totalDistance: 0,
        curvesByType: {},
        interpolatedPointCount: interpolated.length,
      },
    };
  }

  // Stage 2: Curvature computation with smoothing
  const curvaturePoints = computeCurvature(interpolated, smoothingWindow);

  // Stage 3: Segmentation into curve and straight regions
  const rawSegments = segmentRoute(curvaturePoints, interpolated, curvatureThreshold, gapMerge);

  // Cumulative distances for distanceFromStart calculations
  const cumulativeDistances = computeCumulativeDistances(interpolated);

  // Stages 4, 5, 6: Classification + Speed Advisory + Lean Angle
  const segments = rawSegments.map((raw) => {
    const distFromStart = cumulativeDistances[raw.startIndex];

    if (raw.isCurve) {
      // Stage 4: Classification
      const curve = classifyCurve(
        raw, curvaturePoints, interpolated, distFromStart, severityThresholds
      );

      // Stage 5: Speed Advisory
      applyAdvisorySpeed(curve, lateralG);

      // Stage 6: Lean Angle
      applyLeanAngle(curve);

      return curve;
    } else {
      // Straight segment
      return {
        type: 'straight',
        length: computeArcLength(interpolated, raw.startIndex, raw.endIndex),
        startIndex: raw.startIndex,
        endIndex: raw.endIndex,
        distanceFromStart: distFromStart,
        startPoint: interpolated[raw.startIndex],
        endPoint: interpolated[raw.endIndex],
      };
    }
  });

  // Stage 7: Compound Detection
  detectCompounds(segments, interpolated, gapMerge);

  // Stage 8: Narration (for curves only)
  const curves = segments.filter((s) => s.type === 'curve');
  for (const curve of curves) {
    curve.narration = generateNarration(curve);
  }

  // Compute summary statistics
  const totalDistance =
    cumulativeDistances.length > 0
      ? cumulativeDistances[cumulativeDistances.length - 1]
      : 0;

  const curvesByType = {};
  for (const c of curves) {
    curvesByType[c.severity] = (curvesByType[c.severity] || 0) + 1;
  }

  return {
    segments,
    curves,
    stats: {
      totalCurves: curves.length,
      totalDistance,
      curvesByType,
      interpolatedPointCount: interpolated.length,
    },
  };
}
