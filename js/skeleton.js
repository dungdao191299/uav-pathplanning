import { uploadedGrid, validateData } from './partition.js';

/** scipy.ndimage.uniform_filter(size=odd, mode='nearest'), preserving float dtype. */
export function uniformFilter(field, shape, size) {
  if (!Number.isInteger(size) || size < 1 || size % 2 !== 1) throw new Error('Window size must be a positive odd integer.');
  const strides = [shape[1] * shape[2], shape[2], 1], radius = (size - 1) / 2;
  let input = field;
  for (let axis = 0; axis < 3; axis++) {
    const output = new field.constructor(field.length), step = strides[axis], length = shape[axis];
    for (let base = 0; base < field.length; base++) {
      if (Math.floor(base / step) % length !== 0) continue;
      const at = coordinate => input[base + Math.max(0, Math.min(length - 1, coordinate)) * step];
      let sum = 0;
      for (let k = -radius; k <= radius; k++) sum += at(k);
      for (let k = 0; k < length; k++) {
        output[base + k * step] = sum / size;
        sum += at(k + radius + 1) - at(k - radius);
      }
    }
    input = output;
  }
  return input;
}

// Lee94 Euler table and octants, as used by scikit-image (BSD-3-Clause).
// Attribution and license: frontend/THIRD-PARTY-NOTICES.txt.
const EULER = [1,-1,-1,1,-3,-1,-1,1,-1,1,1,-1,3,1,1,-1,-3,-1,3,1,1,-1,3,1,-1,1,1,-1,3,1,1,-1,
  -3,3,-1,1,1,3,-1,1,-1,1,1,-1,3,1,1,-1,1,3,3,1,5,3,3,1,-1,1,1,-1,3,1,1,-1,
  -7,-1,-1,1,-3,-1,-1,1,-1,1,1,-1,3,1,1,-1,-3,-1,3,1,1,-1,3,1,-1,1,1,-1,3,1,1,-1,
  -3,3,-1,1,1,3,-1,1,-1,1,1,-1,3,1,1,-1,1,3,3,1,5,3,3,1,-1,1,1,-1,3,1,1,-1];
const OCTANTS = [[2,1,11,10,5,4,14],[0,9,3,12,1,10,4],[8,7,17,16,5,4,14],[6,15,7,16,3,12,4],
  [20,23,19,22,11,14,10],[18,21,9,12,19,22,10],[26,23,17,14,25,22,16],[24,25,15,16,21,22,12]];
const LOCAL_LINKS = Array.from({ length: 27 }, (_, i) =>
  [...new Set(OCTANTS.filter(octant => octant.includes(i)).flat())].filter(j => j !== i));

/** Exact Lee two-pass thinning order; output is a Uint8Array in C-order XYZ. */
export function skeletonize3D(mask, shape, progress = () => {}) {
  if (shape.length !== 3 || !shape.every(n => Number.isInteger(n) && n > 0) ||
      mask.length !== shape.reduce((a, b) => a * b, 1)) throw new Error('Invalid skeleton mask.');
  const [nx, ny, nz] = shape, sz = nz + 2, sx = (ny + 2) * sz;
  const image = new Uint8Array((nx + 2) * sx);
  let count = 0;
  for (const value of mask) if (value) count++;
  let active = new Uint32Array(count), n = 0;
  for (let x = 0; x < nx; x++) for (let y = 0; y < ny; y++) for (let z = 0; z < nz; z++) {
    if (!mask[(x * ny + y) * nz + z]) continue;
    const i = (x + 1) * sx + (y + 1) * sz + z + 1;
    image[i] = 1; active[n++] = i;
  }
  const offsets = Array.from({ length: 27 }, (_, i) =>
    (Math.floor(i / 9) - 1) * sx + (i % 3 - 1) * sz + Math.floor(i % 9 / 3) - 1);
  const neighbors = new Uint8Array(27), visited = new Uint8Array(27), stack = new Uint8Array(27);
  const neighborhood = i => { for (let j = 0; j < 27; j++) neighbors[j] = image[i + offsets[j]]; };
  function simple() {
    visited.fill(0);
    let components = 0;
    for (let i = 0; i < 27; i++) {
      if (i === 13 || !neighbors[i] || visited[i]) continue;
      if (++components > 1) return false;
      let length = 1; stack[0] = i; visited[i] = 1;
      while (length) {
        const j = stack[--length];
        for (const k of LOCAL_LINKS[j]) {
          if (neighbors[k] && !visited[k]) { visited[k] = 1; stack[length++] = k; }
        }
      }
    }
    return true;
  }
  function eulerInvariant() {
    let sum = 0;
    for (const octant of OCTANTS) {
      let code = 1;
      for (let j = 0; j < 7; j++) if (neighbors[octant[j]]) code |= 1 << (7 - j);
      sum += EULER[code >> 1];
    }
    return sum === 0;
  }
  const borders = [-sz, sz, 1, -1, sx, -sx].slice(0, nx === 1 ? 4 : 6);
  const candidates = new Uint32Array(count);
  let changed = true, iteration = 0;
  while (changed) {
    changed = false;
    for (const border of borders) {
      let length = 0;
      for (const i of active) {
        if (!image[i] || image[i + border]) continue;
        neighborhood(i);
        let sum = 0; for (const value of neighbors) sum += value;
        if (sum !== 2 && eulerInvariant() && simple()) candidates[length++] = i;
      }
      for (let j = 0; j < length; j++) {
        const i = candidates[j]; neighborhood(i);
        if (simple()) { image[i] = 0; changed = true; }
      }
    }
    let length = 0;
    for (const i of active) if (image[i]) active[length++] = i;
    active = active.subarray(0, length);
    progress({ iteration: ++iteration, remaining: length });
  }
  const result = new Uint8Array(mask.length);
  for (let x = 0; x < nx; x++) for (let y = 0; y < ny; y++) for (let z = 0; z < nz; z++) {
    result[(x * ny + y) * nz + z] = image[(x + 1) * sx + (y + 1) * sz + z + 1];
  }
  return result;
}

/** skan graph: 26-neighbors, junction MST, then branch endpoints (including cycles). */
export function skeletonBranchPoints(mask, shape) {
  const points = [], ids = new Map();
  const [nx, ny, nz] = shape;
  for (let x = 0; x < nx; x++) for (let y = 0; y < ny; y++) for (let z = 0; z < nz; z++) {
    const i = (x * ny + y) * nz + z;
    if (mask[i]) { ids.set(i, points.length); points.push([x, y, z]); }
  }
  const graph = points.map(() => []), edges = [];
  points.forEach(([x, y, z], i) => {
    for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) for (let dz = -1; dz <= 1; dz++) {
      const a = x + dx, b = y + dy, c = z + dz;
      if (a < 0 || a >= nx || b < 0 || b >= ny || c < 0 || c >= nz) continue;
      const j = ids.get((a * ny + b) * nz + c);
      if (j === undefined || j <= i) continue;
      edges.push([i, j, Math.sqrt(dx * dx + dy * dy + dz * dz)]);
      graph[i].push(j); graph[j].push(i);
    }
  });
  const junction = graph.map(neighbors => neighbors.length > 2);
  const parents = points.map((_, i) => i);
  function root(i) { while (parents[i] !== i) { parents[i] = parents[parents[i]]; i = parents[i]; } return i; }
  const tree = new Set();
  // Stable weight/CSR order matches scipy's Kruskal tie ordering.
  for (const [i, j] of edges.filter(([a, b]) => junction[a] && junction[b]).sort((a, b) => a[2] - b[2] || a[0] - b[0] || a[1] - b[1])) {
    const a = root(i), b = root(j);
    if (a !== b) { parents[a] = b; tree.add(`${i},${j}`); }
  }
  for (let i = 0; i < graph.length; i++) graph[i] = graph[i].filter(j =>
    !(junction[i] && junction[j]) || tree.has(`${Math.min(i, j)},${Math.max(i, j)}`)).sort((a, b) => a - b);
  const visited = new Set(), endpoints = new Set();
  const key = (a, b) => `${Math.min(a, b)},${Math.max(a, b)}`;
  function walk(start, next) {
    let current = start;
    endpoints.add(start);
    while (!visited.has(key(current, next))) {
      visited.add(key(current, next));
      if (graph[next].length !== 2 || next === start) break;
      const following = graph[next].find(i => i !== current);
      current = next; next = following;
    }
    endpoints.add(next);
  }
  for (let i = 0; i < graph.length; i++) if (graph[i].length === 1 || graph[i].length > 2) {
    for (const j of graph[i]) if (!visited.has(key(i, j))) walk(i, j);
  }
  for (let i = 0; i < graph.length; i++) if (graph[i].length && !visited.has(key(i, graph[i][0]))) walk(i, graph[i][0]);
  return [...endpoints].sort((a, b) => a - b).map(i => points[i]);
}

const comparePoints = (a, b) => a[0] - b[0] || a[1] - b[1] || a[2] - b[2];
// NumPy uses ties-to-even for round; JS Math.round uses a different tie rule.
function numpyRound(value, decimals) {
  const scaled = value * 10 ** decimals, floor = Math.floor(scaled);
  return (scaled - floor === 0.5 ? floor + floor % 2 : Math.round(scaled)) / 10 ** decimals;
}

/** Notebook DBSCAN(eps=radius,min_samples=1) centroids, rounded to 2 decimals. */
export function mergeSkeletonPoints(points, radius) {
  if (!Number.isFinite(radius) || radius <= 0) throw new Error('Merge radius must be positive.');
  const visited = new Uint8Array(points.length), result = [];
  // shortcut: O(P²) radius scan; use a spatial index if branch endpoints reach tens of thousands.
  for (let i = 0; i < points.length; i++) {
    if (visited[i]) continue;
    const cluster = [i]; visited[i] = 1;
    for (let cursor = 0; cursor < cluster.length; cursor++) {
      const p = points[cluster[cursor]];
      for (let j = 0; j < points.length; j++) {
        if (!visited[j] && Math.hypot(...points[j].map((n, c) => n - p[c])) <= radius) {
          visited[j] = 1; cluster.push(j);
        }
      }
    }
    cluster.sort((a, b) => a - b);
    result.push([0, 1, 2].map(c => numpyRound(cluster.reduce((sum, j) => sum + points[j][c], 0) / cluster.length, 2)));
  }
  return result;
}

/** Notebook cells 29–33,39. Coordinates/radii are in displayed voxel space. */
export function buildSkeleton(data, { axes = [0, 1, 2], windowSize = 11, mergeRadius = 15, boxes = [] } = {}, progress = () => {}) {
  validateData(data);
  const { shape, index } = uploadedGrid(data, axes);
  if (!Number.isInteger(windowSize) || windowSize < 1 || windowSize % 2 !== 1 || !Number.isFinite(mergeRadius) || mergeRadius <= 0) {
    throw new Error('Use an odd positive window size and positive merge radius.');
  }
  const Type = data.wind.values instanceof Float32Array ? Float32Array : Float64Array;
  const round = Type === Float32Array ? Math.fround : n => n;
  const count = data.building.values.length, channels = data.wind.shape[3];
  const free = new Uint8Array(count), sums = new Type(count), speedSquared = new Type(count);
  let freeCount = 0;
  for (let x = 0; x < shape[0]; x++) for (let y = 0; y < shape[1]; y++) for (let z = 0; z < shape[2]; z++) {
    const i = (x * shape[1] + y) * shape[2] + z;
    free[i] = data.building.values[index([x, y, z])] === 0 ? 1 : 0; freeCount += free[i];
  }
  for (let c = 0; c < 3; c++) {
    progress(`Filtering wind ${c + 1}/3…`);
    const field = new Type(count);
    for (let x = 0; x < shape[0]; x++) for (let y = 0; y < shape[1]; y++) for (let z = 0; z < shape[2]; z++) {
      field[(x * shape[1] + y) * shape[2] + z] = data.wind.values[index([x, y, z]) * channels + axes[c]];
    }
    const mean = uniformFilter(field, shape, windowSize);
    for (let i = 0; i < count; i++) field[i] = field[i] ** 2;
    const meanSquared = uniformFilter(field, shape, windowSize);
    for (let i = 0; i < count; i++) {
      sums[i] += round(meanSquared[i] - round(mean[i] ** 2));
      speedSquared[i] += round(mean[i] ** 2);
    }
  }
  progress('Computing TI quantiles…');
  const ti = sums, validTI = new Type(freeCount);
  let j = 0;
  for (let i = 0; i < count; i++) {
    ti[i] = free[i] ? round(Math.sqrt(Math.max(round(sums[i] * 0.5), 0))) / round(round(Math.sqrt(speedSquared[i])) + 1e-8) : 0;
    if (free[i]) validTI[j++] = ti[i];
  }
  validTI.sort();
  const quantile = q => {
    if (!freeCount) return 0;
    const position = (freeCount - 1) * q, low = Math.floor(position), fraction = position - low;
    const a = validTI[low], b = validTI[Math.ceil(position)];
    // NumPy's interpolation chooses the nearer endpoint for numerical stability.
    return fraction >= 0.5 ? b - (b - a) * (1 - fraction) : a + (b - a) * fraction;
  };
  const quantiles = [0.25, 0.5, 0.75].map(quantile), regions = [];
  for (const [name, lower, upper] of [['High', quantiles[1], quantiles[2]], ['Very high', quantiles[2], Infinity]]) {
    progress(`Thinning ${name}…`);
    const mask = new Uint8Array(count);
    for (let i = 0; i < count; i++) mask[i] = free[i] && ti[i] > lower && ti[i] <= upper ? 1 : 0;
    const skeleton = skeletonize3D(mask, shape, ({ iteration, remaining }) =>
      progress(`Thinning ${name}: pass ${iteration} · ${remaining} voxels…`));
    progress(`Extracting ${name} branches…`);
    const endpoints = skeletonBranchPoints(skeleton, shape);
    progress(`Merging ${name}: ${endpoints.length} endpoints…`);
    const merged = mergeSkeletonPoints(endpoints, mergeRadius);
    const voxels = [];
    for (let x = 0; x < shape[0]; x++) for (let y = 0; y < shape[1]; y++) for (let z = 0; z < shape[2]; z++) {
      if (skeleton[(x * shape[1] + y) * shape[2] + z]) voxels.push([x, y, z]);
    }
    regions.push({ name, voxels, endpoints, merged });
  }
  const points = [...new Map(regions.flatMap(region => region.merged).map(p => {
    const point = p.map(n => numpyRound(n, 1)); return [point.join(','), point];
  })).values()].sort(comparePoints);
  const freeBoxes = boxes.filter(box => !box.is_obstacle);
  const outside = points.filter(point => !freeBoxes.some(box => [box.x_rng, box.y_rng, box.z_rng].every(([lo, hi], c) =>
    point[c] >= lo - 1e-9 && point[c] < hi + 1e-9)));
  progress(`Skeleton: ${points.length} merged points · ${outside.length} outside free boxes.`);
  return { shape, quantiles, regions, points, outside };
}

