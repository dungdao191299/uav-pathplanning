import { uploadedGrid, calculateGlobalThreshold, partitionSpace, validateData } from './partition.js';
import { buildSkeleton } from './skeleton.js';
import { buildExpandedGraph, SOLVER_DEFAULTS, VOXEL_SIZE_METERS } from './dijkstra.js';

// Defaults from case_all_genMatrix.ipynb, not the smaller defaults of partition_space.
export const GRAPH_DEFAULTS = Object.freeze({
  thresholdPercentage: 0.2, minCubic: Object.freeze([50, 50, 20]),
  voxelSize: Object.freeze([1, 1, 1]), minCubicUnit: 'voxel',
  windowSize: 11, mergeRadius: 15, skeletonRadius: 100, bridgeRadius: 50, neighbors: 5,
});

/** Notebook cell 11 DDA: voxel coordinates, including both endpoints; ties step Z,Y,X. */
export function segmentIntersectsObstacle(start, end, data, axes = [0, 1, 2]) {
  const { shape, index } = uploadedGrid(data, axes);
  const voxel = start.map(Math.floor), target = end.map(Math.floor);
  const delta = end.map((value, c) => value - start[c]);
  const distance = Math.sqrt(delta.reduce((sum, value) => sum + value * value, 0));
  if (!distance) return false;
  const direction = delta.map(value => value / distance), step = direction.map(Math.sign);
  const tDelta = direction.map(value => value ? Math.abs(1 / value) : Infinity);
  const tMax = direction.map((value, c) => value > 0 ? (voxel[c] + 1 - start[c]) * tDelta[c]
    : value < 0 ? (start[c] - voxel[c]) * tDelta[c] : Infinity);
  while (true) {
    if (voxel.some((value, c) => value < 0 || value >= shape[c]) || data.building.values[index(voxel)] !== 0) return true;
    if (voxel.every((value, c) => value === target[c])) return false;
    const axis = tMax[0] < tMax[1] ? (tMax[0] < tMax[2] ? 0 : 2) : (tMax[1] < tMax[2] ? 1 : 2);
    voxel[axis] += step[axis]; tMax[axis] += tDelta[axis];
  }
}

function faceTouch(a, b) {
  const ar = [a.x_rng, a.y_rng, a.z_rng], br = [b.x_rng, b.y_rng, b.z_rng], eps = 1e-9;
  return ar.some(([lo, hi], c) =>
    (Math.abs(hi - br[c][0]) <= eps || Math.abs(br[c][1] - lo) <= eps) &&
    ar.every(([a0, a1], k) => k === c || Math.min(a1, br[k][1]) - Math.max(a0, br[k][0]) > eps));
}

/** Cells 22,25,43,44,46,47,51,52. Build edges from adaptive boxes and merged skeleton points. */
export function buildGraphFromSkeleton(data, boxes, skeletonPoints, options = {}) {
  validateData(data);
  const settings = { ...GRAPH_DEFAULTS, axes: [0, 1, 2], ...options };
  const { axes, skeletonRadius, bridgeRadius, neighbors } = settings;
  const { shape } = uploadedGrid(data, axes);
  if (![skeletonRadius, bridgeRadius].every(n => Number.isFinite(n) && n > 0) || !Number.isSafeInteger(neighbors) || neighbors < 1) {
    throw new Error('Use positive graph radii and a positive integer neighbor count.');
  }
  const free = boxes.filter(box => !box.is_obstacle);
  const centers = free.map(box => [box.x_rng, box.y_rng, box.z_rng].map(([lo, hi]) => (lo + hi) / 2));
  const intersects = (a, b) => segmentIntersectsObstacle(a, b, data, axes);
  const distance = (a, b) => Math.sqrt(a.reduce((sum, value, c) => sum + (value - b[c]) ** 2, 0));
  // shortcut: O(P² + B²) scans; use a spatial index if generated points/boxes reach tens of thousands.
  function candidates(point, points, radius) {
    return points.map((p, id) => ({ id, distance: distance(point, p) }))
      .filter(item => item.distance <= radius).sort((a, b) => a.distance - b.distance || a.id - b.id);
  }
  const adaptive = [];
  for (let i = 0; i < free.length; i++) for (let j = i + 1; j < free.length; j++) {
    if (faceTouch(free[i], free[j])) adaptive.push([centers[i], centers[j]]);
  }
  const skeletonPairs = new Set();
  for (let i = 0; i < skeletonPoints.length; i++) {
    let added = 0;
    for (const { id: j } of candidates(skeletonPoints[i], skeletonPoints, skeletonRadius)) {
      if (i === j) continue;
      const a = Math.min(i, j), b = Math.max(i, j), key = `${a},${b}`;
      if (skeletonPairs.has(key)) continue;
      if (!intersects(skeletonPoints[a], skeletonPoints[b])) { skeletonPairs.add(key); added++; }
      if (added >= neighbors) break;
    }
  }
  const skeleton = [...skeletonPairs].map(pair => pair.split(',').map(Number))
    .sort((a, b) => a[0] - b[0] || a[1] - b[1]).map(([i, j]) => [skeletonPoints[i], skeletonPoints[j]]);
  const bridge = [];
  for (const point of skeletonPoints) {
    // Python selects the nearest N boxes BEFORE collision filtering, without replacement.
    for (const { id } of candidates(point, centers, bridgeRadius).slice(0, neighbors)) {
      if (!intersects(point, centers[id])) bridge.push([point, centers[id]]);
    }
  }
  const edgeTypeNames = ['adaptive', 'skeleton', 'bridge'];
  const groups = [adaptive, skeleton, bridge].map(edges => edges.filter(([a, b]) => !intersects(a, b)));
  const pairs = groups.flat();
  // Like the notebook exporter, point IDs are sorted and only retained edge endpoints are kept.
  const points = [...new Map(pairs.flat().map(point => [point.join(','), point])).values()]
    .sort((a, b) => a[0] - b[0] || a[1] - b[1] || a[2] - b[2]);
  const ids = new Map(points.map((point, id) => [point.join(','), id]));
  return {
    points, edges: pairs.map(pair => pair.map(point => ids.get(point.join(',')))),
    edgeTypes: groups.flatMap((edges, type) => edges.map(() => type)), edgeTypeNames,
    boxes: free.map(box => [box.x_rng[0], box.y_rng[0], box.z_rng[0], box.x_rng[1], box.y_rng[1], box.z_rng[1]]),
    metadata: { shape, coordinateUnit: 'voxel', parameters: settings,
      counts: { points: points.length, edges: pairs.length, boxes: free.length,
        adaptive: groups[0].length, skeleton: groups[1].length, bridge: groups[2].length } },
  };
}

/** Attach sampled wind and a directed energy graph, without start/end or terminal costs. */
export function expandGeneratedGraph(graph, data, axes = [0, 1, 2]) {
  const { shape, index } = uploadedGrid(data, axes);
  const channels = data.wind.shape[3];
  // NumPy round uses ties-to-even; clamp samples on the outer grid boundary.
  const round = n => n % 1 === 0.5 ? 2 * Math.round(n / 2) : Math.round(n);
  function sample(point, transition) {
    const voxel = point.map((n, c) => Math.max(0, Math.min(shape[c] - 1,
      transition ? Math.floor(n / VOXEL_SIZE_METERS[c]) : round(n))));
    const offset = index(voxel) * channels;
    return axes.map(channel => data.wind.values[offset + channel]);
  }
  // generate_directed_edge_dict uses rounded voxels for flight, point/scale for turns.
  const solver = { ...SOLVER_DEFAULTS,
    flightWind: graph.points.map(point => sample(point, false)),
    transitionWind: graph.points.map(point => sample(point, true)) };
  const expandedGraph = buildExpandedGraph({ nodes: graph.points, edges: graph.edges }, solver);
  return { ...graph, solver, expandedGraph };
}

/** Input: loaded building/wind TypedArrays. Output: points/edges, solver and expandedGraph (J). */
export function generateGraph(data, options = {}, progress = () => {}) {
  const settings = { ...GRAPH_DEFAULTS, axes: [0, 1, 2], ...options };
  progress('Partitioning space…');
  const windThreshold = calculateGlobalThreshold(data, settings.axes, settings.thresholdPercentage);
  const boxes = partitionSpace(data, { ...settings, windThreshold });
  const skeleton = buildSkeleton(data, { ...settings, boxes }, progress);
  progress('Connecting graph…');
  const graph = buildGraphFromSkeleton(data, boxes, skeleton.outside, settings);
  progress('Computing energy and expanding graph…');
  return expandGeneratedGraph(graph, data, settings.axes);
}

if (typeof WorkerGlobalScope !== 'undefined' && self instanceof WorkerGlobalScope) {
  self.onmessage = ({ data: { input, options } }) => {
    try { self.postMessage({ result: generateGraph(input, options, status => self.postMessage({ status })) }); }
    catch (error) { self.postMessage({ error: error.message }); }
  };
}
