/** View of a C-order grid after axis permutation; does not copy the arrays. */
export function uploadedGrid(data, axes) {
  if (axes.length !== 3 || new Set(axes).size !== 3 || !axes.every(axis => [0, 1, 2].includes(axis))) {
    throw new Error('Invalid axis order.');
  }
  const original = data.building.shape;
  const shape = axes.map(axis => original[axis]);
  const originalStrides = [original[1] * original[2], original[2], 1];
  const strides = axes.map(axis => originalStrides[axis]);
  const index = point => point[0] * strides[0] + point[1] * strides[1] + point[2] * strides[2];
  return { shape, index, strides };
}

export function validateData({ building, wind }) {
  const shape = building.shape;
  if (shape.length !== 3 || !shape.every(n => Number.isSafeInteger(n) && n > 0) ||
      wind.shape.length !== 4 || ![3, 4].includes(wind.shape[3]) ||
      !shape.every((n, axis) => n === wind.shape[axis]) ||
      building.values.length !== shape.reduce((a, b) => a * b, 1) ||
      wind.values.length !== building.values.length * wind.shape[3]) {
    throw new Error('Invalid building or wind grid.');
  }
  for (let i = 0; i < building.values.length; i++) {
    if (!Number.isFinite(building.values[i])) throw new Error('Building contains non-finite values.');
    for (let channel = 0; channel < 3; channel++) {
      if (!Number.isFinite(wind.values[i * wind.shape[3] + channel])) {
        throw new Error('Wind contains non-finite velocity.');
      }
    }
  }
}

/** Python calculate_global_threshold: mean speed over ALL voxels × fraction. */
export function calculateGlobalThreshold(data, axes = [0, 1, 2], percentage = 0.5) {
  validateData(data);
  uploadedGrid(data, axes);
  if (!Number.isFinite(percentage) || percentage < 0) throw new Error('Invalid threshold fraction.');
  let sum = 0;
  const values = data.wind.values, channels = data.wind.shape[3];
  for (let i = 0; i < values.length; i += channels) sum += Math.hypot(values[i], values[i + 1], values[i + 2]);
  return sum / data.building.values.length * percentage;
}

/**
 * Port of pathfinding_opt.py partition_space, operating on loaded TypedArrays.
 * Input: {building:{shape,values}, wind:{shape,values}}, options in displayed XYZ.
 * Output: Python-compatible boxes with half-open voxel ranges, is_obstacle,
 * avg_wind [vx,vy,vz] and wind_std (RMS vector deviation, in m/s).
 * Only the first three wind channels are velocity; a fourth channel is ignored.
 */
export function partitionSpace(data, {
  axes = [0, 1, 2], windThreshold = 0.5, minCubic = [5, 5, 5],
  voxelSize = [1, 1, 1], minCubicUnit = 'voxel',
} = {}) {
  validateData(data);
  const { shape, strides } = uploadedGrid(data, axes);
  const positiveTriple = values => values.length === 3 && values.every(n => Number.isFinite(n) && n > 0);
  if (!positiveTriple(minCubic) || !positiveTriple(voxelSize) ||
      !Number.isFinite(windThreshold) || windThreshold < 0 || !['voxel', 'meter'].includes(minCubicUnit)) {
    throw new Error('Invalid partition settings.');
  }
  const occupancy = data.building.values, wind = data.wind.values, channels = data.wind.shape[3];
  const results = [];
  // shortcut: scans regions twice at each depth; move this CPU work to a Worker for larger grids.
  function visit(ranges) {
    const size = ranges.map(([lo, hi]) => hi - lo);
    const total = size[0] * size[1] * size[2];
    let free = 0;
    const avg = [0, 0, 0];
    for (let x = ranges[0][0]; x < ranges[0][1]; x++) {
      for (let y = ranges[1][0]; y < ranges[1][1]; y++) {
        for (let z = ranges[2][0]; z < ranges[2][1]; z++) {
          const i = x * strides[0] + y * strides[1] + z * strides[2];
          if (occupancy[i] !== 0) continue;
          free++;
          for (let c = 0; c < 3; c++) avg[c] += wind[i * channels + axes[c]];
        }
      }
    }
    if (free) for (let c = 0; c < 3; c++) avg[c] /= free;
    let squared = 0;
    if (free) {
      for (let x = ranges[0][0]; x < ranges[0][1]; x++) {
        for (let y = ranges[1][0]; y < ranges[1][1]; y++) {
          for (let z = ranges[2][0]; z < ranges[2][1]; z++) {
            const i = x * strides[0] + y * strides[1] + z * strides[2];
            if (occupancy[i] !== 0) continue;
            for (let c = 0; c < 3; c++) squared += (wind[i * channels + axes[c]] - avg[c]) ** 2;
          }
        }
      }
    }
    const dispersion = free ? Math.sqrt(squared / free) : 0;
    const hasObstacle = free !== total;
    const meters = size.map((n, c) => n * voxelSize[c]);
    const minimum = (minCubicUnit === 'meter' ? meters : size).every((n, c) => n <= minCubic[c]);
    // np.argmax resolves ties in X, Y, Z order; keep that order here.
    let axis = 0;
    for (let c = 1; c < 3; c++) if (meters[c] > meters[axis]) axis = c;
    const [lo, hi] = ranges[axis];
    const mid = lo + Math.floor((hi - lo) / 2);
    if (!free || (!hasObstacle && dispersion < windThreshold) || minimum || mid === lo || mid === hi) {
      results.push({ x_rng: ranges[0], y_rng: ranges[1], z_rng: ranges[2],
        is_obstacle: hasObstacle, avg_wind: avg, wind_std: dispersion });
      return;
    }
    const left = ranges.slice(), right = ranges.slice();
    left[axis] = [lo, mid]; right[axis] = [mid, hi];
    visit(left); visit(right);
  }
  visit(shape.map(n => [0, n]));
  return results;
}
