import * as THREE from 'three';
import { createScene } from './scene.js';
import { createWindLayer } from './wind.js';
import { createGraphLayer } from './graph.js';
import { uploadedGrid } from './partition.js';
import { bindTrajectoryControls } from './trajectory.js';

const MAX_ARRAY_BYTES = 512 * 1024 * 1024;

/** Parse một mảng NPY C-order số/bool; shape giữ đúng thứ tự trục của file. */
export function parseNpy(buffer) {
  const bytes = new Uint8Array(buffer);
  const view = new DataView(buffer);
  if (bytes.length < 10 || bytes[0] !== 0x93 ||
      new TextDecoder().decode(bytes.subarray(1, 6)) !== 'NUMPY') {
    throw new Error('Invalid NPY file.');
  }
  const version = bytes[6];
  if (![1, 2, 3].includes(version)) throw new Error('Unsupported NPY version.');
  const headerStart = version === 1 ? 10 : 12;
  if (bytes.length < headerStart) throw new Error('Incomplete NPY header.');
  const headerSize = version === 1 ? view.getUint16(8, true) : view.getUint32(8, true);
  const payloadStart = headerStart + headerSize;
  if (payloadStart > bytes.length) throw new Error('Incomplete NPY header.');
  const header = new TextDecoder().decode(bytes.subarray(headerStart, payloadStart));
  const dtype = /['"]descr['"]:\s*['"]([^'"]+)['"]/.exec(header)?.[1];
  const shape = /['"]shape['"]:\s*\(([^)]*)\)/.exec(header)?.[1]
    .split(',').map(value => value.trim()).filter(Boolean).map(Number);
  if (!shape?.length || !shape.every(size => Number.isSafeInteger(size) && size > 0) ||
      !/['"]fortran_order['"]:\s*False/.test(header)) {
    throw new Error('Use a non-empty C-order NumPy array.');
  }
  const types = { b1: Uint8Array, u1: Uint8Array, i1: Int8Array, u2: Uint16Array,
    i2: Int16Array, u4: Uint32Array, i4: Int32Array, f4: Float32Array, f8: Float64Array };
  const ArrayType = dtype && /^[<|=]/.test(dtype) && types[dtype.slice(1)];
  if (!ArrayType) throw new Error(`Unsupported dtype ${dtype}; use little-endian numbers or bool.`);
  const count = shape.reduce((total, size) => total * size, 1);
  const length = count * ArrayType.BYTES_PER_ELEMENT;
  if (!Number.isSafeInteger(length) || length > MAX_ARRAY_BYTES || length !== bytes.length - payloadStart) {
    throw new Error('Invalid NPY shape or array size (maximum 512 MiB).');
  }
  // TypedArray offsets must align with the element size; older NPY files may not.
  const aligned = payloadStart % ArrayType.BYTES_PER_ELEMENT === 0;
  const values = aligned
    ? new ArrayType(buffer, payloadStart, count)
    : new ArrayType(buffer.slice(payloadStart));
  return { shape, dtype, values };
}

/** NPZ là ZIP chứa NPY; chỉ giải nén mảng được chọn, không ghi file ra đĩa. */
export async function readNumpyFile(file, role) {
  if (file.size > MAX_ARRAY_BYTES) throw new Error('File exceeds 512 MiB.');
  const buffer = await file.arrayBuffer();
  if (/\.npy$/i.test(file.name)) return parseNpy(buffer);
  if (!/\.npz$/i.test(file.name)) throw new Error('Choose an NPY or NPZ file.');
  const view = new DataView(buffer);
  function checkRange(offset, length) {
    if (offset < 0 || length < 0 || offset + length > buffer.byteLength) throw new Error('Incomplete NPZ archive.');
  }
  let footer = buffer.byteLength - 22;
  while (footer >= Math.max(0, buffer.byteLength - 65557)) {
    if (view.getUint32(footer, true) === 0x06054b50 &&
        footer + 22 + view.getUint16(footer + 20, true) === buffer.byteLength) break;
    footer--;
  }
  if (footer < Math.max(0, buffer.byteLength - 65557)) throw new Error('Invalid NPZ archive.');
  const count = view.getUint16(footer + 10, true);
  let cursor = view.getUint32(footer + 16, true);
  if (view.getUint16(footer + 4, true) || view.getUint16(footer + 6, true) || count === 65535 || cursor === 0xffffffff) {
    throw new Error('Multi-disk and large ZIP64 archives are not supported.');
  }
  const entries = [];
  for (let index = 0; index < count; index++) {
    checkRange(cursor, 46);
    if (view.getUint32(cursor, true) !== 0x02014b50) throw new Error('Invalid NPZ directory.');
    const nameSize = view.getUint16(cursor + 28, true);
    const extraSize = view.getUint16(cursor + 30, true);
    const commentSize = view.getUint16(cursor + 32, true);
    checkRange(cursor + 46, nameSize + extraSize + commentSize);
    const name = new TextDecoder().decode(new Uint8Array(buffer, cursor + 46, nameSize));
    if (name.endsWith('.npy')) entries.push({ name,
      flags: view.getUint16(cursor + 8, true), method: view.getUint16(cursor + 10, true),
      packedSize: view.getUint32(cursor + 20, true), size: view.getUint32(cursor + 24, true),
      offset: view.getUint32(cursor + 42, true) });
    cursor += 46 + nameSize + extraSize + commentSize;
  }
  const names = role === 'building' ? ['building.npy'] : ['wind.npy', 'mean.npy'];
  const entry = names.map(name => entries.find(item => item.name === name)).find(Boolean)
    || (entries.length === 1 ? entries[0] : null);
  if (!entry) throw new Error(`NPZ must contain ${names.join(' or ')} or exactly one array.`);
  if (entry.flags & 1 || ![0, 8].includes(entry.method) || entry.size > MAX_ARRAY_BYTES) {
    throw new Error('Unsupported NPZ compression, encryption or array size.');
  }
  checkRange(entry.offset, 30);
  if (view.getUint32(entry.offset, true) !== 0x04034b50) throw new Error('Invalid NPZ entry.');
  const start = entry.offset + 30 + view.getUint16(entry.offset + 26, true) + view.getUint16(entry.offset + 28, true);
  checkRange(start, entry.packedSize);
  let decoded = buffer.slice(start, start + entry.packedSize);
  if (entry.method === 8) {
    const reader = new Blob([decoded]).stream().pipeThrough(new DecompressionStream('deflate-raw')).getReader();
    const chunks = [];
    let size = 0;
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      size += value.length;
      if (size > entry.size) {
        await reader.cancel();
        throw new Error('NPZ array exceeds its declared size.');
      }
      chunks.push(value);
    }
    decoded = await new Blob(chunks).arrayBuffer();
  }
  if (decoded.byteLength !== entry.size) throw new Error('Incomplete NPZ array.');
  return { ...parseNpy(decoded), key: entry.name.slice(0, -4) };
}

export async function loadStartingData(buildingFile, windFile) {
  if (!buildingFile || !windFile) throw new Error('Choose both files.');
  const [building, wind] = await Promise.all([
    readNumpyFile(buildingFile, 'building'), readNumpyFile(windFile, 'wind'),
  ]);
  if (building.shape.length !== 3) throw new Error('Building must be a 3D occupancy array.');
  if (wind.shape.length !== 4 || ![3, 4].includes(wind.shape[3])) {
    throw new Error('Wind must have shape [nx, ny, nz, 3 or 4].');
  }
  if (!building.shape.every((size, axis) => size === wind.shape[axis])) {
    throw new Error('Building and wind grids must have the same dimensions.');
  }
  return { building, wind };
}

/** Tạo building từ mặt ngoài mask, gộp mặt đồng phẳng để giảm số tam giác. */
export function createUploadedBuilding(data, axes = [0, 1, 2]) {
  const { shape, index } = uploadedGrid(data, axes);
  const positions = [], normals = [], indices = [];
  const filled = point => point.every((value, axis) => value >= 0 && value < shape[axis]) &&
    data.building.values[index(point)] !== 0;
  for (let axis = 0; axis < 3; axis++) {
    const u = (axis + 1) % 3, v = (axis + 2) % 3;
    const width = shape[u], height = shape[v];
    const mask = new Int8Array(width * height);
    for (let plane = 0; plane <= shape[axis]; plane++) {
      for (let row = 0; row < height; row++) {
        for (let column = 0; column < width; column++) {
          const point = [0, 0, 0];
          point[axis] = plane - 1; point[u] = column; point[v] = row;
          const before = Boolean(filled(point));
          point[axis]++;
          const after = Boolean(filled(point));
          mask[row * width + column] = before === after ? 0 : before ? 1 : -1;
        }
      }
      for (let row = 0; row < height; row++) {
        for (let column = 0; column < width;) {
          const sign = mask[row * width + column];
          if (!sign) { column++; continue; }
          let w = 1, h = 1;
          while (column + w < width && mask[row * width + column + w] === sign) w++;
          rows: while (row + h < height) {
            for (let offset = 0; offset < w; offset++) {
              if (mask[(row + h) * width + column + offset] !== sign) break rows;
            }
            h++;
          }
          if (positions.length > 3000000) throw new Error('Building surface is too complex for this preview.');
          const base = positions.length / 3;
          for (const [x, y] of [[column, row], [column + w, row], [column + w, row + h], [column, row + h]]) {
            const point = [0, 0, 0], normal = [0, 0, 0];
            point[axis] = plane; point[u] = x; point[v] = y; normal[axis] = sign;
            positions.push(...point); normals.push(...normal);
          }
          for (const offset of sign > 0 ? [0, 1, 2, 0, 2, 3] : [0, 3, 2, 0, 2, 1]) indices.push(base + offset);
          for (let offset = 0; offset < h; offset++) mask.fill(0, (row + offset) * width + column, (row + offset) * width + column + w);
          column += w;
        }
      }
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
  geometry.setIndex(indices);
  const object = new THREE.Group();
  object.add(new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: 0x456882, roughness: 0.85, metalness: 0.1 })));
  return { meta: { shape }, object };
}

/** Lấy mẫu trực tiếp, không copy/swap toàn bộ mảng gió lớn. */
export function sampleUploadedWind(data, axes, { mode, axis, layer, step }) {
  const { shape, index } = uploadedGrid(data, axes);
  if (!['slice', 'volume'].includes(mode) || ![0, 1, 2].includes(axis) ||
      !Number.isInteger(layer) || layer < 0 || layer >= shape[axis] || !Number.isInteger(step) || step < 1) {
    throw new Error('Invalid wind preview settings.');
  }
  const starts = shape.map((_, dimension) => mode === 'slice' && dimension === axis ? layer : 0);
  const ends = shape.map((size, dimension) => mode === 'slice' && dimension === axis ? layer + 1 : size);
  const rows = [], channels = data.wind.shape[3];
  for (let x = starts[0]; x < ends[0]; x += step) {
    for (let y = starts[1]; y < ends[1]; y += step) {
      for (let z = starts[2]; z < ends[2]; z += step) {
        const voxel = index([x, y, z]);
        if (data.building.values[voxel] !== 0) continue;
        const velocity = axes.map(channel => data.wind.values[voxel * channels + channel]);
        if (velocity.every(Number.isFinite)) rows.push(x + 0.5, y + 0.5, z + 0.5, ...velocity, Math.hypot(...velocity));
      }
    }
  }
  return new Float32Array(rows);
}

// Dữ liệu chỉ giữ trong bộ nhớ trang để sử dụng cho bước tiếp theo.
export let startingData = null;

export function bindStartingPage() {
  const element = id => document.getElementById(id);
  const buildingInput = element('buildingFile'), windInput = element('windFile');
  const button = element('loadFiles'), status = element('uploadStatus'), summary = element('dataSummary');
  const controls = ['wind', 'mode', 'axis', 'layer', 'step', 'length', 'scale', 'home', 'top', 'generateGraph'];
  const planningControls = ['solveCase', 'start-x', 'start-y', 'start-z', 'end-x', 'end-y', 'end-z',
    'constraint-turningAngle', 'constraint-trajectoryAngle', 'constraint-thrust',
    'case-trajectory', 'case-expanded', 'case-drone'];
  let view, wind, axes, graph, graphWorker, planning;
  function clearGraph() {
    planning?.clear(); planning = null;
    planningControls.forEach(id => { element(id).disabled = true; });
    element('caseStatus').textContent = 'Generate a graph first.';
    graphWorker?.terminate(); graphWorker = null;
    if (startingData) delete startingData.graph;
    graph?.draw({ boxes: [], nodes: new Float32Array(), edges: new Uint32Array(), edgeTypes: [] });
    element('graphStatus').textContent = '';
    element('generateGraph').disabled = !startingData || !view;
  }
  function drawWind() {
    if (!startingData || !wind) return;
    const shape = axes.map(axis => startingData.building.shape[axis]);
    const axis = Number(element('axis').value);
    const layer = Math.min(Number(element('layer').value), shape[axis] - 1);
    element('layer').value = layer;
    element('layer').max = shape[axis] - 1;
    element('layerValue').textContent = `${layer} / ${shape[axis] - 1}`;
    element('stepValue').textContent = `${element('step').value} cells`;
    element('lengthValue').textContent = `${element('length').value} cells`;
    element('axis').disabled = element('layer').disabled = element('mode').value === 'volume';
    const rows = sampleUploadedWind(startingData, axes, {
      mode: element('mode').value, axis, layer, step: Number(element('step').value),
    });
    const max = wind.draw(rows, Number(element('length').value), element('wind').checked);
    element('speedMax').textContent = max.toFixed(2);
  }
  function renderPreview() {
    if (!startingData) return;
    clearGraph();
    view?.dispose();
    view = wind = null;
    axes = element('swapAxes').value.split(',').map(Number);
    const building = createUploadedBuilding(startingData, axes);
    view = createScene(element('viewport'), building);
    wind = createWindLayer(view.world);
    graph = createGraphLayer(view.world);
    controls.forEach(id => { element(id).disabled = false; });
    view.setScale(element('scale').value.split(',').map(Number));
    drawWind();
    summary.textContent = `Grid: ${building.meta.shape.join(' × ')} · Wind: ${startingData.wind.shape[3]} channels`;
    status.textContent = 'Step 1 complete · Data loaded.';
  }
  function reset() {
    clearGraph();
    view?.dispose();
    view = wind = graph = null;
    startingData = null;
    controls.forEach(id => { element(id).disabled = true; });
    summary.textContent = '';
    element('speedMax').textContent = '—';
    status.textContent = 'Choose both files, then load data.';
  }
  buildingInput.onchange = windInput.onchange = reset;
  element('swapAxes').onchange = () => {
    try { renderPreview(); }
    catch (error) { status.textContent = `Could not display data: ${error.message}`; }
  };
  for (const id of ['mode', 'axis', 'layer', 'step', 'length']) element(id).oninput = drawWind;
  element('wind').onchange = () => wind?.setVisible(element('wind').checked);
  element('scale').onchange = () => view?.setScale(element('scale').value.split(',').map(Number));
  element('home').onclick = () => view?.home();
  element('top').onclick = () => view?.home(true);
  element('graphGenerate').onsubmit = event => {
    event.preventDefault();
    if (!startingData || !view) return;
    clearGraph();
    element('generateGraph').disabled = true;
    element('graphStatus').textContent = 'Generating graph…';
    try {
      graphWorker = new Worker(new URL('./generate-graph.js', import.meta.url), { type: 'module' });
      const worker = graphWorker;
      worker.onmessage = ({ data }) => {
        if (worker !== graphWorker) return;
        if (data.status) element('graphStatus').textContent = data.status;
        if (data.result || data.error) {
          try {
            if (data.error) throw new Error(data.error);
            graph.draw({ boxes: [], nodes: new Float32Array(data.result.points.flat()),
              edges: new Uint32Array(data.result.edges.flat()), edgeTypes: data.result.edgeTypes });
            startingData.graph = data.result;
            if (data.result.points.length) {
              planning = bindTrajectoryControls(view.world, {
                nodes: data.result.points, edges: data.result.edges, solver: data.result.solver,
                defaultPoints: { start: data.result.points[0], end: data.result.points.at(-1) },
              }, { autoSolve: false, graphLayer: graph });
              planningControls.forEach(id => { element(id).disabled = false; });
              element('caseStatus').textContent = 'Graph ready · Set Start / End, then Solve.';
              element('pathPlanning').open = true;
            } else {
              element('caseStatus').textContent = 'Graph has no connected points. Adjust graph settings.';
            }
            const expanded = data.result.expandedGraph;
            const counts = data.result.metadata.counts;
            element('graphStatus').textContent = `${data.result.points.length} points · ${data.result.edges.length} edges\nAdaptive: ${counts.adaptive} · Skeleton: ${counts.skeleton} · Bridge: ${counts.bridge}\nExpanded: ${expanded.nodes.length} states · ${expanded.edges.length} edges · Energy (J)`;
          } catch (error) {
            element('graphStatus').textContent = 'Could not generate graph: ' + error.message;
          } finally {
            worker.terminate(); graphWorker = null; element('generateGraph').disabled = false;
          }
        }
      };
      worker.onerror = event => {
        if (worker !== graphWorker) return;
        element('graphStatus').textContent = 'Could not generate graph: ' + event.message;
        worker.terminate(); graphWorker = null; element('generateGraph').disabled = false;
      };
      worker.postMessage({ input: { building: startingData.building, wind: startingData.wind }, options: {
        axes, thresholdPercentage: Number(element('thresholdPercentage').value) / 100,
        minCubic: ['minX', 'minY', 'minZ'].map(id => Number(element(id).value)),
        voxelSize: ['voxelX', 'voxelY', 'voxelZ'].map(id => Number(element(id).value)),
        minCubicUnit: element('minUnit').value,
        windowSize: Number(element('skeletonWindow').value), mergeRadius: Number(element('mergeRadius').value),
        skeletonRadius: Number(element('skeletonRadius').value), bridgeRadius: Number(element('bridgeRadius').value),
        neighbors: Number(element('neighbors').value),
      } });
    } catch (error) {
      clearGraph();
      element('graphStatus').textContent = 'Could not generate graph: ' + error.message;
    }
  };
  for (const id of ['thresholdPercentage', 'minX', 'minY', 'minZ', 'voxelX', 'voxelY', 'voxelZ', 'minUnit',
    'skeletonWindow', 'mergeRadius', 'skeletonRadius', 'bridgeRadius', 'neighbors']) element(id).onchange = clearGraph;
  element('loadData').onsubmit = async event => {
    event.preventDefault();
    reset();
    button.disabled = buildingInput.disabled = windInput.disabled = true;
    status.textContent = 'Loading data…';
    try {
      startingData = await loadStartingData(buildingInput.files[0], windInput.files[0]);
      renderPreview();
    } catch (error) {
      status.textContent = `Could not load data: ${error.message}`;
    } finally {
      button.disabled = buildingInput.disabled = windInput.disabled = false;
    }
  };
}
