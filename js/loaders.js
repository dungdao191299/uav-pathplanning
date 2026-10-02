import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

/** @typedef {[number, number, number]} Point XYZ theo voxel, Z hướng lên; cho phép số thập phân. */
/**
 * @typedef {object} GraphData
 * @property {Point[]} nodes Node hình học; vị trí i có nodeId=i.
 * @property {[number, number][]} edges Cạnh vô hướng [nodeIdA,nodeIdB], không phải tọa độ.
 * @property {number[][]} boxes Mỗi box=[xmin,ymin,zmin,xmax,ymax,zmax].
 * @property {number[]} edgeTypes Một type/cạnh: 0=adaptive, 1=skeleton, 2=bridge.
 * @property {{start: Point, end: Point}} defaultPoints Điểm mặc định của case, caller có thể thay.
 * @property {{speed: number, parameters: object, flightWind: Point[], transitionWind: Point[]}} solver
 * Mỗi mảng wind có nodes.length vector vận tốc (m/s), không phải vị trí XYZ.
 */

/**
 * Load building (không tạo scene/camera, không vẽ).
 * @param {string} directory Thư mục chứa meta.json và building.glb.gz.
 * @returns {Promise<{meta: object, object: import('three').Group}>}
 * meta.shape = [nx,ny,nz]; object là mesh/group XYZ theo voxel, Z hướng lên.
 * meta.stride là bước lưới gió đã compile. Lỗi HTTP/GLB được throw cho caller.
 */
export async function loadBuilding(directory = './data/') {
  const base = directory.replace(/\/?$/, '/');
  const [meta, glb] = await Promise.all([
    fetchData(base + 'meta.json'),
    fetchData(base + 'building.glb.gz', true),
  ]);
  const model = await new GLTFLoader().parseAsync(glb, '');
  return { meta, object: model.scene };
}

/**
 * Load packed wind (không lấy mẫu lại, không vẽ).
 * @param {string} path File wind-volume.bin.gz hoặc wind-{axis}-{layer}.bin.gz.
 * @returns {Promise<Float32Array>} [vx,vy,vz, vx,vy,vz, ...] theo C-order XYZ.
 * Lưới file cách meta.stride voxel; trục cố định của slice có chiều dài 1.
 * NaN đánh dấu vật cản. Dùng sampleCompiled() để tạo mẫu [x,y,z,vx,vy,vz,speed].
 */
export async function loadWind(path = './data/wind-volume.bin.gz') {
  return new Float32Array(await fetchData(path, true));
}

/**
 * Load graph hình học và dữ liệu gió/tham số để solver chạy độc lập với DOM.
 * @param {string} graphPath File JSON/gzip: nodes Point[], edges [nodeId,nodeId][].
 * @param {string} solverPath File JSON/gzip: speed, parameters, flightWind, transitionWind.
 * @returns {Promise<GraphData>} Xem README mục API cho ví dụ schema đầy đủ.
 * defaultPoints = {start: Point, end: Point}; solver chứa dữ liệu năng lượng.
 * Hàm này KHÔNG tạo graph expand và KHÔNG giải Dijkstra.
 */
export async function loadGraph(
  graphPath = './data/notebook-graph.json.gz',
  solverPath = './data/case1-input.json.gz',
) {
  const [graph, input] = await Promise.all([fetchData(graphPath), fetchData(solverPath)]);
  return {
    ...graph,
    caseName: input.case,
    configurationSource: input.configurationSource,
    defaultPoints: { start: input.start_point, end: input.end_point },
    solver: {
      speed: input.speed,
      parameters: input.parameters,
      flightWind: input.flightWind,
      transitionWind: input.transitionWind,
    },
  };
}

async function fetchData(path, binary = false) {
  const response = await fetch(path);
  if (!response.ok) {
    throw new Error(`Could not load ${path} (HTTP ${response.status}).`);
  }
  // Một đường tải chung cho JSON thường, JSON gzip và binary gzip.
  // binary=true chỉ dành cho GLB/Float32; JSON luôn được parse tại đây.
  const decoded = path.endsWith('.gz')
    ? new Response(response.body.pipeThrough(new DecompressionStream('gzip')))
    : response;
  return binary ? decoded.arrayBuffer() : decoded.json();
}
