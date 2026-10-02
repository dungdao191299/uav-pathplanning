import * as THREE from 'three';
import { solveDijkstra } from './dijkstra.js';

// CHỈNH CẤU HÌNH VẼ TẠI ĐÂY. Bán kính theo voxel; opacity nằm trong [0, 1].
// Scale của toàn cảnh được áp dụng bởi scene.js, không nhân tọa độ lần nữa.
const TRAJECTORY_STYLE = {
  pathColor: 0xff65db,
  pathRadius: 2,
  pathRadialSegments: 8,
  markerRadius: 2.5,
  startColor: 0xb4f277,
  endColor: 0xf8877c,
  stateColor: 0x719fff,
  stateSize: 2,
  expandedOpacity: 0.45,
  edgeColors: {
    flight: 0x719fff,
    maneuver: 0xffae62,
    takeoff: 0xb4f277,
    landing: 0xf8877c,
  },
};

/**
 * Vẽ polyline XYZ; không tải dữ liệu và không chạy solver.
 * @param {import('three').Group} world Group XYZ chưa xoay/scale riêng các điểm.
 * @param {number[][]} trajectory [[x,y,z], ...] theo voxel; [] nếu không có đường.
 * @param {object|null} expandedGraph Tùy chọn: result.expandedGraph để vẽ sơ đồ trạng thái.
 * @returns {{setVisible: function, clear: function}} Layer; name='trajectory' hoặc 'expanded'.
 * clear() gỡ layer và giải phóng tài nguyên GPU trước khi vẽ nghiệm khác.
 */
export function plotTrajectory(world, trajectory, expandedGraph = null) {
  const validPoints = Array.isArray(trajectory) && trajectory.every(
    point => Array.isArray(point) && point.length === 3 && point.every(Number.isFinite),
  );
  if (!validPoints) throw new Error('Trajectory must be an array of finite [x,y,z] points.');
  const caseGroup = new THREE.Group();
  caseGroup.name = 'trajectory-result';
  world.add(caseGroup);

  // Graph expand là sơ đồ trạng thái, không phải các đoạn bay bổ sung.
  const expandedGroup = new THREE.Group();
  expandedGroup.name = 'expanded';
  caseGroup.add(expandedGroup);

  const edgePositions = [];
  const edgeColors = [];
  const palette = Object.fromEntries(
    Object.entries(TRAJECTORY_STYLE.edgeColors).map(([kind, color]) => [kind, new THREE.Color(color)]),
  );
  for (const edge of expandedGraph?.edges || []) {
    for (const nodeIndex of [edge.from, edge.to]) {
      edgePositions.push(...expandedGraph.nodes[nodeIndex].position);
      edgeColors.push(...palette[edge.kind].toArray());
    }
  }

  const edgeGeometry = new THREE.BufferGeometry();
  edgeGeometry.setAttribute('position', new THREE.Float32BufferAttribute(edgePositions, 3));
  edgeGeometry.setAttribute('color', new THREE.Float32BufferAttribute(edgeColors, 3));
  const edgeMaterial = new THREE.LineBasicMaterial({
    vertexColors: true,
    transparent: true,
    opacity: TRAJECTORY_STYLE.expandedOpacity,
  });
  expandedGroup.add(new THREE.LineSegments(edgeGeometry, edgeMaterial));

  const stateGeometry = new THREE.BufferGeometry();
  const statePositions = (expandedGraph?.nodes || []).flatMap(node => node.position);
  stateGeometry.setAttribute('position', new THREE.Float32BufferAttribute(statePositions, 3));
  expandedGroup.add(new THREE.Points(stateGeometry, new THREE.PointsMaterial({
    color: TRAJECTORY_STYLE.stateColor,
    size: TRAJECTORY_STYLE.stateSize,
  })));

  // Mỗi đoạn trajectory là một cylinder: độ dày ổn định trên WebGL.
  // Dùng chung geometry/material qua InstancedMesh để không tạo nhiều draw call.
  const routeGroup = new THREE.Group();
  routeGroup.name = 'trajectory';
  caseGroup.add(routeGroup);

  const segmentCount = Math.max(0, trajectory.length - 1);
  const pathGeometry = new THREE.CylinderGeometry(
    TRAJECTORY_STYLE.pathRadius,
    TRAJECTORY_STYLE.pathRadius,
    1,
    TRAJECTORY_STYLE.pathRadialSegments,
  );
  const pathMaterial = new THREE.MeshBasicMaterial({ color: TRAJECTORY_STYLE.pathColor });
  const pathMesh = new THREE.InstancedMesh(pathGeometry, pathMaterial, segmentCount);
  const matrix = new THREE.Matrix4();
  const rotation = new THREE.Quaternion();
  const cylinderAxis = new THREE.Vector3(0, 1, 0);

  for (let index = 0; index < segmentCount; index++) {
    const startPoint = new THREE.Vector3(...trajectory[index]);
    const endPoint = new THREE.Vector3(...trajectory[index + 1]);
    const direction = endPoint.clone().sub(startPoint);
    const length = direction.length();
    const midpoint = startPoint.add(endPoint).multiplyScalar(0.5);

    rotation.setFromUnitVectors(cylinderAxis, direction.normalize());
    matrix.compose(midpoint, rotation, new THREE.Vector3(1, length, 1));
    pathMesh.setMatrixAt(index, matrix);
  }
  routeGroup.add(pathMesh);

  // Khi truyền graph expand, có thể vẽ S/T cả trong trường hợp no_path.
  const endpointPoints = trajectory.length
    ? [trajectory[0], trajectory.at(-1)]
    : (expandedGraph?.nodes.slice(0, 2).map(node => node.position) || []);
  for (const [index, point] of endpointPoints.entries()) {
    const color = index === 0 ? TRAJECTORY_STYLE.startColor : TRAJECTORY_STYLE.endColor;
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(TRAJECTORY_STYLE.markerRadius, 12, 8),
      new THREE.MeshBasicMaterial({ color }),
    );
    marker.position.set(...point);
    routeGroup.add(marker);
  }

  return {
    setVisible(name, visible) {
      caseGroup.getObjectByName(name).visible = visible;
    },
    clear() {
      // Gỡ object khỏi scene chưa đủ: phải giải phóng buffer/material GPU.
      world.remove(caseGroup);
      caseGroup.traverse(child => {
        child.geometry?.dispose();
        child.material?.dispose();
        if (child.isInstancedMesh) child.dispose();
      });
    },
  };
}

/** UI chỉ gọi solveDijkstra(points, data), rồi plotTrajectory(); không tự load data. */
export function bindTrajectoryControls(world, data) {
  const element = id => document.getElementById(id);
  const status = element('caseStatus');
  const solveButton = element('solveCase');
  const downloadButton = element('downloadCase');
  const input = data.solver;
  for (const name of ['start', 'end']) {
    ['x', 'y', 'z'].forEach((axis, index) => {
      element(`${name}-${axis}`).value = data.defaultPoints[name][index];
    });
  }
  let layer;
  let result;

  function syncVisibility() {
    if (!layer) return;
    for (const name of ['trajectory', 'expanded']) {
      layer.setVisible(name, element(`case-${name}`).checked);
    }
  }
  for (const name of ['trajectory', 'expanded']) {
    element(`case-${name}`).onchange = syncVisibility;
  }

  function solve() {
    solveButton.disabled = true;
    if (downloadButton) downloadButton.disabled = true;
    try {
      // Đọc lại input mỗi lần Solve; không thay đổi điểm mặc định trong graph.
      const points = Object.fromEntries(['start', 'end'].map(name => [name,
        ['x', 'y', 'z'].map(axis => {
          const value = element(`${name}-${axis}`).value.trim();
          return value === '' ? NaN : Number(value);
        }),
      ]));
      if (!Object.values(points).flat().every(Number.isFinite)) {
        throw new Error('Enter six valid coordinates.');
      }
      const constraints = Object.fromEntries(
        ['turningAngle', 'trajectoryAngle', 'thrust'].map(name => [name, element(`constraint-${name}`).checked]),
      );
      result = solveDijkstra(points, data, constraints);

      layer?.clear();
      layer = plotTrajectory(world, result.trajectory, result.expandedGraph);
      syncVisibility();

      const graph = result.expandedGraph;
      const lines = [
        `${graph.nodes.length} states · ${graph.edges.length} expanded edges`,
      ];
      if (result.status === 'no_path') {
        lines.push('No path found.');
      } else {
        lines.push(
          `${result.lengthM.toFixed(1)} m · ${(result.energyJ / 1000).toFixed(2)} kJ · ${result.flightSeconds.toFixed(1)} s`,
        );
      }
      status.textContent = lines.join('\n');
      if (downloadButton) downloadButton.disabled = false;
    } catch (error) {
      status.textContent = `Could not solve: ${error.message}`;
    } finally {
      solveButton.disabled = false;
    }
  }
  solveButton.onclick = solve;

  // Xuất cả cấu hình và graph để có thể kiểm tra lại nghiệm ngoài trình duyệt.
  if (downloadButton) downloadButton.onclick = () => {
    const payload = {
      case: data.caseName,
      configurationSource: data.configurationSource,
      parameters: input.parameters,
      speed: input.speed,
      ...result,
    };
    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'case1-dijkstra-expand.json';
    link.click();
    URL.revokeObjectURL(url);
  };

  solve();
}
