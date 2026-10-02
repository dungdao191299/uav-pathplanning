import * as THREE from 'three';

// CHỈNH CẤU HÌNH VẼ GRAPH GỐC TẠI ĐÂY.
// edgeColors theo edgeTypeNames: 0=adaptive, 1=skeleton, 2=bridge.
const GRAPH_STYLE = {
  freeBoxColor: "gray",
  boxOpacity: 0.1,
  nodeColor: 0xb4f277,
  nodeSize: 2,
  edgeColors: [0x3bcfdd, 0x3bcfdd, 0x3bcfdd],
  edgeOpacity: 0.4,
};

export function createGraphLayer(world) {
  const group = new THREE.Group();
  world.add(group);

  function clear() {
    for (const child of [...group.children]) {
      group.remove(child);
      child.geometry.dispose();
      child.material.dispose();
    }
  }

  // nodes: Float32 XYZ liên tiếp; edges: chỉ số node theo từng cặp.
  function draw({ boxes, nodes, edges, edgeTypes }) {
    clear();

    // Lấy 12 cạnh của box đơn vị, rồi biến đổi chúng theo min/max mỗi box.
    const boxShape = new THREE.BoxGeometry(1, 1, 1);
    const unitEdges = new THREE.EdgesGeometry(boxShape);
    boxShape.dispose();
    const template = unitEdges.getAttribute('position');
    const boxPositions = [];

    for (const box of boxes) {
      for (let vertex = 0; vertex < template.count; vertex++) {
        for (let axis = 0; axis < 3; axis++) {
          const unitCoordinate = template.array[vertex * 3 + axis] + 0.5;
          const size = box.max[axis] - box.min[axis];
          boxPositions.push(box.min[axis] + unitCoordinate * size);
        }
      }
    }
    unitEdges.dispose();

    const boxGeometry = new THREE.BufferGeometry();
    boxGeometry.setAttribute('position', new THREE.Float32BufferAttribute(boxPositions, 3));
    const boxLines = new THREE.LineSegments(boxGeometry, new THREE.LineBasicMaterial({
      color: GRAPH_STYLE.freeBoxColor,
      transparent: true,
      opacity: GRAPH_STYLE.boxOpacity,
    }));
    boxLines.name = 'boxes';
    group.add(boxLines);

    const nodeGeometry = new THREE.BufferGeometry();
    nodeGeometry.setAttribute('position', new THREE.BufferAttribute(nodes, 3));
    const points = new THREE.Points(nodeGeometry, new THREE.PointsMaterial({
      color: GRAPH_STYLE.nodeColor,
      size: GRAPH_STYLE.nodeSize,
    }));
    points.name = 'nodes';
    group.add(points);

    // LineSegments cần hai vertex cho mỗi cạnh, không nhận graph adjacency.
    const linePositions = new Float32Array(edges.length * 3);
    edges.forEach((nodeIndex, endpointIndex) => {
      const position = nodes.subarray(nodeIndex * 3, nodeIndex * 3 + 3);
      linePositions.set(position, endpointIndex * 3);
    });
    const palette = GRAPH_STYLE.edgeColors.map(color => new THREE.Color(color));
    const lineColors = Array.from(edges, (_, endpointIndex) => {
      const edgeIndex = Math.floor(endpointIndex / 2);
      return palette[edgeTypes[edgeIndex]].toArray();
    }).flat();

    const edgeGeometry = new THREE.BufferGeometry();
    edgeGeometry.setAttribute('position', new THREE.BufferAttribute(linePositions, 3));
    edgeGeometry.setAttribute('color', new THREE.Float32BufferAttribute(lineColors, 3));
    const edgeLines = new THREE.LineSegments(edgeGeometry, new THREE.LineBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: GRAPH_STYLE.edgeOpacity,
    }));
    edgeLines.name = 'edges';
    group.add(edgeLines);
  }

  function setVisible(name, visible) {
    const layer = group.getObjectByName(name);
    if (layer) layer.visible = visible;
  }

  return { draw, setVisible };
}

// Chỉ vẽ/gắn UI với dữ liệu loadGraph() đã trả về; không gọi fetch.
export function bindGraphControls(graph, data) {
  const $ = id => document.getElementById(id);
  const status = $('graphStatus');
  function syncVisibility() {
    for (const name of ['boxes', 'nodes', 'edges']) {
      graph.setVisible(name, $(`graph-${name}`).checked);
    }
  }
  for (const name of ['boxes', 'nodes', 'edges']) $(`graph-${name}`).onchange = syncVisibility;
  try {
    // File lưu mảng JSON; renderer dùng tọa độ/chỉ số dạng TypedArray.
    graph.draw({
      boxes: data.boxes.map(bounds => ({ min: bounds.slice(0, 3), max: bounds.slice(3) })),
      nodes: new Float32Array(data.nodes.flat()),
      edges: new Uint32Array(data.edges.flat()),
      edgeTypes: data.edgeTypes,
    });
    syncVisibility();
    status.textContent = `${data.boxes.length} free boxes · ${data.nodes.length} nodes · ${data.edges.length} edges`;
    return data;
  } catch (error) {
    status.textContent = `Could not display saved graph: ${error.message}`;
  }
}
