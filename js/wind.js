import * as THREE from 'three';
import { loadWind } from './loaders.js';

// File chứa velocity XYZ trên lưới thưa; vật cản được đánh dấu NaN.
// Kết quả mỗi mẫu: [x, y, z, vx, vy, vz, speed], tọa độ ở tâm voxel.
export function sampleCompiled(values, shape, { axis, layer, step, mode }, stride = 4) {
  const validLayer = Number.isInteger(layer) && layer >= 0 && layer < shape[axis];
  const validStep = Number.isInteger(step) && step >= 4 && step <= 24 && step % stride === 0;
  if (![0, 1, 2].includes(axis) || !validLayer || !validStep || !['slice', 'volume'].includes(mode)) {
    throw new Error('Invalid wind field parameters.');
  }

  const packedShape = shape.map((size, dimension) => (
    mode === 'slice' && dimension === axis ? 1 : Math.ceil(size / stride)
  ));
  const expectedLength = packedShape.reduce((count, size) => count * size, 1) * 3;
  if (values.length !== expectedLength) throw new Error('Unexpected wind field file size.');

  const rows = [];
  const starts = shape.map((_, dimension) => mode === 'slice' && dimension === axis ? layer : 0);
  const ends = shape.map((size, dimension) => mode === 'slice' && dimension === axis ? layer + 1 : size);
  for (let x = starts[0]; x < ends[0]; x += step) {
    for (let y = starts[1]; y < ends[1]; y += step) {
      for (let z = starts[2]; z < ends[2]; z += step) {
        // Trục cố định của file slice có chiều dài 1, nên chỉ số packed là 0.
        const packedPoint = [x, y, z].map((value, dimension) => (
          mode === 'slice' && dimension === axis ? 0 : value / stride
        ));
        const index = ((packedPoint[0] * packedShape[1] + packedPoint[1]) * packedShape[2] + packedPoint[2]) * 3;
        const velocity = values.subarray(index, index + 3);
        if (velocity.every(Number.isFinite)) {
          rows.push(x + 0.5, y + 0.5, z + 0.5, ...velocity, Math.hypot(...velocity));
        }
      }
    }
  }
  return new Float32Array(rows);
}

export function createWindLayer(world) {
  let arrows;
  const arrowShape = new THREE.ConeGeometry(0.8, 3, 5);
  arrowShape.translate(0, 1.5, 0);
  const shaftShape = new THREE.CylinderGeometry(0.12, 0.12, 1, 4);
  shaftShape.translate(0, 0.5, 0);
  const material = new THREE.MeshBasicMaterial();
  const object = new THREE.Object3D();
  const direction = new THREE.Vector3(), position = new THREE.Vector3();
  const color = new THREE.Color(), yAxis = new THREE.Vector3(0, 1, 0);

  function draw(rows, length, visible) {
    // InstancedMesh cũ được bỏ; geometry/material dùng chung giữa các lần vẽ.
    if (arrows) {
      world.remove(arrows);
      for (const child of arrows.children) child.dispose();
    }
    arrows = new THREE.Group();
    arrows.visible = visible;
    const count = rows.length / 7; // 7 giá trị cho mỗi mẫu gió.
    const heads = new THREE.InstancedMesh(arrowShape, material, count);
    const shafts = new THREE.InstancedMesh(shaftShape, material, count);
    let max = 0;
    for (let i = 6; i < rows.length; i += 7) max = Math.max(max, rows[i]);
    let used = 0;
    for (let i = 0; i < rows.length; i += 7) {
      const speed = rows[i + 6];
      if (speed < 1e-8) continue;
      position.fromArray(rows, i);
      direction.fromArray(rows, i + 3).normalize();
      color.setHSL((1 - speed / (max || 1)) * 0.65, 0.85, 0.55);
      object.quaternion.setFromUnitVectors(yAxis, direction);
      object.position.copy(position);
      // Thân chiếm 72%, đầu chiếm 28% tổng chiều dài mũi tên.
      object.scale.set(1, length * 0.72, 1);
      object.updateMatrix();
      shafts.setMatrixAt(used, object.matrix);
      shafts.setColorAt(used, color);

      object.position.copy(position).addScaledVector(direction, length * 0.72);
      object.scale.set(length * 0.09, length * 0.28 / 3, length * 0.09);
      object.updateMatrix();
      heads.setMatrixAt(used, object.matrix);
      heads.setColorAt(used, color);
      used++;
    }
    heads.count = shafts.count = used;
    arrows.add(heads, shafts);
    world.add(arrows);
    return max;
  }

  function setVisible(visible) {
    if (arrows) arrows.visible = visible;
  }

  return { draw, setVisible };
}


export async function bindControls(meta, view, wind) {
  const $ = id => document.getElementById(id);
  const status = $('status');
  const shape = meta.shape;
  let rows = new Float32Array();
  let sequence = 0;
  let fieldPath = '';
  let field;

  function drawWind() {
    const max = wind.draw(rows, Number($('length').value), true);
    $('speedMax').textContent = max.toFixed(2);
  }

  function syncLabels() {
    const axis = Number($('axis').value);
    $('layer').max = shape[axis] - 1;
    $('layerValue').textContent = `${$('layer').value} / ${shape[axis] - 1}`;
    $('stepValue').textContent = `${$('step').value} cells`;
    $('lengthValue').textContent = `${$('length').value} cells`;
    $('axis').disabled = $('layer').disabled = $('mode').value === 'volume';
  }

  async function updateWind() {
    // Người dùng có thể đổi slice khi fetch cũ chưa xong.
    // Chỉ request mới nhất được cập nhật scene; không cho dữ liệu cũ ghi đè.
    const request = ++sequence;
    syncLabels();
    status.textContent = 'Updating wind field…';
    const options = {
      axis: Number($('axis').value),
      layer: Number($('layer').value),
      step: Number($('step').value),
      mode: $('mode').value,
    };
    const path = options.mode === 'volume'
      ? './data/wind-volume.bin.gz'
      : `./data/wind-${options.axis}-${options.layer}.bin.gz`;
    try {
      if (fieldPath !== path) {
        const loaded = await loadWind(path);
        if (request !== sequence) return;
        field = loaded;
        fieldPath = path;
      }
      if (request !== sequence) return;
      rows = sampleCompiled(field, shape, options, meta.stride);
      drawWind();
      status.textContent = rows.length ? '' : 'No valid vectors in this slice.';
    } catch (error) {
      if (request === sequence) {
        status.textContent = error.message;
        wind.setVisible(false);
      }
    }
  }

  $('home').onclick = () => view.home();
  $('top').onclick = () => view.home(true);
  $('scale').onchange = () => view.setScale($('scale').value.split(',').map(Number));

  for (const id of ['axis', 'layer', 'step', 'mode']) $(id).onchange = updateWind;
  for (const id of ['layer', 'step']) $(id).oninput = syncLabels;
  $('length').oninput = () => {
    syncLabels();
    drawWind();
  };

  await updateWind();
}
