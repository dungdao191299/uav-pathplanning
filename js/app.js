import { loadBuilding, loadGraph } from './loaders.js';
import { createScene } from './scene.js';
import { createWindLayer, bindControls } from './wind.js';
import { createGraphLayer, bindGraphControls } from './graph.js';
import { bindTrajectoryControls } from './trajectory.js';

const status = document.getElementById('status');

try {
  // 1. Tải geometry và kích thước chung trước khi khởi tạo các layer.
  const building = await loadBuilding();
  const view = createScene(document.getElementById('viewport'), building);
  const wind = createWindLayer(view.world);
  // 2. Gió và graph tải độc lập. Solver cần graph đã tải xong.
  await Promise.all([
    loadGraph().then(data => {
      bindGraphControls(createGraphLayer(view.world), data);
      bindTrajectoryControls(view.world, data);
    }).catch(error => {
      document.getElementById('graphStatus').textContent = `Could not load graph: ${error.message}`;
      document.getElementById('caseStatus').textContent = 'Case data unavailable.';
      document.getElementById('solveCase').disabled = true;
    }),
    // bindControls gọi loadWind(path) khi đổi slice và vẽ bằng createWindLayer.
    bindControls(building.meta, view, wind),
  ]);
} catch (error) {
  console.error(error);
  status.textContent = `Could not display 3D: ${error.message} Check the data and your browser's WebGL support.`;
}
