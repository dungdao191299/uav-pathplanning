import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// Nhận kết quả loadBuilding(); chỉ dựng scene và gắn object đã load.
export function createScene(viewport, { meta, object: building }) {
  const shape = meta.shape;
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  viewport.append(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#09111d');
  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 3000);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;

  scene.add(new THREE.HemisphereLight(0xdcecff, 0x243645, 2.4));
  const light = new THREE.DirectionalLight(0xffffff, 2.8);
  light.position.set(100, 300, 200);
  scene.add(light);

  // Toàn bộ data dùng Z hướng lên; Three.js mặc định dùng Y hướng lên.
  // Xoay group chung thay vì sửa riêng building, graph, gió và trajectory.
  const world = new THREE.Group();
  world.rotation.x = -Math.PI / 2;
  scene.add(world);
  world.add(building);

  const bounds = new THREE.Box3(new THREE.Vector3(), new THREE.Vector3(...shape));
  world.add(new THREE.Box3Helper(bounds, 0x2b4055), new THREE.AxesHelper(45));
  const center = new THREE.Vector3(...shape.map(n => n / 2));
  const floor = new THREE.GridHelper(1, 30, 0x365166, 0x1e3041);
  floor.rotation.x = Math.PI / 2;
  floor.scale.set(shape[0], 1, shape[1]);
  floor.position.set(center.x, center.y, -0.05);
  world.add(floor);

  function home(top = false) {
    // Camera căn theo kích thước hiện tại của world sau khi đổi scale.
    const target = world.localToWorld(center.clone());
    const radius = Math.max(...shape.map((n, a) => n * world.scale.getComponent(a)));
    camera.far = radius * 10;
    camera.updateProjectionMatrix();
    controls.target.copy(target);
    camera.up.set(0, top ? 0 : 1, top ? -1 : 0);
    camera.position.copy(target).add(
      top
        ? new THREE.Vector3(0, radius * 1.8, 0)
        : new THREE.Vector3(radius * 1.15, radius * 0.85, radius * 1.35)
    );
    controls.update();
  }

  function setScale(scale) {
    // Chỉ đổi hiển thị, không đổi dữ liệu hay kết quả năng lượng của solver.
    world.scale.set(...scale);
    home();
  }

  new ResizeObserver(() => {
    const { width, height } = viewport.getBoundingClientRect();
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
  }).observe(viewport);

  home();
  renderer.setAnimationLoop(() => {
    controls.update();
    renderer.render(scene, camera);
  });

  return { world, building, home, setScale };
}
