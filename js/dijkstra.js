// Dựng graph trạng thái theo case_all_dijkstra_expand.ipynb.
// Đầu vào: nodes XYZ theo voxel, edges là cặp chỉ số node.
// Đầu ra: graph có trọng số năng lượng (J), không dùng khoảng cách làm trọng số.
export const VOXEL_SIZE_METERS = Object.freeze([4, 4, 1.5]);

// Chỉ dùng để tách out/in khi HIỂN THỊ graph expand.
// Không thay đổi độ dài cạnh bay hay công thức tính năng lượng.
const STATE_DISPLAY_OFFSET = 0.15;

const subtractVectors = (a, b) => a.map((value, axis) => value - b[axis]);
const addVectors = (a, b) => a.map((value, axis) => value + b[axis]);
const scaleVector = (vector, factor) => vector.map(value => value * factor);
const dotProduct = (a, b) => a.reduce((sum, value, axis) => sum + value * b[axis], 0);
const vectorLength = vector => Math.hypot(...vector);
const toMeters = point => point.map((value, axis) => value * VOXEL_SIZE_METERS[axis]);

function angleDegrees(a, b) {
  const cosine = dotProduct(a, b) / (vectorLength(a) * vectorLength(b));
  // Sai số số thực có thể làm cosine lệch nhẹ ra ngoài [-1, 1].
  return Math.acos(Math.max(-1, Math.min(1, cosine))) * 180 / Math.PI;
}

// get_energy trong notebook dùng hằng số riêng, không đọc config.json.
// Giữ nguyên để đối chiếu kết quả với notebook.
const FLIGHT_PARAMETERS = {
  rho: 1.18,
  Cd: 1,
  Af: 0.25,
  m: 4,
  A: 0.45,
  Pp_hover: 65,
  g_acc: [0, 0, -9.81],
};

// config.json defaults for case 1; flightPower retains get_energy's own constants.
export const SOLVER_DEFAULTS = Object.freeze({
  speed: 15,
  parameters: Object.freeze({ ...FLIGHT_PARAMETERS, g_acc: Object.freeze([0, 0, -9.81]),
    dt: 4, dt_takeoff: 4, dt_landing: 4, Tmax: 70, angle_threshold: 60 }),
});

/** Công suất (W) và lực đẩy (N) cho rẽ/cất/hạ cánh; dùng drone_params từ config. */
export function dronePower(velocity, wind, acceleration, parameters) {
  const airVelocity = subtractVectors(velocity, wind);
  const dragForce = scaleVector(
    airVelocity,
    -0.5 * parameters.rho * parameters.Cd * parameters.Af * vectorLength(airVelocity),
  );
  const gravityForce = scaleVector(parameters.g_acc, parameters.m);
  const accelerationForce = scaleVector(acceleration, parameters.m);
  const thrustVector = subtractVectors(
    subtractVectors(accelerationForce, gravityForce),
    dragForce,
  );
  const thrust = vectorLength(thrustVector);
  const climbSpeed = thrust ? dotProduct(airVelocity, thrustVector) / thrust : airVelocity[2];
  const inducedSpeed = -climbSpeed / 2 + Math.sqrt(
    (climbSpeed / 2) ** 2 + thrust / (2 * parameters.rho * parameters.A),
  );

  const usefulPower = dotProduct(thrustVector, airVelocity);
  const inducedPower = thrust * inducedSpeed;
  const hoverThrust = parameters.m * vectorLength(parameters.g_acc);
  const profilePower = parameters.Pp_hover * (thrust / hoverThrust) ** 1.5;

  return { thrust, power: usefulPower + inducedPower + profilePower };
}

/** Công suất tại một đầu cạnh bay thẳng, theo công thức get_energy của notebook. */
function flightPower(velocity, wind) {
  const parameters = FLIGHT_PARAMETERS;
  const airVelocity = subtractVectors(velocity, wind);
  const dragForce = scaleVector(
    airVelocity,
    -0.5 * parameters.rho * parameters.Cd * parameters.Af * vectorLength(airVelocity),
  );
  const thrustVector = subtractVectors(scaleVector(parameters.g_acc, -parameters.m), dragForce);
  const thrust = vectorLength(thrustVector);
  const climbSpeed = dotProduct(airVelocity, thrustVector) / thrust;
  const inducedSpeed = -climbSpeed / 2 + Math.sqrt(
    (climbSpeed / 2) ** 2 + thrust / (2 * parameters.rho * parameters.A),
  );

  const idealPower = thrust * (inducedSpeed + climbSpeed);
  const dragPower = 0.5 * parameters.rho * parameters.Cd * parameters.Af * vectorLength(airVelocity) ** 3;
  const profilePower = parameters.Pp_hover * (thrust / (parameters.m * 9.81)) ** 1.5;

  return { thrust, power: idealPower + dragPower + profilePower };
}

/** Chi phí rẽ bổ sung giữa hai cạnh bay; không tính lại năng lượng bay nền. */
export function maneuver(incomingFlight, outgoingFlight, parameters) {
  // Notebook xét chuyển vận tốc từ giữa cạnh vào đến giữa cạnh ra.
  const duration = 0.5 * (
    incomingFlight.length / incomingFlight.speed +
    outgoingFlight.length / outgoingFlight.speed
  );
  const velocitySum = addVectors(incomingFlight.velocity, outgoingFlight.velocity);
  const sumLength = vectorLength(velocitySum);
  const averageSpeed = (incomingFlight.speed + outgoingFlight.speed) / 2;
  const middleVelocity = sumLength > 1e-12
    ? scaleVector(velocitySum, averageSpeed / sumLength)
    : incomingFlight.velocity;
  const wind = scaleVector(addVectors(incomingFlight.wind, outgoingFlight.wind), 0.5);
  const acceleration = scaleVector(
    subtractVectors(outgoingFlight.velocity, incomingFlight.velocity),
    1 / duration,
  );

  const turning = dronePower(middleVelocity, wind, acceleration, parameters);
  const baseline = dronePower(middleVelocity, wind, [0, 0, 0], parameters);
  return {
    weight: Math.max(0, turning.power - baseline.power) * duration,
    thrust: turning.thrust,
    angle: angleDegrees(incomingFlight.velocity, outgoingFlight.velocity),
  };
}

/**
 * Input: data={nodes:Point[],edges:[i,j][]}, input={speed,parameters,flightWind,transitionWind}.
 * Optional input.start_point/end_point add S/T, takeoff/landing and endpoint filters.
 * Without them, return all out/in states and flight/maneuver edges; start/end=null.
 * Output edges={from,to,kind,weight,...}, with directed energy weight in Joules.
 */
export function buildExpandedGraph(data, input, options = {}) {
  const constraints = { turningAngle: false, trajectoryAngle: false, thrust: false, ...options };
  if (!Object.values(constraints).every(value => typeof value === 'boolean')) {
    throw new Error('Constraint options must be boolean.');
  }
  const parameters = input.parameters;
  const speed = input.speed;
  const isVector3 = value => Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);
  const hasEndpoints = input.start_point !== undefined || input.end_point !== undefined;
  if (constraints.trajectoryAngle && !hasEndpoints) throw new Error('Trajectory angle requires start/end points.');

  // Kiểm tra dữ liệu JSON trước khi chia độ dài/thời gian hoặc tính năng lượng.
  const positiveValues = [
    speed, parameters.rho, parameters.Cd, parameters.Af, parameters.m,
    parameters.A, parameters.Tmax, parameters.dt_takeoff, parameters.dt_landing,
  ];
  const validVectors = data.nodes.every(isVector3) &&
    (!hasEndpoints || (data.nodes.length > 0 && isVector3(input.start_point) && isVector3(input.end_point))) &&
    isVector3(parameters.g_acc) && vectorLength(parameters.g_acc) > 0;
  const validParameters = positiveValues.every(value => Number.isFinite(value) && value > 0) &&
    Number.isFinite(parameters.Pp_hover) && parameters.Pp_hover >= 0 &&
    Number.isFinite(parameters.angle_threshold) &&
    parameters.angle_threshold >= 0 && parameters.angle_threshold <= 180;
  const validWind = [input.flightWind, input.transitionWind].every(
    rows => rows.length === data.nodes.length && rows.every(isVector3),
  );
  if (!validVectors || !validParameters || !validWind) {
    throw new Error('Invalid case input or drone parameters.');
  }

  // 1. Snap start/end theo khoảng cách voxel, giống nearest_point_index của notebook.
  function nearestNodeIndex(point) {
    let bestIndex = 0;
    for (let index = 1; index < data.nodes.length; index++) {
      const distance = vectorLength(subtractVectors(point, data.nodes[index]));
      const bestDistance = vectorLength(subtractVectors(point, data.nodes[bestIndex]));
      if (distance < bestDistance) bestIndex = index;
    }
    return bestIndex;
  }

  const start = hasEndpoints ? nearestNodeIndex(input.start_point) : null;
  const end = hasEndpoints ? nearestNodeIndex(input.end_point) : null;
  if (hasEndpoints && start === end) {
    // Hai điểm dùng chung node gần nhất: không cần tìm đường bên trong graph.
    return {
      nodes: [{ kind: 'start', position: data.nodes[start] }, { kind: 'end', position: data.nodes[end] }],
      edges: [{ from: 0, to: 1, kind: 'maneuver', weight: 0 }],
      flights: [], start, end, removed: 0, constraints,
      startOffset: vectorLength(subtractVectors(input.start_point, data.nodes[start])),
      endOffset: vectorLength(subtractVectors(input.end_point, data.nodes[end])),
    };
  }
  const heading = hasEndpoints ? subtractVectors(toMeters(data.nodes[end]), toMeters(data.nodes[start])) : null;

  // 2. Checkbox chỉ bật/tắt bộ lọc; công thức năng lượng luôn được tính.
  const flights = [];
  for (const pair of data.edges) {
    const validPair = pair.length === 2 && pair[0] !== pair[1] &&
      pair.every(index => Number.isInteger(index) && index >= 0 && index < data.nodes.length);
    if (!validPair) throw new Error('Invalid graph edge.');

    for (const [u, v] of [pair, [...pair].reverse()]) {
      const delta = subtractVectors(toMeters(data.nodes[v]), toMeters(data.nodes[u]));
      const length = vectorLength(delta);
      if (!length) throw new Error('Graph edges must have distinct positions.');
      const velocity = scaleVector(delta, speed / length);
      const powers = [u, v].map(index => flightPower(velocity, input.flightWind[index]));
      const maximumThrust = Math.max(...powers.map(sample => sample.thrust));

      const returnsToStart = v === start;
      const leavesEnd = u === end;
      const exceedsThrust = constraints.thrust && maximumThrust > parameters.Tmax;
      const pointsAwayFromEnd = constraints.trajectoryAngle && angleDegrees(delta, heading) >= 90;
      if (returnsToStart || leavesEnd || exceedsThrust || pointsAwayFromEnd) continue;

      // Tích phân hình thang: công suất trung bình hai đầu × thời gian bay.
      flights.push({
        u, v, length, speed, velocity,
        wind: scaleVector(addVectors(input.transitionWind[u], input.transitionWind[v]), 0.5),
        energy: (powers[0].power + powers[1].power) * 0.5 * length / speed,
      });
    }
  }

  // Without endpoints, keep every directed flight/turn and omit S/T and terminal links.
  const stateOffset = hasEndpoints ? 2 : 0;
  const nodes = hasEndpoints ? [
    { kind: 'start', position: data.nodes[start] },
    { kind: 'end', position: data.nodes[end] },
  ] : [];
  const edges = [];
  const incoming = new Map();
  const outgoing = new Map();

  function appendFlight(map, junction, flightIndex) {
    if (!map.has(junction)) map.set(junction, []);
    map.get(junction).push(flightIndex);
  }

  function addEdge(from, to, kind, weight, flight, turn) {
    if (!Number.isFinite(weight) || weight < 0) {
      throw new Error('Dijkstra requires finite nonnegative energy.');
    }
    edges.push({ from, to, kind, weight, flight, ...turn });
  }

  for (const [flightIndex, flight] of flights.entries()) {
    const startPoint = data.nodes[flight.u];
    const endPoint = data.nodes[flight.v];
    const outState = stateOffset + flightIndex * 2;
    const inState = outState + 1;

    // Tọa độ sơ đồ 15%/85% giúp thấy rõ hai trạng thái tại cùng junction.
    nodes.push(
      { kind: 'out', flight: flightIndex, position: addVectors(
        scaleVector(startPoint, 1 - STATE_DISPLAY_OFFSET),
        scaleVector(endPoint, STATE_DISPLAY_OFFSET),
      ) },
      { kind: 'in', flight: flightIndex, position: addVectors(
        scaleVector(startPoint, STATE_DISPLAY_OFFSET),
        scaleVector(endPoint, 1 - STATE_DISPLAY_OFFSET),
      ) },
    );
    addEdge(outState, inState, 'flight', flight.energy, flightIndex);
    appendFlight(outgoing, flight.u, flightIndex);
    appendFlight(incoming, flight.v, flightIndex);

    if (flight.u === start) {
      const duration = parameters.dt_takeoff;
      const power = dronePower(
        scaleVector(flight.velocity, 0.5), flight.wind,
        scaleVector(flight.velocity, 1 / duration), parameters,
      ).power;
      addEdge(0, outState, 'takeoff', power * duration, flightIndex);
    }
    if (flight.v === end) {
      const duration = parameters.dt_landing;
      // Notebook dùng gia tốc hạ cánh dương; giữ nguyên để tái lập kết quả.
      const power = dronePower(
        scaleVector(flight.velocity, 0.5), flight.wind,
        scaleVector(flight.velocity, 1 / duration), parameters,
      ).power;
      addEdge(inState, 1, 'landing', power * duration, flightIndex);
    }
  }

  // 4. Nối in(cạnh vào) -> out(cạnh ra) tại mỗi junction đủ điều kiện rẽ.
  let removed = 0;
  for (const [junction, arrivals] of incoming) {
    if (junction === start || junction === end) continue;
    const departures = outgoing.get(junction) || [];
    for (const arrivalIndex of arrivals) {
      for (const departureIndex of departures) {
        const turn = maneuver(flights[arrivalIndex], flights[departureIndex], parameters);
        if ((constraints.thrust && turn.thrust > parameters.Tmax) ||
            (constraints.turningAngle && turn.angle > parameters.angle_threshold)) {
          removed++;
          continue;
        }
        const arrivalState = stateOffset + 1 + arrivalIndex * 2;
        const departureState = stateOffset + departureIndex * 2;
        addEdge(arrivalState, departureState, 'maneuver', turn.weight, undefined, turn);
      }
    }
  }

  return {
    nodes, edges, flights, start, end, removed, constraints,
    startOffset: hasEndpoints ? vectorLength(subtractVectors(input.start_point, data.nodes[start])) : null,
    endOffset: hasEndpoints ? vectorLength(subtractVectors(input.end_point, data.nodes[end])) : null,
  };
}

/** Tìm đường theo weight (J). path chứa CHỈ SỐ CẠNH expand, không phải node XYZ. */
export function dijkstra(graph, start = 0, end = 1) {
  const adjacency = Array.from({ length: graph.nodes.length }, () => []);
  graph.edges.forEach((edge, edgeIndex) => {
    if (!Number.isFinite(edge.weight) || edge.weight < 0) {
      throw new Error('Invalid Dijkstra weight.');
    }
    adjacency[edge.from].push(edgeIndex);
  });

  const distance = new Float64Array(graph.nodes.length).fill(Infinity);
  const previousEdge = new Int32Array(graph.nodes.length).fill(-1);
  const visited = new Uint8Array(graph.nodes.length);
  distance[start] = 0;

  // ponytail: O(V^2) minimum scan fits this ~2,000-state demo; use a heap for larger graphs.
  for (let step = 0; step < graph.nodes.length; step++) {
    let currentNode = -1;
    for (let index = 0; index < distance.length; index++) {
      if (!visited[index] && (currentNode === -1 || distance[index] < distance[currentNode])) {
        currentNode = index;
      }
    }
    if (currentNode === -1 || distance[currentNode] === Infinity) break;

    if (currentNode === end) {
      // Đi ngược các cạnh tiền nhiệm từ T về S, rồi đảo để có thứ tự bay.
      const path = [];
      let node = end;
      while (node !== start) {
        const edgeIndex = previousEdge[node];
        path.push(edgeIndex);
        node = graph.edges[edgeIndex].from;
      }
      return { cost: distance[end], path: path.reverse() };
    }

    visited[currentNode] = 1;
    for (const edgeIndex of adjacency[currentNode]) {
      const edge = graph.edges[edgeIndex];
      const candidateCost = distance[currentNode] + edge.weight;
      if (candidateCost < distance[edge.to]) {
        distance[edge.to] = candidateCost;
        previousEdge[edge.to] = edgeIndex;
      }
    }
  }

  // Không có đường hợp lệ: tránh trả Infinity để có thể export JSON.
  return { cost: null, path: [] };
}

/**
 * @typedef {object} SolveResult
 * @property {'ok'|'no_path'} status
 * @property {number[][]} trajectory Polyline Point[] theo voxel, thứ tự start -> end.
 * @property {number|null} energyJ Năng lượng trên graph (J).
 * @property {number} lengthM Chiều dài trajectory trên graph (m).
 * @property {number} flightSeconds Thời gian cruise, không bao gồm cất/hạ cánh (s).
 * @property {object} expandedGraph Graph có hướng: nodes, edges, flights và metadata snap.
 * @property {number[]} expandedPath Chỉ số cạnh trong expandedGraph.edges theo thứ tự nghiệm.
 * @property {{requested: number[], snapped: number[], nodeIndex: number, offsetCells: number}} start
 * @property {{requested: number[], snapped: number[], nodeIndex: number, offsetCells: number}} end
 */

/**
 * Nhận điểm yêu cầu, dựng graph expand rồi giải Dijkstra; không dùng fetch/DOM/Three.js.
 * @param {{start: number[], end: number[]}} points Hai Point=[x,y,z] hữu hạn theo voxel XYZ.
 * @param {import('./loaders.js').GraphData} data GraphData từ loadGraph(): nodes, edges và solver.
 * @param {{turningAngle?: boolean, trajectoryAngle?: boolean, thrust?: boolean}} options
 * Bộ lọc góc rẽ, hướng Start -> End <90°, lực đẩy: mặc định đều false.
 * @returns {SolveResult} status, trajectory Point[], energyJ, lengthM,
 * flightSeconds, expandedGraph, expandedPath (chỉ số cạnh trạng thái), start, end.
 * No path: status='no_path', trajectory=[], energyJ=null. Input sai sẽ throw.
 * Trajectory bắt đầu/kết thúc tại node gần nhất; không nối thêm tới điểm yêu cầu.
 */
export function solveDijkstra(points, data, options = {}) {
  if (!points || !data?.solver) throw new Error('Points and graph.solver are required.');
  const input = { ...data.solver, start_point: points.start, end_point: points.end };
  const graph = buildExpandedGraph(data, input, options);
  const solution = dijkstra(graph);
  const chosenFlights = solution.path
    .map(edgeIndex => graph.edges[edgeIndex])
    .filter(edge => edge.kind === 'flight')
    .map(edge => graph.flights[edge.flight]);
  const trajectory = chosenFlights.length
    ? [data.nodes[chosenFlights[0].u], ...chosenFlights.map(flight => data.nodes[flight.v])]
    : solution.cost === null ? [] : [data.nodes[graph.start]];

  return {
    status: solution.cost === null ? 'no_path' : 'ok',
    expandedGraph: graph,
    expandedPath: solution.path,
    energyJ: solution.cost,
    trajectory,
    lengthM: chosenFlights.reduce((sum, flight) => sum + flight.length, 0),
    // Chỉ thời gian cruise; không cộng takeoff/landing hoặc thời gian rẽ lần nữa.
    flightSeconds: chosenFlights.reduce((sum, flight) => sum + flight.length / flight.speed, 0),
    start: { requested: points.start, snapped: data.nodes[graph.start], nodeIndex: graph.start, offsetCells: graph.startOffset },
    end: { requested: points.end, snapped: data.nodes[graph.end], nodeIndex: graph.end, offsetCells: graph.endOffset },
  };
}
