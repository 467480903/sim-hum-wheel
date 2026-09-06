'use strict';
/* =====================================================================
 * G2 仿真控制台
 *  - 轻量 URDF 解析器 (three.js, 手写, 支持 box/cylinder/sphere/STL mesh)
 *  - three.js 3D 视图 (ROS 坐标系 Z 朝上, 自带轨道相机控制)
 *  - rosbridge 通信: /joint_states /odom /map /scan /amcl_pose
 *    /nav_status /nav_feedback /chassis_move_status
 *    发布: *_joint_controller/joint_trajectory /cmd_vel
 *    /chassis_relative_move /nav_goal_pose /nav_cancel
 * ===================================================================== */

/* ---------------- 常量 ---------------- */
const GROUPS = [
  { key: 'waist', title: '腰部 (5轴)', ctrl: 'waist_joint_controller',
    joints: ['idx01_body_joint1', 'idx02_body_joint2', 'idx03_body_joint3',
             'idx04_body_joint4', 'idx05_body_joint5'] },
  { key: 'head', title: '头部 (3轴)', ctrl: 'head_joint_controller',
    joints: ['idx11_head_joint1', 'idx12_head_joint2', 'idx13_head_joint3'] },
  { key: 'larm', title: '左臂 (7轴)', ctrl: 'left_arm_joint_controller',
    joints: ['idx21_arm_l_joint1', 'idx22_arm_l_joint2', 'idx23_arm_l_joint3',
             'idx24_arm_l_joint4', 'idx25_arm_l_joint5', 'idx26_arm_l_joint6',
             'idx27_arm_l_joint7'] },
  { key: 'rarm', title: '右臂 (7轴)', ctrl: 'right_arm_joint_controller',
    joints: ['idx61_arm_r_joint1', 'idx62_arm_r_joint2', 'idx63_arm_r_joint3',
             'idx64_arm_r_joint4', 'idx65_arm_r_joint5', 'idx66_arm_r_joint6',
             'idx67_arm_r_joint7'] },
];
const ALL_JOINTS = GROUPS.flatMap(g => g.joints);
const $ = s => document.querySelector(s);

/* =====================================================================
 * 1. 轻量 URDF 解析器
 * ===================================================================== */
const _AX = new THREE.Vector3(1, 0, 0);
const _AY = new THREE.Vector3(0, 1, 0);
const _AZ = new THREE.Vector3(0, 0, 1);

function parseVec(s, def) {
  if (!s) return def.slice();
  const v = s.trim().split(/\s+/).map(Number);
  return (v.length === 3 && v.every(x => !isNaN(x))) ? v : def.slice();
}

/* URDF rpy (fixed-axis): R = Rz(y) * Ry(p) * Rx(r) */
function rpyToQuat(r, p, y) {
  const q = new THREE.Quaternion().setFromAxisAngle(_AZ, y);
  q.multiply(new THREE.Quaternion().setFromAxisAngle(_AY, p));
  q.multiply(new THREE.Quaternion().setFromAxisAngle(_AX, r));
  return q;
}

const meshProgress = { total: 0, done: 0 };
function meshDone() {
  meshProgress.done++;
  const hint = $('#loadModelHint');
  if (hint) {
    if (meshProgress.done >= meshProgress.total) hint.style.display = 'none';
    else hint.textContent = `模型加载 ${meshProgress.done}/${meshProgress.total}…`;
  }
}

function urdfColor(rgba) {
  return new THREE.Color(rgba[0], rgba[1], rgba[2]);
}

function makeVisualObject(vizEl, materials) {
  const grp = new THREE.Group();
  const origin = vizEl.querySelector('origin');
  const xyz = parseVec(origin && origin.getAttribute('xyz'), [0, 0, 0]);
  const rpy = parseVec(origin && origin.getAttribute('rpy'), [0, 0, 0]);
  grp.position.fromArray(xyz);
  grp.quaternion.copy(rpyToQuat(rpy[0], rpy[1], rpy[2]));

  /* 材质 */
  const matEl = vizEl.querySelector('material');
  let color = 0x9aa0aa;
  if (matEl) {
    const cEl = matEl.querySelector('color');
    if (cEl) {
      const rgba = parseVec(cEl.getAttribute('rgba'), [0.6, 0.6, 0.6, 1]);
      color = urdfColor(rgba.slice(0, 3));
    } else {
      const name = matEl.getAttribute('name');
      const def = name && materials[name];
      if (def) color = urdfColor(def.slice(0, 3));
    }
  }
  const mat = new THREE.MeshPhongMaterial({ color, shininess: 30, specular: 0x333333 });

  const geoEl = vizEl.querySelector('geometry');
  if (!geoEl) return grp;
  const box = geoEl.querySelector('box');
  const cyl = geoEl.querySelector('cylinder');
  const sph = geoEl.querySelector('sphere');
  const mesh = geoEl.querySelector('mesh');

  if (box) {
    const s = parseVec(box.getAttribute('size'), [0.1, 0.1, 0.1]);
    grp.add(new THREE.Mesh(new THREE.BoxGeometry(s[0], s[1], s[2]), mat));
  } else if (cyl) {
    const r = parseFloat(cyl.getAttribute('radius')) || 0.05;
    const l = parseFloat(cyl.getAttribute('length')) || 0.1;
    const g = new THREE.CylinderGeometry(r, r, l, 24);
    g.rotateX(Math.PI / 2);          // URDF 圆柱轴沿 Z
    grp.add(new THREE.Mesh(g, mat));
  } else if (sph) {
    const r = parseFloat(sph.getAttribute('radius')) || 0.05;
    grp.add(new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14), mat));
  } else if (mesh) {
    let fn = mesh.getAttribute('filename') || '';
    const scale = parseVec(mesh.getAttribute('scale'), [1, 1, 1]);
    fn = fn.replace(/^package:\/\/g2_description\//, '/');
    if (/\.stl$/i.test(fn)) {
      meshProgress.total++;
      const holder = new THREE.Group();
      grp.add(holder);
      new THREE.STLLoader().load(fn, geo => {
        const m = new THREE.Mesh(geo, mat);
        m.scale.set(scale[0], scale[1], scale[2]);
        holder.add(m);
        meshDone();
      }, undefined, () => { console.warn('mesh 加载失败:', fn); meshDone(); });
    }
  }
  return grp;
}

/**
 * 解析 URDF -> {root:Object3D, joints:Map, links:Map}
 * joints: name -> {obj, axisVec, type, limit, originQuat, basePos}
 */
function parseURDF(text) {
  const doc = new DOMParser().parseFromString(text, 'text/xml');
  if (doc.querySelector('parsererror')) throw new Error('URDF 解析失败');

  /* 顶层材质表 */
  const materials = {};
  for (const m of doc.querySelectorAll('robot > material')) {
    const c = m.querySelector('color');
    if (c) materials[m.getAttribute('name')] = parseVec(c.getAttribute('rgba'), [0.6, 0.6, 0.6, 1]);
  }

  const joints = [];
  const childSet = new Set();
  for (const j of doc.querySelectorAll('joint')) {
    const child = j.querySelector('child').getAttribute('link');
    childSet.add(child);
    joints.push({
      name: j.getAttribute('name'),
      type: j.getAttribute('type') || 'fixed',
      origin: j.querySelector('origin'),
      parent: j.querySelector('parent').getAttribute('link'),
      child,
      axisEl: j.querySelector('axis'),
      limitEl: j.querySelector('limit'),
    });
  }

  const linkEls = new Map();
  for (const l of doc.querySelectorAll('link')) linkEls.set(l.getAttribute('name'), l);

  /* 根 link: 从不出现在 child 中 */
  let rootName = null;
  for (const name of linkEls.keys()) {
    if (!childSet.has(name)) { rootName = name; break; }
  }
  if (!rootName) throw new Error('未找到根 link');

  const jointMap = new Map();
  const linkMap = new Map();

  const buildLink = (linkName, parentObj) => {
    const linkObj = new THREE.Object3D();
    linkObj.name = linkName;
    parentObj.add(linkObj);
    linkMap.set(linkName, linkObj);
    const el = linkEls.get(linkName);
    if (el) {
      for (const viz of el.querySelectorAll('visual')) linkObj.add(makeVisualObject(viz, materials));
    }
    for (const j of joints) {
      if (j.parent !== linkName) continue;
      const jObj = new THREE.Object3D();
      jObj.name = j.name;
      const xyz = parseVec(j.origin && j.origin.getAttribute('xyz'), [0, 0, 0]);
      const rpy = parseVec(j.origin && j.origin.getAttribute('rpy'), [0, 0, 0]);
      jObj.position.fromArray(xyz);
      jObj.quaternion.copy(rpyToQuat(rpy[0], rpy[1], rpy[2]));
      linkObj.add(jObj);
      const axis = parseVec(j.axisEl && j.axisEl.getAttribute('xyz'), [1, 0, 0]);
      const lim = j.limitEl;
      jointMap.set(j.name, {
        obj: jObj,
        axisVec: new THREE.Vector3(...axis).normalize(),
        type: j.type,
        limit: {
          lower: lim ? (parseFloat(lim.getAttribute('lower')) || -Math.PI) : -Math.PI,
          upper: lim ? (parseFloat(lim.getAttribute('upper')) || Math.PI) : Math.PI,
        },
        originQuat: jObj.quaternion.clone(),
        basePos: jObj.position.clone(),
      });
      buildLink(j.child, jObj);
    }
  };

  const root = new THREE.Group();
  buildLink(rootName, root);
  return { root, joints: jointMap, links: linkMap };
}

const _jq = new THREE.Quaternion();
function setJointValue(j, v) {
  if (!j) return;
  if (j.type === 'prismatic') {
    j.obj.position.copy(j.basePos).addScaledVector(j.axisVec, v);
  } else if (j.type === 'revolute' || j.type === 'continuous') {
    _jq.setFromAxisAngle(j.axisVec, v);
    j.obj.quaternion.copy(j.originQuat).multiply(_jq);
  }
}

/* =====================================================================
 * 2. three.js 场景 (ROS 坐标系: X 前 Y 左 Z 上)
 * ===================================================================== */
let scene, camera, renderer, robotRoot, robotModel, roomModel, scanLine;
const world = { followRobot: false };

class SimpleOrbit {
  constructor(cam, dom) {
    this.cam = cam; this.dom = dom;
    this.target = new THREE.Vector3(0, 0, 0.8);
    this.r = 8.0; this.theta = -1.05; this.phi = 1.12;   // phi: 与 +Z 夹角
    this.dragBtn = -1; this.lastX = 0; this.lastY = 0;
    this.enabled = true;
    dom.addEventListener('contextmenu', e => e.preventDefault());
    dom.addEventListener('pointerdown', e => {
      if (!this.enabled) return;
      this.dragBtn = e.button; this.lastX = e.clientX; this.lastY = e.clientY;
      dom.setPointerCapture(e.pointerId);
    });
    dom.addEventListener('pointermove', e => {
      if (this.dragBtn < 0 || !this.enabled) return;
      const dx = e.clientX - this.lastX, dy = e.clientY - this.lastY;
      this.lastX = e.clientX; this.lastY = e.clientY;
      if (this.dragBtn === 0) {                       // 左键: 旋转
        this.theta -= dx * 0.006;
        this.phi = Math.max(0.08, Math.min(Math.PI - 0.08, this.phi - dy * 0.006));
      } else {                                        // 右/中键: 平移
        const s = this.r * 0.0016;
        const right = new THREE.Vector3().setFromMatrixColumn(this.cam.matrix, 0);
        const up = new THREE.Vector3().setFromMatrixColumn(this.cam.matrix, 1);
        this.target.addScaledVector(right, -dx * s).addScaledVector(up, dy * s);
      }
    });
    const end = () => { this.dragBtn = -1; };
    dom.addEventListener('pointerup', end);
    dom.addEventListener('pointercancel', end);
    dom.addEventListener('wheel', e => {
      if (!this.enabled) return;
      e.preventDefault();
      this.r = Math.max(1.2, Math.min(30, this.r * (e.deltaY > 0 ? 1.12 : 0.89)));
    }, { passive: false });
  }
  reset() { this.target.set(0, 0, 0.8); this.r = 8.0; this.theta = -1.05; this.phi = 1.12; }
  update() {
    const sp = Math.sin(this.phi);
    this.cam.position.set(
      this.target.x + this.r * sp * Math.cos(this.theta),
      this.target.y + this.r * sp * Math.sin(this.theta),
      this.target.z + this.r * Math.cos(this.phi));
    this.cam.up.set(0, 0, 1);
    this.cam.lookAt(this.target);
  }
}

let controls;

function init3D() {
  const view = $('#view3d');
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x14161a);

  camera = new THREE.PerspectiveCamera(55, 1, 0.05, 100);
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  view.appendChild(renderer.domElement);
  controls = new SimpleOrbit(camera, renderer.domElement);

  const hemi = new THREE.HemisphereLight(0xbfd0e8, 0x2c2620, 0.95);
  hemi.position.set(0, 0, 1);
  scene.add(hemi);
  const dir = new THREE.DirectionalLight(0xffffff, 0.85);
  dir.position.set(5, -4, 9);
  scene.add(dir);

  const grid = new THREE.GridHelper(9, 45, 0x39404e, 0x222833);
  grid.rotation.x = Math.PI / 2;      // XY 平面 (地面)
  scene.add(grid);

  robotRoot = new THREE.Group();
  robotRoot.position.z = 0.04;
  scene.add(robotRoot);

  const resize = () => {
    const w = view.clientWidth, h = view.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  };
  new ResizeObserver(resize).observe(view);
  resize();

  (function animate() {
    requestAnimationFrame(animate);
    if (world.followRobot && robotRoot) {
      controls.target.lerp(robotRoot.position, 0.06);
    }
    controls.update();
    updateEEInfo();
    renderer.render(scene, camera);
  })();
}

/* 激光扫描可视化 (挂在 slam_sensor_link 下) */
function makeScanLine() {
  const N = 720;                       // LineSegments: 每束 2 点
  const pos = new Float32Array(N * 3);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  scanLine = new THREE.LineSegments(geo,
    new THREE.LineBasicMaterial({ color: 0xff5555, transparent: true, opacity: 0.5 }));
  scanLine.frustumCulled = false;
  return scanLine;
}

function updateScanLine(msg) {
  if (!scanLine) return;
  const pos = scanLine.geometry.attributes.position.array;
  const n = msg.ranges.length;
  let k = 0;
  for (let i = 0; i < n && k < pos.length - 6; i++) {
    let r = msg.ranges[i];
    if (r === null || r === undefined || !isFinite(r)) continue;
    r = Math.min(r, msg.range_max || 12);
    const a = msg.angle_min + i * msg.angle_increment;
    pos[k++] = 0; pos[k++] = 0; pos[k++] = 0;
    pos[k++] = r * Math.cos(a); pos[k++] = r * Math.sin(a); pos[k++] = 0;
  }
  for (; k < pos.length; k++) pos[k] = 0;
  scanLine.geometry.attributes.position.needsUpdate = true;
}

/* =====================================================================
 * 3. rosbridge 连接
 * ===================================================================== */
let ros = null;
let pubs = null;
const jointPos = {};          // 最新关节位置
let lastJointMsg = null;
const robotOdom = { x: 0, y: 0, yaw: 0, t: 0 };
const amclPose = { x: 0, y: 0, yaw: 0, t: 0 };
let navState = { status: '', goal: null, path: [] };

function setBadge(id, cls, text) {
  const el = $(id);
  el.className = 'badge ' + cls;
  el.textContent = text;
}

function connectROS() {
  setBadge('#connStatus', 'warn', 'ROS 连接中…');
  ros = new ROSLIB.Ros({ url: `ws://${location.hostname}:9090` });
  ros.on('connection', () => { setBadge('#connStatus', 'ok', 'ROS 已连接'); setupTopics(); });
  ros.on('error', () => setBadge('#connStatus', 'err', 'ROS 连接错误'));
  ros.on('close', () => {
    setBadge('#connStatus', 'err', 'ROS 未连接, 重试中…');
    pubs = null;
    setTimeout(connectROS, 3000);
  });
}

function sub(name, type, cb, throttle) {
  const opt = { ros, name, messageType: type };
  if (throttle) { opt.throttle_rate = throttle; opt.queue_length = 1; }
  new ROSLIB.Topic(opt).subscribe(cb);
}

function setupTopics() {
  pubs = {
    cmdVel: new ROSLIB.Topic({ ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' }),
    relMove: new ROSLIB.Topic({ ros, name: '/chassis_relative_move', messageType: 'geometry_msgs/Vector3' }),
    navGoal: new ROSLIB.Topic({ ros, name: '/nav_goal_pose', messageType: 'geometry_msgs/PoseStamped' }),
    navCancel: new ROSLIB.Topic({ ros, name: '/nav_cancel', messageType: 'std_msgs/Empty' }),
    jtc: {},
  };
  for (const g of GROUPS) {
    pubs.jtc[g.key] = new ROSLIB.Topic({
      ros, name: `/${g.ctrl}/joint_trajectory`,
      messageType: 'trajectory_msgs/JointTrajectory',
    });
  }

  sub('/joint_states', 'sensor_msgs/JointState', msg => {
    lastJointMsg = msg;
    for (let i = 0; i < msg.name.length; i++) jointPos[msg.name[i]] = msg.position[i];
    if (robotModel) applyJointStates();
    syncSliders();
  }, 50);

  sub('/odom', 'nav_msgs/Odometry', msg => {
    const p = msg.pose.pose;
    robotOdom.x = p.position.x; robotOdom.y = p.position.y;
    robotOdom.yaw = quatYaw(p.orientation); robotOdom.t = performance.now();
    if (robotRoot) {
      robotRoot.position.set(p.position.x, p.position.y, p.position.z || 0.04);
      robotRoot.quaternion.set(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    }
    $('#odomText').textContent =
      `x ${robotOdom.x.toFixed(2)} y ${robotOdom.y.toFixed(2)} θ ${deg(robotOdom.yaw)}°`;
  }, 100);

  sub('/scan', 'sensor_msgs/LaserScan', msg => { if (scanLine && scanLine.visible) updateScanLine(msg); }, 200);
  sub('/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped', msg => {
    const p = msg.pose.pose;
    amclPose.x = p.position.x; amclPose.y = p.position.y;
    amclPose.yaw = quatYaw(p.orientation); amclPose.t = performance.now();
    drawNavMap();
  }, 250);
  sub('/map', 'nav_msgs/OccupancyGrid', msg => { navMap.data = msg; buildMapImage(); drawNavMap(); });
  sub('/nav_status', 'std_msgs/String', msg => {
    navState.status = msg.data;
    const map = {
      accepted: ['run', '导航: 已接受'], executing: ['run', '导航: 执行中'],
      succeeded: ['ok', '导航: 已到达'], canceled: ['warn', '导航: 已取消'],
      failed: ['err', '导航: 失败'], canceling: ['run', '导航: 取消中…'],
    };
    const m = map[msg.data];
    if (m) setBadge('#navStatus', m[0], m[1]);
    else setBadge('#navStatus', 'warn', '导航: ' + msg.data);
    if (msg.data === 'succeeded') navState.path = [];
    drawNavMap();
  });
  sub('/nav_feedback', 'geometry_msgs/PoseStamped', msg => {
    const x = msg.pose.position.x, y = msg.pose.position.y;
    const last = navState.path[navState.path.length - 1];
    if (!last || Math.hypot(x - last[0], y - last[1]) > 0.05) {
      navState.path.push([x, y]);
      if (navState.path.length > 800) navState.path.shift();
    }
    drawNavMap();
  }, 200);
  sub('/chassis_move_status', 'std_msgs/String', msg => {
    const el = $('#chassisStatus');
    el.textContent = msg.data === 'moving' ? '相对运动执行中…' :
                     msg.data === 'done' ? '相对运动完成 ✓' : '';
  });
}

function applyJointStates() {
  for (const [name, val] of Object.entries(jointPos)) {
    setJointValue(robotModel.joints.get(name), val);
  }
}

function quatYaw(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
function deg(a) { return Math.round(a * 180 / Math.PI); }
function nowStamp() {
  const ms = Date.now();
  return { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1e6 };
}

/* 发布 JointTrajectory 到某组控制器 */
function publishTrajectory(groupKey, positions, dur) {
  if (!pubs) return;
  const g = GROUPS.find(x => x.key === groupKey);
  pubs.jtc[groupKey].publish({
    joint_names: g.joints,
    points: [{
      positions: positions.map(Number),
      time_from_start: { sec: Math.floor(dur), nanosec: Math.round((dur % 1) * 1e9) },
    }],
  });
}

/* =====================================================================
 * 4. 页签切换
 * ===================================================================== */
for (const btn of document.querySelectorAll('#tabs button')) {
  btn.addEventListener('click', () => {
    document.querySelectorAll('#tabs button').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.tab-page').forEach(p => p.classList.remove('active'));
    btn.classList.add('active');
    $('#tab-' + btn.dataset.tab).classList.add('active');
  });
}

/* =====================================================================
 * 5. 轴运动页签
 * ===================================================================== */
const sliderActive = new Set();

function buildAxisTab() {
  const wrap = $('#axisGroups');
  wrap.innerHTML = '';
  for (const g of GROUPS) {
    const sec = document.createElement('div');
    const h = document.createElement('h3');
    h.className = 'sec';
    h.innerHTML = `${g.title} <button class="mini" data-zero="${g.key}">全部回零</button>`;
    h.querySelector('button').addEventListener('click', () => zeroGroup(g));
    sec.appendChild(h);

    g.joints.forEach((jn, i) => {
      const j = robotModel.joints.get(jn);
      let lo = -Math.PI, hi = Math.PI;
      if (j) { lo = Math.min(j.limit.lower, 0); hi = Math.max(j.limit.upper, 0); }
      if (hi - lo < 1e-3) { lo = -Math.PI; hi = Math.PI; }
      const row = document.createElement('div');
      row.className = 'jrow';
      row.innerHTML =
        `<div class="jlabel"><span class="jname" title="${jn}">关节 ${i + 1}</span>` +
        `<span class="jval" id="val_${jn}">0.000</span></div>` +
        `<input type="range" id="sl_${jn}" min="${lo.toFixed(4)}" max="${hi.toFixed(4)}" ` +
        `step="0.005" value="0">`;
      const sl = row.querySelector('input');
      sl.addEventListener('pointerdown', () => sliderActive.add(jn));
      sl.addEventListener('pointerup', () => sliderActive.delete(jn));
      sl.addEventListener('pointercancel', () => sliderActive.delete(jn));
      sl.addEventListener('input', () => {
        const v = parseFloat(sl.value);
        $('#val_' + CSS.escape(jn)).textContent = v.toFixed(3);
        setJointValue(robotModel.joints.get(jn), v);   // 本地预览
      });
      sl.addEventListener('change', () => sendGroupFromSliders(g));
      sec.appendChild(row);
    });
    wrap.appendChild(sec);
  }
  syncSliders();
}

/* 滑块 -> 控制器 (整组发送, allow_partial_joints_goal=false) */
function sendGroupFromSliders(g) {
  const cur = g.joints.map(jn => jointPos[jn] ?? 0);
  const target = g.joints.map(jn => {
    const el = document.getElementById('sl_' + jn);
    return el ? parseFloat(el.value) : (jointPos[jn] ?? 0);
  });
  let maxd = 0;
  for (let i = 0; i < cur.length; i++) maxd = Math.max(maxd, Math.abs(target[i] - cur[i]));
  const dur = Math.min(6, Math.max(0.5, 0.4 + maxd * 1.5));
  publishTrajectory(g.key, target, dur);
}

function zeroGroup(g) {
  for (const jn of g.joints) {
    const el = document.getElementById('sl_' + jn);
    if (el) { el.value = 0; $('#val_' + CSS.escape(jn)).textContent = '0.000'; }
  }
  publishTrajectory(g.key, g.joints.map(() => 0), 2.5);
}

/* 关节状态 -> 滑块同步 */
function syncSliders() {
  for (const jn of ALL_JOINTS) {
    if (sliderActive.has(jn) || !(jn in jointPos)) continue;
    const el = document.getElementById('sl_' + jn);
    if (!el) continue;
    el.value = jointPos[jn];
    const v = $('#val_' + CSS.escape(jn));
    if (v) v.textContent = jointPos[jn].toFixed(3);
  }
}

/* =====================================================================
 * 6. 位置运动页签 (示教点位, localStorage 持久化)
 * ===================================================================== */
const LS_KEY = 'g2_points_v1';
const loadPoints = () => { try { return JSON.parse(localStorage.getItem(LS_KEY)) || []; } catch { return []; } };
const savePoints = pts => localStorage.setItem(LS_KEY, JSON.stringify(pts));

function renderPoints() {
  const list = $('#ptList');
  list.innerHTML = '';
  for (const pts of loadPoints()) {
    const row = document.createElement('div');
    row.className = 'pt-row';
    row.innerHTML =
      `<span class="pt-name">${pts.name}</span><span class="pt-n">${Object.keys(pts.joints).length} 关节</span>` +
      `<button class="btn primary mini" data-go="1">前往</button>` +
      `<button class="btn mini" data-del="1">删除</button>`;
    row.querySelector('[data-go]').addEventListener('click', () => gotoPoint(pts));
    row.querySelector('[data-del]').addEventListener('click', () => {
      savePoints(loadPoints().filter(p => p !== pts));
      renderPoints();
    });
    list.appendChild(row);
  }
}

function gotoPoint(pts) {
  let maxd = 0;
  for (const g of GROUPS) {
    const has = g.joints.some(jn => jn in pts.joints);
    if (!has) continue;
    const target = g.joints.map(jn => pts.joints[jn] ?? jointPos[jn] ?? 0);
    target.forEach((t, i) => {
      maxd = Math.max(maxd, Math.abs(t - (jointPos[g.joints[i]] ?? 0)));
    });
    const dur = Math.min(8, Math.max(1.5, 1 + maxd * 1.5));
    publishTrajectory(g.key, target, dur);
  }
}

function saveCurrentPose() {
  const name = prompt('点位名称:', `点位 ${loadPoints().length + 1}`);
  if (!name) return;
  const joints = {};
  for (const jn of ALL_JOINTS) if (jn in jointPos) joints[jn] = jointPos[jn];
  if (Object.keys(joints).length === 0) { alert('尚未收到关节状态'); return; }
  const pts = loadPoints();
  pts.push({ name, joints });
  savePoints(pts);
  renderPoints();
}

/* =====================================================================
 * 6.5 末端笛卡尔控制 (位置运动): x/y/z 引导轴拖拽 + 数值 IK (阻尼最小二乘)
 * ===================================================================== */
const ee = {
  L: { group: GROUPS[2], endLink: null, target: null, gizmo: null, dragging: false },
  R: { group: GROUPS[3], endLink: null, target: null, gizmo: null, dragging: false },
};

function eeBasePos(endObj, out) {
  robotRoot.updateMatrixWorld(true);
  const wp = endObj.getWorldPosition(out || new THREE.Vector3());
  return robotRoot.worldToLocal(wp);
}

/* 3x3 线性方程求解 */
function solve3(a, b) {
  const m = [a[0].slice(), a[1].slice(), a[2].slice(), b];
  for (let col = 0; col < 3; col++) {
    let piv = col;
    for (let r = col + 1; r < 3; r++) if (Math.abs(m[r][col]) > Math.abs(m[piv][col])) piv = r;
    if (Math.abs(m[piv][col]) < 1e-12) return null;
    [m[col], m[piv]] = [m[piv], m[col]];
    for (let r = 0; r < 3; r++) {
      if (r === col) continue;
      const f = m[r][col] / m[col][col];
      for (let c = col; c < 4; c++) m[r][c] -= f * m[col][c];
    }
  }
  return [m[0][3] / m[0][0], m[1][3] / m[1][1], m[2][3] / m[2][2]];
}

/**
 * 一次 IK 迭代: 把 endObj (基座系) 移向 targetBase。
 * 关节限位内阻尼最小二乘, 完成后发送轨迹并乐观更新本地状态。
 * 返回 false 表示已收敛 (误差 < 3mm)。
 */
function ikStep(hand, targetBase) {
  const g = hand.group;
  const js = g.joints.map(jn => robotModel.joints.get(jn));
  const q = g.joints.map(jn => jointPos[jn] ?? 0);
  const lo = js.map(j => Math.min(j.limit.lower, 0));
  const hi = js.map(j => Math.max(j.limit.upper, 0));

  const p0 = eeBasePos(hand.endLink);
  const e = targetBase.clone().sub(p0);
  const err = e.length();
  if (err < 0.003) return false;
  e.setLength(Math.min(err, 0.08));          // 单步限幅

  /* 数值雅可比 3xN (有限差分) */
  const h = 1e-3;
  const N = q.length;
  const J = [new Float64Array(N), new Float64Array(N), new Float64Array(N)];
  const tmp = new THREE.Vector3();
  for (let i = 0; i < N; i++) {
    const qp = q.slice(); qp[i] += h;
    g.joints.forEach((jn, k) => setJointValue(robotModel.joints.get(jn), qp[k]));
    const pi = eeBasePos(hand.endLink, tmp).clone();
    J[0][i] = (pi.x - p0.x) / h;
    J[1][i] = (pi.y - p0.y) / h;
    J[2][i] = (pi.z - p0.z) / h;
  }
  g.joints.forEach((jn, k) => setJointValue(robotModel.joints.get(jn), q[k]));   // 恢复

  /* A = J·Jᵀ + λ²I,  y = A⁻¹·e,  dq = Jᵀ·y */
  const lam2 = 0.05;
  const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let r = 0; r < 3; r++) {
    for (let c = r; c < 3; c++) {
      let s = 0;
      for (let i = 0; i < N; i++) s += J[r][i] * J[c][i];
      A[r][c] = A[c][r] = s;
    }
    A[r][r] += lam2;
  }
  const y = solve3(A, [e.x, e.y, e.z]);
  if (!y) return false;

  const qn = q.map((v, i) => {
    const dq = J[0][i] * y[0] + J[1][i] * y[1] + J[2][i] * y[2];
    const d = Math.max(-0.15, Math.min(0.15, dq));
    return Math.max(lo[i], Math.min(hi[i], v + d));
  });
  g.joints.forEach((jn, k) => {
    jointPos[jn] = qn[k];
    setJointValue(robotModel.joints.get(jn), qn[k]);
  });
  publishTrajectory(g.key, qn, 0.3);
  return true;
}

function initCartesian() {
  ee.L.endLink = robotModel.links.get('arm_l_end_link');
  ee.R.endLink = robotModel.links.get('arm_r_end_link');
  if (!ee.L.endLink || !ee.R.endLink) { console.warn('未找到臂末端 link'); return; }

  const defs = [['L', ee.L, 0xff9a3c], ['R', ee.R, 0x3cd0ff]];
  for (const [k, hand, color] of defs) {
    const target = new THREE.Object3D();
    target.add(new THREE.Mesh(new THREE.SphereGeometry(0.022, 12, 8),
      new THREE.MeshBasicMaterial({ color })));
    scene.add(target);
    hand.target = target;

    const gizmo = new THREE.TransformControls(camera, renderer.domElement);
    gizmo.attach(target);
    gizmo.setMode('translate');
    gizmo.setSize(0.5);
    scene.add(gizmo);
    hand.gizmo = gizmo;

    gizmo.addEventListener('dragging-changed', ev => {
      hand.dragging = ev.value;
      controls.enabled = !ev.value;         // 拖拽时禁用轨道相机
    });
    const chk = $('#gizmo' + k);
    if (chk) chk.addEventListener('change', ev => {
      gizmo.visible = ev.target.checked;
      gizmo.enabled = ev.target.checked;
    });
  }

  /* 拖拽期间 150ms 闭环: IK 收敛到引导轴位置 */
  setInterval(() => {
    if (!pubs) return;
    robotRoot.updateMatrixWorld(true);
    for (const hand of [ee.L, ee.R]) {
      if (!hand.dragging) continue;
      const tb = robotRoot.worldToLocal(hand.target.position.clone());
      for (let it = 0; it < 3; it++) {
        if (!ikStep(hand, tb)) break;
      }
    }
  }, 150);
}

/* 坐标显示 + 引导轴跟随末端 (非拖拽时), 由渲染循环调用 */
let _eeTick = 0;
function updateEEInfo() {
  if (!robotModel || !ee.L.endLink) return;
  if (performance.now() - _eeTick < 150) return;
  _eeTick = performance.now();
  const tmp = new THREE.Vector3();
  for (const [k, hand] of [['L', ee.L], ['R', ee.R]]) {
    const p = eeBasePos(hand.endLink, tmp);
    const el = $('#ee' + k);
    if (el) el.textContent = `x ${p.x.toFixed(3)} y ${p.y.toFixed(3)} z ${p.z.toFixed(3)}`;
    if (!hand.dragging && hand.target) hand.endLink.getWorldPosition(hand.target.position);
  }
}

/* =====================================================================
 * 7. 底盘运动页签
 * ===================================================================== */
const joy = { active: false, vx: 0, vy: 0, rot: 0 };
const JOY_MAX = 0.6, ROT_MAX = 0.8;

function initChassisTab() {
  const pad = $('#joy'), knob = $('#joyKnob');
  let pid = null;

  const setKnob = (dx, dy) => { knob.style.transform = `translate(${dx}px, ${dy}px)`; };
  const handle = e => {
    const r = pad.getBoundingClientRect();
    let dx = e.clientX - (r.left + r.width / 2);
    let dy = e.clientY - (r.top + r.height / 2);
    const len = Math.hypot(dx, dy), max = r.width / 2 - 18;
    if (len > max) { dx *= max / len; dy *= max / len; }
    setKnob(dx, dy);
    joy.vx = (-dy / max) * JOY_MAX;      // 屏幕上 = 前 (+x)
    joy.vy = (-dx / max) * JOY_MAX;      // 屏幕左 = 左 (+y)
    joy.active = true;
    updateVelText();
  };
  pad.addEventListener('pointerdown', e => { pid = e.pointerId; pad.setPointerCapture(pid); handle(e); });
  pad.addEventListener('pointermove', e => { if (pid !== null) handle(e); });
  const release = () => {
    pid = null; joy.active = false; joy.vx = 0; joy.vy = 0;
    setKnob(0, 0); updateVelText();
  };
  pad.addEventListener('pointerup', release);
  pad.addEventListener('pointercancel', release);

  const hold = (id, dir) => {
    const el = $(id);
    el.addEventListener('pointerdown', e => { e.preventDefault(); joy.rot = dir; updateVelText(); });
    for (const ev of ['pointerup', 'pointerleave', 'pointercancel'])
      el.addEventListener(ev, () => { joy.rot = 0; updateVelText(); });
  };
  hold('#btnRotL', ROT_MAX);
  hold('#btnRotR', -ROT_MAX);

  /* 10Hz 速度发布 */
  let wasActive = false;
  setInterval(() => {
    const active = joy.active || joy.rot !== 0;
    if (!pubs) return;
    if (active || wasActive) {
      pubs.cmdVel.publish({
        linear: { x: joy.vx, y: joy.vy, z: 0 },
        angular: { x: 0, y: 0, z: joy.rot },
      });
    }
    wasActive = active;
  }, 100);

  /* 相对运动 */
  for (const b of document.querySelectorAll('[data-rel]')) {
    b.addEventListener('click', () => sendRelMove(...b.dataset.rel.split(',').map(Number)));
  }
  $('#btnRelGo').addEventListener('click', () => {
    const dx = parseFloat($('#relDx').value) || 0;
    const dy = parseFloat($('#relDy').value) || 0;
    const dth = (parseFloat($('#relDth').value) || 0) * Math.PI / 180;
    sendRelMove(dx, dy, dth);
  });
}

function sendRelMove(dx, dy, dth) {
  if (!pubs) return;
  $('#chassisStatus').textContent = '相对运动已发送…';
  pubs.relMove.publish({ x: dx, y: dy, z: dth });
}

function updateVelText() {
  $('#velText').textContent =
    `v ${joy.vx.toFixed(2)}, ${joy.vy.toFixed(2)} m/s ω ${joy.rot.toFixed(2)} rad/s`;
}

/* =====================================================================
 * 8. 地图导航页签
 * ===================================================================== */
const navMap = { data: null, img: null, tf: null };
const navCanvas = () => $('#navMap');

function buildMapImage() {
  const m = navMap.data;
  if (!m || !m.data) return;
  const W = m.info.width, H = m.info.height;
  const off = document.createElement('canvas');
  off.width = W; off.height = H;
  const ctx = off.getContext('2d');
  const img = ctx.createImageData(W, H);
  for (let i = 0; i < W * H; i++) {
    const v = m.data[i];
    let r, g, b;
    if (v < 0)          { r = 16; g = 18; b = 23; }        // 未知
    else if (v === 0)   { r = 40; g = 46; b = 58; }        // 空闲
    else if (v > 60)    { r = 225; g = 231; b = 240; }     // 占据
    else                { r = 70; g = 78; b = 92; }        // 半占据
    const o = i * 4;
    img.data[o] = r; img.data[o + 1] = g; img.data[o + 2] = b; img.data[o + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
  navMap.img = off;

  /* 计算画布变换: 地图等比铺满画布 */
  const cv = navCanvas();
  const res = m.info.resolution;
  const s = Math.min(cv.width / (W * res), cv.height / (H * res));
  navMap.tf = {
    s, res, W, H,
    ox: m.info.origin.position.x,
    oy: m.info.origin.position.y,
    offX: (cv.width - W * res * s) / 2,
    offY: (cv.height - H * res * s) / 2,
  };
  $('#navMapHint').style.display = 'none';
}

const w2c = (x, y) => {
  const t = navMap.tf;
  return [t.offX + (x - t.ox) * t.s, t.offY + (t.H * t.res - (y - t.oy)) * t.s];
};
const c2w = (cx, cy) => {
  const t = navMap.tf;
  return [t.ox + (cx - t.offX) / t.s, t.oy + t.H * t.res - (cy - t.offY) / t.s];
};

function drawArrow(ctx, x, y, yaw, color, size = 14) {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-yaw);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.6, size * 0.55);
  ctx.lineTo(-size * 0.25, 0);
  ctx.lineTo(-size * 0.6, -size * 0.55);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function robotPose() {
  return (performance.now() - amclPose.t < 5000) ? { ...amclPose, src: 'AMCL' }
                                                 : { ...robotOdom, src: '里程计' };
}

function drawNavMap() {
  const cv = navCanvas();
  if (!cv) return;
  const ctx = cv.getContext('2d');
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.fillStyle = '#0b0d10';
  ctx.fillRect(0, 0, cv.width, cv.height);
  if (!navMap.img || !navMap.tf) return;

  const t = navMap.tf;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(navMap.img, t.offX, t.offY, t.W * t.res * t.s, t.H * t.res * t.s);

  /* 导航路径 */
  if (navState.path.length > 1) {
    ctx.strokeStyle = '#e8b74a';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    const p0 = w2c(navState.path[0][0], navState.path[0][1]);
    ctx.moveTo(p0[0], p0[1]);
    for (const p of navState.path.slice(1)) {
      const c = w2c(p[0], p[1]);
      ctx.lineTo(c[0], c[1]);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  /* 目标点 */
  if (navState.goal) {
    const c = w2c(navState.goal.x, navState.goal.y);
    drawArrow(ctx, c[0], c[1], navState.goal.yaw, '#58b6e8', 16);
  }

  /* 机器人 */
  const rp = robotPose();
  const c = w2c(rp.x, rp.y);
  drawArrow(ctx, c[0], c[1], rp.yaw, '#5ade7f', 14);

  const goalTxt = navState.goal
    ? `目标: x ${navState.goal.x.toFixed(2)} y ${navState.goal.y.toFixed(2)} θ ${deg(navState.goal.yaw)}°`
    : '目标: -';
  $('#navInfo').textContent =
    `机器人 (${rp.src}): x ${rp.x.toFixed(2)} y ${rp.y.toFixed(2)} θ ${deg(rp.yaw)}°\n` +
    `${goalTxt}\n状态: ${navState.status || '待机'}`;
}

/* 地图点击/拖拽 -> 导航目标 */
function initNavTab() {
  const cv = navCanvas();
  let dragStart = null, dragCur = null;

  const pos = e => {
    const r = cv.getBoundingClientRect();
    return [(e.clientX - r.left) * cv.width / r.width,
            (e.clientY - r.top) * cv.height / r.height];
  };

  cv.addEventListener('pointerdown', e => {
    dragStart = pos(e); dragCur = null;
    cv.setPointerCapture(e.pointerId);
  });
  cv.addEventListener('pointermove', e => {
    if (!dragStart) return;
    const p = pos(e);
    if (Math.hypot(p[0] - dragStart[0], p[1] - dragStart[1]) > 8) dragCur = p;
  });
  cv.addEventListener('pointerup', e => {
    if (!dragStart || !navMap.tf) { dragStart = null; return; }
    const p = pos(e);
    const [wx, wy] = c2w(dragStart[0], dragStart[1]);
    let yaw;
    if (dragCur) {
      const [cx1, cy1] = dragStart, [cx2, cy2] = dragCur;
      yaw = Math.atan2(-(cy2 - cy1), cx2 - cx1);   // 画布 y 向下
    } else {
      yaw = robotPose().yaw;
    }
    dragStart = null; dragCur = null;
    navState.goal = { x: wx, y: wy, yaw };
    if (pubs) {
      pubs.navGoal.publish({
        header: { frame_id: 'map', stamp: nowStamp() },
        pose: {
          position: { x: wx, y: wy, z: 0 },
          orientation: {
            x: 0, y: 0,
            z: Math.sin(yaw / 2), w: Math.cos(yaw / 2),
          },
        },
      });
    }
    drawNavMap();
  });

  $('#btnNavCancel').addEventListener('click', () => {
    if (!pubs) return;
    pubs.navCancel.publish({});
    navState.goal = null;
    drawNavMap();
  });
}

/* =====================================================================
 * 9. 顶部工具条
 * ===================================================================== */
function initOverlay() {
  $('#btnCamReset').addEventListener('click', () => controls.reset());
  $('#chkFollow').addEventListener('change', e => { world.followRobot = e.target.checked; });
  $('#chkScan').addEventListener('change', e => { if (scanLine) scanLine.visible = e.target.checked; });
}

/* =====================================================================
 * 10. 启动
 * ===================================================================== */
async function init() {
  init3D();
  initOverlay();
  initChassisTab();
  initNavTab();
  $('#btnSavePose').addEventListener('click', saveCurrentPose);
  $('#btnGoHome').addEventListener('click', () =>
    gotoPoint({ name: '零位', joints: Object.fromEntries(ALL_JOINTS.map(j => [j, 0])) }));
  renderPoints();
  connectROS();

  /* 加载机器人 + 房间 URDF */
  try {
    const [robotText, roomText] = await Promise.all([
      fetch('/urdf/model.urdf').then(r => r.text()),
      fetch('/urdf/model_room.urdf').then(r => r.text()),
    ]);
    robotModel = parseURDF(robotText);
    robotRoot.add(robotModel.root);
    roomModel = parseURDF(roomText);
    scene.add(roomModel.root);
    if (meshProgress.total === 0) $('#loadModelHint').style.display = 'none';

    /* 激光可视化挂到 slam_sensor_link */
    const sensor = robotModel.links.get('slam_sensor_link') || robotRoot;
    sensor.add(makeScanLine());

    buildAxisTab();
    initCartesian();
    if (lastJointMsg) { applyJointStates(); syncSliders(); }
  } catch (err) {
    console.error(err);
    $('#loadModelHint').textContent = '模型加载失败: ' + err.message;
  }
}

init();
