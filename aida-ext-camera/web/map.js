// Top-down animated floor map shared by the robot page and the fullscreen map.
// Floor frame: X right, Y up (toward the printed top of the reference board).
class FloorMap {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.map = null;
    this.robotSize = [.9, .6];
    this.target = null;      // latest measured robot pose
    this.shown = null;       // smoothed pose being drawn
    this.lastUpdate = 0;     // performance.now() of last measurement
    this.trail = [];         // {x, y, t}
    this.updates = [];       // measurement times for the rate
    this.view = null;
    this.frame = this.frame.bind(this);
    new ResizeObserver(() => this.resize()).observe(canvas);
    this.resize();
    requestAnimationFrame(this.frame);
  }

  resize() {
    const ratio = window.devicePixelRatio || 1, rect = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.max(1, Math.round(rect.width * ratio));
    this.canvas.height = Math.max(1, Math.round(rect.height * ratio));
    this.ratio = ratio;
  }

  setMap(map) { this.map = map; this.view = null; }

  setRobotSize(size) { if (size) this.robotSize = size; }

  push(robot) {
    const now = performance.now();
    if (!robot) { this.target = null; return; }
    const pose = {x: robot.x_m, y: robot.y_m, yaw: robot.yaw_rad, std: robot.std_xy_m || 0,
                  z: robot.z_m, free: robot.source === 'free_6dof'};
    if (!this.shown) this.shown = {...pose};
    this.shown.z = pose.z;
    this.shown.free = pose.free;
    this.target = pose;
    this.lastUpdate = now;
    this.updates.push(now);
    this.updates = this.updates.filter(t => now - t < 2000);
    const last = this.trail[this.trail.length - 1];
    if (!last || Math.hypot(last.x - pose.x, last.y - pose.y) > .004) this.trail.push({x: pose.x, y: pose.y, t: now});
    this.trail = this.trail.filter(p => now - p.t < 25000);
  }

  stats() {
    const now = performance.now(), recent = this.trail.filter(p => now - p.t < 1000);
    let speed = 0;
    if (recent.length > 1) {
      const a = recent[0], b = recent[recent.length - 1];
      speed = Math.hypot(b.x - a.x, b.y - a.y) / Math.max(.2, (b.t - a.t) / 1000);
    }
    return {rate: this.updates.length / 2, speed, age: this.target ? (now - this.lastUpdate) / 1000 : Infinity};
  }

  bounds() {
    const points = [];
    if (this.map) {
      const half = this.map.floor_board_m ? Math.max(...this.map.floor_board_m) / 2 : .2;
      for (const [x, y] of this.map.placements) points.push([x - half, y - half], [x + half, y + half]);
      points.push(this.map.camera_floor_m.slice(0, 2), [0, 0]);
    }
    if (this.shown) points.push([this.shown.x, this.shown.y]);
    if (!points.length) points.push([-1, -1], [1, 1]);
    const xs = points.map(p => p[0]), ys = points.map(p => p[1]);
    return [Math.min(...xs) - .7, Math.min(...ys) - .7, Math.max(...xs) + .7, Math.max(...ys) + .7];
  }

  updateView(dt) {
    const [x0, y0, x1, y1] = this.bounds(), w = this.canvas.width, h = this.canvas.height;
    const pad = 28 * this.ratio;
    const target = {cx: (x0 + x1) / 2, cy: (y0 + y1) / 2,
                    scale: Math.min((w - 2 * pad) / (x1 - x0), (h - 2 * pad) / (y1 - y0))};
    if (!this.view) { this.view = target; return; }
    const k = 1 - Math.exp(-dt / .6);
    for (const key of ['cx', 'cy', 'scale']) this.view[key] += (target[key] - this.view[key]) * k;
  }

  toScreen(x, y) {
    const v = this.view;
    return [(x - v.cx) * v.scale + this.canvas.width / 2, this.canvas.height / 2 - (y - v.cy) * v.scale];
  }

  frame(time) {
    const dt = Math.min(.1, (time - (this.previousTime || time)) / 1000);
    this.previousTime = time;
    if (this.target && this.shown) {
      const k = 1 - Math.exp(-dt / .12);
      this.shown.x += (this.target.x - this.shown.x) * k;
      this.shown.y += (this.target.y - this.shown.y) * k;
      const delta = Math.atan2(Math.sin(this.target.yaw - this.shown.yaw), Math.cos(this.target.yaw - this.shown.yaw));
      this.shown.yaw += delta * k;
      this.shown.std += (this.target.std - this.shown.std) * k;
    }
    this.updateView(dt);
    this.draw(time);
    requestAnimationFrame(this.frame);
  }

  draw(time) {
    const c = this.ctx, w = this.canvas.width, h = this.canvas.height, r = this.ratio, s = this.view.scale;
    const background = c.createRadialGradient(w / 2, h / 2, 0, w / 2, h / 2, Math.hypot(w, h) / 1.6);
    background.addColorStop(0, '#13221f');
    background.addColorStop(1, '#0b1312');
    c.fillStyle = background;
    c.fillRect(0, 0, w, h);
    if (!this.map) return;

    // Camera field of view on the floor.
    const footprint = this.map.footprint || [];
    if (footprint.length > 2) {
      c.beginPath();
      footprint.forEach(([x, y], i) => { const p = this.toScreen(x, y); i ? c.lineTo(...p) : c.moveTo(...p); });
      c.closePath();
      c.fillStyle = 'rgba(70,110,255,.07)';
      c.fill();
      c.setLineDash([6 * r, 6 * r]);
      c.strokeStyle = 'rgba(130,160,255,.35)';
      c.lineWidth = r;
      c.stroke();
      c.setLineDash([]);
    }

    // Grid: 0.5 m fine, 1 m major, labels on the major lines.
    const [x0, y1] = this.fromScreen(0, 0), [x1, y0] = this.fromScreen(w, h);
    for (let x = Math.floor(x0 * 2) / 2; x <= x1; x += .5) this.gridLine([x, y0], [x, y1], x);
    for (let y = Math.floor(y0 * 2) / 2; y <= y1; y += .5) this.gridLine([x0, y], [x1, y], y);
    c.font = `${10 * r}px ui-monospace,monospace`;
    c.fillStyle = 'rgba(205,220,214,.38)';
    for (let x = Math.ceil(x0); x <= x1; x++) { const p = this.toScreen(x, y0); c.fillText(`${x} м`, p[0] + 4 * r, h - 8 * r); }
    for (let y = Math.ceil(y0); y <= y1; y++) { const p = this.toScreen(x0, y); c.fillText(`${y} м`, 8 * r, p[1] - 4 * r); }

    // Calibration placements of the floor board.
    const half = this.map.floor_board_m ? this.map.floor_board_m.map(v => v / 2) : [.1, .1];
    c.strokeStyle = 'rgba(255,255,255,.12)';
    c.lineWidth = r;
    for (const [x, y] of this.map.placements) {
      const [a, b] = this.toScreen(x - half[0], y + half[1]);
      c.strokeRect(a, b, half[0] * 2 * s, half[1] * 2 * s);
    }

    // Origin and axes.
    this.arrow([0, 0], [.5, 0], '#ff6b5a', 'X');
    this.arrow([0, 0], [0, .5], '#59cd8c', 'Y');

    // Camera.
    const [camX, camY, camZ] = this.map.camera_floor_m, cam = this.toScreen(camX, camY);
    c.fillStyle = '#8aa7ff';
    c.beginPath();
    c.arc(cam[0], cam[1], 5 * r, 0, 2 * Math.PI);
    c.fill();
    c.strokeStyle = 'rgba(138,167,255,.35)';
    c.beginPath();
    c.arc(cam[0], cam[1], 11 * r, 0, 2 * Math.PI);
    c.stroke();
    c.fillStyle = 'rgba(190,205,255,.8)';
    c.fillText(`камера · ${camZ.toFixed(2)} м`, cam[0] + 15 * r, cam[1] + 4 * r);

    this.drawTrail();
    this.drawRobot(time);
  }

  fromScreen(px, py) {
    const v = this.view;
    return [(px - this.canvas.width / 2) / v.scale + v.cx, (this.canvas.height / 2 - py) / v.scale + v.cy];
  }

  gridLine(a, b, value) {
    const c = this.ctx, p = this.toScreen(...a), q = this.toScreen(...b);
    const major = Math.abs(value - Math.round(value)) < 1e-6;
    c.strokeStyle = major ? 'rgba(255,255,255,.09)' : 'rgba(255,255,255,.04)';
    c.lineWidth = this.ratio;
    c.beginPath();
    c.moveTo(...p);
    c.lineTo(...q);
    c.stroke();
  }

  arrow(from, to, color, label) {
    const c = this.ctx, r = this.ratio, a = this.toScreen(...from), b = this.toScreen(...to);
    const angle = Math.atan2(b[1] - a[1], b[0] - a[0]);
    c.strokeStyle = c.fillStyle = color;
    c.lineWidth = 2 * r;
    c.beginPath();
    c.moveTo(...a);
    c.lineTo(...b);
    c.stroke();
    c.beginPath();
    c.moveTo(b[0] + Math.cos(angle) * 8 * r, b[1] + Math.sin(angle) * 8 * r);
    c.lineTo(b[0] + Math.cos(angle + 2.4) * 7 * r, b[1] + Math.sin(angle + 2.4) * 7 * r);
    c.lineTo(b[0] + Math.cos(angle - 2.4) * 7 * r, b[1] + Math.sin(angle - 2.4) * 7 * r);
    c.fill();
    c.font = `600 ${11 * r}px ui-monospace,monospace`;
    c.fillText(label, b[0] + Math.cos(angle) * 14 * r - 4 * r, b[1] + Math.sin(angle) * 14 * r + 4 * r);
  }

  drawTrail() {
    const c = this.ctx, now = performance.now(), points = this.trail;
    c.lineCap = 'round';
    for (let i = 1; i < points.length; i++) {
      const age = (now - points[i].t) / 25000;
      c.strokeStyle = `rgba(89,205,140,${(.55 * (1 - age) ** 2).toFixed(3)})`;
      c.lineWidth = (1 + 2.5 * (1 - age)) * this.ratio;
      c.beginPath();
      c.moveTo(...this.toScreen(points[i - 1].x, points[i - 1].y));
      c.lineTo(...this.toScreen(points[i].x, points[i].y));
      c.stroke();
    }
  }

  drawRobot(time) {
    if (!this.shown) return;
    const c = this.ctx, r = this.ratio, s = this.view.scale, pose = this.shown;
    const lost = !this.target || performance.now() - this.lastUpdate > 1200;
    // Grey: not visible; amber: approximate 6-DoF pose (board not at its height).
    const color = lost ? '147,162,155' : pose.free ? '235,176,84' : '89,205,140';
    const [x, y] = this.toScreen(pose.x, pose.y), [length, width] = this.robotSize;

    if (!lost) {
      // Pulse ring and 3-sigma position uncertainty.
      const phase = (time % 1600) / 1600;
      c.strokeStyle = `rgba(${color},${(.45 * (1 - phase)).toFixed(3)})`;
      c.lineWidth = 2 * r;
      c.beginPath();
      c.arc(x, y, (Math.max(length, width) * s / 2) * (1 + phase * .9), 0, 2 * Math.PI);
      c.stroke();
      const sigma = Math.max(3 * pose.std * s, 3 * r);
      c.fillStyle = `rgba(${color},.18)`;
      c.beginPath();
      c.arc(x, y, sigma, 0, 2 * Math.PI);
      c.fill();
    }

    c.save();
    c.translate(x, y);
    c.rotate(-pose.yaw);
    c.shadowColor = `rgba(${color},.7)`;
    c.shadowBlur = lost ? 0 : 24 * r;
    const L = length * s, W = width * s, corner = Math.min(L, W) * .14;
    c.beginPath();
    c.roundRect(-L / 2, -W / 2, L, W, corner);
    c.fillStyle = `rgba(${color},.16)`;
    c.fill();
    c.shadowBlur = 0;
    c.setLineDash(lost || pose.free ? [5 * r, 5 * r] : []);
    c.strokeStyle = `rgb(${color})`;
    c.lineWidth = 2 * r;
    c.stroke();
    c.setLineDash([]);
    // Heading chevron at the front.
    c.fillStyle = `rgb(${color})`;
    c.beginPath();
    c.moveTo(L / 2 + 14 * r, 0);
    c.lineTo(L / 2 - 4 * r, -W * .28);
    c.lineTo(L / 2 + 2 * r, 0);
    c.lineTo(L / 2 - 4 * r, W * .28);
    c.closePath();
    c.fill();
    c.beginPath();
    c.arc(0, 0, 3.5 * r, 0, 2 * Math.PI);
    c.fill();
    c.restore();

    c.font = `${11 * r}px ui-monospace,monospace`;
    c.fillStyle = lost ? 'rgba(200,210,205,.7)' : 'rgba(225,245,235,.95)';
    const label = lost ? 'робот не виден' : `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)} м`;
    const lx = x + Math.max(L, W) / 2 + 12 * r, ly = y - Math.max(L, W) / 2;
    c.fillText(label, lx, ly);
    if (!lost && pose.z !== undefined && pose.z !== null) {
      c.fillStyle = `rgba(${color},.9)`;
      c.fillText(`z ${pose.z.toFixed(2)} м${pose.free ? ' · не на своей высоте' : ''}`, lx, ly + 15 * r);
    }
  }
}

// Shared live connection: map geometry + SSE robot poses.
function connectMap(map, onPose) {
  fetch('/api/robot/map').then(r => r.ok ? r.json() : null).then(data => data && map.setMap(data));
  const source = new EventSource('/api/robot/events');
  source.onmessage = event => {
    const data = JSON.parse(event.data);
    map.push(data.robot);
    onPose && onPose(data);
  };
  return source;
}

function formatPose(robot) {
  if (!robot) return {x: '—', y: '—', yaw: '—', z: '—', tilt: '—'};
  const degrees = robot.yaw_rad * 180 / Math.PI;
  return {x: robot.x_m.toFixed(3), y: robot.y_m.toFixed(3), yaw: `${degrees.toFixed(1)}°`,
          z: robot.z_m === undefined || robot.z_m === null ? '—' : robot.z_m.toFixed(3),
          tilt: robot.tilt_deg === undefined || robot.tilt_deg === null ? '—' : `${robot.tilt_deg.toFixed(1)}°`};
}
