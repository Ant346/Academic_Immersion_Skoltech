const $ = id => document.getElementById(id);
let state = null, busy = false, first = true, timer;
const map = new FloorMap($('map'));

function toast(message, error = false) {
  clearTimeout(timer);
  $('robot-toast').textContent = message;
  $('robot-toast').className = 'toast' + (error ? ' error' : '');
  $('robot-toast').hidden = false;
  timer = setTimeout(() => $('robot-toast').hidden = true, 6000);
}

async function api(path, method = 'GET', body) {
  const r = await fetch('/api' + path, {method, headers: body ? {'Content-Type': 'application/json'} : {}, body: body ? JSON.stringify(body) : undefined});
  const data = await r.json();
  if (!r.ok) throw new Error(typeof data.detail === 'string' ? data.detail : 'Проверьте параметры.');
  return data;
}

async function action(path, body) {
  if (busy) return;
  busy = true;
  try { const s = await api(path, 'POST', body); render(s); return s; }
  catch (e) { toast(e.message, true); }
  finally { busy = false; if (state) render(state); }
}

const mm = value => `${(value * 1000).toFixed(1)} мм`;

function render(s) {
  state = s;
  const c = s.calibration, live = s.live, recording = s.recording;
  $('robot-connection').classList.toggle('on', s.connected);
  $('robot-connection').lastChild.textContent = s.connected ? ' USB-камера подключена' : ' Камера отключена';
  $('robot-connect').textContent = s.connected ? 'Отключить камеру' : 'Подключить камеру';
  const p = s.performance || {};
  $('robot-performance').textContent = `видео ${(p.preview_fps || 0).toFixed(0)} · анализ ${(p.analysis_fps || 0).toFixed(1)} fps`;
  map.setRobotSize(c.robot_size_m);
  if (first) {
    if (c.height_m !== null) $('height-mm').value = Math.round(c.height_m * 1000);
    $('robot-length').value = c.robot_size_m[0];
    $('robot-width').value = c.robot_size_m[1];
    first = false;
  }
  const estimate = s.height_estimate;
  $('height-estimate').textContent = estimate
    ? `Оценка камеры: ${Math.round(estimate.height_m * 1000)} мм ± ${Math.round(estimate.spread_m * 1000)}\nНаклон доски: ${estimate.tilt_deg.toFixed(1)}° · кадров ${estimate.frames}`
    : 'Оценка камеры: покажите доску на роботе';
  $('height-estimate').style.whiteSpace = 'pre-line';
  $('height-use').disabled = !estimate || busy;
  setTag('height-tag', c.height_m !== null, c.height_m !== null ? `${Math.round(c.height_m * 1000)} ММ` : 'НЕ ЗАДАНА');
  setTag('spin-tag', !!c.spin, c.spin ? 'ГОТОВО' : 'НЕ ЗАДАН');
  setTag('drive-tag', !!c.drive, c.drive ? 'ГОТОВО' : 'ОСЬ X ДОСКИ');

  recordButton('spin-record', recording === 'spin', 'вращения', s.recorded);
  recordButton('drive-record', recording === 'drive', 'движения', s.recorded);
  $('spin-record').disabled = busy || c.height_m === null || (recording && recording !== 'spin');
  $('drive-record').disabled = busy || !c.spin || (recording && recording !== 'drive');
  $('spin-result').hidden = !c.spin;
  if (c.spin) {
    $('spin-result').textContent = `Смещение центра от доски: ${mm(c.spin.offset_m[0])}, ${mm(c.spin.offset_m[1])}\nРазброс центра: ${mm(c.spin.residual_m)} · поворот ${c.spin.span_deg.toFixed(0)}°`;
    $('spin-result').classList.toggle('bad', c.spin.residual_m > .015);
  }
  $('drive-result').hidden = !c.drive;
  if (c.drive) {
    $('drive-result').textContent = `«Вперёд» = ось X доски ${(c.heading_offset_rad * 180 / Math.PI).toFixed(1)}°\nПроехал ${c.drive.distance_m.toFixed(2)} м · уход вбок ${mm(c.drive.lateral_rms_m)}`;
    $('drive-result').classList.toggle('bad', c.drive.lateral_rms_m > .02 || c.drive.yaw_change_deg > 5);
  }
  for (const el of ['spin-result', 'drive-result']) $(el).style.whiteSpace = 'pre-line';
  $('robot-reset').disabled = busy || (!c.spin && !c.drive);

  const robot = live && live.robot;
  $('robot-tags').textContent = live ? live.tags : '—';
  $('robot-std').textContent = robot ? `${(robot.std_xy_m * 1000).toFixed(1)} мм` : '—';
  $('robot-rms').textContent = robot ? `${robot.rms_px.toFixed(2)} px` : '—';
  let reason = s.camera_error || (!s.floor_ready ? 'Нет привязки к полу: завершите шаг 2.' : !s.connected ? 'Подключите камеру.' : live ? live.reason : 'Робот не виден: доска на роботе вне кадра или закрыта.');
  if (recording) reason = `Идёт запись ${recording === 'spin' ? 'вращения' : 'движения'}: ${s.recorded} кадров. Нажмите «Закончить», когда закончите.`;
  $('robot-reason').textContent = reason;
  $('robot-message').classList.toggle('ready', !!robot && robot.source !== 'free_6dof');
}

function setTag(id, done, text) { $(id).textContent = text; $(id).classList.toggle('done', done); }

function recordButton(id, active, noun, count) {
  $(id).classList.toggle('recording', active);
  $(id).textContent = active ? `Закончить запись · ${count} кадров` : `Начать запись ${noun}`;
}

function onPose(data) {
  const f = formatPose(data.robot);
  $('pose-x').textContent = f.x;
  $('pose-y').textContent = f.y;
  $('pose-yaw').textContent = f.yaw;
  $('pose-z').textContent = f.z;
  $('pose-tilt').textContent = f.tilt;
  $('robot-dot').classList.toggle('on', !!data.robot);
  $('robot-status').textContent = data.robot ? 'РОБОТ НА КАРТЕ' : 'ПОИСК РОБОТА';
}

setInterval(() => {
  const st = map.stats();
  $('pose-speed').textContent = st.age < 1.5 ? st.speed.toFixed(2) : '—';
  $('robot-rate').textContent = st.rate ? `${st.rate.toFixed(1)} Гц` : '—';
}, 250);

$('height-use').onclick = () => { if (state?.height_estimate) $('height-mm').value = Math.round(state.height_estimate.height_m * 1000); };
$('height-apply').onclick = async () => {
  const height = Number($('height-mm').value);
  if (!$('height-mm').value || !(height >= 0)) { toast('Введите высоту доски в миллиметрах.', true); return; }
  const s = await action('/robot/settings', {height_m: height / 1000, robot_length_m: Number($('robot-length').value), robot_width_m: Number($('robot-width').value)});
  if (s) toast('Высота сохранена. Робот появится на карте.');
};
for (const [id, kind] of [['spin-record', 'spin'], ['drive-record', 'drive']]) {
  $(id).onclick = async () => {
    if (state?.recording === kind) {
      const s = await action('/robot/stop');
      if (s) toast(kind === 'spin' ? 'Центр робота найден.' : 'Направление «вперёд» найдено.');
    } else {
      await action('/robot/record', {kind});
    }
  };
}
$('robot-reset').onclick = () => action('/robot/reset');
$('robot-connect').onclick = async () => {
  if (state?.connected) return action('/camera/disconnect');
  try {
    const floor = await api('/floor/state');
    if (floor.intrinsics) await action('/camera/connect', {width: floor.intrinsics.image_width, height: floor.intrinsics.image_height});
  } catch (e) { toast(e.message, true); }
};
$('camera-toggle').onchange = () => {
  $('camera-panel').hidden = !$('camera-toggle').checked;
  if ($('camera-toggle').checked) $('robot-preview').src = '/api/stream';
  else $('robot-preview').removeAttribute('src');
};

async function init() {
  try { await api('/mode', 'POST', {mode: 'robot'}); } catch (e) { toast(e.message, true); }
  connectMap(map, onPose);
  setInterval(async () => { if (!busy) try { render(await api('/robot/state')); } catch (e) { $('robot-reason').textContent = 'Нет связи с приложением.'; } }, 700);
  try { render(await api('/robot/state')); } catch (e) { toast(e.message, true); }
}
init();
