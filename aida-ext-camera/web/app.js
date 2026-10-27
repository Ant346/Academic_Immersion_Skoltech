const $ = id => document.getElementById(id);
let state = null, busy = false, boardDirty = false, sampleSignature = '', toastTimer, pollBusy = false, autoBusy = false;

function toast(message, error=false) {
  clearTimeout(toastTimer); $('toast').textContent = message;
  $('toast').className = 'toast' + (error ? ' error' : ''); $('toast').hidden = false;
  toastTimer = setTimeout(() => $('toast').hidden = true, 5500);
}
async function api(path, method='GET', body) {
  const response = await fetch('/api' + path, {method, headers: body ? {'Content-Type':'application/json'} : {}, body: body ? JSON.stringify(body) : undefined});
  const data = await response.json();
  if (!response.ok) throw new Error(typeof data.detail === 'string' ? data.detail : 'Проверьте значения в форме.');
  return data;
}
async function action(path, body, method='POST', message) {
  if (busy) return;
  busy = true; if (state) render(state);
  try { const data = await api(path, method, body); render(data); if (message) toast(message); return data; }
  catch (error) { toast(error.message, true); }
  finally { busy = false; if (state) render(state); }
}
function fillBoard(s) {
  const b = s.board;
  for (const [id, key] of [['columns','columns'],['rows','rows'],['tag-size','tag_size_mm'],['gap','gap_mm'],['first-id','first_id'],['rotation','rotation'],['border-bits','border_bits']]) $(id).value = b[key];
  $('id-order').value = String(b.ids_bottom_up); $('confirmed').checked = s.confirmed;
}
function render(s) {
  const first = state === null; state = s;
  if (first) {
    fillBoard(s);
    if (s.migration?.frames.length) toast(`Сохранённые кадры перераспознаны новым детектором: ${s.migration.frames.filter(f=>f.accepted).length} из ${s.migration.frames.length}. Исходные файлы сохранены.`);
  }
  const connected = s.connected, count = s.samples.length, running = s.job.running;
  $('connection').classList.toggle('on', connected); $('connection').lastChild.textContent = connected ? ' USB-камера подключена' : ' Камера отключена';
  $('connect').textContent = connected ? 'Отключить камеру' : 'Подключить камеру';
  $('connect').disabled = busy || running;
  $('resolution').disabled = connected || count > 0 || busy || running;
  $('resolution-note').textContent = count ? 'Разрешение зафиксировано сохранёнными кадрами. Для другого разрешения начните новую сессию.' : connected ? 'Чтобы изменить разрешение, сначала отключите камеру.' : 'Доступные режимы подключённой USB-камеры.';
  if (s.camera.width) {
    $('camera-actual').textContent = `${s.camera.width} × ${s.camera.height} · ${s.camera.fps || '—'} fps`;
    if (s.camera.width !== s.camera.requested_width || s.camera.height !== s.camera.requested_height) $('camera-actual').textContent += ' · камера выбрала другой режим';
    if (count && [...$('resolution').options].some(option=>option.value===`${s.camera.width}x${s.camera.height}`)) $('resolution').value = `${s.camera.width}x${s.camera.height}`;
  }
  $('live-dot').classList.toggle('on', connected);
  $('preview-title').textContent = connected ? s.camera.name : 'Ожидание камеры';
  const img = $('preview');
  if (connected && !img.getAttribute('src')) img.src = '/api/stream';
  if (!connected && img.getAttribute('src')) img.removeAttribute('src');
  img.hidden = !connected; $('placeholder').hidden = connected; $('preview-badge').hidden = !connected;
  $('preview-badge').textContent = s.preview_undistort ? 'ИСКАЖЕНИЯ ИСПРАВЛЕНЫ · ТОЛЬКО ПРОСМОТР' : 'ИСХОДНОЕ ИЗОБРАЖЕНИЕ';
  $('undistort').disabled = !s.result || !connected || busy;
  $('undistort').checked = s.preview_undistort;
  const q = s.quality;
  $('tags').textContent = connected ? `${q.tags} / ${s.board.columns*s.board.rows}` : '—';
  $('sharpness').textContent = q.sharpness || '—'; $('area').textContent = q.area_percent ? `${q.area_percent}%` : '—';
  $('layout-error').textContent = q.layout_error_px ?? '—';
  $('quality-text').textContent = s.camera_error || (boardDirty ? 'Примените изменённые настройки доски.' : !s.confirmed ? 'Подтвердите размеры доски и постоянные настройки объектива.' : q.reason);
  const ready = connected && q.ready && s.confirmed && !boardDirty && !s.camera_error;
  $('quality-message').classList.toggle('ready', ready);
  $('capture').disabled = !ready || busy || running;
  $('auto-capture').disabled = !connected || running || !s.confirmed || boardDirty;
  if (!connected || running || !s.confirmed || boardDirty) $('auto-capture').checked = false;
  $('capture-count').textContent = `${count} / 25 кадров`; $('sample-total').textContent = count;
  $('calibrate').disabled = count < 12 || busy || running || boardDirty;
  $('calibrate').textContent = running ? 'Вычисляем…' : 'Рассчитать intrinsics ↗';
  $('new-session').disabled = running || busy;
  for (const id of ['columns','rows','tag-size','gap','first-id','rotation','id-order','border-bits']) $(id).disabled = count > 0 || running;
  $('save-board').disabled = busy || running; $('confirmed').disabled = running;
  $('board-status').textContent = boardDirty ? 'Есть несохранённые изменения.' : s.confirmed ? 'Параметры применены. Готово к съёмке.' : 'Подтвердите параметры перед сохранением кадров.';
  $('step1').className = connected && s.confirmed ? 'done' : 'active';
  $('step2').className = s.result ? 'done' : connected && s.confirmed ? 'active' : '';
  $('step3').className = s.result ? 'active' : '';
  const cells = s.coverage.flat(), covered = cells.filter(v=>v>0).length;
  $('coverage-percent').textContent = `${Math.round(100*covered/24)}%`;
  $('coverage').replaceChildren(...cells.map(value=>{const el = document.createElement('div'); el.title = `${value} кадров`; if(value) el.style.background = `rgba(37,88,236,${Math.min(.2 + value*.08,.9)})`; return el;}));
  $('job-error').textContent = s.job.error || '';
  $('result').hidden = !s.result; $('result-empty').hidden = !!s.result;
  if (s.result) renderResult(s.result);
  const signature = JSON.stringify(s.samples);
  if (signature !== sampleSignature) { sampleSignature = signature; renderSamples(s.samples); }
}
function renderResult(result) {
  $('rms').textContent = result.rms_px.toFixed(3); $('result-resolution').textContent = `${result.image_width} × ${result.image_height}`; $('result-count').textContent = result.frame_count;
  $('matrix').textContent = result.camera_matrix.map(row=>row.map(v=>v.toFixed(3).padStart(10)).join(' ')).join('\n');
  $('distortion').replaceChildren(...result.distortion_coefficients.map((v,i)=>{const el=document.createElement('div');const label=document.createElement('span'); label.textContent=['k₁','k₂','p₁','p₂','k₃'][i];el.append(label,document.createTextNode(v.toFixed(6)));return el;}));
  $('result-warnings').replaceChildren(...result.warnings.map(text=>{const el=document.createElement('p');el.textContent=text;return el;}));
}
function renderSamples(samples) {
  const parent = $('samples'); parent.replaceChildren();
  if (!samples.length) {const el = document.createElement('div');el.className='samples-empty';el.textContent='Здесь появятся кадры, которые участвуют в калибровке.';parent.append(el);return;}
  samples.forEach((sample,index)=>{
    const card=document.createElement('article');card.className='sample';
    const visual=document.createElement('div');visual.className='sample-image';
    const img=document.createElement('img');img.src=sample.thumbnail;img.alt=`Наблюдение ${index+1}, ${sample.tags} маркеров`;img.loading='lazy';
    const remove=document.createElement('button');remove.className='sample-delete';remove.textContent='×';remove.title='Исключить кадр из расчёта';remove.setAttribute('aria-label',`Исключить кадр ${index+1}`);
    remove.onclick=()=>action(`/samples/${sample.id}`,undefined,'DELETE','Кадр исключён. Пересчитайте intrinsics.');
    visual.append(img,remove);
    const info=document.createElement('div');info.className='sample-info';
    const number=document.createElement('strong');number.textContent=`${String(index+1).padStart(2,'0')} · ${sample.tags} tags`;
    const error=document.createElement('span');error.textContent=sample.rms_px===null?'— px':`${sample.rms_px.toFixed(2)} px`;if(sample.rms_px>1)error.className='bad';
    info.append(number,error);card.append(visual,info);parent.append(card);
  });
}
$('connect').onclick=()=>{
  if (state?.connected) return action('/camera/disconnect');
  const [width,height]=$('resolution').value.split('x').map(Number);
  action('/camera/connect',{width,height});
};
$('board-form').addEventListener('input',()=>{boardDirty=true;if(state)render(state);});
$('board-form').onsubmit=async event=>{
  event.preventDefault();
  const body={columns:Number($('columns').value),rows:Number($('rows').value),tag_size_mm:Number($('tag-size').value),gap_mm:Number($('gap').value),first_id:Number($('first-id').value),rotation:Number($('rotation').value),border_bits:Number($('border-bits').value),ids_bottom_up:$('id-order').value==='true',confirmed:$('confirmed').checked};
  const result=await action('/board',body,'POST','Настройки доски применены.');
  if(result){boardDirty=false;render(result);}
};
$('capture').onclick=()=>action('/samples',undefined,'POST','Кадр сохранён. Переместите или наклоните доску.');
$('calibrate').onclick=()=>action('/calibrate');
$('undistort').onchange=()=>action('/preview',{undistort:$('undistort').checked});
$('new-session').onclick=()=>$('reset-dialog').showModal();
$('cancel-reset').onclick=()=>$('reset-dialog').close();
$('confirm-reset').onclick=async()=>{const result=await action('/session/new');if(result){boardDirty=false;fillBoard(result);render(result);$('reset-dialog').close();toast('Новая сессия. Предыдущие файлы сохранены.');}};
document.addEventListener('keydown',event=>{if(event.code==='Space'&&!['INPUT','SELECT','TEXTAREA','BUTTON'].includes(event.target.tagName)&&!$('reset-dialog').open){event.preventDefault();if(!$('capture').disabled)$('capture').click();}});
setInterval(async()=>{if(pollBusy)return;pollBusy=true;try{render(await api('/state'));}catch(error){$('connection').lastChild.textContent=' Нет связи с приложением';$('capture').disabled=true;$('auto-capture').checked=false;}finally{pollBusy=false;}},900);
setInterval(async()=>{if(!$('auto-capture').checked||$('capture').disabled||autoBusy)return;autoBusy=true;try{await action('/samples');}finally{autoBusy=false;}},3000);
async function init(){
  try {
    await api('/mode','POST',{mode:'intrinsics'});
    const [camera,s]=await Promise.all([api('/camera'),api('/state')]);
    $('camera-name').replaceChildren(new Option(camera.name,camera.device));
    $('resolution').replaceChildren(...camera.modes.map(m=>new Option(`${m.width} × ${m.height} · ${m.fourcc}`,`${m.width}x${m.height}`)));
    const desired=s.camera.width?`${s.camera.width}x${s.camera.height}`:null;
    if (!camera.modes.length) $('resolution').replaceChildren(new Option('Камера не сообщает доступные режимы',''));
    if (s.samples.length && desired && ![...$('resolution').options].some(o=>o.value===desired)) $('resolution').add(new Option(`${s.camera.width} × ${s.camera.height} — режим сохранённой сессии`,desired));
    if([...$('resolution').options].some(o=>o.value===desired))$('resolution').value=desired;
    render(s);
    if(!camera.available)toast('USB-камера не найдена. Проверьте подключение и USB_CAMERA_DEVICE.',true);
  } catch(error){toast(`Не удалось загрузить приложение: ${error.message}`,true);}
}
init();
