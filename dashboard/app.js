"use strict";

const $ = (selector) => document.querySelector(selector);
const state = { status: null, roi: null, pointerStart: null, lastMarker: null, toastTimer: null };

function number(value, digits = 2) {
  return value === null || value === undefined || !Number.isFinite(Number(value)) ? "—" : Number(value).toFixed(digits);
}

function toast(message, error = false) {
  const element = $("#toast");
  element.textContent = message;
  element.classList.toggle("error", error);
  element.classList.add("show");
  clearTimeout(state.toastTimer);
  state.toastTimer = setTimeout(() => element.classList.remove("show"), 4200);
}

async function api(path, options = {}, retried = false) {
  const config = { method: options.method || "GET", headers: {} };
  const operatorToken = sessionStorage.getItem("cartpoleOperatorToken");
  if (operatorToken) config.headers["X-Operator-Token"] = operatorToken;
  if (options.body !== undefined) {
    config.headers["Content-Type"] = "application/json";
    config.body = JSON.stringify(options.body);
  }
  const response = await fetch(path, config);
  let payload = {};
  try { payload = await response.json(); } catch (_) { /* empty response */ }
  if (response.status === 401 && !retried) {
    const token = window.prompt("운영자 토큰을 입력하세요. 이 값은 현재 브라우저 세션에만 보관됩니다.");
    if (token) {
      sessionStorage.setItem("cartpoleOperatorToken", token);
      return api(path, options, true);
    }
  }
  if (!response.ok) throw new Error(payload.detail || `${response.status} ${response.statusText}`);
  return payload;
}

function setBadge(element, status) {
  element.classList.remove("good", "warn", "bad");
  const good = ["ok", "IDLE", "RUNNING"];
  const warn = ["low_confidence", "partial_loss", "ARMED", "UNHOMED", "CALIBRATION_REQUIRED"];
  element.classList.add(good.includes(status) ? "good" : warn.includes(status) ? "warn" : "bad");
}

function renderStatus(data) {
  state.status = data;
  $("#connection").classList.add("online");
  $("#connection").lastChild.textContent = " 실시간 연결";
  const safety = $("#safety-state");
  safety.textContent = data.safety_state;
  safety.className = `state state-${data.safety_state.toLowerCase()}`;
  $("#safety-reason").textContent = data.safety_reason;
  $("#fps").textContent = number(data.fps, 1);
  $("#processing-ms").textContent = number(data.tracking.processing_ms, 1);
  $("#latency-ms").textContent = number(data.control_latency_ms, 1);
  const power = data.system_health;
  $("#pi-power").textContent = power
    ? (power.clean === true ? "OK" : (power.clean === false ? power.raw_hex : "N/A"))
    : "N/A";
  $("#brightness").textContent = number(data.tracking.brightness, 0);

  const tracking = data.tracking.status;
  $("#tracking-pill").textContent = tracking;
  setBadge($("#tracking-pill"), tracking);
  $("#tracking-reason").textContent = data.tracking.reason || "—";

  $("#theta").textContent = data.state.theta_rad == null ? "—" : number(data.state.theta_rad * 180 / Math.PI, 2);
  $("#omega").textContent = number(data.state.omega_rad_s, 2);
  $("#cart-position").textContent = number(data.state.cart_position_mm, 1);
  $("#cart-velocity").textContent = number(data.state.cart_velocity_mm_s, 1);
  $("#pid-output").textContent = number(data.control.limited_velocity_mm_s, 1);
  const swing = data.swing_up;
  const modeNames = { idle: "스윙업 대기", swing_up: "에너지 스윙업", balance: "수직 균형", fault: "스윙업 중단" };
  $("#control-mode").textContent = swing
    ? `${swing.enabled ? (modeNames[swing.mode] || swing.mode) : "균형 전용 · 스윙업 꺼짐"} · ${data.control.reason || "대기"}`
    : (data.control.reason || "—");
  $("#motor-state").textContent = data.motor.connected ? (data.motor.homed ? "ONLINE · HOMED" : "ONLINE · UNHOMED") : "OFFLINE";
  $("#motor-detail").textContent = data.motor.fault || `firmware x=${number(data.motor.position_mm, 1)} mm`;

  const calibrated = data.calibration.live_verified;
  $("#calibration-badge").textContent = calibrated ? "실시간 검증됨" : `검증 ${data.calibration.verification_frames || 0}/15`;
  setBadge($("#calibration-badge"), calibrated ? "ok" : "CALIBRATION_REQUIRED");
  renderGains(data.gains);
  $("#gain-source").textContent = data.gain_source || "config";
  $("#experiment-state").textContent = data.experiment.active ? "기록 중" : "대기";
  setBadge($("#experiment-state"), data.experiment.active ? "RUNNING" : "IDLE");
  updateProfiles(data.calibration.available_profiles, data.calibration.profile);
}

function renderGains(gains) {
  const names = {
    kp_angle: "Kp · angle", ki_angle: "Ki · angle", kd_angle: "Kd · angle",
    kp_position: "Kp · position", ki_position: "Ki · position", kd_position: "Kd · position"
  };
  $("#gains").innerHTML = Object.entries(names).map(([key, label]) => `<div class="gain"><span>${label}</span><b>${number(gains[key], 5)}</b></div>`).join("");
}

function escapeHtml(value) {
  return String(value).replace(/[&<>'"]/g, char => ({"&":"&amp;", "<":"&lt;", ">":"&gt;", "'":"&#39;", '"':"&quot;"}[char]));
}

let knownProfiles = "";
function updateProfiles(profiles, active) {
  const signature = JSON.stringify([profiles, active]);
  if (signature === knownProfiles) return;
  knownProfiles = signature;
  $("#profile-select").innerHTML = `<option value="">—</option>` + profiles.map(name => `<option value="${escapeHtml(name)}" ${name === active ? "selected" : ""}>${escapeHtml(name)}</option>`).join("");
}

function connectSocket() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  const socket = new WebSocket(`${protocol}//${location.host}/ws`);
  socket.onmessage = event => renderStatus(JSON.parse(event.data));
  socket.onclose = () => {
    $("#connection").classList.remove("online");
    $("#connection").lastChild.textContent = " 재연결 중";
    setTimeout(connectSocket, 1300);
  };
  socket.onerror = () => socket.close();
}

async function postControl(action, message) {
  try {
    await api(`/api/control/${action}`, { method: "POST" });
    toast(message);
  } catch (error) { toast(error.message, true); }
}

// ROI selection is drawn in CSS pixels and converted to native camera pixels on submit.
const overlay = $("#roi-overlay");
function resizeOverlay() {
  const rect = overlay.getBoundingClientRect();
  overlay.width = Math.max(1, Math.round(rect.width * devicePixelRatio));
  overlay.height = Math.max(1, Math.round(rect.height * devicePixelRatio));
  drawROI();
}
function pointerPosition(event) {
  const rect = overlay.getBoundingClientRect();
  return { x: Math.max(0, Math.min(rect.width, event.clientX - rect.left)), y: Math.max(0, Math.min(rect.height, event.clientY - rect.top)) };
}
function drawROI() {
  const context = overlay.getContext("2d");
  const scale = devicePixelRatio;
  context.clearRect(0, 0, overlay.width, overlay.height);
  if (!state.roi) return;
  context.save(); context.scale(scale, scale);
  context.strokeStyle = "#44e3c4"; context.lineWidth = 2; context.setLineDash([7, 4]);
  context.fillStyle = "rgba(68,227,196,.10)";
  context.fillRect(state.roi.x, state.roi.y, state.roi.width, state.roi.height);
  context.strokeRect(state.roi.x, state.roi.y, state.roi.width, state.roi.height);
  context.restore();
}
overlay.addEventListener("pointerdown", event => { state.pointerStart = pointerPosition(event); overlay.setPointerCapture(event.pointerId); });
overlay.addEventListener("pointermove", event => {
  if (!state.pointerStart) return;
  const point = pointerPosition(event);
  state.roi = { x: Math.min(point.x, state.pointerStart.x), y: Math.min(point.y, state.pointerStart.y), width: Math.abs(point.x - state.pointerStart.x), height: Math.abs(point.y - state.pointerStart.y) };
  drawROI();
  $("#roi-readout").textContent = `${Math.round(state.roi.width)} × ${Math.round(state.roi.height)} 화면 px`;
});
overlay.addEventListener("pointerup", () => { state.pointerStart = null; });
window.addEventListener("resize", resizeOverlay);
$("#video").addEventListener("load", () => { $("#video-placeholder").hidden = true; resizeOverlay(); });

async function sampleROI() {
  if (!state.roi || state.roi.width < 4 || state.roi.height < 4) return toast("영상에서 충분한 크기의 ROI를 드래그하세요.", true);
  const image = $("#video");
  const rect = image.getBoundingClientRect();
  const nativeWidth = image.naturalWidth || 1280;
  const nativeHeight = image.naturalHeight || 720;
  // object-fit: contain may letterbox; account for the rendered content rectangle.
  const scale = Math.min(rect.width / nativeWidth, rect.height / nativeHeight);
  const contentWidth = nativeWidth * scale, contentHeight = nativeHeight * scale;
  const offsetX = (rect.width - contentWidth) / 2, offsetY = (rect.height - contentHeight) / 2;
  const native = {
    x: Math.round((state.roi.x - offsetX) / scale), y: Math.round((state.roi.y - offsetY) / scale),
    width: Math.round(state.roi.width / scale), height: Math.round(state.roi.height / scale)
  };
  try {
    const payload = await api("/api/calibration/sample-roi", { method: "POST", body: {
      name: $("#marker-name").value, role: $("#marker-role").value, required: $("#marker-required").checked, ...native
    }});
    state.lastMarker = payload.marker.name;
    $("#marker-json").value = JSON.stringify(payload.marker, null, 2);
    $("#mask-preview").src = `/api/calibration/mask/${encodeURIComponent(payload.marker.name)}.jpg?t=${Date.now()}`;
    toast("HSV 범위를 계산했습니다. 마스크를 확인한 뒤 프로파일을 저장하세요.");
  } catch (error) { toast(error.message, true); }
}

async function updateMarker() {
  try {
    const marker = JSON.parse($("#marker-json").value);
    const name = marker.name || state.lastMarker;
    const payload = await api(`/api/calibration/markers/${encodeURIComponent(name)}`, { method: "PUT", body: {
      ranges: marker.hsv_ranges, min_area_px: marker.min_area_px, max_area_px: marker.max_area_px
    }});
    $("#marker-json").value = JSON.stringify(payload.marker, null, 2);
    $("#mask-preview").src = `/api/calibration/mask/${encodeURIComponent(name)}.jpg?t=${Date.now()}`;
    toast("수동 HSV/면적 범위를 반영했습니다.");
  } catch (error) { toast(error.message, true); }
}

function optionalNumber(selector) { const value = $(selector).value.trim(); return value === "" ? null : Number(value); }
async function saveProfile() {
  try {
    const body = {
      name: $("#profile-name").value,
      notes: "dashboard calibration",
      coordinates: {
        rail_left_px: Number($("#rail-left").value), rail_right_px: Number($("#rail-right").value),
        rail_length_mm: Number($("#rail-length").value), camera_x_sign: Number($("#camera-sign").value),
        pivot_y_px: optionalNumber("#pivot-y"), vertical_offset_rad: Number($("#vertical-offset").value),
        pendulum_length_mm: optionalNumber("#pendulum-length"), homography: null
      }
    };
    const payload = await api("/api/calibration/save", { method: "POST", body });
    toast(payload.structurally_valid ? "프로파일을 저장했습니다. 15프레임 실시간 검증을 시작합니다." : "프로파일은 저장했지만 필수 기하 정보가 부족합니다.", !payload.structurally_valid);
  } catch (error) { toast(error.message, true); }
}

async function refreshExperiments() {
  try {
    const { experiments } = await api("/api/experiments?limit=30");
    $("#experiment-table").innerHTML = experiments.map(item => {
      const m = item.metrics || {};
      const result = m.stabilization_success ? "성공" : (m.failure_type || "미달");
      const source = item.context?.gain_source || "config";
      return `<tr><td title="${escapeHtml(item.experiment_id)}">${escapeHtml(item.experiment_id.slice(-8))}</td><td>${escapeHtml(source)}</td><td>${number(m.continuous_stable_time_s, 2)} s</td><td>${m.rms_angle_rad == null ? "—" : number(m.rms_angle_rad * 180 / Math.PI, 2) + "°"}</td><td>${number((m.marker_detection_rate || 0) * 100, 1)}%</td><td>${escapeHtml(result)}</td></tr>`;
    }).join("") || `<tr><td colspan="6" class="muted">측정된 실험이 없습니다.</td></tr>`;
    const history = experiments.slice().reverse().map((item, index) => ({ ...item, elapsed_s: index + 1 }));
    drawChart($("#performance-chart"), history, [
      {label:"stable s",color:"#44e3c4",value:x=>x.metrics?.continuous_stable_time_s},
      {label:"RMS °",color:"#ffb648",value:x=>x.metrics?.rms_angle_rad==null?NaN:x.metrics.rms_angle_rad*180/Math.PI}
    ]);
    drawChart($("#angle-gain-chart"), history, [
      {label:"Kp",color:"#44e3c4",value:x=>x.gains?.kp_angle},
      {label:"Ki",color:"#5d8dff",value:x=>x.gains?.ki_angle},
      {label:"Kd",color:"#ffb648",value:x=>x.gains?.kd_angle}
    ]);
    drawChart($("#position-gain-chart"), history, [
      {label:"Kp",color:"#44e3c4",value:x=>x.gains?.kp_position},
      {label:"Ki",color:"#5d8dff",value:x=>x.gains?.ki_position},
      {label:"Kd",color:"#ffb648",value:x=>x.gains?.kd_position}
    ]);
  } catch (_) { /* dashboard remains usable */ }
}

function drawChart(canvas, samples, series, zeroCentered = false) {
  const rect = canvas.getBoundingClientRect(), dpr = devicePixelRatio;
  canvas.width = Math.max(2, Math.round(rect.width * dpr)); canvas.height = Math.max(2, Math.round(rect.height * dpr));
  const ctx = canvas.getContext("2d"); ctx.scale(dpr, dpr);
  const width = rect.width, height = rect.height, pad = {l: 42, r: 12, t: 10, b: 24};
  ctx.clearRect(0, 0, width, height); ctx.strokeStyle = "#27313d"; ctx.lineWidth = 1;
  for (let i=0;i<=4;i++) { const y=pad.t+(height-pad.t-pad.b)*i/4; ctx.beginPath();ctx.moveTo(pad.l,y);ctx.lineTo(width-pad.r,y);ctx.stroke(); }
  const valid = samples.filter(item => series.some(s => Number.isFinite(Number(s.value(item)))));
  if (valid.length < 2) { ctx.fillStyle="#71808f";ctx.fillText("실험 데이터 대기 중", pad.l, height/2);return; }
  const xs = valid.map(item => Number(item.elapsed_s)); let xMin=Math.min(...xs),xMax=Math.max(...xs); if(xMax===xMin)xMax=xMin+1;
  let values=[]; series.forEach(s => valid.forEach(item => { const value=Number(s.value(item)); if(Number.isFinite(value))values.push(value); }));
  let yMin=Math.min(...values),yMax=Math.max(...values); if(zeroCentered){const span=Math.max(Math.abs(yMin),Math.abs(yMax),.01);yMin=-span;yMax=span;} if(yMax===yMin){yMax+=1;yMin-=1;} const margin=(yMax-yMin)*.08;yMin-=margin;yMax+=margin;
  const xScale=x=>pad.l+(x-xMin)/(xMax-xMin)*(width-pad.l-pad.r); const yScale=y=>pad.t+(yMax-y)/(yMax-yMin)*(height-pad.t-pad.b);
  ctx.font="10px ui-monospace";ctx.fillStyle="#748494";ctx.fillText(yMax.toFixed(1),2,pad.t+5);ctx.fillText(yMin.toFixed(1),2,height-pad.b);
  series.forEach((s,index)=>{ctx.strokeStyle=s.color;ctx.lineWidth=1.7;ctx.beginPath();let started=false;valid.forEach(item=>{const v=Number(s.value(item));if(!Number.isFinite(v)){started=false;return;}const x=xScale(Number(item.elapsed_s)),y=yScale(v);if(!started){ctx.moveTo(x,y);started=true;}else ctx.lineTo(x,y);});ctx.stroke();ctx.fillStyle=s.color;ctx.fillText(s.label,pad.l+index*90,height-6);});
}

async function refreshCharts() {
  try {
    const { samples } = await api("/api/recent?limit=700");
    drawChart($("#angle-chart"), samples, [
      {label:"theta °",color:"#44e3c4",value:x=>x.theta_rad==null?NaN:x.theta_rad*180/Math.PI},
      {label:"omega",color:"#5d8dff",value:x=>x.omega_rad_s}
    ], true);
    drawChart($("#cart-chart"), samples, [
      {label:"x mm",color:"#ffb648",value:x=>x.cart_position_mm},
      {label:"PID v",color:"#ff6c76",value:x=>x.requested_velocity_mm_s},
      {label:"limited v",color:"#5d8dff",value:x=>x.limited_velocity_mm_s}
    ]);
  } catch (_) { /* no samples yet */ }
}

function bindActions() {
  $("#estop").onclick = () => postControl("estop", "ESTOP을 전송하고 래치했습니다.");
  $("#home").onclick = () => confirm("카트가 저속으로 양쪽 리미트까지 이동합니다. 레일이 비어 있고 감독 중입니까?") && postControl("home", "호밍을 시작했습니다.");
  $("#arm").onclick = () => postControl("arm", "시스템을 ARM 상태로 전환했습니다.");
  $("#start-control").onclick = () => postControl("start", "PID 제어를 시작했습니다.");
  $("#stop-control").onclick = () => postControl("stop", "감속 정지를 요청했습니다.");
  $("#clear-fault").onclick = () => postControl("clear-fault", "FAULT를 해제했습니다.");
  $("#clear-estop").onclick = () => confirm("물리 리미트와 위험 원인을 확인했습니까? 해제 후 반드시 재호밍해야 합니다.") && postControl("clear-estop", "ESTOP을 해제했습니다. 재호밍이 필요합니다.");
  $("#sample-roi").onclick = sampleROI;
  $("#update-marker").onclick = updateMarker;
  $("#save-profile").onclick = saveProfile;
  $("#load-profile").onclick = async () => { const name=$("#profile-select").value;if(!name)return;try{await api(`/api/calibration/profiles/${encodeURIComponent(name)}/load`,{method:"POST"});toast("프로파일을 불러왔습니다. 실시간 품질을 재검증합니다.");}catch(error){toast(error.message,true);} };
  $("#start-experiment").onclick = async () => { try{await api("/api/experiments/start",{method:"POST",body:{notes:$("#experiment-notes").value}});toast("실험 기록을 시작했습니다.");}catch(error){toast(error.message,true);} };
  $("#finish-experiment").onclick = async () => { try{const result=await api("/api/experiments/finish",{method:"POST"});toast(`평가 완료: 안정 ${number(result.metrics.continuous_stable_time_s,2)}초`);refreshExperiments();}catch(error){toast(error.message,true);} };
}

bindActions();
connectSocket();
resizeOverlay();
refreshExperiments();
refreshCharts();
setInterval(refreshExperiments, 5000);
setInterval(refreshCharts, 1000);
setInterval(() => {
  if (state.lastMarker) {
    $("#mask-preview").src = `/api/calibration/mask/${encodeURIComponent(state.lastMarker)}.jpg?t=${Date.now()}`;
  }
}, 700);

