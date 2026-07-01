const state = {
  configApplied: false,
  polling: false,
};

const elevatorFsmStates = ["WAIT_BOARD", "BOARDING", "RIDING", "EXITING", "DONE"];

const $ = (id) => document.getElementById(id);

const api = async (path, options = {}) => {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const data = await response.json();
  if (!response.ok || data.ok === false) {
    throw new Error(data.message || `HTTP ${response.status}`);
  }
  return data;
};

const escapeHtml = (value) =>
  String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

const formatAge = (ms) => {
  if (ms === null || ms === undefined) return "n/a";
  if (ms < 1000) return `${Math.round(ms)} ms`;
  return `${(ms / 1000).toFixed(1)} s`;
};

const formatTime = (iso) => {
  if (!iso) return "-";
  const date = new Date(iso);
  if (Number.isNaN(date.getTime())) return String(iso);
  return date.toLocaleTimeString();
};

const formatNumber = (value, digits = 3) => {
  if (value === null || value === undefined || Number.isNaN(Number(value))) {
    return "-";
  }
  return Number(value).toFixed(digits);
};

const formatHex = (value) => {
  if (value === null || value === undefined || Number.isNaN(Number(value))) {
    return "-";
  }
  return `0x${Number(value).toString(16).toUpperCase().padStart(2, "0")}`;
};

const setConnection = (text, className) => {
  const node = $("connectionStatus");
  node.textContent = text;
  node.className = `connection-pill ${className}`;
};

const applyConfig = (config) => {
  if (state.configApplied || !config) return;

  const locations = Array.isArray(config.locations) ? config.locations : [];
  const defaults = config.default_goal || {};
  const pickup = $("pickupLocation");
  const delivery = $("deliveryLocation");

  const options = locations.length
    ? locations
    : [
        { name: defaults.pickup_location || "room_402" },
        { name: defaults.delivery_location || "room_501" },
      ];

  for (const select of [pickup, delivery]) {
    select.innerHTML = options
      .map((location) => {
        const label = location.type
          ? `${location.name} (${location.type})`
          : location.name;
        return `<option value="${escapeHtml(location.name)}">${escapeHtml(label)}</option>`;
      })
      .join("");
  }

  $("missionId").value = defaults.mission_id || "";
  $("objectLabel").value = defaults.object_label || "box";
  $("targetFloor").value = defaults.target_floor ?? 5;
  pickup.value = defaults.pickup_location || "room_402";
  delivery.value = defaults.delivery_location || "room_501";

  renderFlow(config.mission_steps || []);
  state.configApplied = true;
};

const renderFlow = (steps) => {
  const list = $("flowList");
  $("flowMeta").textContent = `${steps.length} steps`;

  if (!steps.length) {
    list.innerHTML = `<li class="empty">No flow</li>`;
    return;
  }

  list.innerHTML = steps
    .map(
      (step) => `
        <li>
          <span class="flow-state">${escapeHtml(step.state)}</span>
          <span class="flow-task">${escapeHtml(step.task)} | ${escapeHtml(step.location)}</span>
        </li>
      `,
    )
    .join("");
};

const renderMission = (mission) => {
  const status = mission.status;
  const feedback = mission.feedback;
  const result = mission.result;
  const goal = mission.goal;

  $("missionReady").textContent = mission.action_ready ? "Ready" : "Offline";
  $("missionReady").className = `status-chip ${mission.action_ready ? "success" : "danger"}`;

  const stateText =
    status?.state ||
    goal?.state ||
    result?.status ||
    (mission.active ? "ACTIVE" : "IDLE");
  $("missionState").textContent = `${stateText} | age ${formatAge(mission.status_age_ms)}`;

  const progress = clamp(
    Number(status?.progress ?? feedback?.progress ?? (result?.success ? 1 : 0)),
    0,
    1,
  );
  $("missionProgressText").textContent = `${Math.round(progress * 100)}%`;
  $("missionProgressBar").style.width = `${progress * 100}%`;

  $("missionTask").textContent =
    status?.active_task || feedback?.current_task || "-";
  $("missionMessage").textContent =
    status?.message || feedback?.detail || goal?.state || "-";
  $("missionResult").textContent = result
    ? `${result.status}: ${result.message}`
    : "-";
  renderElevatorFsm(status, feedback);

  $("startMissionButton").disabled = !mission.action_ready || mission.active;
  $("cancelMissionButton").disabled = !mission.active;
};

const extractElevatorFsmState = (status, feedback) => {
  const candidates = [
    feedback?.current_state,
    feedback?.detail,
    status?.state,
    status?.message,
    status?.active_task,
  ];

  for (const value of candidates) {
    const text = String(value || "");
    const direct = elevatorFsmStates.find((fsmState) => text === fsmState);
    if (direct) return direct;

    const embedded = elevatorFsmStates.find((fsmState) =>
      text.includes(fsmState),
    );
    if (embedded) return embedded;
  }

  return "";
};

const renderElevatorFsm = (status, feedback) => {
  const currentState = extractElevatorFsmState(status, feedback);
  const currentIndex = elevatorFsmStates.indexOf(currentState);
  $("elevatorFsmState").textContent = currentState || "-";

  $("elevatorFsmSteps").innerHTML = elevatorFsmStates
    .map((fsmState, index) => {
      const className =
        index === currentIndex
          ? "active"
          : currentIndex >= 0 && index < currentIndex
            ? "done"
            : "";
      return `<div class="fsm-step ${className}">${escapeHtml(fsmState)}</div>`;
    })
    .join("");
};

const boardHealth = (fields, notes) => {
  if (notes?.includes("no status")) return "danger";
  if (fields.stale === true) return "warning";
  if (fields.state === "ESTOP" || fields.state === "ERROR") return "danger";
  if (fields.error && fields.error !== "NONE") return "danger";
  if (Number(fields.fault || 0) !== 0) return "danger";
  if (fields.enabled === true && fields.stale === false) return "success";
  return "neutral";
};

const metric = (label, value) => `
  <div class="metric">
    <div class="metric-label">${escapeHtml(label)}</div>
    <div class="metric-value">${escapeHtml(value)}</div>
  </div>
`;

const renderBoards = (arm) => {
  $("armAge").textContent = `status age ${formatAge(arm.status_age_ms)}`;

  const parsed = arm.parsed_status;
  const controllers = parsed?.controllers || [];

  if (!controllers.length) {
    $("boardGrid").innerHTML = `<div class="empty">No board status</div>`;
    return;
  }

  const cards = [];
  for (const controller of controllers) {
    for (const board of controller.boards || []) {
      const fields = board.fields || {};
      const notes = board.notes || [];
      const health = boardHealth(fields, notes);
      const title = `${controller.name} / Board ${board.board_id}`;
      const stateText = notes.length
        ? notes.join(", ")
        : `${fields.state || "-"} / ${fields.error || "-"}`;

      cards.push(`
        <article class="board-card ${health}">
          <div class="board-title">
            <span>${escapeHtml(title)}</span>
            <span>${escapeHtml(controller.accept_traj === null ? "-" : `traj ${controller.accept_traj}`)}</span>
          </div>
          <div class="metric-grid">
            ${metric("State", stateText)}
            ${metric("Enabled", fields.enabled ?? "-")}
            ${metric("Ready", formatHex(fields.ready))}
            ${metric("Fault", formatHex(fields.fault))}
            ${metric("Queue", `${fields.local_queue_free ?? "-"} / ${fields.queue_free ?? "-"}`)}
            ${metric("Moving", fields.moving ?? "-")}
            ${metric("Stale", fields.stale ?? "-")}
            ${metric("Age", fields.age_ms === undefined ? "-" : `${formatNumber(fields.age_ms, 1)} ms`)}
            ${metric("Position", fields.position_valid ?? "-")}
          </div>
        </article>
      `);
    }
  }

  $("boardGrid").innerHTML = cards.join("") || `<div class="empty">No board status</div>`;
};

const renderJoints = (joints) => {
  $("jointAge").textContent = `state age ${formatAge(joints.age_ms)}`;
  const rows = joints.state?.joints || [];

  if (!rows.length) {
    $("jointTableBody").innerHTML = `<tr><td colspan="3">No data</td></tr>`;
    return;
  }

  $("jointTableBody").innerHTML = rows
    .map((joint) => {
      const rad = joint.position;
      const deg = rad === null || rad === undefined ? null : (rad * 180) / Math.PI;
      return `
        <tr>
          <td>${escapeHtml(joint.name)}</td>
          <td>${formatNumber(rad, 4)}</td>
          <td>${formatNumber(deg, 2)}</td>
        </tr>
      `;
    })
    .join("");
};

const renderLogs = (snapshot) => {
  $("serverTime").textContent = formatTime(snapshot.server.time);

  const events = (snapshot.events || []).slice(-60).reverse();
  $("eventLog").innerHTML = events.length
    ? events
        .map(
          (event) => `
            <li>
              <span class="log-time">${escapeHtml(formatTime(event.time))} | ${escapeHtml(event.kind)} | ${escapeHtml(event.level)}</span>
              <span class="log-message">${escapeHtml(event.message)}</span>
            </li>
          `,
        )
        .join("")
    : `<li class="empty">No events</li>`;

  const armLog = (snapshot.arm.log || []).slice(-40).reverse();
  $("armLog").innerHTML = armLog.length
    ? armLog
        .map(
          (entry) => `
            <li>
              <span class="log-time">${escapeHtml(formatTime(entry.received_at))}</span>
              <span class="log-message">${escapeHtml(entry.message)}</span>
            </li>
          `,
        )
        .join("")
    : `<li class="empty">No status log</li>`;
};

const renderSnapshot = (snapshot) => {
  applyConfig(snapshot.config);
  renderMission(snapshot.mission);
  renderBoards(snapshot.arm);
  renderJoints(snapshot.joints);
  renderLogs(snapshot);
  setConnection(`Online | uptime ${Math.floor(snapshot.server.uptime_s)} s`, "online");
};

const poll = async () => {
  if (state.polling) return;
  state.polling = true;
  try {
    const snapshot = await api("/api/snapshot");
    renderSnapshot(snapshot);
  } catch (error) {
    setConnection(`Offline | ${error.message}`, "offline");
  } finally {
    state.polling = false;
  }
};

const postArmCommand = async (command) => {
  const buttons = document.querySelectorAll("[data-arm-command], #statusButton, #estopTopButton");
  buttons.forEach((button) => {
    button.disabled = true;
  });

  try {
    await api(`/api/arm/${command}`, { method: "POST", body: "{}" });
    await poll();
  } catch (error) {
    setConnection(error.message, "offline");
  } finally {
    buttons.forEach((button) => {
      button.disabled = false;
    });
  }
};

const startMission = async (event) => {
  event.preventDefault();
  const payload = {
    mission_id: $("missionId").value.trim(),
    pickup_location: $("pickupLocation").value,
    delivery_location: $("deliveryLocation").value,
    target_floor: Number($("targetFloor").value),
    object_label: $("objectLabel").value.trim(),
  };

  try {
    await api("/api/mission/start", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    await poll();
  } catch (error) {
    setConnection(error.message, "offline");
  }
};

const cancelMission = async () => {
  try {
    await api("/api/mission/cancel", { method: "POST", body: "{}" });
    await poll();
  } catch (error) {
    setConnection(error.message, "offline");
  }
};

document.addEventListener("DOMContentLoaded", () => {
  $("missionForm").addEventListener("submit", startMission);
  $("cancelMissionButton").addEventListener("click", cancelMission);
  $("statusButton").addEventListener("click", () => postArmCommand("status"));
  $("estopTopButton").addEventListener("click", () => postArmCommand("estop"));

  document.querySelectorAll("[data-arm-command]").forEach((button) => {
    button.addEventListener("click", () => {
      postArmCommand(button.dataset.armCommand);
    });
  });

  poll();
  setInterval(poll, 700);
});
