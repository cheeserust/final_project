const state = {
  configApplied: false,
  polling: false,
  manualConfigs: {},
  manualDirty: { arm: false, gripper: false },
  manualInitialized: { arm: false, gripper: false },
  manualSending: { arm: false, gripper: false },
  lastJointState: null,
};

const elevatorFsmStates = [
  "PICK_OBJECT_AT_START",
  "GO_TO_ELEVATOR_FRONT_4F",
  "ALIGN_ELEVATOR_TAG_4F",
  "PRESS_ELEVATOR_CALL_BUTTON_4F",
  "WAIT_ELEVATOR_OPEN_4F",
  "ENTER_ELEVATOR_4F",
  "ALIGN_INSIDE_ELEVATOR_TAG_TO_5F",
  "PRESS_5F_BUTTON",
  "WAIT_5F",
  "SWITCH_5F_MAP",
  "EXIT_ELEVATOR_5F",
  "GO_TO_DELIVERY_LOCATION",
  "PLACE_OBJECT_AT_DELIVERY",
  "RETURN_TO_ELEVATOR_5F",
  "ALIGN_ELEVATOR_TAG_5F",
  "PRESS_ELEVATOR_CALL_BUTTON_5F",
  "WAIT_ELEVATOR_OPEN_5F",
  "ENTER_ELEVATOR_5F",
  "ALIGN_INSIDE_ELEVATOR_TAG_TO_4F",
  "PRESS_4F_BUTTON",
  "WAIT_4F",
  "SWITCH_4F_MAP",
  "EXIT_ELEVATOR_4F",
  "RETURN_TO_START",
  "DONE",
];

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

const formatDuration = (seconds) => {
  if (seconds === null || seconds === undefined) return "-";
  if (seconds < 60) return `${Math.round(seconds)} s`;
  const minutes = Math.floor(seconds / 60);
  const rest = Math.round(seconds % 60);
  return `${minutes} m ${rest} s`;
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

const connectionClass = (stateName) => {
  if (stateName === "ONLINE") return "success";
  if (stateName === "RECOVERED") return "warning";
  if (stateName === "LOST") return "danger";
  return "neutral";
};

const setOverviewCard = (cardId, className) => {
  const card = $(cardId);
  if (card) {
    card.className = `overview-card ${className}`;
  }
};

const manualDom = {
  arm: {
    grid: "armManualControls",
    duration: "manualArmDuration",
    send: "sendArmManualButton",
    meta: "manualArmMeta",
  },
  gripper: {
    grid: "gripperManualControls",
    duration: "manualGripperDuration",
    send: "sendGripperManualButton",
    meta: "manualGripperMeta",
  },
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
        { name: defaults.pickup_location || "object" },
        { name: defaults.delivery_location || "object_place" },
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
  pickup.value = defaults.pickup_location || "object";
  delivery.value = defaults.delivery_location || "object_place";

  renderFlow(config.mission_steps || []);
  renderManualControls(config.manual || {});
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

const renderManualControls = (manualConfig) => {
  state.manualConfigs = manualConfig || {};
  renderManualControllerControls("arm");
  renderManualControllerControls("gripper");

  const armConfig = state.manualConfigs.arm || {};
  const gripperConfig = state.manualConfigs.gripper || {};
  $("manualArmDuration").value = armConfig.default_duration_sec ?? 2.0;
  $("manualGripperDuration").value = gripperConfig.default_duration_sec ?? 1.0;
  $("manualGripperLoad").value = gripperConfig.default_target_load_raw ?? 500;
};

const renderManualControllerControls = (controller) => {
  const config = state.manualConfigs[controller];
  const dom = manualDom[controller];
  const grid = $(dom.grid);
  const joints = Array.isArray(config?.joints) ? config.joints : [];

  if (!joints.length) {
    grid.innerHTML = `<div class="empty">No manual joints</div>`;
    return;
  }

  grid.innerHTML = joints
    .map((joint) => {
      const value = Number(joint.default_deg ?? 0).toFixed(1);
      return `
        <div class="manual-control-row">
          <div class="manual-control-label">
            <strong>${escapeHtml(joint.label)}</strong>
            <span>${escapeHtml(joint.joint_name)}</span>
          </div>
          <input
            id="manual-${escapeHtml(controller)}-${escapeHtml(joint.key)}-range"
            type="range"
            min="${escapeHtml(joint.min_deg)}"
            max="${escapeHtml(joint.max_deg)}"
            step="0.1"
            value="${escapeHtml(value)}"
          >
          <input
            id="manual-${escapeHtml(controller)}-${escapeHtml(joint.key)}-number"
            class="manual-number"
            type="number"
            min="${escapeHtml(joint.min_deg)}"
            max="${escapeHtml(joint.max_deg)}"
            step="0.1"
            value="${escapeHtml(value)}"
          >
        </div>
      `;
    })
    .join("");

  joints.forEach((joint) => {
    const range = $(`manual-${controller}-${joint.key}-range`);
    const number = $(`manual-${controller}-${joint.key}-number`);
    const sync = (source) => {
      const safe = clampManualValue(source.value, joint);
      const formatted = safe.toFixed(1);
      range.value = formatted;
      number.value = formatted;
      state.manualDirty[controller] = true;
    };

    range.addEventListener("input", () => sync(range));
    number.addEventListener("change", () => sync(number));
  });
};

const clampManualValue = (value, joint) => {
  const fallback = Number(joint.default_deg ?? 0);
  const numeric = Number(value);
  const min = Number(joint.min_deg);
  const max = Number(joint.max_deg);
  return clamp(Number.isFinite(numeric) ? numeric : fallback, min, max);
};

const setManualControlValue = (controller, joint, valueDeg) => {
  const range = $(`manual-${controller}-${joint.key}-range`);
  const number = $(`manual-${controller}-${joint.key}-number`);
  if (!range || !number) return;

  const safe = clampManualValue(valueDeg, joint);
  const formatted = safe.toFixed(1);
  range.value = formatted;
  number.value = formatted;
};

const applyManualCurrent = (controller, requireAll) => {
  const config = state.manualConfigs[controller];
  const joints = Array.isArray(config?.joints) ? config.joints : [];
  const jointStateRows = state.lastJointState?.joints || [];
  if (!joints.length || !jointStateRows.length) return false;

  const byName = new Map();
  jointStateRows.forEach((joint) => {
    if (Number.isFinite(Number(joint.position))) {
      byName.set(joint.name, Number(joint.position));
    }
  });

  if (requireAll && joints.some((joint) => !byName.has(joint.joint_name))) {
    return false;
  }

  let applied = 0;
  joints.forEach((joint) => {
    if (!byName.has(joint.joint_name)) return;
    setManualControlValue(controller, joint, (byName.get(joint.joint_name) * 180) / Math.PI);
    applied += 1;
  });

  return applied > 0;
};

const syncManualDefaultsFromJoints = () => {
  ["arm", "gripper"].forEach((controller) => {
    if (state.manualInitialized[controller] || state.manualDirty[controller]) {
      return;
    }
    if (applyManualCurrent(controller, true)) {
      state.manualInitialized[controller] = true;
    }
  });
};

const newestManualCommand = (commands) => {
  const rows = Object.values(commands || {}).filter(Boolean);
  if (!rows.length) return null;

  return rows.sort((left, right) => {
    const leftTime = Date.parse(left.finished_at || left.accepted_at || left.sent_at || "");
    const rightTime = Date.parse(right.finished_at || right.accepted_at || right.sent_at || "");
    return (Number.isFinite(rightTime) ? rightTime : 0) - (Number.isFinite(leftTime) ? leftTime : 0);
  })[0];
};

const renderManualStatus = (manual) => {
  const controllers = manual?.controllers || {};
  const missionActive = Boolean(manual?.mission_active);
  const arm = controllers.arm || {};
  const gripper = controllers.gripper || {};
  const armReady = Boolean(arm.ready);
  const gripperReady = Boolean(gripper.ready);
  const anyActive = Boolean(arm.active || gripper.active);
  const allReady = armReady && gripperReady;

  $("manualArmMeta").textContent = `${armReady ? "Ready" : "Offline"} | ${arm.active ? "Moving" : "Idle"}`;
  $("manualGripperMeta").textContent = `${gripperReady ? "Ready" : "Offline"} | ${gripper.active ? "Moving" : "Idle"}`;

  $("sendArmManualButton").disabled =
    missionActive || !armReady || arm.active || state.manualSending.arm;
  $("sendGripperManualButton").disabled =
    missionActive || !gripperReady || gripper.active || state.manualSending.gripper;

  if (missionActive) {
    $("manualReady").textContent = "Mission";
    $("manualReady").className = "status-chip warning";
  } else if (anyActive) {
    $("manualReady").textContent = "Moving";
    $("manualReady").className = "status-chip warning";
  } else if (allReady) {
    $("manualReady").textContent = "Ready";
    $("manualReady").className = "status-chip success";
  } else {
    $("manualReady").textContent = "Offline";
    $("manualReady").className = "status-chip danger";
  }

  const last = newestManualCommand(manual?.last_commands);
  if (!last) {
    $("manualStatus").textContent = missionActive
      ? "Mission active | manual send disabled"
      : "No manual command";
    return;
  }

  const result = last.result?.error_string || "";
  $("manualStatus").textContent =
    `${last.controller || "manual"} ${last.state || "-"} | ${formatDuration(last.duration_sec)}${result ? ` | ${result}` : ""}`;
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
  $("overviewMissionState").textContent = stateText;

  const progress = clamp(
    Number(status?.progress ?? feedback?.progress ?? (result?.success ? 1 : 0)),
    0,
    1,
  );
  $("missionProgressText").textContent = `${Math.round(progress * 100)}%`;
  $("missionProgressBar").style.width = `${progress * 100}%`;
  $("overviewMissionMeta").textContent = `Progress ${Math.round(progress * 100)}% | age ${formatAge(mission.status_age_ms)}`;

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

  const missionHealth = status?.error || result?.success === false
    ? "danger"
    : mission.active
      ? "warning"
      : result?.success
        ? "success"
        : "neutral";
  setOverviewCard("overviewMissionCard", missionHealth);
  setOverviewCard("overviewTaskCard", mission.active ? "warning" : "neutral");
  $("overviewActiveTask").textContent =
    status?.active_task || feedback?.current_task || "-";
  $("overviewTaskMeta").textContent =
    status?.message || feedback?.detail || result?.message || "No active task";
};

const renderRobotConnection = (connection) => {
  const stateName = connection?.state || "WAITING";
  const chipClass = connectionClass(stateName);
  $("robotLinkState").textContent = stateName;
  $("robotLinkState").className = `status-chip ${chipClass}`;
  setOverviewCard("overviewConnectionCard", chipClass);
  setOverviewCard(
    "overviewRecoveryCard",
    stateName === "RECOVERED" ? "warning" : stateName === "LOST" ? "danger" : "neutral",
  );

  const heartbeat = connection?.heartbeat || {};
  const lastState =
    heartbeat.mission_state ||
    connection?.events_during_disconnect?.at(-1)?.state ||
    "-";
  const lastTask =
    heartbeat.active_task ||
    connection?.events_during_disconnect?.at(-1)?.active_task ||
    "-";
  const progressValue = Number(heartbeat.progress);
  const progressText = Number.isFinite(progressValue)
    ? `${Math.round(clamp(progressValue, 0, 1) * 100)}%`
    : "-";

  $("robotLastSeen").textContent = formatTime(connection?.last_seen);
  $("robotHeartbeatAge").textContent = formatAge(connection?.heartbeat_age_ms);
  $("robotLastMissionState").textContent = lastState;
  $("robotLastTask").textContent = lastTask || "-";
  $("robotLastProgress").textContent = progressText;
  $("robotDisconnectedDuration").textContent = formatDuration(
    connection?.disconnected_duration_s,
  );
  $("overviewConnectionState").textContent = stateName;
  $("overviewConnectionMeta").textContent =
    `Last seen ${formatTime(connection?.last_seen)} | age ${formatAge(connection?.heartbeat_age_ms)}`;
  $("overviewRecoveryState").textContent =
    stateName === "LOST"
      ? "Disconnected"
      : stateName === "RECOVERED"
        ? "Recovered"
        : connection?.disconnected_duration_s
          ? "Recovered"
          : "No Gap";
  $("overviewRecoveryMeta").textContent =
    connection?.disconnected_duration_s
      ? `Gap ${formatDuration(connection.disconnected_duration_s)} | events ${(connection.events_during_disconnect || []).length}`
      : "No disconnect window";

  if (stateName === "LOST") {
    $("robotLinkDetail").textContent =
      `Last seen ${formatTime(connection?.last_seen)} | timeout ${connection?.timeout_s ?? "-"} s`;
    setConnection(
      `Robot LOST | last seen ${formatTime(connection?.last_seen)}`,
      "offline",
    );
  } else if (stateName === "RECOVERED") {
    $("robotLinkDetail").textContent =
      `Recovered ${formatTime(connection?.recovered_at)} | gap ${formatDuration(connection?.disconnected_duration_s)}`;
    setConnection(
      `Robot RECOVERED | gap ${formatDuration(connection?.disconnected_duration_s)}`,
      "recovered",
    );
  } else if (stateName === "ONLINE") {
    $("robotLinkDetail").textContent =
      `Online via ${connection?.source || "heartbeat"} | age ${formatAge(connection?.heartbeat_age_ms)}`;
    setConnection("Robot ONLINE", "online");
  } else {
    $("robotLinkDetail").textContent = "Waiting for robot heartbeat";
    setConnection("Waiting for robot", "offline");
  }

  const events = connection?.events_during_disconnect || [];
  $("disconnectEvents").innerHTML = events.length
    ? events
        .slice()
        .reverse()
        .map(
          (event) => `
            <li class="${escapeHtml(event.level || "info")}">
              <span>${escapeHtml(formatTime(event.time))}</span>
              <strong>${escapeHtml(event.state || "-")}</strong>
              <em>${escapeHtml(event.message || "")}</em>
            </li>
          `,
        )
        .join("")
    : `<li class="empty">No recovered events</li>`;
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

  const missionEvents = (snapshot.mission_events || []).slice(-60).reverse();
  $("missionEventLog").innerHTML = missionEvents.length
    ? missionEvents
        .map(
          (event) => `
            <li>
              <span class="log-time">${escapeHtml(formatTime(event.time))} | ${escapeHtml(event.state || "-")} | ${escapeHtml(event.level || "info")}</span>
              <span class="log-message">${escapeHtml(event.message || "")}</span>
            </li>
          `,
        )
        .join("")
    : `<li class="empty">No mission events</li>`;

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
  state.lastJointState = snapshot.joints.state;
  syncManualDefaultsFromJoints();
  renderRobotConnection(snapshot.robot_connection);
  renderMission(snapshot.mission);
  renderManualStatus(snapshot.manual);
  renderBoards(snapshot.arm);
  renderJoints(snapshot.joints);
  renderLogs(snapshot);
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

const readManualPositions = (controller) => {
  const config = state.manualConfigs[controller];
  const joints = Array.isArray(config?.joints) ? config.joints : [];
  const positions = {};

  joints.forEach((joint) => {
    const number = $(`manual-${controller}-${joint.key}-number`);
    const value = clampManualValue(number?.value, joint);
    setManualControlValue(controller, joint, value);
    positions[joint.key] = value;
  });

  return positions;
};

const sendManual = async (controller) => {
  const dom = manualDom[controller];
  const payload = {
    positions_deg: readManualPositions(controller),
    duration_sec: Number($(dom.duration).value),
  };
  if (controller === "gripper") {
    payload.target_load_raw = Number($("manualGripperLoad").value);
  }

  state.manualSending[controller] = true;
  $(dom.send).disabled = true;

  try {
    await api(`/api/manual/${controller}`, {
      method: "POST",
      body: JSON.stringify(payload),
    });
    state.manualDirty[controller] = false;
    await poll();
  } catch (error) {
    $("manualStatus").textContent = error.message;
    setConnection(error.message, "offline");
  } finally {
    state.manualSending[controller] = false;
    await poll();
  }
};

const useCurrentManual = (controller) => {
  if (applyManualCurrent(controller, true)) {
    state.manualInitialized[controller] = true;
    state.manualDirty[controller] = false;
    $("manualStatus").textContent = `${controller} values loaded from joint state`;
    return;
  }

  $("manualStatus").textContent = `${controller} joint state is not available`;
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
  $("sendArmManualButton").addEventListener("click", () => sendManual("arm"));
  $("sendGripperManualButton").addEventListener("click", () => sendManual("gripper"));
  $("useCurrentArmButton").addEventListener("click", () => useCurrentManual("arm"));
  $("useCurrentGripperButton").addEventListener("click", () => useCurrentManual("gripper"));

  document.querySelectorAll("[data-arm-command]").forEach((button) => {
    button.addEventListener("click", () => {
      postArmCommand(button.dataset.armCommand);
    });
  });

  poll();
  setInterval(poll, 700);
});
