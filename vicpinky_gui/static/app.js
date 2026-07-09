const state = {
  configApplied: false,
  polling: false,
  manualConfigs: {},
  manualDirty: { arm: false, gripper: false },
  manualInitialized: { arm: false, gripper: false },
  manualSending: { arm: false, gripper: false },
  lastJointState: null,
  locations: [],
  directNavLocations: [],
  navSending: false,
  lastSeq: null,
  pendingSync: false,
  syncing: false,
  syncStatus: "LIVE",
  lastSyncAt: null,
  recoveredLogs: [],
  drivingMapRevision: null,
  drivingMap: null,
  drivingMapLoading: false,
};

const elevatorFsmStates = [
  "GO_TO_ELEVATOR_FRONT",
  "ALIGN_ELEVATOR_TAG",
  "PRESS_ELEVATOR_CALL_BUTTON",
  "WAIT_ELEVATOR_OPEN",
  "ENTER_ELEVATOR",
  "PRESS_5F_BUTTON",
  "WAIT_5F",
  "EXIT_ELEVATOR",
  "SWITCH_5F_MAP",
  "GO_TO_TARGET_PLACE",
  "ARM_TASK_AT_TARGET",
  "RETURN_TO_ELEVATOR",
  "ALIGN_ELEVATOR_TAG_RETURN",
  "PRESS_ELEVATOR_CALL_BUTTON_RETURN",
  "WAIT_ELEVATOR_OPEN_RETURN",
  "ENTER_ELEVATOR_RETURN",
  "PRESS_4F_BUTTON",
  "WAIT_4F",
  "EXIT_ELEVATOR_RETURN",
  "SWITCH_4F_MAP",
  "RETURN_HOME",
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

const taskHealthClass = (task = {}) => {
  const taskState = String(task.state || "");
  const result = String(task.button_press_result || "");
  if (task.arm_fault || task.gripper_fault || result.includes("FAILED") || taskState === "FAULT") {
    return "danger";
  }
  if (["PRESSING", "WAITING_TARGET_FLOOR", "EXITING", "ROBOT_STOPPED"].includes(taskState)) {
    return "warning";
  }
  if (["SUCCESS", "ACTION_SUCCESS"].includes(result) || task.target_floor_arrived || task.exit_done) {
    return "success";
  }
  return "neutral";
};

const latestSeqFromSnapshot = (snapshot) => {
  const candidates = [Number(snapshot?.log_sync?.latest_seq)];
  for (const collection of [snapshot?.events, snapshot?.mission_events]) {
    if (!Array.isArray(collection)) continue;
    collection.forEach((entry) => {
      const seq = Number(entry?.seq);
      if (Number.isFinite(seq)) candidates.push(seq);
    });
  }
  return Math.max(0, ...candidates.filter(Number.isFinite));
};

const mergeRecoveredLogs = (logs) => {
  const bySeq = new Map(
    state.recoveredLogs
      .filter((log) => Number.isFinite(Number(log.seq)))
      .map((log) => [Number(log.seq), log]),
  );

  (Array.isArray(logs) ? logs : []).forEach((log) => {
    const seq = Number(log?.seq);
    if (Number.isFinite(seq)) {
      bySeq.set(seq, log);
    }
  });

  state.recoveredLogs = [...bySeq.entries()]
    .sort((left, right) => left[0] - right[0])
    .map((entry) => entry[1])
    .slice(-160);
};

const renderSyncStatus = () => {
  const chip = $("logSyncState");
  if (!chip) return;

  if (state.syncing) {
    chip.textContent = "SYNCING";
    chip.className = "status-chip warning";
  } else if (state.syncStatus === "SYNCED") {
    chip.textContent = "SYNCED";
    chip.className = "status-chip success";
  } else if (state.syncStatus === "FAILED") {
    chip.textContent = "SYNC FAILED";
    chip.className = "status-chip danger";
  } else if (state.syncStatus === "OFFLINE") {
    chip.textContent = "OFFLINE";
    chip.className = "status-chip danger";
  } else {
    chip.textContent = "LIVE";
    chip.className = "status-chip neutral";
  }
};

const renderSyncReplay = () => {
  const node = $("syncEventLog");
  if (!node) return;

  const recovered = state.recoveredLogs.slice(-60).reverse();
  node.innerHTML = recovered.length
    ? recovered
        .map(
          (event) => `
            <li>
              <span class="log-time">${escapeHtml(formatTime(event.time || event.timestamp))} | #${escapeHtml(event.seq ?? "-")} | ${escapeHtml(event.event_type || event.source || "-")}</span>
              <span class="log-message">${escapeHtml(event.message || "")}</span>
            </li>
          `,
        )
        .join("")
    : `<li class="empty">No replayed logs</li>`;
};

const manualDom = {
  arm: {
    section: "manualArmSection",
    grid: "armManualControls",
    duration: "manualArmDuration",
    send: "sendArmManualButton",
    current: "useCurrentArmButton",
    meta: "manualArmMeta",
  },
  gripper: {
    section: "manualGripperSection",
    grid: "gripperManualControls",
    duration: "manualGripperDuration",
    send: "sendGripperManualButton",
    current: "useCurrentGripperButton",
    meta: "manualGripperMeta",
  },
};

const applyConfig = (config) => {
  if (state.configApplied || !config) return;

  const locations = Array.isArray(config.locations) ? config.locations : [];
  const directNavLocations = Array.isArray(config.direct_nav_locations)
    ? config.direct_nav_locations
    : [];
  const missionLocations = Array.isArray(config.mission_locations)
    ? config.mission_locations
    : [];
  const defaults = config.default_goal || {};
  const navDefaults = config.default_nav || {};
  const pickup = $("pickupLocation");
  const delivery = $("deliveryLocation");
  state.locations = locations;
  state.directNavLocations = directNavLocations;

  const options = missionLocations.length
    ? missionLocations
    : [
        { name: defaults.pickup_location || "home" },
        { name: defaults.delivery_location || "object_place" },
      ];

  for (const select of [pickup, delivery]) {
    select.innerHTML = options
      .map((location) => {
        const label = location.label || location.name;
        return `<option value="${escapeHtml(location.name)}">${escapeHtml(label)}</option>`;
      })
      .join("");
  }

  $("missionId").value = defaults.mission_id || "";
  $("objectLabel").value = defaults.object_label || "box";
  $("targetFloor").value = defaults.target_floor ?? 5;
  pickup.value = defaults.pickup_location || "home";
  delivery.value = defaults.delivery_location || "object_place";
  renderNavFloorOptions(navDefaults.target_floor ?? 4);
  renderNavLocationOptions(
    navDefaults.location_id || navDefaults.location_name || "4:home",
  );

  renderFlow(config.mission_steps || []);
  renderManualControls(config.manual || {});
  state.configApplied = true;
};

const numericFloors = (locations) => {
  const floors = locations
    .map((location) => Number(location.floor))
    .filter((floor) => Number.isInteger(floor));
  return [...new Set(floors)].sort((left, right) => left - right);
};

const renderNavFloorOptions = (preferredFloor) => {
  const floors = numericFloors(state.directNavLocations);
  const options = floors.length ? floors : [4, 5];
  $("navFloor").innerHTML = options
    .map((floor) => `<option value="${escapeHtml(floor)}">${escapeHtml(floor)}F</option>`)
    .join("");
  const safeFloor = options.includes(Number(preferredFloor))
    ? Number(preferredFloor)
    : options[0];
  $("navFloor").value = String(safeFloor);
};

const renderNavLocationOptions = (preferredLocation = "") => {
  const floor = Number($("navFloor").value);
  const candidates = state.directNavLocations.filter((location) => {
    const locationFloor = Number(location.floor);
    return Number.isInteger(locationFloor) && locationFloor === floor;
  });
  const options = candidates.length ? candidates : state.directNavLocations;
  const previous = preferredLocation || $("navLocation").value;

  $("navLocation").innerHTML = options
    .map((location) => {
      const labelParts = [location.name];
      if (location.type) labelParts.push(location.type);
      return `<option value="${escapeHtml(location.id)}">${escapeHtml(labelParts.join(" | "))}</option>`;
    })
    .join("");

  const ids = options.map((location) => location.id);
  if (ids.includes(previous)) {
    $("navLocation").value = previous;
  }
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
  if (state.manualConfigs.arm) {
    $("manualArmDuration").value = armConfig.default_duration_sec ?? 2.0;
  }
  if (state.manualConfigs.gripper) {
    $("manualGripperDuration").value = gripperConfig.default_duration_sec ?? 1.0;
    $("manualGripperLoad").value = gripperConfig.default_target_load_raw ?? 500;
  }
};

const renderManualControllerControls = (controller) => {
  const config = state.manualConfigs[controller];
  const dom = manualDom[controller];
  const configured = Boolean(config);
  $(dom.section).hidden = !configured;
  $(dom.duration).disabled = !configured;
  $(dom.send).disabled = !configured;
  $(dom.current).disabled = !configured;
  if (controller === "gripper") {
    $("manualGripperLoad").disabled = !configured;
  }

  if (!configured) {
    return;
  }

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
  const armConfigured = Boolean(state.manualConfigs.arm);
  const gripperConfigured = Boolean(state.manualConfigs.gripper);
  const armReady = Boolean(arm.ready);
  const gripperReady = Boolean(gripper.ready);
  const anyActive = Boolean(
    (armConfigured && arm.active) ||
    (gripperConfigured && gripper.active)
  );
  const configuredReady = [];
  if (armConfigured) configuredReady.push(armReady);
  if (gripperConfigured) configuredReady.push(gripperReady);
  const allReady = configuredReady.length > 0 && configuredReady.every(Boolean);

  if (armConfigured) {
    $("manualArmMeta").textContent = `${armReady ? "Ready" : "Offline"} | ${arm.active ? "Moving" : "Idle"}`;
  }
  if (gripperConfigured) {
    $("manualGripperMeta").textContent = `${gripperReady ? "Ready" : "Offline"} | ${gripper.active ? "Moving" : "Idle"}`;
  }

  $("sendArmManualButton").disabled =
    !armConfigured || missionActive || !armReady || arm.active || state.manualSending.arm;
  $("sendGripperManualButton").disabled =
    !gripperConfigured || missionActive || !gripperReady || gripper.active || state.manualSending.gripper;

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

const renderElevatorButtonTask = (task = {}) => {
  const health = taskHealthClass(task);
  const taskState = task.state || "IDLE";
  const floorText = task.target_floor ? `${task.target_floor}F` : "-";
  const motionResult = task.button_press_result || "UNKNOWN";
  const resultSource = task.button_press_result_source || "none";
  const physicalResult = task.physical_button_result || "UNKNOWN";
  const armFault = task.arm_fault ? "FAULT" : "OK";
  const gripperFault = task.gripper_fault ? "FAULT" : "OK";

  setOverviewCard("overviewButtonCard", health);
  $("overviewButtonState").textContent = taskState;
  $("overviewButtonMeta").textContent =
    `${floorText} | motion ${motionResult} | physical ${physicalResult}`;

  $("buttonTaskChip").textContent = taskState;
  $("buttonTaskChip").className = `status-chip ${health === "neutral" ? "neutral" : health}`;
  $("buttonTaskMeta").textContent =
    task.last_message || task.last_event || "Waiting for mission events";
  $("buttonTaskState").textContent = taskState;
  $("buttonTaskTargetFloor").textContent = floorText;
  $("buttonTaskMotionResult").textContent = `${motionResult} | ${resultSource}`;
  $("buttonTaskPhysicalResult").textContent = physicalResult;
  $("buttonTaskStarted").textContent = formatTime(task.button_press_started_at);
  $("buttonTaskLastEvent").textContent =
    task.last_event
      ? `${task.last_event} | ${formatTime(task.last_event_time)}`
      : "-";
  $("buttonTaskArmFault").textContent = armFault;
  $("buttonTaskGripperFault").textContent = gripperFault;
};

const renderMission = (mission, directNav) => {
  const status = mission.status;
  const feedback = mission.feedback;
  const result = mission.result;
  const goal = mission.goal;
  const directNavActive = Boolean(directNav?.active);

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

  $("startMissionButton").disabled =
    !mission.action_ready || mission.active || directNavActive;
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

const renderDirectNav = (directNav, mission) => {
  const ready = Boolean(directNav?.action_ready);
  const active = Boolean(directNav?.active);
  const missionActive = Boolean(mission?.active);
  const goal = directNav?.goal;
  const feedback = directNav?.feedback;
  const result = directNav?.result;

  $("navReady").textContent = ready ? "Ready" : "Offline";
  $("navReady").className = `status-chip ${ready ? "success" : "danger"}`;

  const stateText =
    goal?.state ||
    result?.status ||
    (active ? "ACTIVE" : "IDLE");
  $("navState").textContent =
    `${stateText} | ${active ? "moving" : "idle"}`;

  const targetText = goal
    ? `${goal.location_name} -> ${goal.target_name} (${goal.target_floor}F)`
    : "-";
  $("navTarget").textContent = targetText;

  const progress = Number(feedback?.progress);
  $("navFeedback").textContent = feedback
    ? `${feedback.phase || "-"} | ${Number.isFinite(progress) ? `${Math.round(progress * 100)}%` : "-"} | ${feedback.detail || ""}`
    : "-";

  $("navResult").textContent = result
    ? `${result.status}: ${result.message}`
    : "-";

  const hasLocation = Boolean($("navLocation").value);
  $("startNavButton").disabled =
    !ready || active || missionActive || state.navSending || !hasLocation;
  $("cancelNavButton").disabled = !active;
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

const drivingChipClass = (stateName) => {
  if (stateName === "NAVIGATING") return "success";
  if (stateName === "MAP_SWITCHING" || stateName === "WAITING_ELEVATOR") return "warning";
  if (stateName === "FAILED" || stateName === "ERROR") return "danger";
  return "neutral";
};

const resizeDrivingCanvas = (canvas) => {
  const wrapper = canvas.parentElement;
  const cssWidth = Math.max(320, Math.floor(wrapper?.clientWidth || 960));
  const cssHeight = Math.max(260, Math.min(620, Math.round(cssWidth * 0.58)));
  const ratio = window.devicePixelRatio || 1;

  canvas.style.width = `${cssWidth}px`;
  canvas.style.height = `${cssHeight}px`;

  const width = Math.floor(cssWidth * ratio);
  const height = Math.floor(cssHeight * ratio);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return { width, height, ratio };
};

const drawEmptyDrivingMap = (message = "Waiting for /map") => {
  const canvas = $("drivingMapCanvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const { width, height } = resizeDrivingCanvas(canvas);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#f9fbfd";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#d7dee9";
  ctx.lineWidth = Math.max(1, width / 960);
  ctx.strokeRect(0, 0, width, height);
  ctx.fillStyle = "#687386";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = `${Math.max(13, Math.round(width / 64))}px sans-serif`;
  ctx.fillText(message, width / 2, height / 2);
};

const DRIVING_MAP_ROTATION = 90;

const mapFit = (map, canvasWidth, canvasHeight) => {
  const width = Number(map?.width || 0);
  const height = Number(map?.height || 0);
  if (!width || !height) return null;
  const rotated = DRIVING_MAP_ROTATION === 90 || DRIVING_MAP_ROTATION === 270;
  const displayWidth = rotated ? height : width;
  const displayHeight = rotated ? width : height;
  const scale = Math.min(canvasWidth / displayWidth, canvasHeight / displayHeight);
  return {
    scale,
    left: (canvasWidth - displayWidth * scale) / 2,
    top: (canvasHeight - displayHeight * scale) / 2,
    width,
    height,
    displayWidth,
    displayHeight,
  };
};

const worldToCanvas = (map, fit, x, y) => {
  const origin = map.origin || {};
  const resolution = Number(map.resolution || 0.05);
  const mx = (Number(x) - Number(origin.x || 0)) / resolution;
  const my = fit.height - (Number(y) - Number(origin.y || 0)) / resolution;
  if (DRIVING_MAP_ROTATION === 90) {
    return {
      x: fit.left + (fit.height - my) * fit.scale,
      y: fit.top + mx * fit.scale,
    };
  }
  return {
    x: fit.left + mx * fit.scale,
    y: fit.top + my * fit.scale,
  };
};

const makeMapCanvas = (map) => {
  const width = Number(map?.width || 0);
  const height = Number(map?.height || 0);
  const data = Array.isArray(map?.data) ? map.data : [];
  if (!width || !height || data.length < width * height) return null;

  const offscreen = document.createElement("canvas");
  offscreen.width = width;
  offscreen.height = height;
  const ctx = offscreen.getContext("2d");
  const image = ctx.createImageData(width, height);

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const sourceIndex = x + y * width;
      const targetY = height - 1 - y;
      const targetIndex = (x + targetY * width) * 4;
      const occ = Number(data[sourceIndex]);
      let shade = 232;
      if (occ === -1) shade = 218;
      else if (occ >= 65) shade = 35;
      else if (occ > 0) shade = Math.max(70, 245 - occ * 2);
      else shade = 252;

      image.data[targetIndex] = shade;
      image.data[targetIndex + 1] = shade;
      image.data[targetIndex + 2] = shade;
      image.data[targetIndex + 3] = 255;
    }
  }

  ctx.putImageData(image, 0, 0);
  return offscreen;
};

const drawPath = (ctx, map, fit, path, color, lineWidth) => {
  const poses = Array.isArray(path?.poses) ? path.poses : [];
  if (poses.length < 2) return;

  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.beginPath();
  poses.forEach((pose, index) => {
    const point = worldToCanvas(map, fit, pose.x, pose.y);
    if (index === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.stroke();
  ctx.restore();
};

const drawPose = (ctx, map, fit, pose, canvasWidth) => {
  if (!pose?.available) return;
  const point = worldToCanvas(map, fit, pose.x, pose.y);
  const yaw = Number(pose.yaw || 0);
  const yawRotation = DRIVING_MAP_ROTATION === 90 ? Math.PI / 2 : 0;
  const radius = Math.max(7, canvasWidth / 90);
  const arrow = radius * 2.2;

  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(-yaw + yawRotation);
  ctx.fillStyle = "#e63946";
  ctx.strokeStyle = "#8b1e27";
  ctx.lineWidth = Math.max(1.5, canvasWidth / 620);
  ctx.beginPath();
  ctx.arc(0, 0, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(arrow, 0);
  ctx.lineTo(arrow - radius * 0.55, -radius * 0.45);
  ctx.moveTo(arrow, 0);
  ctx.lineTo(arrow - radius * 0.55, radius * 0.45);
  ctx.stroke();
  ctx.restore();
};

const drawDrivingMap = (driving = {}) => {
  const canvas = $("drivingMapCanvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const { width, height } = resizeDrivingCanvas(canvas);
  const map = state.drivingMap;

  if (!map) {
    drawEmptyDrivingMap(driving?.map?.available ? "Loading map data..." : "Waiting for /map");
    return;
  }

  const offscreen = makeMapCanvas(map);
  const fit = mapFit(map, width, height);
  if (!offscreen || !fit) {
    drawEmptyDrivingMap("Invalid map data");
    return;
  }

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#f0f3f8";
  ctx.fillRect(0, 0, width, height);
  ctx.imageSmoothingEnabled = false;
  ctx.save();
  if (DRIVING_MAP_ROTATION === 90) {
    ctx.translate(fit.left + fit.displayWidth * fit.scale, fit.top);
    ctx.rotate(Math.PI / 2);
    ctx.drawImage(offscreen, 0, 0, fit.width * fit.scale, fit.height * fit.scale);
  } else {
    ctx.drawImage(
      offscreen,
      fit.left,
      fit.top,
      fit.width * fit.scale,
      fit.height * fit.scale,
    );
  }
  ctx.restore();
  ctx.strokeStyle = "#18202f";
  ctx.lineWidth = Math.max(1, width / 900);
  ctx.strokeRect(
    fit.left,
    fit.top,
    fit.displayWidth * fit.scale,
    fit.displayHeight * fit.scale,
  );

  drawPath(ctx, map, fit, driving.global_path, "#006d77", Math.max(2, width / 360));
  drawPath(ctx, map, fit, driving.local_path, "#f0b429", Math.max(2, width / 420));
  drawPose(ctx, map, fit, driving.pose, width);
};

const loadDrivingMapIfNeeded = async (mapMeta = {}) => {
  if (!mapMeta.available || !mapMeta.data_url) {
    state.drivingMap = null;
    state.drivingMapRevision = null;
    drawDrivingMap({ map: mapMeta });
    return;
  }

  if (state.drivingMapRevision === mapMeta.revision && state.drivingMap) {
    return;
  }
  if (state.drivingMapLoading) return;

  state.drivingMapLoading = true;
  try {
    const payload = await api(mapMeta.data_url);
    state.drivingMap = payload.map || null;
    state.drivingMapRevision = state.drivingMap?.revision ?? null;
  } catch (error) {
    $("drivingMapMeta").textContent = `Map fetch failed: ${error.message}`;
  } finally {
    state.drivingMapLoading = false;
  }
};

const renderDriving = (driving = {}) => {
  const map = driving.map || {};
  const pose = driving.pose || {};
  const odom = driving.odom || {};
  const navState = driving.state || {};
  const globalPath = driving.global_path || {};
  const localPath = driving.local_path || {};

  const label = navState.label || navState.state || "Waiting";
  $("drivingStateChip").textContent = label;
  $("drivingStateChip").className = `status-chip ${drivingChipClass(navState.state)}`;

  if (map.available) {
    $("drivingMapMeta").textContent = `${map.topic || "/map"} | rev ${map.revision} | age ${formatAge(map.age_ms)}`;
    $("drivingMapInfo").textContent = `${map.width}x${map.height}, res ${formatNumber(map.resolution, 3)} m/px, origin (${formatNumber(map.origin?.x, 2)}, ${formatNumber(map.origin?.y, 2)})`;
  } else {
    $("drivingMapMeta").textContent = `Waiting for ${map.topic || "/map"}`;
    $("drivingMapInfo").textContent = "No map";
  }

  $("drivingPose").textContent = pose.available
    ? `x ${formatNumber(pose.x, 3)}, y ${formatNumber(pose.y, 3)}, yaw ${formatNumber(pose.yaw_deg, 1)}° | age ${formatAge(pose.age_ms)}`
    : `Waiting for ${pose.topic || "/amcl_pose"}`;

  $("drivingOdom").textContent = odom.available
    ? `v ${formatNumber(odom.linear_x, 3)} m/s, w ${formatNumber(odom.angular_z, 3)} rad/s | age ${formatAge(odom.age_ms)}`
    : `Waiting for ${odom.topic || "/odom"}`;

  $("drivingPathInfo").textContent = `global ${globalPath.available ? `${globalPath.count} pts` : "-"}, local ${localPath.available ? `${localPath.count} pts` : "-"}`;
  $("drivingNavState").textContent = `${navState.state || "IDLE"}${navState.mission_state ? ` | mission ${navState.mission_state}` : ""}${navState.detail ? ` | ${navState.detail}` : ""}`;

  loadDrivingMapIfNeeded(map).then(() => drawDrivingMap(driving));
  drawDrivingMap(driving);
};

const renderLogs = (snapshot) => {
  $("serverTime").textContent = formatTime(snapshot.server.time);
  renderSyncStatus();

  const events = (snapshot.events || []).slice(-60).reverse();
  $("eventLog").innerHTML = events.length
    ? events
        .map(
          (event) => `
            <li>
              <span class="log-time">${escapeHtml(formatTime(event.time))} | #${escapeHtml(event.seq ?? "-")} | ${escapeHtml(event.event_type || event.kind)} | ${escapeHtml(event.level)}</span>
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
              <span class="log-time">${escapeHtml(formatTime(event.time))} | #${escapeHtml(event.seq ?? "-")} | ${escapeHtml(event.event_type || event.state || "-")} | ${escapeHtml(event.level || "info")}</span>
              <span class="log-message">${escapeHtml(event.message || "")}</span>
            </li>
          `,
        )
        .join("")
    : `<li class="empty">No mission events</li>`;

  renderSyncReplay();

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
  renderMission(snapshot.mission, snapshot.direct_nav);
  renderElevatorButtonTask(snapshot.elevator_button_task || {});
  renderDirectNav(snapshot.direct_nav, snapshot.mission);
  renderDriving(snapshot.driving || {});
  renderManualStatus(snapshot.manual);
  renderBoards(snapshot.arm);
  renderJoints(snapshot.joints);
  renderLogs(snapshot);
};

const syncMissingLogs = async (afterSeq) => {
  if (!Number.isFinite(Number(afterSeq)) || Number(afterSeq) <= 0) {
    return;
  }

  state.syncing = true;
  state.syncStatus = "SYNCING";
  renderSyncStatus();
  setConnection("SYNCING | replaying missed event logs", "syncing");

  try {
    const data = await api(`/api/logs?after_seq=${encodeURIComponent(afterSeq)}`);
    const logs = Array.isArray(data.logs) ? data.logs : [];
    mergeRecoveredLogs(logs);
    renderSyncReplay();
    const newestSeq = Math.max(
      Number(data.latest_seq || 0),
      ...logs.map((log) => Number(log.seq || 0)).filter(Number.isFinite),
    );
    if (newestSeq > 0) {
      state.lastSeq = Math.max(Number(state.lastSeq || 0), newestSeq);
    }
    state.pendingSync = false;
    state.syncStatus = "SYNCED";
    state.lastSyncAt = new Date().toISOString();
    setConnection(`SYNCED | replayed ${logs.length} logs`, "online");
  } catch (error) {
    state.pendingSync = true;
    state.syncStatus = "FAILED";
    setConnection(`SYNC FAILED | ${error.message}`, "recovered");
  } finally {
    state.syncing = false;
    renderSyncStatus();
  }
};

const poll = async () => {
  if (state.polling) return;
  state.polling = true;
  try {
    const previousSeq = Number(state.lastSeq || 0);
    const shouldTrySync = state.pendingSync && previousSeq > 0;
    const snapshot = await api("/api/snapshot");
    renderSnapshot(snapshot);
    const latestSeq = latestSeqFromSnapshot(snapshot);

    if (shouldTrySync && latestSeq > previousSeq) {
      await syncMissingLogs(previousSeq);
    } else if (latestSeq > 0) {
      state.lastSeq = Math.max(previousSeq, latestSeq);
      if (state.pendingSync && latestSeq <= previousSeq) {
        state.pendingSync = false;
        state.syncStatus = "SYNCED";
        state.lastSyncAt = new Date().toISOString();
      } else if (!state.pendingSync && state.syncStatus !== "SYNCED") {
        state.syncStatus = "LIVE";
      }
      renderSyncStatus();
    }
  } catch (error) {
    state.pendingSync = state.lastSeq !== null;
    state.syncStatus = "OFFLINE";
    setConnection(
      `OFFLINE | browser cannot reach central Flask snapshot: ${error.message}`,
      "offline",
    );
    renderSyncStatus();
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
  if (!config) return {};

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
  if (!state.manualConfigs[controller]) return;

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
  if (!state.manualConfigs[controller]) return;

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

const startDirectNav = async (event) => {
  event.preventDefault();
  const selected = state.directNavLocations.find(
    (location) => location.id === $("navLocation").value,
  );
  const payload = {
    location_id: $("navLocation").value,
    location_name: selected?.name || "",
    target_floor: Number($("navFloor").value),
  };

  state.navSending = true;
  $("startNavButton").disabled = true;

  try {
    await api("/api/nav/go-to", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    await poll();
  } catch (error) {
    $("navState").textContent = error.message;
    setConnection(error.message, "offline");
  } finally {
    state.navSending = false;
    await poll();
  }
};

const cancelDirectNav = async () => {
  try {
    await api("/api/nav/cancel", { method: "POST", body: "{}" });
    await poll();
  } catch (error) {
    $("navState").textContent = error.message;
    setConnection(error.message, "offline");
  }
};

window.addEventListener("resize", () => drawDrivingMap({}));

document.addEventListener("DOMContentLoaded", () => {
  $("missionForm").addEventListener("submit", startMission);
  $("cancelMissionButton").addEventListener("click", cancelMission);
  $("navForm").addEventListener("submit", startDirectNav);
  $("cancelNavButton").addEventListener("click", cancelDirectNav);
  $("navFloor").addEventListener("change", () => renderNavLocationOptions());
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
