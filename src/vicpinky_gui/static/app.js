const state = {
  configApplied: false,
  polling: false,
  manualConfigs: {},
  manualDirty: { arm: false, gripper: false },
  manualInitialized: { arm: false, gripper: false },
  manualSending: { arm: false, gripper: false },
  manualNotice: "",
  manualNoticeUntil: 0,
  savedPoses: [],
  nextSavedPoseId: 1,
  savedPosesLoading: false,
  savedPoseMutationPending: false,
  sequence: {},
  sequenceRequestError: "",
  sequenceOperationPending: false,
  lastJointState: null,
  locations: [],
  directNavLocations: [],
  navSending: false,
  lastSeq: null,
  pendingSync: false,
  syncing: false,
  syncStatus: "LIVE",
  lastSyncAt: null,
  snapshotFailures: 0,
  recoveredLogs: [],
  drivingMapRevision: null,
  drivingMap: null,
  drivingMapLoading: false,
  latestDriving: {},
  initialPoseMode: false,
  initialPoseDraft: null,
  initialPoseSending: false,
};

const elevatorFsmStates = [
  "ARM_HOMING",
  "ARM_READY_AT_PICKUP",
  "PICK_OBJECT_TO_TRAY",
  "GO_TO_ELEVATOR_FRONT",
  "ALIGN_ELEVATOR_TAG",
  "PRESS_ELEVATOR_CALL_BUTTON",
  "READY_AND_APPROACH_ELEVATOR_4F",
  "FACE_ELEVATOR_4F",
  "WAIT_ELEVATOR_OPEN",
  "ENTER_ELEVATOR",
  "PRESS_5F_BUTTON",
  "WAIT_5F",
  "EXIT_ELEVATOR",
  "SWITCH_5F_MAP",
  "GO_TO_TARGET_PLACE",
  "ROTATE_AT_DELIVERY",
  "DELIVER_OBJECT_FROM_TRAY",
  "RETURN_TO_ELEVATOR",
  "ALIGN_ELEVATOR_TAG_RETURN",
  "PRESS_ELEVATOR_CALL_BUTTON_RETURN",
  "READY_AND_APPROACH_ELEVATOR_5F",
  "FACE_ELEVATOR_5F",
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

const API_DEFAULT_TIMEOUT_MS = 5000;
const SNAPSHOT_TIMEOUT_MS = 2000;
const HOME_COMMAND_TIMEOUT_MS = 190000;
const CLEAR_COMMAND_TIMEOUT_MS = 20000;
const OFFLINE_FAILURE_THRESHOLD = 2;

const apiErrorMessage = (data, status) => {
  const primary = data?.message || (typeof data?.error === "string" ? data.error : "") || `HTTP ${status}`;
  const details = data?.details
    ?? data?.errors
    ?? data?.offline_controllers
    ?? (typeof data?.error === "object" ? data.error : null);
  if (details === null || details === undefined || details === "") return primary;

  if (Array.isArray(details)) {
    const rendered = details
      .map((detail) => typeof detail === "string" ? detail : JSON.stringify(detail))
      .filter(Boolean)
      .join(", ");
    return rendered ? `${primary} | ${rendered}` : primary;
  }
  if (typeof details === "object") {
    const rendered = Object.entries(details)
      .map(([key, value]) => `${key}: ${typeof value === "string" ? value : JSON.stringify(value)}`)
      .join(", ");
    return rendered ? `${primary} | ${rendered}` : primary;
  }
  return `${primary} | ${details}`;
};

const api = async (path, options = {}) => {
  const { timeoutMs = API_DEFAULT_TIMEOUT_MS, ...fetchOptions } = options;
  const controller = new AbortController();
  const providedSignal = fetchOptions.signal;
  let abortListener = null;
  let timeoutId = null;

  if (providedSignal) {
    abortListener = () => controller.abort();
    if (providedSignal.aborted) {
      controller.abort();
    } else {
      providedSignal.addEventListener("abort", abortListener, { once: true });
    }
  }

  if (Number.isFinite(timeoutMs) && timeoutMs > 0) {
    timeoutId = window.setTimeout(() => controller.abort(), timeoutMs);
  }

  try {
    const headers = {
      "Content-Type": "application/json",
      ...(fetchOptions.headers || {}),
    };
    delete fetchOptions.headers;
    delete fetchOptions.signal;

    const response = await fetch(path, {
      headers,
      ...fetchOptions,
      signal: controller.signal,
    });
    const data = await response.json();
    if (!response.ok || data.ok === false) {
      throw new Error(apiErrorMessage(data, response.status));
    }
    return data;
  } catch (error) {
    if (error?.name === "AbortError") {
      throw new Error(`Request timed out after ${(timeoutMs / 1000).toFixed(1)} s`);
    }
    throw error;
  } finally {
    if (timeoutId !== null) window.clearTimeout(timeoutId);
    if (providedSignal && abortListener) {
      providedSignal.removeEventListener("abort", abortListener);
    }
  }
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
  const armTasks = Array.isArray(config.arm_tasks) ? config.arm_tasks : [];
  const navDefaults = config.default_nav || {};
  const pickup = $("pickupLocation");
  const delivery = $("deliveryLocation");
  state.locations = locations;
  state.directNavLocations = directNavLocations;

  const options = missionLocations.length
    ? missionLocations
    : [
        { name: defaults.pickup_location || "402" },
        { name: defaults.delivery_location || "object_place" },
      ];

  const pickupOptions = options.filter((location) =>
    ["402", "402_4f", "room_402"].includes(location.name),
  );
  const deliveryOptions = options.filter((location) => location.name === "object_place");

  for (const [select, selectOptions] of [
    [pickup, pickupOptions.length ? pickupOptions : [{ name: "402", label: "4F 402" }]],
    [delivery, deliveryOptions.length ? deliveryOptions : [{ name: "object_place", label: "5F object_place" }]],
  ]) {
    select.innerHTML = selectOptions
      .map((location) => {
        const label = location.label || location.name;
        return `<option value="${escapeHtml(location.name)}">${escapeHtml(label)}</option>`;
      })
      .join("");
  }

  $("missionId").value = defaults.mission_id || "";
  $("objectLabel").value = defaults.object_label || "object_1";
  $("armTaskName").innerHTML = armTasks
    .map((task) => `<option value="${escapeHtml(task.name)}">${escapeHtml(task.label || task.name)}</option>`)
    .join("");
  $("armTaskName").value = defaults.arm_task_name || "deliver_object_1_from_tray";
  $("targetFloor").value = defaults.target_floor ?? 5;
  pickup.value = defaults.pickup_location || "402";
  delivery.value = defaults.delivery_location || "object_place";
  renderNavFloorOptions(navDefaults.target_floor ?? 4);
  renderNavLocationOptions(
    navDefaults.location_id || navDefaults.location_name || "4:402",
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
  renderSavedPoseTable();
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
      const value = formatAngleInput(joint.default_deg ?? 0);
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
      const formatted = formatAngleInput(safe);
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

const formatAngleInput = (value) => {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) return "0";
  const rounded = Math.round(numeric * 10) / 10;
  return Number.isInteger(rounded) ? String(rounded) : rounded.toFixed(1);
};

const setManualControlValue = (controller, joint, valueDeg) => {
  const range = $(`manual-${controller}-${joint.key}-range`);
  const number = $(`manual-${controller}-${joint.key}-number`);
  if (!range || !number) return;

  const safe = clampManualValue(valueDeg, joint);
  const formatted = formatAngleInput(safe);
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

const sequenceStateValue = (sequence = {}) =>
  String(sequence.state || sequence.status || "IDLE").toUpperCase();

const sequenceIsActive = (sequence = {}) => {
  if (typeof sequence.active === "boolean") return sequence.active;
  const sequenceState = sequenceStateValue(sequence);
  if (["STARTING", "RUNNING", "DWELL", "DWELLING", "WAITING", "STOPPING", "CANCELLING"].includes(sequenceState)) {
    return true;
  }
  if (["IDLE", "COMPLETED", "FAILED", "CANCELLED", "STOPPED", "ERROR"].includes(sequenceState)) {
    return false;
  }
  return Boolean(sequence.run_id && !sequence.finished_at);
};

const sequenceErrorText = (sequence = {}) => {
  const error = sequence.latest_error ?? sequence.last_error ?? sequence.error;
  if (!error) return "";
  if (typeof error === "string") return error;
  if (typeof error !== "object") return String(error);

  const message = error.message || error.error_string || error.detail || error.reason || "";
  const code = error.code || error.error_code || "";
  if (code && message) return `${code}: ${message}`;
  if (message) return String(message);
  try {
    return JSON.stringify(error);
  } catch (_error) {
    return String(error);
  }
};

const renderSequenceStatus = (sequence) => {
  if (sequence && typeof sequence === "object") {
    state.sequence = sequence;
  }

  const current = state.sequence || {};
  const currentPose = current.current_pose || {};
  const sequenceState = sequenceStateValue(current);
  const active = sequenceIsActive(current);
  const poseId = current.current_pose_id ?? current.current_id ?? currentPose.id;
  const poseName = current.current_pose_name ?? current.current_name ?? current.name ?? currentPose.name;
  const currentStep = current.current_step ?? current.step ?? current.current_index ?? 0;
  const totalSteps = current.total_steps ?? current.total ?? current.pose_count ?? current.pose_ids?.length ?? 0;
  const completed = current.completed_count ?? current.completed ?? 0;
  // A rejected start request does not create a backend run, so the next
  // snapshot is usually IDLE with no error. Keep that request error visible
  // until the operator explicitly tries to start another sequence.
  const latestError = sequenceErrorText(current) || state.sequenceRequestError;

  $("sequenceRunId").textContent = current.run_id || "-";
  $("sequenceState").textContent = sequenceState;
  $("sequenceStep").textContent = `${currentStep} / ${totalSteps}`;
  $("sequenceCurrentPose").textContent = poseId === null || poseId === undefined
    ? "-"
    : `${poseId}${poseName ? ` | ${poseName}` : ""}`;
  $("sequenceCompleted").textContent = String(completed);
  $("sequenceLatestError").textContent = latestError || "-";
  $("sequenceLatestError").classList.toggle("has-error", Boolean(latestError));
  $("sequenceSummary").textContent = active
    ? `${sequenceState} | ${currentStep} / ${totalSteps}`
    : latestError || sequenceState;

  $("runSequenceButton").disabled =
    state.sequenceOperationPending || active || state.savedPoses.length === 0;
  $("stopSequenceButton").disabled = state.sequenceOperationPending || !active;
};

const renderManualStatus = (manual) => {
  const controllers = manual?.controllers || {};
  const missionActive = Boolean(manual?.mission_active);
  const sequenceActive = sequenceIsActive(state.sequence);
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
    !armConfigured || missionActive || sequenceActive || !armReady || state.manualSending.arm;
  $("sendGripperManualButton").disabled =
    !gripperConfigured || missionActive || sequenceActive || !gripperReady || gripper.active || state.manualSending.gripper;

  if (missionActive) {
    $("manualReady").textContent = "Mission";
    $("manualReady").className = "status-chip warning";
  } else if (sequenceActive) {
    $("manualReady").textContent = "Sequence";
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

  const sequenceError = sequenceErrorText(state.sequence) || state.sequenceRequestError;
  if (sequenceError) {
    $("manualStatus").textContent = `Sequence 오류 | ${sequenceError}`;
    return;
  }

  if (state.manualNotice && state.manualNoticeUntil > Date.now()) {
    $("manualStatus").textContent = state.manualNotice;
    return;
  }
  state.manualNotice = "";
  state.manualNoticeUntil = 0;

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
  const display = mission.display || {};
  const directNavActive = Boolean(directNav?.active);

  $("missionReady").textContent = mission.action_ready ? "Ready" : "Offline";
  $("missionReady").className = `status-chip ${mission.action_ready ? "success" : "danger"}`;

  const stateText =
    display.state ||
    status?.state ||
    goal?.state ||
    result?.status ||
    (mission.active ? "ACTIVE" : "IDLE");
  const sourceText = display.source ? ` | ${display.source}` : "";
  $("missionState").textContent =
    `${stateText}${sourceText} | age ${formatAge(display.status_age_ms ?? mission.status_age_ms)}`;
  $("overviewMissionState").textContent = stateText;

  const progress = clamp(
    Number(display.progress ?? status?.progress ?? feedback?.progress ?? (result?.success ? 1 : 0)),
    0,
    1,
  );
  $("missionProgressText").textContent = `${Math.round(progress * 100)}%`;
  $("missionProgressBar").style.width = `${progress * 100}%`;
  $("overviewMissionMeta").textContent =
    `Progress ${Math.round(progress * 100)}% | age ${formatAge(display.status_age_ms ?? mission.status_age_ms)}`;

  $("missionTask").textContent =
    display.active_task || status?.active_task || feedback?.current_task || "-";
  $("missionMessage").textContent =
    display.message || status?.message || feedback?.detail || goal?.state || "-";
  $("missionResult").textContent = result
    ? `${result.status}: ${result.message}`
    : "-";
  renderElevatorFsm(display, status, feedback);

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
    display.active_task || status?.active_task || feedback?.current_task || "-";
  $("overviewTaskMeta").textContent =
    display.message || status?.message || feedback?.detail || result?.message || "No active task";
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

const extractElevatorFsmState = (display, status, feedback) => {
  const candidates = [
    display?.state,
    display?.event_state,
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

const renderElevatorFsm = (display, status, feedback) => {
  const currentState = extractElevatorFsmState(display, status, feedback);
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
  if (fields.error && !["NONE", "ERR_NONE"].includes(fields.error)) return "danger";
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
  if (
    stateName === "MAP_SWITCHING" ||
    stateName === "WAITING_MAP" ||
    stateName === "SWITCHING" ||
    stateName === "WAITING_ELEVATOR"
  ) return "warning";
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

const drivingMapRotation = (map) => {
  const width = Number(map?.width || 0);
  const height = Number(map?.height || 0);
  return height > width ? 90 : 0;
};

const mapFit = (map, canvasWidth, canvasHeight) => {
  const width = Number(map?.width || 0);
  const height = Number(map?.height || 0);
  if (!width || !height) return null;
  const rotation = drivingMapRotation(map);
  const rotated = rotation === 90 || rotation === 270;
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
    rotation,
  };
};

const worldToCanvas = (map, fit, x, y) => {
  const origin = map.origin || {};
  const resolution = Number(map.resolution || 0.05);
  const mx = (Number(x) - Number(origin.x || 0)) / resolution;
  const my = fit.height - (Number(y) - Number(origin.y || 0)) / resolution;
  if (fit.rotation === 90) {
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

const canvasToWorld = (map, fit, x, y) => {
  const origin = map.origin || {};
  const resolution = Number(map.resolution || 0.05);
  let mx;
  let my;

  if (fit.rotation === 90) {
    mx = (y - fit.top) / fit.scale;
    my = fit.height - (x - fit.left) / fit.scale;
  } else {
    mx = (x - fit.left) / fit.scale;
    my = (y - fit.top) / fit.scale;
  }

  if (mx < 0 || my < 0 || mx > fit.width || my > fit.height) {
    return null;
  }

  return {
    x: Number(origin.x || 0) + mx * resolution,
    y: Number(origin.y || 0) + (fit.height - my) * resolution,
  };
};

const pointerCanvasPoint = (canvas, event) => {
  const rect = canvas.getBoundingClientRect();
  return {
    x: (event.clientX - rect.left) * (canvas.width / rect.width),
    y: (event.clientY - rect.top) * (canvas.height / rect.height),
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
  const yawRotation = fit.rotation === 90 ? Math.PI / 2 : 0;
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

const drawInitialPoseDraft = (ctx, map, fit, canvasWidth) => {
  const draft = state.initialPoseDraft;
  if (!draft) return;

  const point = worldToCanvas(map, fit, draft.x, draft.y);
  const yawRotation = fit.rotation === 90 ? Math.PI / 2 : 0;
  const radius = Math.max(8, canvasWidth / 85);
  const arrow = radius * 2.5;

  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(-draft.yaw + yawRotation);
  ctx.fillStyle = "rgba(45, 106, 207, 0.88)";
  ctx.strokeStyle = "#153e90";
  ctx.lineWidth = Math.max(2, canvasWidth / 520);
  ctx.beginPath();
  ctx.arc(0, 0, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(arrow, 0);
  ctx.lineTo(arrow - radius * 0.6, -radius * 0.5);
  ctx.moveTo(arrow, 0);
  ctx.lineTo(arrow - radius * 0.6, radius * 0.5);
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
  if (fit.rotation === 90) {
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
  drawInitialPoseDraft(ctx, map, fit, width);
};

const loadDrivingMapIfNeeded = async (mapMeta = {}) => {
  if (!mapMeta.available || !mapMeta.data_url) {
    state.drivingMap = null;
    state.drivingMapRevision = null;
    state.initialPoseDraft = null;
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
    state.initialPoseDraft = null;
  } catch (error) {
    $("drivingMapMeta").textContent = `Map fetch failed: ${error.message}`;
  } finally {
    state.drivingMapLoading = false;
  }
};

const updateInitialPoseUi = () => {
  const button = $("initialPoseButton");
  const wrapper = $("drivingMapCanvas")?.parentElement;
  if (!button || !wrapper) return;

  wrapper.classList.toggle("pose-mode", state.initialPoseMode);
  button.textContent = state.initialPoseMode ? "Cancel Pose" : "Set Pose";
  button.classList.toggle("primary-button", state.initialPoseMode);
  button.classList.toggle("secondary-button", !state.initialPoseMode);
};

const setInitialPoseMode = (enabled) => {
  state.initialPoseMode = Boolean(enabled);
  state.initialPoseDraft = null;
  updateInitialPoseUi();
  drawDrivingMap(state.latestDriving || {});
};

const sendInitialPose = async (pose) => {
  if (state.initialPoseSending) return;
  state.initialPoseSending = true;
  try {
    await api("/api/driving/initial-pose", {
      method: "POST",
      body: JSON.stringify({
        x: pose.x,
        y: pose.y,
        yaw: pose.yaw,
        frame_id: state.drivingMap?.frame_id || "map",
      }),
    });
    setInitialPoseMode(false);
    await poll();
  } catch (error) {
    setConnection(`INITIAL POSE FAILED | ${error.message}`, "offline");
    state.initialPoseDraft = null;
    drawDrivingMap(state.latestDriving || {});
  } finally {
    state.initialPoseSending = false;
  }
};

const beginInitialPoseDrag = (event) => {
  if (!state.initialPoseMode || !state.drivingMap) return;

  const canvas = $("drivingMapCanvas");
  const fit = mapFit(state.drivingMap, canvas.width, canvas.height);
  if (!fit) return;

  const point = pointerCanvasPoint(canvas, event);
  const world = canvasToWorld(state.drivingMap, fit, point.x, point.y);
  if (!world) return;

  event.preventDefault();
  canvas.setPointerCapture?.(event.pointerId);
  state.initialPoseDraft = {
    ...world,
    yaw: Number(state.latestDriving?.pose?.yaw || 0),
    pointerId: event.pointerId,
  };
  drawDrivingMap(state.latestDriving || {});
};

const updateInitialPoseDrag = (event) => {
  if (!state.initialPoseMode || !state.initialPoseDraft || !state.drivingMap) return;
  if (state.initialPoseDraft.pointerId !== event.pointerId) return;

  const canvas = $("drivingMapCanvas");
  const fit = mapFit(state.drivingMap, canvas.width, canvas.height);
  if (!fit) return;

  const point = pointerCanvasPoint(canvas, event);
  const world = canvasToWorld(state.drivingMap, fit, point.x, point.y);
  if (!world) return;

  const dx = world.x - state.initialPoseDraft.x;
  const dy = world.y - state.initialPoseDraft.y;
  if (Math.hypot(dx, dy) > Number(state.drivingMap.resolution || 0.05) * 2) {
    state.initialPoseDraft.yaw = Math.atan2(dy, dx);
  }
  drawDrivingMap(state.latestDriving || {});
};

const finishInitialPoseDrag = (event) => {
  if (!state.initialPoseMode || !state.initialPoseDraft) return;
  if (state.initialPoseDraft.pointerId !== event.pointerId) return;

  const pose = {
    x: state.initialPoseDraft.x,
    y: state.initialPoseDraft.y,
    yaw: state.initialPoseDraft.yaw,
  };
  state.initialPoseDraft = null;
  sendInitialPose(pose);
};

const renderDriving = (driving = {}) => {
  state.latestDriving = driving;
  const map = driving.map || {};
  const pose = driving.pose || {};
  const odom = driving.odom || {};
  const navState = driving.state || {};
  const mapSwitch = driving.map_switch || {};
  const globalPath = driving.global_path || {};
  const localPath = driving.local_path || {};
  const activeMapSwitch = ["SWITCHING", "WAITING_MAP"].includes(mapSwitch.state);

  const label = activeMapSwitch
    ? mapSwitch.message || `${mapSwitch.target_floor || "-"}F map switching`
    : navState.label || navState.state || "Waiting";
  $("drivingStateChip").textContent = label;
  $("drivingStateChip").className =
    `status-chip ${drivingChipClass(activeMapSwitch ? mapSwitch.state : navState.state)}`;

  const mapSwitchText = mapSwitch.state && mapSwitch.state !== "IDLE"
    ? ` | ${mapSwitch.message || mapSwitch.state}`
    : "";

  if (map.available) {
    $("drivingMapMeta").textContent =
      `${map.topic || "/map"} | rev ${map.revision} | age ${formatAge(map.age_ms)}${mapSwitchText}`;
    $("drivingMapInfo").textContent = `${map.width}x${map.height}, res ${formatNumber(map.resolution, 3)} m/px, origin (${formatNumber(map.origin?.x, 2)}, ${formatNumber(map.origin?.y, 2)})`;
  } else {
    $("drivingMapMeta").textContent = `Waiting for ${map.topic || "/map"}${mapSwitchText}`;
    $("drivingMapInfo").textContent = "No map";
  }

  $("drivingPose").textContent = pose.available
    ? `x ${formatNumber(pose.x, 3)}, y ${formatNumber(pose.y, 3)}, yaw ${formatNumber(pose.yaw_deg, 1)}° | age ${formatAge(pose.age_ms)}`
    : `Waiting for ${pose.topic || "/amcl_pose"}`;

  $("drivingOdom").textContent = odom.available
    ? `v ${formatNumber(odom.linear_x, 3)} m/s, w ${formatNumber(odom.angular_z, 3)} rad/s | age ${formatAge(odom.age_ms)}`
    : `Waiting for ${odom.topic || "/odom"}`;

  $("drivingPathInfo").textContent = `global ${globalPath.available ? `${globalPath.count} pts` : "-"}, local ${localPath.available ? `${localPath.count} pts` : "-"}`;
  const mapSwitchDetail = mapSwitch.state && mapSwitch.state !== "IDLE"
    ? ` | map ${mapSwitch.state}${mapSwitch.target_floor ? ` ${mapSwitch.target_floor}F` : ""}`
    : "";
  $("drivingNavState").textContent =
    `${navState.state || "IDLE"}${navState.mission_state ? ` | mission ${navState.mission_state}` : ""}${mapSwitchDetail}${navState.detail ? ` | ${navState.detail}` : ""}`;

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
  if (snapshot.manual?.sequence && typeof snapshot.manual.sequence === "object") {
    renderSequenceStatus(snapshot.manual.sequence);
  } else {
    renderSequenceStatus();
  }
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
    const snapshot = await api("/api/snapshot", { timeoutMs: SNAPSHOT_TIMEOUT_MS });
    state.snapshotFailures = 0;
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
    } else if (!state.pendingSync && state.syncStatus !== "LIVE") {
      state.syncStatus = "LIVE";
      renderSyncStatus();
    }
  } catch (error) {
    state.snapshotFailures += 1;
    state.pendingSync = state.lastSeq !== null;
    if (state.snapshotFailures >= OFFLINE_FAILURE_THRESHOLD) {
      state.syncStatus = "OFFLINE";
      setConnection(
        `OFFLINE | browser cannot reach central Flask snapshot: ${error.message}`,
        "offline",
      );
    } else {
      state.syncStatus = "FAILED";
      setConnection(
        `LINK CHECK ${state.snapshotFailures}/${OFFLINE_FAILURE_THRESHOLD} | ${error.message}`,
        "recovered",
      );
    }
    renderSyncStatus();
  } finally {
    state.polling = false;
  }
};

const postArmCommand = async (command, payload = {}) => {
  const buttons = Array.from(
    document.querySelectorAll("[data-arm-command], #statusButton, #estopTopButton"),
  ).filter((button) => (
    button.id !== "estopTopButton"
    && button.dataset.armCommand !== "estop"
  ));
  buttons.forEach((button) => {
    button.disabled = true;
  });

  try {
    const timeoutMs = command === "home_all"
      ? HOME_COMMAND_TIMEOUT_MS
      : command === "clear_error"
        ? CLEAR_COMMAND_TIMEOUT_MS
        : API_DEFAULT_TIMEOUT_MS;
    await api(`/api/arm/${command}`, {
      method: "POST",
      body: JSON.stringify(payload),
      timeoutMs,
    });
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
  if (sequenceIsActive(state.sequence)) {
    setManualNotice("Sequence 실행 중에는 개별 수동 명령을 보낼 수 없습니다.");
    return;
  }

  const dom = manualDom[controller];
  const payload = {
    positions_deg: readManualPositions(controller),
    duration_sec: Number($(dom.duration).value),
    request_id: (
      globalThis.crypto?.randomUUID?.()
      || `manual-${controller}-${Date.now()}-${Math.random().toString(16).slice(2)}`
    ),
    client_created_unix_ms: Date.now(),
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

const setManualNotice = (message, durationMs = 8000) => {
  state.manualNotice = String(message || "");
  state.manualNoticeUntil = Date.now() + durationMs;
  $("manualStatus").textContent = state.manualNotice;
};

const setSavedPoseStatus = (message, isError = false) => {
  const node = $("savedPoseStatus");
  node.textContent = message;
  node.className = `meta-line${isError ? " status-error" : ""}`;
};

const sortedSavedPoses = () => [...state.savedPoses].sort(
  (left, right) => Number(left.id) - Number(right.id),
);

const hasSavableManualController = () => ["arm", "gripper"].some((controller) => {
  const joints = state.manualConfigs[controller]?.joints;
  return Array.isArray(joints) && joints.length > 0;
});

const renderSavedPoseTable = () => {
  const body = $("savedPoseTableBody");
  const poses = sortedSavedPoses();
  if (!poses.length) {
    body.innerHTML = `
      <tr>
        <td colspan="5" class="empty">저장된 자세가 없습니다.</td>
      </tr>
    `;
  } else {
    body.innerHTML = poses
      .map((pose) => {
        const dwell = Number(pose.dwell_sec);
        const dwellText = Number.isFinite(dwell) ? dwell : 0;
        return `
          <tr>
            <td class="pose-id">${escapeHtml(pose.id)}</td>
            <td class="pose-name-cell">
              <strong>${escapeHtml(pose.name)}</strong>
              <small>대기 ${escapeHtml(dwellText)}초</small>
            </td>
            <td><button class="pose-action-button" type="button" data-pose-action="load" data-pose-id="${escapeHtml(pose.id)}">불러오기</button></td>
            <td><button class="pose-action-button" type="button" data-pose-action="edit" data-pose-id="${escapeHtml(pose.id)}">수정하기</button></td>
            <td><button class="pose-action-button pose-delete-button" type="button" data-pose-action="delete" data-pose-id="${escapeHtml(pose.id)}">삭제하기</button></td>
          </tr>
        `;
      })
      .join("");
  }

  $("openSavePoseButton").disabled =
    state.savedPosesLoading || state.savedPoseMutationPending || !hasSavableManualController();
  body.querySelectorAll("button").forEach((button) => {
    button.disabled = state.savedPoseMutationPending;
  });
  renderSequenceStatus();
};

const captureSavedPoseControllers = () => {
  const controllers = {};

  ["arm", "gripper"].forEach((controller) => {
    const config = state.manualConfigs[controller];
    const joints = Array.isArray(config?.joints) ? config.joints : [];
    if (!joints.length) return;

    const keyedPositions = readManualPositions(controller);
    const positions = {};
    joints.forEach((joint) => {
      const value = keyedPositions[joint.key];
      if (Number.isFinite(Number(value))) {
        positions[joint.joint_name] = Number(value);
      }
    });

    controllers[controller] = {
      positions_deg: positions,
      duration_sec: Number($(manualDom[controller].duration).value),
    };
    if (controller === "gripper") {
      controllers.gripper.target_load_raw = Number($("manualGripperLoad").value);
    }
  });

  if (!Object.keys(controllers).length) {
    throw new Error("저장할 수동 제어 설정이 없습니다.");
  }
  return controllers;
};

const upsertSavedPose = (pose) => {
  if (!pose || pose.id === null || pose.id === undefined) return;
  const index = state.savedPoses.findIndex((candidate) => Number(candidate.id) === Number(pose.id));
  if (index >= 0) {
    state.savedPoses[index] = pose;
  } else {
    state.savedPoses.push(pose);
  }
  state.savedPoses = sortedSavedPoses();
};

const refreshSavedPoses = async () => {
  if (state.savedPosesLoading) return;
  state.savedPosesLoading = true;
  renderSavedPoseTable();

  try {
    const data = await api("/api/manual/poses");
    state.savedPoses = Array.isArray(data.poses) ? data.poses : [];
    if (Number.isInteger(Number(data.next_id)) && Number(data.next_id) > 0) {
      state.nextSavedPoseId = Number(data.next_id);
    }
    setSavedPoseStatus(`${state.savedPoses.length}개 자세 저장됨`);
  } catch (error) {
    setSavedPoseStatus(`저장 자세 불러오기 실패: ${error.message}`, true);
    setManualNotice(`저장 자세 오류 | ${error.message}`);
  } finally {
    state.savedPosesLoading = false;
    renderSavedPoseTable();
  }
};

const refreshSequenceStatus = async () => {
  try {
    const data = await api("/api/manual/sequence");
    renderSequenceStatus(data.sequence || data);
  } catch (error) {
    state.sequence = {
      ...state.sequence,
      latest_error: error.message,
    };
    renderSequenceStatus();
    setManualNotice(`Sequence 오류 | ${error.message}`);
  }
};

const openSavePoseDialog = () => {
  $("savePoseId").value = String(state.nextSavedPoseId);
  $("savePoseName").value = "";
  $("savePoseDwell").value = "0";
  $("savePoseDialog").showModal();
  $("savePoseName").focus();
};

const openEditPoseDialog = (pose) => {
  if (!pose) return;
  $("editPoseDialog").dataset.originalPoseId = String(pose.id);
  $("editPoseId").value = String(pose.id);
  $("editPoseName").value = pose.name || "";
  $("editPoseDwell").value = String(pose.dwell_sec ?? 0);
  $("editPoseControllers").innerHTML = ["arm", "gripper"]
    .map((controller) => {
      const config = state.manualConfigs[controller];
      const joints = Array.isArray(config?.joints) ? config.joints : [];
      const stored = pose.controllers?.[controller];
      const positions = stored?.positions_deg || {};
      if (!stored || !joints.length) return "";
      const title = controller === "arm" ? "로봇팔 각도" : "그리퍼 각도";
      const targetLoad = stored.target_load_raw ?? config.default_target_load_raw ?? 500;
      const targetLoadMax = config.target_load_max ?? 1023;
      return `
        <fieldset class="edit-pose-controller">
          <legend>${title}</legend>
          ${joints.map((joint) => {
            const value = formatAngleInput(
              positions[joint.joint_name] ?? positions[joint.key] ?? joint.default_deg ?? 0,
            );
            return `
              <label>
                ${escapeHtml(joint.label || joint.joint_name)} (°)
                <input
                  type="number"
                  min="${escapeHtml(joint.min_deg)}"
                  max="${escapeHtml(joint.max_deg)}"
                  step="0.1"
                  value="${escapeHtml(value)}"
                  data-edit-pose-controller="${escapeHtml(controller)}"
                  data-edit-pose-joint="${escapeHtml(joint.joint_name)}"
                  required
                >
              </label>
            `;
          }).join("")}
          ${controller === "gripper" ? `
            <label>
              부하값
              <input
                type="number"
                min="0"
                max="${escapeHtml(targetLoadMax)}"
                step="1"
                value="${escapeHtml(targetLoad)}"
                data-edit-pose-load
                required
              >
            </label>
          ` : ""}
        </fieldset>
      `;
    })
    .join("");
  $("editPoseDialog").showModal();
  $("editPoseName").focus();
};

const loadSavedPoseIntoManual = (pose) => {
  if (!pose) return;
  let appliedJoints = 0;

  ["arm", "gripper"].forEach((controller) => {
    const stored = pose.controllers?.[controller];
    const config = state.manualConfigs[controller];
    const joints = Array.isArray(config?.joints) ? config.joints : [];
    if (!stored || !joints.length) return;

    const positions = stored.positions_deg || stored.positions || stored.joints || {};
    let controllerApplied = 0;
    joints.forEach((joint) => {
      const rawValue = positions[joint.joint_name] ?? positions[joint.key];
      if (!Number.isFinite(Number(rawValue))) return;
      setManualControlValue(controller, joint, Number(rawValue));
      appliedJoints += 1;
      controllerApplied += 1;
    });
    if (!controllerApplied) return;

    if (Number.isFinite(Number(stored.duration_sec))) {
      $(manualDom[controller].duration).value = String(stored.duration_sec);
    }
    if (controller === "gripper" && Number.isFinite(Number(stored.target_load_raw))) {
      $("manualGripperLoad").value = String(stored.target_load_raw);
    }
    state.manualInitialized[controller] = true;
    state.manualDirty[controller] = true;
  });

  if (!appliedJoints) {
    const message = `자세 ${pose.id}에 현재 구성과 일치하는 관절이 없습니다.`;
    setSavedPoseStatus(message, true);
    setManualNotice(message);
    return;
  }

  const message = `자세 ${pose.id} (${pose.name}) 값을 불러왔습니다. 로봇에는 전송하지 않았습니다.`;
  setSavedPoseStatus(message);
  setManualNotice(message);
};

const submitSavePose = async (event) => {
  event.preventDefault();
  const id = Number($("savePoseId").value);
  const name = $("savePoseName").value.trim();
  const dwellSec = Number($("savePoseDwell").value);
  if (!Number.isInteger(id) || id <= 0 || !name || !Number.isFinite(dwellSec) || dwellSec < 0) {
    setSavedPoseStatus("빈 양의 정수 ID, 이름과 0 이상의 대기시간을 확인해 주세요.", true);
    return;
  }

  state.savedPoseMutationPending = true;
  renderSavedPoseTable();
  $("savePoseSubmitButton").disabled = true;
  try {
    const data = await api("/api/manual/poses", {
      method: "POST",
      body: JSON.stringify({
        id,
        name,
        dwell_sec: dwellSec,
        controllers: captureSavedPoseControllers(),
      }),
    });
    upsertSavedPose(data.pose);
    if (Number.isInteger(Number(data.next_id)) && Number(data.next_id) > 0) {
      state.nextSavedPoseId = Number(data.next_id);
    }
    $("savePoseDialog").close("saved");
    setSavedPoseStatus(`자세 ${data.pose?.id ?? ""} 저장 완료`);
    setManualNotice(`현재 수동 자세를 ${data.pose?.id ?? "새 ID"}번으로 저장했습니다.`);
  } catch (error) {
    setSavedPoseStatus(`저장 실패: ${error.message}`, true);
    setManualNotice(`저장 자세 오류 | ${error.message}`);
  } finally {
    state.savedPoseMutationPending = false;
    $("savePoseSubmitButton").disabled = false;
    renderSavedPoseTable();
  }
};

const submitEditPose = async (event) => {
  event.preventDefault();
  const originalId = Number($("editPoseDialog").dataset.originalPoseId);
  const id = Number($("editPoseId").value);
  const name = $("editPoseName").value.trim();
  const dwellSec = Number($("editPoseDwell").value);
  if (!Number.isInteger(originalId) || !Number.isInteger(id) || id <= 0 || !name || !Number.isFinite(dwellSec) || dwellSec < 0) {
    setSavedPoseStatus("빈 양의 정수 ID, 이름과 0 이상의 대기시간을 확인해 주세요.", true);
    return;
  }

  state.savedPoseMutationPending = true;
  renderSavedPoseTable();
  $("editPoseSubmitButton").disabled = true;
  try {
    const original = state.savedPoses.find((pose) => Number(pose.id) === originalId) || {};
    const controllers = JSON.parse(JSON.stringify(original.controllers || {}));
    document.querySelectorAll("[data-edit-pose-controller][data-edit-pose-joint]").forEach((input) => {
      const controller = input.dataset.editPoseController;
      const joint = input.dataset.editPoseJoint;
      const value = Number(input.value);
      if (!Number.isFinite(value)) {
        throw new Error(`${joint} 각도를 확인해 주세요.`);
      }
      controllers[controller].positions_deg[joint] = value;
    });
    const loadInput = document.querySelector("[data-edit-pose-load]");
    if (loadInput) {
      const value = Number(loadInput.value);
      const maxValue = Number(loadInput.max || 1023);
      if (!Number.isInteger(value) || value < 0 || value > maxValue) {
        throw new Error(`부하값은 0~${maxValue} 정수로 입력해 주세요.`);
      }
      controllers.gripper.target_load_raw = value;
    }
    const data = await api(`/api/manual/poses/${encodeURIComponent(originalId)}`, {
      method: "PATCH",
      body: JSON.stringify({ id, name, dwell_sec: dwellSec, controllers }),
    });
    state.savedPoses = state.savedPoses.filter((pose) => Number(pose.id) !== originalId);
    upsertSavedPose(data.pose || { ...original, id, name, dwell_sec: dwellSec });
    if (Number.isInteger(Number(data.next_id)) && Number(data.next_id) > 0) {
      state.nextSavedPoseId = Number(data.next_id);
    }
    $("editPoseDialog").close("saved");
    setSavedPoseStatus(`자세 ${id} 수정 완료`);
    setManualNotice(`저장 자세 ${id}의 이름, 대기시간, 각도와 부하값을 수정했습니다.`);
  } catch (error) {
    setSavedPoseStatus(`수정 실패: ${error.message}`, true);
    setManualNotice(`저장 자세 오류 | ${error.message}`);
  } finally {
    state.savedPoseMutationPending = false;
    $("editPoseSubmitButton").disabled = false;
    renderSavedPoseTable();
  }
};

const deleteSavedPose = async (pose) => {
  if (!pose || !window.confirm(`자세 ${pose.id} (${pose.name})를 삭제하시겠습니까?`)) return;

  state.savedPoseMutationPending = true;
  renderSavedPoseTable();
  try {
    const data = await api(`/api/manual/poses/${encodeURIComponent(pose.id)}`, {
      method: "DELETE",
    });
    state.savedPoses = state.savedPoses.filter((candidate) => Number(candidate.id) !== Number(pose.id));
    if (Number.isInteger(Number(data.next_id)) && Number(data.next_id) > 0) {
      state.nextSavedPoseId = Number(data.next_id);
    }
    setSavedPoseStatus(`자세 ${pose.id} 삭제 완료`);
    setManualNotice(`저장 자세 ${pose.id}를 삭제했습니다.`);
  } catch (error) {
    setSavedPoseStatus(`삭제 실패: ${error.message}`, true);
    setManualNotice(`저장 자세 오류 | ${error.message}`);
  } finally {
    state.savedPoseMutationPending = false;
    renderSavedPoseTable();
  }
};

const handleSavedPoseTableClick = (event) => {
  const button = event.target.closest("button[data-pose-action]");
  if (!button) return;
  const pose = state.savedPoses.find((candidate) => Number(candidate.id) === Number(button.dataset.poseId));
  if (!pose) return;

  if (button.dataset.poseAction === "load") {
    loadSavedPoseIntoManual(pose);
  } else if (button.dataset.poseAction === "edit") {
    openEditPoseDialog(pose);
  } else if (button.dataset.poseAction === "delete") {
    deleteSavedPose(pose);
  }
};

const startManualSequence = async () => {
  if (!state.savedPoses.length || sequenceIsActive(state.sequence)) return;

  const rawOrder = $("sequencePoseOrder").value.trim();
  let poseIds = null;
  if (rawOrder) {
    const tokens = rawOrder.split(/[\s,]+/).filter(Boolean);
    if (!tokens.every((token) => /^\d+$/.test(token) && Number(token) > 0)) {
      state.sequenceRequestError = "실행 순서는 양의 정수 ID를 공백 또는 쉼표로 구분해 입력해 주세요.";
      renderSequenceStatus();
      setManualNotice(`Sequence 실행 실패 | ${state.sequenceRequestError}`);
      return;
    }
    poseIds = tokens.map(Number);
  }

  state.sequenceRequestError = "";
  state.sequenceOperationPending = true;
  renderSequenceStatus();
  try {
    const data = await api("/api/manual/sequence/start", {
      method: "POST",
      body: JSON.stringify(poseIds ? { pose_ids: poseIds } : {}),
    });
    renderSequenceStatus(data.sequence || data);
    setManualNotice(`Sequence 실행 요청 | 자세 ${poseIds?.length ?? state.savedPoses.length}개`);
    await poll();
  } catch (error) {
    state.sequenceRequestError = error.message;
    state.sequence = {
      ...state.sequence,
      active: false,
      state: "ERROR",
      latest_error: error.message,
    };
    renderSequenceStatus();
    setManualNotice(`Sequence 실행 실패 | ${error.message}`);
  } finally {
    state.sequenceOperationPending = false;
    renderSequenceStatus();
  }
};

const stopManualSequence = async () => {
  if (!sequenceIsActive(state.sequence)) return;

  state.sequenceOperationPending = true;
  renderSequenceStatus();
  try {
    const data = await api("/api/manual/sequence/stop", {
      method: "POST",
      body: "{}",
    });
    renderSequenceStatus(data.sequence || data);
    setManualNotice("Sequence 중지 요청을 보냈습니다.");
    await poll();
  } catch (error) {
    state.sequence = {
      ...state.sequence,
      latest_error: error.message,
    };
    renderSequenceStatus();
    setManualNotice(`Sequence 중지 실패 | ${error.message}`);
  } finally {
    state.sequenceOperationPending = false;
    renderSequenceStatus();
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
    arm_task_name: $("armTaskName").value,
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
  $("initialPoseButton").addEventListener("click", () => {
    setInitialPoseMode(!state.initialPoseMode);
  });
  $("drivingMapCanvas").addEventListener("pointerdown", beginInitialPoseDrag);
  $("drivingMapCanvas").addEventListener("pointermove", updateInitialPoseDrag);
  $("drivingMapCanvas").addEventListener("pointerup", finishInitialPoseDrag);
  $("drivingMapCanvas").addEventListener("pointercancel", () => {
    state.initialPoseDraft = null;
    drawDrivingMap(state.latestDriving || {});
  });
  $("statusButton").addEventListener("click", () => postArmCommand("status"));
  $("estopTopButton").addEventListener("click", () => postArmCommand("estop"));
  $("sendArmManualButton").addEventListener("click", () => sendManual("arm"));
  $("sendGripperManualButton").addEventListener("click", () => sendManual("gripper"));
  $("useCurrentArmButton").addEventListener("click", () => useCurrentManual("arm"));
  $("useCurrentGripperButton").addEventListener("click", () => useCurrentManual("gripper"));
  $("openSavePoseButton").addEventListener("click", openSavePoseDialog);
  $("savePoseForm").addEventListener("submit", submitSavePose);
  $("editPoseForm").addEventListener("submit", submitEditPose);
  $("savedPoseTableBody").addEventListener("click", handleSavedPoseTableClick);
  $("runSequenceButton").addEventListener("click", startManualSequence);
  $("stopSequenceButton").addEventListener("click", stopManualSequence);

  document.querySelectorAll("[data-dialog-close]").forEach((button) => {
    button.addEventListener("click", () => {
      const dialog = $(button.dataset.dialogClose);
      if (dialog?.open) dialog.close("cancel");
    });
  });

  $("disableConfirmForm").addEventListener("submit", (event) => {
    event.preventDefault();
    $("disableConfirmDialog").close("confirmed");
    postArmCommand("disable", { confirmed: true });
  });

  document.querySelectorAll("[data-arm-command]").forEach((button) => {
    button.addEventListener("click", () => {
      if (button.dataset.armCommand === "disable") {
        $("disableConfirmDialog").showModal();
        $("confirmDisableButton").focus();
        return;
      }
      postArmCommand(button.dataset.armCommand);
    });
  });

  renderSavedPoseTable();
  renderSequenceStatus();
  refreshSavedPoses();
  refreshSequenceStatus();
  poll();
  setInterval(poll, 700);
});
