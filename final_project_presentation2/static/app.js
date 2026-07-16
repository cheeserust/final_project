(() => {
  "use strict";

  const API_ROOT = "/api";
  const POLL_INTERVAL_MS = 1000;
  const ARM_JOINT_NAMES = ["base_joint", "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"];
  const GRIPPER_JOINT_NAMES = [
    "finger_1_base", "finger_1_middle", "finger_1_tip",
    "finger_2_base", "finger_2_middle", "finger_2_tip",
    "finger_3_base", "finger_3_middle", "finger_3_tip",
  ];
  const ACTIVE_STATES = new Set([
    "VERIFY_PICKUP", "ACQUIRE_DROPOFF", "OUTBOUND_TO_DROPOFF",
    "PLACE_OBJECT", "ACQUIRE_PICKUP", "RETURN_TO_PICKUP", "STOPPING", "RUNNING",
  ]);
  const ERROR_STATES = new Set(["ERROR", "ERROR_LATCHED", "CONFIG_ERROR", "FAULT"]);
  const READY_STATES = new Set([
    "AT_PICKUP", "AT_DROPOFF", "IDLE", "READY",
    "WAIT_RETURN_CONFIRM", "WAITING_RETURN_CONFIRM",
  ]);
  const STEP_LABELS = {
    POSE: "저장 동작 실행",
    GO_DROPOFF: "놓기 위치로 이동",
    WAIT_SECONDS: "시간 대기",
    WAIT_RETURN_CONFIRM: "기존 복귀 확인 · 자동 통과",
    GO_PICKUP: "집기 위치로 복귀",
  };

  const app = {
    snapshot: null,
    connected: false,
    firstSnapshotReceived: false,
    polling: false,
    pollTimer: null,
    pollIntervalMs: POLL_INTERVAL_MS,
    active: false,
    errorLatched: false,
    motionReady: false,
    driveReady: false,
    poseReady: false,
    workflowReady: false,
    configReady: false,
    turnReady: false,
    armReady: false,
    gripperReady: false,
    waitingReturn: false,
    checkpoint: null,
    workflowProgress: null,
    workflowRecovery: null,
    currentState: "UNKNOWN",
    categories: [],
    poses: [],
    workflows: [],
    selectedWorkflowId: null,
    selectedWorkflowStep: null,
    armFeedbackDeg: [],
    gripperFeedbackDeg: [],
    workflowDraft: [],
    events: [],
    serverEventKeys: new Set(),
    libraryFallbackAttempted: false,
    routeVisual: {
      phase: "",
      phaseStartedAt: performance.now(),
      initialDistance: null,
      progress: 0,
      x: 89,
      y: 50,
      heading: 180,
      direction: "outbound",
      destination: "pickup",
      status: "대기 · A 집기",
    },
  };

  const $ = (selector, root = document) => root.querySelector(selector);
  const $$ = (selector, root = document) => Array.from(root.querySelectorAll(selector));

  class ApiError extends Error {
    constructor(message, status = 0, details = null) {
      super(message);
      this.name = "ApiError";
      this.status = status;
      this.details = details;
    }
  }

  function firstDefined(...values) {
    return values.find((value) => value !== undefined && value !== null);
  }

  function asBoolean(value) {
    if (typeof value === "boolean") return value;
    if (typeof value === "number") return value !== 0;
    if (typeof value === "string") {
      const normalized = value.trim().toLowerCase();
      if (["true", "ok", "ready", "online", "connected", "healthy", "active", "fresh", "yes"].includes(normalized)) return true;
      if (["false", "bad", "offline", "disconnected", "fault", "stale", "no", "error"].includes(normalized)) return false;
    }
    return undefined;
  }

  function finiteNumber(value) {
    if (value === "" || value === null || value === undefined) return null;
    const number = Number(value);
    return Number.isFinite(number) ? number : null;
  }

  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  function formatNumber(value, digits = 2, fallback = "—") {
    const number = finiteNumber(value);
    return number === null ? fallback : number.toFixed(digits);
  }

  function formatAngle(value, digits = 1) {
    const number = finiteNumber(value);
    if (number === null) return "—°";
    return `${number > 0 ? "+" : ""}${number.toFixed(digits)}°`;
  }

  function formatTime(value = new Date()) {
    const date = value instanceof Date ? value : new Date(value);
    if (Number.isNaN(date.getTime())) return "—";
    return new Intl.DateTimeFormat("ko-KR", {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      hour12: false,
    }).format(date);
  }

  function unwrapApiPayload(payload) {
    if (!payload || typeof payload !== "object") return payload;
    if (Object.prototype.hasOwnProperty.call(payload, "data") &&
        (Object.prototype.hasOwnProperty.call(payload, "ok") || Object.keys(payload).length <= 3)) {
      return payload.data;
    }
    return payload;
  }

  async function api(path, { method = "GET", body, signal } = {}) {
    const options = {
      method,
      headers: { Accept: "application/json" },
      cache: "no-store",
      signal,
    };
    if (body !== undefined) {
      options.headers["Content-Type"] = "application/json";
      options.body = JSON.stringify(body);
    }

    let response;
    try {
      response = await fetch(`${API_ROOT}${path}`, options);
    } catch (error) {
      if (error.name === "AbortError") throw error;
      throw new ApiError("서버에 연결할 수 없습니다. 네트워크와 노드 상태를 확인하세요.", 0, error);
    }

    const contentType = response.headers.get("content-type") || "";
    let payload = null;
    if (response.status !== 204) {
      try {
        payload = contentType.includes("application/json") ? await response.json() : await response.text();
      } catch (_error) {
        payload = null;
      }
    }

    if (!response.ok) {
      const message = typeof payload === "string"
        ? payload
        : firstDefined(payload?.message, payload?.error?.message, payload?.error, payload?.detail, `요청 실패 (${response.status})`);
      throw new ApiError(String(message), response.status, payload);
    }
    return unwrapApiPayload(payload);
  }

  function showToast(message, level = "info", timeoutMs = 3800) {
    const region = $("#toastRegion");
    const toast = document.createElement("div");
    toast.className = `toast ${level}`;
    toast.setAttribute("role", level === "error" ? "alert" : "status");
    toast.textContent = message;
    region.append(toast);
    const remove = () => {
      toast.classList.add("removing");
      window.setTimeout(() => toast.remove(), 190);
    };
    window.setTimeout(remove, timeoutMs);
  }

  function addEvent(message, level = "info", timestamp = new Date(), source = "UI") {
    app.events.unshift({ message: String(message), level, timestamp, source });
    if (app.events.length > 80) app.events.length = 80;
    renderEvents();
  }

  function renderEvents() {
    const list = $("#eventList");
    list.replaceChildren();
    if (!app.events.length) {
      const empty = document.createElement("li");
      empty.className = "muted-event";
      empty.textContent = "아직 기록이 없습니다.";
      list.append(empty);
      return;
    }
    app.events.forEach((event) => {
      const item = document.createElement("li");
      item.className = "event-item";
      const time = document.createElement("time");
      time.className = "event-time";
      time.dateTime = new Date(event.timestamp).toISOString();
      time.textContent = formatTime(event.timestamp);
      const level = document.createElement("span");
      level.className = `event-level ${event.level}`;
      level.textContent = event.source || event.level.toUpperCase();
      const message = document.createElement("span");
      message.className = "event-message";
      message.textContent = event.message;
      item.append(time, level, message);
      list.append(item);
    });
  }

  function errorMessage(error) {
    if (error instanceof ApiError) {
      if (error.status === 409) return `현재 상태에서는 실행할 수 없습니다: ${error.message}`;
      if (error.status === 503) return `하드웨어가 준비되지 않았습니다: ${error.message}`;
      return error.message;
    }
    return error?.message || String(error);
  }

  function handleActionError(error, context) {
    const message = `${context}: ${errorMessage(error)}`;
    showToast(message, "error", 5500);
    addEvent(message, "error", new Date(), "API");
  }

  async function confirmAction(title, message, { danger = false, acceptLabel = "확인" } = {}) {
    const dialog = $("#confirmDialog");
    if (!dialog?.showModal) return window.confirm(`${title}\n\n${message}`);
    $("#confirmTitle").textContent = title;
    $("#confirmMessage").textContent = message;
    const accept = $("#confirmAccept");
    accept.textContent = acceptLabel;
    accept.className = `btn ${danger ? "btn-danger" : "btn-primary"}`;
    dialog.returnValue = "";
    dialog.showModal();
    return new Promise((resolve) => {
      const onCancel = () => { dialog.returnValue = "cancel"; };
      dialog.addEventListener("cancel", onCancel, { once: true });
      dialog.addEventListener("close", () => {
        dialog.removeEventListener("cancel", onCancel);
        const confirmed = dialog.returnValue === "confirm";
        dialog.returnValue = "";
        resolve(confirmed);
      }, { once: true });
    });
  }

  function normalizeCollection(raw, kind) {
    if (!raw) return [];
    if (typeof raw === "object" && !Array.isArray(raw)) {
      const wrapperKeys = {
        category: ["categories", "category"],
        pose: ["poses", "pose"],
        workflow: ["workflows", "workflow"],
      }[kind] || [`${kind}s`, kind];
      const wrapped = wrapperKeys.map((key) => raw[key]).find((value) => Array.isArray(value));
      if (wrapped !== undefined) raw = wrapped;
      else if (raw.data && typeof raw.data === "object") return normalizeCollection(raw.data, kind);
    }
    let items;
    if (Array.isArray(raw)) {
      items = raw;
    } else if (typeof raw === "object") {
      items = Object.entries(raw).map(([id, value]) => {
        if (value && typeof value === "object") return { id, ...value };
        return { id, name: String(value) };
      });
    } else {
      return [];
    }

    return items.map((item, index) => {
      const clone = { ...item };
      clone.id = firstDefined(item.id, item[`${kind}_id`], item.key, index + 1);
      clone.name = firstDefined(item.name, item.display_name, item.label, String(clone.id));
      return clone;
    });
  }

  function normalizeHealth(value) {
    if (value && typeof value === "object") {
      const bool = asBoolean(firstDefined(value.ready, value.healthy, value.connected, value.fresh, value.ok));
      const label = firstDefined(value.label, value.message, value.state, value.status);
      if (bool === true) return { className: "ok", label: label || "정상", bool: true };
      if (bool === false) return { className: "bad", label: label || "오류", bool: false };
      if (String(label || "").toLowerCase().includes("warn")) return { className: "warn", label: String(label), bool: false };
      return { className: "unknown", label: label || "확인 중", bool: undefined };
    }
    const bool = asBoolean(value);
    if (bool === true) return { className: "ok", label: "정상", bool: true };
    if (bool === false) return { className: "bad", label: "오류", bool: false };
    if (typeof value === "string" && value) return { className: "unknown", label: value, bool: undefined };
    return { className: "unknown", label: "확인 중", bool: undefined };
  }

  function readinessBoolean(value) {
    if (value && typeof value === "object") {
      return asBoolean(firstDefined(value.ready, value.ok, value.healthy));
    }
    return asBoolean(value);
  }

  function getDeviceValues(snapshot) {
    const readiness = snapshot?.readiness || snapshot?.health || snapshot?.devices || {};
    const cameras = readiness.cameras || snapshot?.cameras || {};
    return {
      front: firstDefined(readiness.front_camera, readiness.front, cameras.front?.health, cameras.front?.ready, cameras.front),
      rear: firstDefined(readiness.rear_camera, readiness.rear, cameras.rear?.health, cameras.rear?.ready, cameras.rear),
      odom: firstDefined(readiness.odom, readiness.odometry, snapshot?.odom?.health, snapshot?.odom?.ready),
      arm: firstDefined(readiness.arm, snapshot?.arm?.health, snapshot?.arm?.ready, snapshot?.arm_online),
      gripper: firstDefined(readiness.gripper, snapshot?.gripper?.health, snapshot?.gripper?.ready, snapshot?.gripper_online),
      watchdog: firstDefined(readiness.watchdog, snapshot?.watchdog?.health, snapshot?.watchdog?.ready, snapshot?.watchdog_online),
    };
  }

  function stateClass(state) {
    if (ERROR_STATES.has(state)) return "state-error";
    if (ACTIVE_STATES.has(state)) return "state-active";
    if (READY_STATES.has(state)) return "state-ready";
    if (state === "UNKNOWN" || state === "OFFLINE") return "state-unknown";
    return "state-warning";
  }

  function ingestSnapshot(snapshot) {
    app.snapshot = snapshot || {};
    const configuredPollSeconds = finiteNumber(snapshot?.poll_interval_sec);
    if (configuredPollSeconds !== null) {
      app.pollIntervalMs = clamp(configuredPollSeconds * 1000, 100, 10000);
    }
    const stateValue = firstDefined(snapshot?.state, snapshot?.current_state, snapshot?.runtime?.state, snapshot?.status?.state, "UNKNOWN");
    app.currentState = typeof stateValue === "string" ? stateValue.toUpperCase() : String(stateValue?.name || "UNKNOWN").toUpperCase();

    const run = snapshot?.run || snapshot?.execution || snapshot?.current_run || {};
    const explicitActive = asBoolean(firstDefined(snapshot?.active, snapshot?.run_active, run?.active, run?.running));
    app.active = explicitActive !== undefined
      ? explicitActive
      : ACTIVE_STATES.has(app.currentState) || (run?.status && ["RUNNING", "ACTIVE", "PAUSED"].includes(String(run.status).toUpperCase()));

    const errorValue = firstDefined(snapshot?.error_latched, snapshot?.error?.latched, snapshot?.fault_latched);
    app.errorLatched = asBoolean(errorValue) === true || ERROR_STATES.has(app.currentState);
    app.waitingReturn = asBoolean(firstDefined(snapshot?.waiting_return_confirm, run?.waiting_return_confirm)) === true || ["WAIT_RETURN_CONFIRM", "WAITING_RETURN_CONFIRM"].includes(app.currentState);
    app.checkpoint = firstDefined(snapshot?.checkpoint, snapshot?.state?.checkpoint, null);
    app.workflowProgress = firstDefined(snapshot?.workflow_progress, null);
    app.workflowRecovery = firstDefined(snapshot?.workflow_recovery, snapshot?.state?.workflow_recovery, null);
    const progressWorkflowId = firstDefined(
      app.workflowRecovery?.workflow_id,
      app.workflowProgress?.workflow_id,
    );
    const progressStatus = String(app.workflowProgress?.status || "").toLowerCase();
    if (
      progressWorkflowId !== undefined && progressWorkflowId !== null &&
      (
        app.selectedWorkflowId === null ||
        app.workflowRecovery ||
        ["starting", "running", "paused"].includes(progressStatus)
      )
    ) {
      app.selectedWorkflowId = String(progressWorkflowId);
    }
    if (app.workflowRecovery) {
      app.selectedWorkflowStep = Math.trunc(
        finiteNumber(app.workflowRecovery.step_number) || 0
      ) || null;
    }

    const readiness = snapshot?.readiness || {};
    const devices = getDeviceValues(snapshot);
    const deviceHealth = Object.values(devices).map(normalizeHealth);
    const explicitReady = asBoolean(firstDefined(snapshot?.motion_ready, snapshot?.readiness?.motion_ready, snapshot?.ready_to_move));
    if (explicitReady !== undefined) {
      app.motionReady = explicitReady;
    } else {
      const known = deviceHealth.filter((health) => health.bool !== undefined);
      app.motionReady = known.length >= 4 && known.every((health) => health.bool);
    }
    const readinessWithFallback = (name) => {
      const explicit = readinessBoolean(readiness[name]);
      return explicit === undefined ? app.motionReady : explicit;
    };
    app.driveReady = readinessWithFallback("drive_ready");
    app.workflowReady = readinessWithFallback("workflow_ready");
    app.turnReady = readinessWithFallback("turn_ready");
    const configReady = readinessBoolean(readiness.config);
    const armStatusReady = readinessBoolean(readiness.arm_status_service);
    app.poseReady = configReady !== undefined || armStatusReady !== undefined
      ? (configReady ?? true) && (armStatusReady ?? true)
      : readinessWithFallback("pose_ready");
    app.configReady = configReady ?? app.motionReady;
    app.armReady = readinessBoolean(readiness.arm) ?? normalizeHealth(devices.arm).bool ?? app.poseReady;
    app.gripperReady = readinessBoolean(readiness.gripper) ?? normalizeHealth(devices.gripper).bool ?? app.poseReady;

    const config = snapshot?.config || {};
    const categoryRaw = firstDefined(snapshot?.categories, snapshot?.library?.categories, config?.categories);
    const poseRaw = firstDefined(snapshot?.poses, snapshot?.library?.poses, config?.poses);
    const workflowRaw = firstDefined(snapshot?.workflows, snapshot?.library?.workflows, config?.workflows);
    if (categoryRaw !== undefined) app.categories = normalizeCollection(categoryRaw, "category");
    if (poseRaw !== undefined) app.poses = normalizeCollection(poseRaw, "pose");
    if (workflowRaw !== undefined) app.workflows = normalizeCollection(workflowRaw, "workflow");

    extractJointFeedback(snapshot);
    ingestServerEvents(snapshot?.events || snapshot?.recent_events || []);
    renderSnapshot();
    renderLibrary();
  }

  function refreshCameraPreviews() {
    const cacheKey = Date.now();
    ["front", "rear"].forEach((camera) => {
      const image = $(`#${camera}CameraPreview`);
      if (!image) return;
      const figure = image.closest("figure");
      const status = $(`#${camera}CameraStatus`);
      image.onerror = () => {
        figure?.classList.remove("camera-available");
        figure?.classList.add("camera-unavailable");
        image.removeAttribute("src");
        if (status) status.textContent = "영상 없음 · 카메라 stale/unavailable";
      };
      image.onload = () => {
        figure?.classList.remove("camera-unavailable");
        figure?.classList.add("camera-available");
        if (status) status.textContent = "영상 수신 중";
      };
      image.src = `${API_ROOT}/camera/${camera}.jpg?t=${cacheKey}`;
    });
  }

  function markCameraPreviewsUnavailable(message) {
    ["front", "rear"].forEach((camera) => {
      const image = $(`#${camera}CameraPreview`);
      const figure = image?.closest("figure");
      figure?.classList.remove("camera-available");
      figure?.classList.add("camera-unavailable");
      const status = $(`#${camera}CameraStatus`);
      if (status) status.textContent = message;
    });
  }

  function extractJointFeedback(snapshot) {
    const arm = snapshot?.arm || {};
    const gripper = snapshot?.gripper || {};
    const degrees = firstDefined(arm.feedback_deg, arm.positions_deg, snapshot?.arm_feedback_deg, snapshot?.joint_states_deg);
    const radians = firstDefined(arm.feedback_rad, arm.positions_rad, snapshot?.arm_feedback_rad);
    app.armFeedbackDeg = jointVector(degrees, ARM_JOINT_NAMES);
    if (!app.armFeedbackDeg.length && radians) {
      app.armFeedbackDeg = jointVector(radians, ARM_JOINT_NAMES).map((value) => value * 180 / Math.PI);
    }

    const gripperDegrees = firstDefined(gripper.feedback_deg, gripper.positions_deg, snapshot?.gripper_feedback_deg);
    const gripperRadians = firstDefined(gripper.feedback_rad, gripper.positions_rad, snapshot?.gripper_feedback_rad);
    app.gripperFeedbackDeg = jointVector(gripperDegrees, GRIPPER_JOINT_NAMES);
    if (!app.gripperFeedbackDeg.length && gripperRadians) {
      app.gripperFeedbackDeg = jointVector(gripperRadians, GRIPPER_JOINT_NAMES).map((value) => value * 180 / Math.PI);
    }
  }

  function jointVector(raw, names) {
    if (Array.isArray(raw)) return raw.slice(0, names.length).map(Number).filter(Number.isFinite);
    if (raw && typeof raw === "object") {
      const result = names.map((name) => finiteNumber(raw[name]));
      return result.every((value) => value !== null) ? result : [];
    }
    return [];
  }

  function ingestServerEvents(events) {
    if (!Array.isArray(events)) return;
    events.forEach((event) => {
      const message = firstDefined(event.message, event.text, event.detail);
      if (!message) return;
      const timestamp = firstDefined(event.timestamp, event.time, new Date().toISOString());
      const key = String(firstDefined(event.id, `${timestamp}:${message}`));
      if (app.serverEventKeys.has(key)) return;
      app.serverEventKeys.add(key);
      const rawLevel = String(firstDefined(event.level, event.severity, "info")).toLowerCase();
      const level = ["success", "warning", "error"].includes(rawLevel) ? rawLevel : "info";
      addEvent(message, level, timestamp, "NODE");
    });
  }

  function renderRouteRobot(snapshot, run, rawPhase) {
    const robot = $("#routeRobot");
    const body = $("#routeRobotBody");
    const statusElement = $("#routeMotionText");
    if (!robot || !body || !statusElement) return;

    const visual = app.routeVisual;
    const points = {
      pickup: { x: 89, y: 50 },
      dropoff: { x: 11, y: 50 },
    };
    const phase = String(rawPhase || "").toLowerCase();
    const explicitDirection = String(firstDefined(
      run?.route,
      snapshot?.state?.route_direction,
      snapshot?.route?.name,
      "",
    )).toLowerCase();
    if (["outbound", "return"].includes(explicitDirection)) {
      visual.direction = explicitDirection;
    }
    if (phase !== visual.phase) {
      visual.phase = phase;
      visual.phaseStartedAt = performance.now();
      visual.initialDistance = null;
      visual.progress = 0;
    }

    const metrics = snapshot?.vision || snapshot?.metrics || {};
    const detection = metrics.detection || {};
    const detectedId = finiteNumber(firstDefined(
      detection.id,
      detection.marker_id,
      metrics.detected_id,
      snapshot?.metrics?.detected_marker_id,
    ));
    const distance = finiteNumber(firstDefined(
      detection.distance_m,
      metrics.distance_m,
      snapshot?.metrics?.distance_m,
    ));
    const markers = snapshot?.markers || snapshot?.config?.markers || {};
    let start = null;
    let end = null;
    let markerName = null;
    let status = visual.status;
    let heading = visual.heading;
    let mode = "idle";

    if (phase.includes("verify_pickup")) {
      Object.assign(visual, points.pickup);
      visual.destination = "pickup";
      heading = 180;
      status = "A 집기 · 출발 준비";
    } else if (phase.includes("verify_dropoff")) {
      Object.assign(visual, points.dropoff);
      visual.destination = "dropoff";
      heading = 180;
      status = "C 놓기 · 복귀 준비";
    } else if (phase.includes("align_dropoff")) {
      start = points.pickup;
      end = points.dropoff;
      markerName = "dropoff";
      heading = 180;
      status = "A → C · 마커 3 직선 전진·거리 정지";
      mode = "moving";
    } else if (phase.includes("align_pickup")) {
      start = points.dropoff;
      end = points.pickup;
      markerName = "pickup";
      heading = 180;
      status = "C → A · 마커 1 직선 후진·거리 정지";
      mode = "moving";
    } else if (app.waitingReturn) {
      Object.assign(visual, points.dropoff);
      visual.destination = "dropoff";
      heading = 180;
      status = "C 놓기 · 복귀 확인 대기";
    } else if (phase === "complete") {
      const destination = visual.direction === "return" ? "pickup" : "dropoff";
      Object.assign(visual, points[destination]);
      visual.destination = destination;
      heading = 180;
      status = destination === "pickup" ? "A 집기 · 도착" : "C 놓기 · 도착";
    } else if (phase === "emergency_stop" || phase === "stopped") {
      status = visual.status.startsWith("정지 ·")
        ? visual.status
        : `정지 · ${visual.status}`;
      mode = "stopped";
    }

    if (start && end && markerName) {
      const marker = markers[markerName] || {};
      const markerId = finiteNumber(marker.id);
      const targetDistance = finiteNumber(marker.target_distance_m);
      const matchingDetection = (
        distance !== null && markerId !== null && detectedId === markerId
      );
      let progress = visual.progress;
      if (matchingDetection && targetDistance !== null) {
        if (visual.initialDistance === null || distance > visual.initialDistance) {
          visual.initialDistance = distance;
        }
        const span = visual.initialDistance - targetDistance;
        if (span > 0.03) {
          progress = Math.max(progress, clamp(
            (visual.initialDistance - distance) / span,
            0,
            0.98,
          ));
        } else if (distance <= targetDistance + 0.05) {
          progress = Math.max(progress, 0.98);
        }
      }
      visual.progress = progress;
      visual.x = start.x + (end.x - start.x) * progress;
      visual.y = start.y + (end.y - start.y) * progress;
      visual.destination = markerName;
      if (!matchingDetection) mode = "searching";
    }

    if (app.errorLatched) {
      status = status.startsWith("오류 ·") ? status : `오류 · ${status}`;
      mode = "stopped";
    }

    visual.heading = heading;
    visual.status = status;
    robot.style.left = `${visual.x}%`;
    robot.style.top = `${visual.y}%`;
    robot.className = `route-robot ${mode}`;
    body.style.transform = `rotate(${heading}deg)`;
    robot.setAttribute("aria-label", `로봇 주행 표시 ${status}`);
    statusElement.textContent = status;
    statusElement.className = `route-motion-text ${mode}`;
  }

  function boardFieldText(value, { hex = false } = {}) {
    if (value === true) return "예";
    if (value === false) return "아니오";
    if (value === null || value === undefined || value === "") return "—";
    if (hex && Number.isFinite(Number(value))) {
      return `0x${Number(value).toString(16).toUpperCase().padStart(2, "0")}`;
    }
    return String(value);
  }

  function boardAssessment(board) {
    const fields = board?.fields || {};
    const notes = board?.notes || [];
    const state = String(fields.state || "").toUpperCase();
    const error = String(fields.error || "").toUpperCase();
    const stale = fields.stale === true;
    const noStatus = notes.some((note) => String(note).toLowerCase().includes("no status"));
    const ready = finiteNumber(firstDefined(fields.ready_mask, fields.ready));
    const clearErrors = new Set(["", "0", "0X00", "NONE", "ERR_NONE", "OK", "NO_ERROR", "NORMAL"]);
    if (noStatus || stale) return { level: "danger", action: "상태 통신 확인" };
    if (state === "ESTOP" || error.includes("ESTOP")) {
      return { level: "danger", action: "ESTOP · Enable 필요" };
    }
    if (["ERROR", "FAULT"].includes(state) || !clearErrors.has(error)) {
      return { level: "danger", action: "Clear 필요" };
    }
    if (fields.enabled !== true) return { level: "warning", action: "Enable 필요" };
    if (fields.position_valid === false || ready === 0) {
      return { level: "warning", action: "Home 필요" };
    }
    if (state === "MOVING" || state === "HOMING") {
      return { level: "warning", action: "동작 진행 중" };
    }
    return { level: "success", action: "동작 가능" };
  }

  function boardMetric(label, value) {
    const item = document.createElement("div");
    const term = document.createElement("dt");
    const description = document.createElement("dd");
    term.textContent = label;
    description.textContent = value;
    item.append(term, description);
    return item;
  }

  function renderArmBoardStatus(snapshot) {
    const status = snapshot?.arm_board_status || {};
    const boards = Array.isArray(status.boards) ? status.boards : [];
    const grid = $("#armBoardGrid");
    const age = $("#armBoardStatusAge");
    const recommendation = $("#armBoardRecommendation");
    if (!grid || !age || !recommendation) return;

    const ageSeconds = finiteNumber(status.age_sec);
    age.textContent = status.available
      ? `자동 갱신 · ${ageSeconds === null ? "방금" : `${ageSeconds.toFixed(1)}초 전`}`
      : String(status.message || "상태 서비스 연결 대기 중");
    grid.replaceChildren();
    if (!boards.length) {
      const empty = document.createElement("div");
      empty.className = "board-status-empty";
      empty.textContent = "보드 상태가 없습니다. 상태 새로고침을 눌러 확인하세요.";
      grid.append(empty);
      recommendation.textContent = "상태 서비스와 CAN 보드 연결을 확인하세요.";
      recommendation.className = "board-recommendation danger";
      return;
    }

    const assessments = [];
    boards.forEach((board) => {
      const fields = board.fields || {};
      const assessment = boardAssessment(board);
      assessments.push(assessment);
      const card = document.createElement("article");
      card.className = `board-status-card ${assessment.level}`;
      const title = document.createElement("div");
      title.className = "board-status-title";
      const name = document.createElement("strong");
      name.textContent = `${board.controller || "arm"} · Board ${board.board_id ?? "—"}`;
      const action = document.createElement("span");
      action.textContent = assessment.action;
      title.append(name, action);
      const metrics = document.createElement("dl");
      metrics.className = "board-status-metrics";
      metrics.append(
        boardMetric("State", boardFieldText(fields.state)),
        boardMetric("Enabled", boardFieldText(fields.enabled)),
        boardMetric("Home/Ready", boardFieldText(firstDefined(fields.ready_mask, fields.ready), { hex: true })),
        boardMetric("Error", boardFieldText(fields.error)),
        boardMetric("Limit bits", boardFieldText(fields.fault, { hex: true })),
        boardMetric("Status age", fields.age_ms === undefined ? "—" : `${formatNumber(fields.age_ms, 0)} ms`),
      );
      card.append(title, metrics);
      grid.append(card);
    });

    const danger = assessments.find((item) => item.level === "danger");
    const warning = assessments.find((item) => item.level === "warning");
    const controllerBlocked = (status.controllers || []).some(
      (controller) => controller.accept_traj === false,
    );
    if (danger) {
      recommendation.textContent = danger.action;
      recommendation.className = "board-recommendation danger";
    } else if (warning) {
      recommendation.textContent = warning.action;
      recommendation.className = "board-recommendation warning";
    } else if (controllerBlocked) {
      recommendation.textContent = "보드는 정상이나 trajectory 수락 상태를 확인하세요.";
      recommendation.className = "board-recommendation warning";
    } else {
      recommendation.textContent = "모든 보드 정상 · 로봇팔 동작 가능";
      recommendation.className = "board-recommendation success";
    }
  }

  function workflowById(workflowId) {
    return app.workflows.find(
      (workflow) => String(workflow.id) === String(workflowId)
    );
  }

  function completedWorkflowStepNumbers(progress, total, skippedSteps) {
    const explicit = Array.isArray(progress?.completed_step_numbers)
      ? progress.completed_step_numbers
        .map((value) => Math.trunc(finiteNumber(value) || 0))
        .filter((value) => value >= 1 && value <= total)
      : [];
    if (explicit.length) return new Set(explicit);
    const completedCount = Math.max(
      0,
      Math.trunc(finiteNumber(progress?.completed_steps) || 0),
    );
    const inferred = new Set();
    for (let stepNumber = 1; stepNumber <= total && inferred.size < completedCount; stepNumber += 1) {
      if (!skippedSteps.has(stepNumber)) inferred.add(stepNumber);
    }
    return inferred;
  }

  function selectWorkflowForStages(workflow) {
    app.selectedWorkflowId = String(workflow.id);
    app.selectedWorkflowStep = null;
    renderWorkflowTable();
    renderWorkflowProgress();
    applyControlLocks();
  }

  function renderWorkflowProgress() {
    const panel = $("#workflowProgressPanel");
    if (!panel) return;
    const recoveryActions = $("#workflowRecoveryActions");
    recoveryActions.hidden = false;
    const rawProgress = app.workflowProgress;
    const rawRecovery = app.workflowRecovery;
    const fallbackWorkflowId = firstDefined(
      rawRecovery?.workflow_id,
      rawProgress?.workflow_id,
    );
    const selectedId = firstDefined(app.selectedWorkflowId, fallbackWorkflowId);
    const workflow = workflowById(selectedId);
    const progress = (
      rawProgress && String(rawProgress.workflow_id) === String(selectedId)
    ) ? rawProgress : null;
    const recovery = (
      rawRecovery && String(rawRecovery.workflow_id) === String(selectedId)
    ) ? rawRecovery : null;
    const visible = Boolean(workflow || progress);
    panel.hidden = false;
    if (!visible) {
      $("#workflowProgressTitle").textContent = "워크플로우 진행 단계";
      const statusBadge = $("#workflowProgressStatus");
      statusBadge.textContent = "선택 대기";
      statusBadge.className = "workflow-progress-status status-ready";
      const track = $(".workflow-progress-track", panel);
      track.setAttribute("aria-valuenow", "0");
      $("#workflowProgressBar").style.width = "0%";
      $("#workflowCurrentStep").textContent = app.workflows.length
        ? "워크플로우 표에서 확인할 항목을 선택하세요."
        : "저장된 워크플로우가 없습니다.";
      $("#workflowProgressSummary").textContent = "성공 0 · 건너뜀 0 · 전체 0";
      const executionList = $("#workflowExecutionSteps");
      executionList.replaceChildren();
      const empty = document.createElement("li");
      empty.className = "workflow-execution-empty";
      empty.textContent = app.workflows.length
        ? "워크플로우 표에서 확인할 항목을 선택하세요."
        : "워크플로우를 만들면 단계가 여기에 순서대로 표시됩니다.";
      executionList.append(empty);
      $("#workflowRecoveryMessage").hidden = true;
      $("#workflowStepSelectionHint").textContent = "실행 전 단계는 회색, 완료는 초록색, 실패는 빨간색으로 표시됩니다.";
      return;
    }

    const steps = normalizeWorkflowSteps(firstDefined(workflow?.steps, workflow?.workflow, []));
    const status = String(firstDefined(
      progress?.status,
      recovery ? "paused" : "ready",
    )).toLowerCase();
    const statusLabels = {
      ready: "실행 전",
      starting: "시작 중",
      running: "실행 중",
      paused: "단계 일시정지",
      stopped: "사용자 중지",
      completed: "완료",
      completed_with_skips: "건너뜀 포함 완료",
      aborted: "종료됨",
      failed_safety: "안전 오류",
    };
    const total = Math.max(
      0,
      Math.trunc(finiteNumber(progress?.total_steps) || steps.length),
    );
    const skippedSteps = new Set(
      (Array.isArray(progress?.skipped_steps) ? progress.skipped_steps : [])
        .map((value) => Math.trunc(finiteNumber(value) || 0))
        .filter((value) => value >= 1 && value <= total),
    );
    const completedSteps = completedWorkflowStepNumbers(
      progress,
      total,
      skippedSteps,
    );
    const current = Math.max(0, Math.trunc(finiteNumber(progress?.current_step) || 0));
    const processed = Math.min(total, completedSteps.size + skippedSteps.size);
    const finished = ["completed", "completed_with_skips"].includes(status);
    const percent = total > 0 ? clamp((finished ? total : processed) / total * 100, 0, 100) : 0;
    const workflowId = firstDefined(workflow?.id, progress?.workflow_id, recovery?.workflow_id, "—");
    const workflowName = firstDefined(workflow?.name, progress?.workflow_name, recovery?.workflow_name, "워크플로우");
    const stepLabel = firstDefined(progress?.current_step_label, recovery?.step_label, "단계 준비 중");

    $("#workflowProgressTitle").textContent = `#${workflowId} · ${workflowName}`;
    const statusBadge = $("#workflowProgressStatus");
    statusBadge.textContent = statusLabels[status] || status;
    statusBadge.className = `workflow-progress-status status-${status}`;
    const track = $(".workflow-progress-track", panel);
    track.setAttribute("aria-valuenow", String(Math.round(percent)));
    $("#workflowProgressBar").style.width = `${percent}%`;
    $("#workflowCurrentStep").textContent = !progress
      ? "아직 실행하지 않았습니다. 모든 단계는 대기 상태입니다."
      : finished
      ? "모든 단계 처리가 끝났습니다."
      : current > 0 && total > 0
        ? `현재 ${current}/${total} · ${stepLabel}`
        : "워크플로우 시작 준비 중";
    $("#workflowProgressSummary").textContent = `성공 ${completedSteps.size} · 건너뜀 ${skippedSteps.size} · 전체 ${total}`;

    const failedStep = Math.trunc(
      finiteNumber(recovery?.step_number) ||
      (status === "failed_safety" ? current : 0)
    );
    if (recovery && (!app.selectedWorkflowStep || app.selectedWorkflowStep > total)) {
      app.selectedWorkflowStep = failedStep || null;
    }
    const executionList = $("#workflowExecutionSteps");
    executionList.replaceChildren();
    if (!steps.length) {
      const empty = document.createElement("li");
      empty.className = "workflow-execution-empty";
      empty.textContent = "이 워크플로우에는 표시할 단계가 없습니다.";
      executionList.append(empty);
    } else {
      steps.forEach((step, index) => {
        const stepNumber = index + 1;
        let stepState = "pending";
        if (failedStep === stepNumber) stepState = "failed";
        else if (completedSteps.has(stepNumber)) stepState = "completed";
        else if (skippedSteps.has(stepNumber)) stepState = "skipped";
        else if (["starting", "running"].includes(status) && current === stepNumber) stepState = "running";
        const stateLabels = {
          pending: "대기",
          running: "실행 중",
          completed: "완료",
          failed: "실패",
          skipped: "건너뜀",
        };
        const item = document.createElement("li");
        item.className = `workflow-execution-step ${stepState}`;
        if (recovery && app.selectedWorkflowStep === stepNumber) {
          item.classList.add("selected");
        }
        const button = document.createElement("button");
        button.type = "button";
        button.className = "workflow-stage-button";
        button.disabled = !recovery || app.active;
        button.setAttribute("aria-pressed", String(
          recovery && app.selectedWorkflowStep === stepNumber
        ));
        if (stepState === "running") button.setAttribute("aria-current", "step");
        const number = document.createElement("span");
        number.className = "workflow-stage-number";
        number.textContent = String(stepNumber).padStart(2, "0");
        const copy = document.createElement("span");
        copy.className = "workflow-stage-copy";
        const description = describeStep(step);
        const title = document.createElement("strong");
        title.textContent = description.title;
        const detail = document.createElement("small");
        detail.textContent = description.detail;
        copy.append(title, detail);
        const stateText = document.createElement("span");
        stateText.className = "workflow-stage-state";
        stateText.textContent = stateLabels[stepState];
        button.append(number, copy, stateText);
        button.addEventListener("click", () => {
          app.selectedWorkflowStep = stepNumber;
          renderWorkflowProgress();
          applyControlLocks();
        });
        item.append(button);
        executionList.append(item);
      });
    }

    const recoveryMessage = $("#workflowRecoveryMessage");
    recoveryMessage.hidden = !recovery;
    if (recovery) {
      const routeResume = asBoolean(recovery.can_resume_route_destination) === true
        ? " 재시도하면 현재 목적지 정렬부터 계속합니다."
        : "";
      recoveryMessage.textContent = `단계 실패: ${firstDefined(recovery.error, progress?.error, "원인 미상")}.${routeResume}`;
    }
    const selectionHint = $("#workflowStepSelectionHint");
    selectionHint.textContent = recovery
      ? app.selectedWorkflowStep
        ? `${app.selectedWorkflowStep}단계가 선택되었습니다. 재시도, 다음 단계 또는 선택 단계 실행을 누르세요.`
        : "다시 시작할 단계를 선택하세요."
      : "실행 전 단계는 회색, 완료는 초록색, 실패는 빨간색으로 표시됩니다.";
  }

  function renderSnapshot() {
    const snapshot = app.snapshot || {};
    const run = snapshot.run || snapshot.execution || snapshot.current_run || {};
    const error = snapshot.error || snapshot.last_error || {};
    const errorText = typeof error === "string" ? error : firstDefined(error.message, error.detail, snapshot.error_message, "오류 원인을 확인한 뒤 해제하세요.");

    const displayState = app.workflowRecovery ? "WORKFLOW_PAUSED" : app.currentState;
    $("#stateBadge").textContent = displayState;
    $("#stateBadge").className = `state-badge ${stateClass(displayState)}`;
    const workflowStatus = String(app.workflowProgress?.status || "").toLowerCase();
    const workflowVisibleAsRun = ["starting", "running", "paused"].includes(workflowStatus);
    $("#runText").textContent = String(firstDefined(
      run.name,
      run.workflow_name,
      run.id,
      snapshot.run_id,
      workflowVisibleAsRun ? `#${app.workflowProgress.workflow_id} · ${app.workflowProgress.workflow_name}` : undefined,
      app.active ? "실행 중" : "없음",
    ));
    $("#routeText").textContent = String(firstDefined(snapshot.route?.name, snapshot.route_name, run.route, "—"));
    const workflowStepText = workflowVisibleAsRun && app.workflowProgress?.current_step
      ? `${app.workflowProgress.current_step}/${app.workflowProgress.total_steps} · ${app.workflowProgress.current_step_label}`
      : undefined;
    const currentPhase = firstDefined(workflowStepText, run.step_name, run.current_step, snapshot.step, snapshot.current_step, "—");
    $("#stepText").textContent = String(currentPhase);
    $("#lastUpdateText").textContent = formatTime(firstDefined(snapshot.timestamp, snapshot.updated_at, new Date()));
    renderRouteRobot(snapshot, run, currentPhase);
    renderArmBoardStatus(snapshot);
    renderWorkflowProgress();

    $("#errorPanel").hidden = !app.errorLatched;
    $("#errorText").textContent = String(errorText);
    $("#checkpointActions").hidden = !app.checkpoint || Boolean(app.workflowRecovery);

    const devices = getDeviceValues(snapshot);
    updateHealth("#frontHealth", devices.front);
    updateHealth("#rearHealth", devices.rear);
    updateHealth("#odomHealth", devices.odom);
    updateHealth("#armHealth", devices.arm);
    updateHealth("#gripperHealth", devices.gripper);
    updateHealth("#watchdogHealth", devices.watchdog);

    const vision = snapshot.vision || snapshot.marker || snapshot.aruco || {};
    const target = vision.target || snapshot.target_marker || {};
    const targetId = firstDefined(target.id, vision.target_id, snapshot.target_marker_id);
    const idleMarker = targetId === undefined
      ? latestVisibleMarker(snapshot)
      : null;
    const detection = idleMarker || vision.detection || vision.detected || snapshot.detected_marker || {};
    const activeCamera = firstDefined(
      idleMarker?.camera_name,
      vision.active_camera,
      snapshot.active_camera,
      snapshot.camera,
      "—",
    );
    const detectedId = firstDefined(
      detection.id,
      detection.marker_id,
      vision.detected_id,
      snapshot.detected_marker_id,
    );
    const detected = asBoolean(firstDefined(detection.visible, detection.detected, vision.detected, detectedId !== undefined)) === true;
    const distance = firstDefined(detection.distance_m, vision.distance_m, snapshot.marker_distance_m);
    const lateral = firstDefined(detection.lateral_m, detection.x_m, vision.lateral_m, snapshot.marker_lateral_m);
    const yawRad = finiteNumber(detection.yaw_rad);
    const yaw = firstDefined(
      detection.yaw_error_deg,
      detection.yaw_deg,
      yawRad === null ? undefined : yawRad * 180 / Math.PI,
      vision.yaw_error_deg,
      snapshot.marker_yaw_deg,
    );
    const turn = snapshot.turn || snapshot.turn_progress || {};
    const turnActual = firstDefined(turn.actual_deg, turn.current_deg, snapshot.turn_actual_deg);
    const turnTarget = firstDefined(turn.target_deg, snapshot.turn_target_deg);

    $("#activeCameraBadge").textContent = `카메라 ${String(activeCamera).toUpperCase()}`;
    $("#activeCameraText").textContent = cameraLabel(activeCamera);
    $("#markerIdText").textContent = `${targetId ?? "—"} / ${detectedId ?? "—"}`;
    $("#distanceText").textContent = `${formatNumber(distance, 3)} m`;
    $("#lateralText").textContent = `${formatNumber(lateral, 3)} m`;
    $("#yawText").textContent = formatAngle(yaw, 1);
    $("#turnText").textContent = `${formatAngle(turnActual, 1)} / ${formatAngle(turnTarget, 1)}`;
    $("#markerEmptyText").textContent = targetId === undefined
      ? "카메라 마커 대기 중"
      : "목표 마커 대기 중";
    renderMarkerIndicator(detection, detected, detectedId, lateral, distance);
    const targetStatus = String(firstDefined(vision.target_status, "idle"));
    const controlReason = String(firstDefined(snapshot.metrics?.control_reason, ""));
    const acquireCreeping = controlReason.startsWith("acquire_creep:");
    const diagnosticMessages = {
      idle: "주행 대기 중 · 현재 보이는 마커를 표시합니다.",
      tracking: `${cameraLabel(activeCamera)} 카메라에서 목표 ID ${targetId ?? "—"} 추적 중`,
      pose_invalid: `${cameraLabel(activeCamera)}에서 ID ${targetId ?? "—"}는 보이지만 거리 자세 계산에 실패했습니다.`,
      wrong_camera: acquireCreeping
        ? `반대 카메라의 ID ${targetId ?? "—"}를 무시하고 목표 방향으로 저속 이동 중`
        : `목표 ID ${targetId ?? "—"}가 반대 카메라에 보여 무시 중입니다.`,
      wrong_id: acquireCreeping
        ? `${cameraLabel(activeCamera)}의 다른 ID를 무시하고 목표 ID ${targetId ?? "—"}를 찾으며 저속 이동 중`
        : `${cameraLabel(activeCamera)}에 다른 ID만 보입니다. 목표 ID ${targetId ?? "—"}를 보여주세요.`,
      not_visible: acquireCreeping
        ? `${cameraLabel(activeCamera)} 카메라의 목표 ID ${targetId ?? "—"}를 찾으며 저속 이동 중`
        : `${cameraLabel(activeCamera)} 카메라에서 목표 ID ${targetId ?? "—"} 대기 중`,
    };
    const markerDiagnostic = $("#markerTargetDiagnostic");
    markerDiagnostic.textContent = diagnosticMessages[targetStatus]
      || String(vision.target_status_message || "목표 마커 상태 확인 중");
    markerDiagnostic.className = `marker-target-diagnostic ${targetStatus}`;

    const configuredNudges = firstDefined(snapshot.turn_control?.manual_steps_deg, [1, 5])
      .map(Number)
      .filter((value) => Number.isFinite(value) && value > 0)
      .sort((a, b) => a - b);
    if (configuredNudges.length >= 2) {
      const values = [-configuredNudges[1], -configuredNudges[0], configuredNudges[0], configuredNudges[1]];
      $$("[data-turn-degrees]").forEach((button, index) => {
        button.dataset.turnDegrees = String(values[index]);
        button.textContent = formatAngle(values[index], 1);
      });
    }

    const markers = snapshot.markers || snapshot.config?.markers || {};
    const markerLabels = {
      pickup: "집기",
      dropoff: "놓기",
    };
    $$('[data-marker-test]').forEach((button) => {
      const name = button.dataset.markerTest;
      const markerId = firstDefined(markers[name]?.id, markers[name]?.marker_id, "—");
      button.textContent = `${markerLabels[name] || name} ${markerId} 시험`;
    });

    const configInfo = snapshot.config_info || snapshot.config || {};
    const revision = firstDefined(snapshot.revision, configInfo.revision);
    $("#revisionText").textContent = `rev ${revision ?? "—"}`;
    $("#configPathText").textContent = String(firstDefined(configInfo.path, "config/final_project_presentation2.json"));
    $("#schemaVersionText").textContent = String(firstDefined(configInfo.schema_version, snapshot.schema_version, "—"));
    $("#configRevisionText").textContent = String(revision ?? "—");
    const configValid = asBoolean(firstDefined(configInfo.valid, snapshot.config_valid));
    $("#configValidationText").textContent = configValid === true ? "정상" : configValid === false ? "오류" : "—";
    const restartRaw = firstDefined(configInfo.restart_required, snapshot.restart_required, []);
    const restartFields = Array.isArray(restartRaw)
      ? restartRaw.filter(Boolean).map(String)
      : restartRaw ? [String(restartRaw)] : [];
    $("#restartRequiredPanel").hidden = restartFields.length === 0;
    $("#restartRequiredText").textContent = restartFields.length
      ? `적용 대기 설정: ${restartFields.join(", ")} · 노드를 재시작하세요.`
      : "변경된 설정을 적용하려면 재시작하세요.";

    app.armFeedbackDeg.forEach((value, index) => {
      const output = $(`#armFeedback${index}`);
      if (output) output.textContent = `${value.toFixed(1)}°`;
    });
    applyControlLocks();
  }

  function cameraLabel(value) {
    const normalized = String(value || "").toLowerCase();
    if (normalized.includes("front") || normalized.includes("전방")) return "전방";
    if (normalized.includes("rear") || normalized.includes("back") || normalized.includes("후방")) return "후방";
    return value ? String(value) : "—";
  }

  function latestVisibleMarker(snapshot) {
    const groups = snapshot?.visible_markers;
    if (!groups || typeof groups !== "object") return null;
    const candidates = [];
    Object.entries(groups).forEach(([fallbackCamera, markers]) => {
      if (!Array.isArray(markers)) return;
      markers.forEach((marker) => {
        if (!marker || typeof marker !== "object") return;
        const markerId = finiteNumber(firstDefined(marker.marker_id, marker.id));
        if (markerId === null) return;
        candidates.push({
          ...marker,
          marker_id: Math.trunc(markerId),
          camera_name: String(firstDefined(marker.camera_name, fallbackCamera)),
          _timestamp: finiteNumber(firstDefined(marker.timestamp_sec, marker.timestamp)) ?? -Infinity,
        });
      });
    });
    candidates.sort((left, right) => right._timestamp - left._timestamp);
    return candidates[0] || null;
  }

  function updateHealth(selector, value) {
    const target = $(selector);
    const health = normalizeHealth(value);
    target.className = `health ${health.className}`;
    target.textContent = health.label;
  }

  function renderMarkerIndicator(detection, detected, detectedId, lateral, distance) {
    const indicator = $("#markerIndicator");
    const empty = $("#markerEmptyText");
    indicator.hidden = !detected;
    empty.hidden = detected;
    if (!detected) return;

    let x = finiteNumber(firstDefined(detection.center_x_normalized, detection.normalized_x));
    let y = finiteNumber(firstDefined(detection.center_y_normalized, detection.normalized_y));
    const centerX = finiteNumber(detection.center_x_px);
    const centerY = finiteNumber(detection.center_y_px);
    const width = finiteNumber(firstDefined(detection.image_width_px, detection.image_width));
    const height = finiteNumber(firstDefined(detection.image_height_px, detection.image_height));
    if (x === null && centerX !== null && width) x = centerX / width;
    if (y === null && centerY !== null && height) y = centerY / height;
    if (x === null) x = 0.5 + clamp(finiteNumber(lateral) || 0, -0.4, 0.4);
    if (y === null) y = 0.52 - clamp(((finiteNumber(distance) || 1) - 1) * 0.18, -0.25, 0.25);
    indicator.style.left = `${clamp(x, 0.1, 0.9) * 100}%`;
    indicator.style.top = `${clamp(y, 0.14, 0.86) * 100}%`;
    $("#markerIndicatorId").textContent = `ID ${detectedId ?? "—"}`;
  }

  function updateConnection(connected, message = "") {
    const changed = app.connected !== connected;
    app.connected = connected;
    const badge = $("#connectionBadge");
    badge.classList.remove("online", "offline", "degraded");
    badge.classList.add(connected ? "online" : "offline");
    $("#connectionText").textContent = connected ? "노드 연결됨" : "노드 연결 끊김";
    if (changed) {
      if (connected) addEvent("final_project_presentation2 노드에 연결되었습니다.", "success", new Date(), "LINK");
      else addEvent(message || "노드 연결이 끊겼습니다.", "error", new Date(), "LINK");
    }
    if (!connected) markCameraPreviewsUnavailable("노드 연결 끊김 · 영상 사용 불가");
    applyControlLocks();
  }

  function applyControlLocks() {
    const commonMotionLocked = !app.connected || app.active || app.errorLatched;
    const workflowRecoveryPending = Boolean(app.workflowRecovery);
    const configLocked = !app.connected || app.active || Boolean(app.checkpoint) || workflowRecoveryPending;
    const readiness = {
      motion: app.motionReady,
      drive: app.driveReady,
      pose: app.poseReady,
      workflow: app.workflowReady,
      turn: app.turnReady,
    };
    $$('[data-lock="motion"], [data-lock="drive"], [data-lock="pose"], [data-lock="workflow"], [data-lock="turn"]').forEach((control) => {
      const kind = control.dataset.lock;
      const blockedByCheckpoint = Boolean(app.checkpoint) && !control.hasAttribute("data-checkpoint-allowed");
      const blockedByWorkflowRecovery = workflowRecoveryPending && !control.hasAttribute("data-workflow-recovery-allowed");
      control.disabled = commonMotionLocked || !readiness[kind] || blockedByCheckpoint || blockedByWorkflowRecovery;
    });
    $$('[data-lock="checkpoint"]').forEach((control) => {
      control.disabled = !app.connected || app.active || !app.checkpoint;
    });
    $$('[data-lock="config"]').forEach((control) => {
      const blockedByCheckpoint = Boolean(app.checkpoint) && !control.hasAttribute("data-checkpoint-allowed");
      const blockedByWorkflowRecovery = workflowRecoveryPending && !control.hasAttribute("data-workflow-recovery-allowed");
      control.disabled = !app.connected || app.active || blockedByCheckpoint || blockedByWorkflowRecovery;
    });
    $$([
      "#categoryAddForm input", "#renameCategoryName",
      "#poseForm input", "#poseForm select",
      "#workflowForm input", "#workflowForm select",
    ].join(",")).forEach((control) => { control.disabled = configLocked; });
    $("#poseId").disabled = configLocked || Boolean($("#poseEditId").value);
    $("#workflowId").disabled = configLocked || Boolean($("#workflowEditId").value);
    $$('[data-lock="return-confirm"]').forEach((control) => {
      control.disabled = !app.connected || !app.waitingReturn || app.errorLatched || Boolean(app.checkpoint) || workflowRecoveryPending;
    });

    const driveLocked = commonMotionLocked || !app.driveReady || Boolean(app.checkpoint) || workflowRecoveryPending;
    const readyBadge = $("#motionReadyBadge");
    readyBadge.className = `ready-badge ${!driveLocked ? "ready" : "not-ready"}`;
    readyBadge.textContent = app.workflowRecovery
      ? "워크플로우 일시정지"
      : app.checkpoint
        ? "목적지 이동 체크포인트"
      : !driveLocked ? "주행 준비 완료" : app.active ? "실행 중" : app.errorLatched ? "오류 잠김" : !app.connected ? "연결 끊김" : "주행 준비 안 됨";

    const reason = $("#motionBlockReason");
    if (!app.connected) reason.textContent = "노드 연결이 없어 명령이 잠겼습니다.";
    else if (app.errorLatched) reason.textContent = "오류 원인을 해결하고 오류 해제를 눌러주세요.";
    else if (app.workflowRecovery) reason.textContent = "워크플로우가 실패 단계에서 일시정지되었습니다. 현재 단계 재시도, 다음 단계 진행 또는 워크플로우 종료를 선택하세요.";
    else if (app.checkpoint) reason.textContent = "체크포인트 복구 중입니다. 설정된 ± 미세 회전, 목적지 이동 계속, 체크포인트 취소, 전체 중지만 사용할 수 있습니다.";
    else if (app.waitingReturn) reason.textContent = "물건 놓기가 완료되었습니다. 복귀 확인 버튼을 누르면 원상복귀합니다.";
    else if (app.active) reason.textContent = "현재 실행이 끝날 때까지 다른 명령과 설정 변경이 잠깁니다.";
    else if (!app.driveReady) reason.textContent = "카메라·odom·watchdog 준비 상태를 확인하세요.";
    else reason.textContent = "주행 준비가 완료되었습니다. 경로가 비어 있는지 확인하세요.";

    const selectedCategory = selectedCategoryObject();
    $("#deleteCategory").disabled = configLocked || !selectedCategory || Boolean(selectedCategory.protected) || isUncategorized(selectedCategory);
    $("#renameCategory").disabled = configLocked || !selectedCategory || Boolean(selectedCategory.protected);
    $("#armDisable").disabled = !app.connected;
    $("#armEstop").disabled = !app.connected;
    $("#armStatusRefresh").disabled = !app.connected;
    updateActuatorDisabledState();
    updateTableControlLocks();
  }

  function updateActuatorDisabledState() {
    const configLocked = !app.connected || app.active || Boolean(app.checkpoint) || Boolean(app.workflowRecovery);
    const armEnabled = $("#armEnabled").checked;
    const gripperEnabled = $("#gripperEnabled").checked;
    $("#armFieldset").classList.toggle("inactive", !armEnabled);
    $("#gripperFieldset").classList.toggle("inactive", !gripperEnabled);
    $$("input:not(#armEnabled)", $("#armFieldset")).forEach((input) => { input.disabled = configLocked || !armEnabled; });
    $$("input:not(#gripperEnabled)", $("#gripperFieldset")).forEach((input) => { input.disabled = configLocked || !gripperEnabled; });
    const requiredControllersReady = (!armEnabled || app.armReady) && (!gripperEnabled || app.gripperReady);
    $("#testPose").disabled = !app.connected || app.active || app.errorLatched || Boolean(app.checkpoint) || Boolean(app.workflowRecovery) || !app.poseReady || !requiredControllersReady;
    $("#manualArmExecute").disabled = !app.connected || app.active || app.errorLatched || Boolean(app.checkpoint) || Boolean(app.workflowRecovery) || !app.poseReady || !app.armReady || !armEnabled;
    $("#manualGripperExecute").disabled = !app.connected || app.active || app.errorLatched || Boolean(app.checkpoint) || Boolean(app.workflowRecovery) || !app.poseReady || !app.gripperReady || !gripperEnabled;
  }

  function updateTableControlLocks() {
    const configLocked = !app.connected || app.active || Boolean(app.checkpoint) || Boolean(app.workflowRecovery);
    const commonMotionLocked = !app.connected || app.active || app.errorLatched || Boolean(app.checkpoint) || Boolean(app.workflowRecovery);
    $$('[data-table-lock="config"]').forEach((button) => { button.disabled = configLocked; });
    $$('[data-table-lock="motion"], [data-table-lock="drive"], [data-table-lock="pose"], [data-table-lock="workflow"], [data-table-lock="turn"]').forEach((button) => {
      const kind = button.dataset.tableLock;
      let ready = {
        motion: app.motionReady,
        drive: app.driveReady,
        pose: app.poseReady,
        workflow: app.workflowReady,
        turn: app.turnReady,
      }[kind];
      if (kind === "workflow" && button.dataset.workflowSpecific === "true") {
        ready = (
          app.configReady &&
          (button.dataset.requiresDrive !== "true" || app.driveReady) &&
          (button.dataset.requiresPose !== "true" || app.poseReady)
        );
      }
      const controllersReady = (
        (button.dataset.requiresArm !== "true" || app.armReady) &&
        (button.dataset.requiresGripper !== "true" || app.gripperReady)
      );
      button.disabled = commonMotionLocked || !ready || !controllersReady;
    });
  }

  async function pollSnapshot({ immediate = false } = {}) {
    if (app.polling) return;
    app.polling = true;
    try {
      const snapshot = await api("/snapshot");
      const wasConnected = app.connected;
      updateConnection(true);
      ingestSnapshot(snapshot || {});
      refreshCameraPreviews();
      if (app.active) {
        await api("/lease", { method: "POST" });
      }
      if (!app.firstSnapshotReceived) {
        app.firstSnapshotReceived = true;
        if (!app.categories.length && !app.poses.length && !app.workflows.length) refreshLibraryFallback();
      }
      if (!wasConnected && !immediate) showToast("노드 연결이 복구되었습니다.", "success");
    } catch (error) {
      if (!(error instanceof DOMException && error.name === "AbortError")) {
        updateConnection(false, errorMessage(error));
        app.currentState = "OFFLINE";
        $("#stateBadge").textContent = "OFFLINE";
        $("#stateBadge").className = "state-badge state-error";
      }
    } finally {
      app.polling = false;
    }
  }

  async function refreshLibraryFallback() {
    if (app.libraryFallbackAttempted) return;
    app.libraryFallbackAttempted = true;
    const requests = [
      ["categories", "category", "/categories"],
      ["poses", "pose", "/poses"],
      ["workflows", "workflow", "/workflows"],
    ];
    const results = await Promise.allSettled(requests.map(([, , path]) => api(path)));
    results.forEach((result, index) => {
      if (result.status === "fulfilled") {
        app[requests[index][0]] = normalizeCollection(result.value, requests[index][1]);
      }
    });
    renderLibrary();
  }

  async function mutate(path, { method = "POST", body, successMessage, context = "요청 실패" } = {}) {
    try {
      const result = await api(path, { method, body });
      const returnedRevision = finiteNumber(result?.revision);
      if (returnedRevision !== null && app.snapshot) {
        app.snapshot.revision = Math.trunc(returnedRevision);
        if (app.snapshot.config_info) {
          app.snapshot.config_info.revision = Math.trunc(returnedRevision);
        }
      }
      if (successMessage) {
        showToast(successMessage, "success");
        addEvent(successMessage, "success", new Date(), "UI");
      }
      await pollSnapshot({ immediate: true });
      return result ?? { ok: true };
    } catch (error) {
      handleActionError(error, context);
      throw error;
    }
  }

  function withExpectedRevision(payload = {}) {
    const configInfo = app.snapshot?.config_info || app.snapshot?.config || {};
    const revision = finiteNumber(firstDefined(app.snapshot?.revision, configInfo.revision));
    return {
      ...payload,
      expected_revision: revision === null ? 0 : Math.trunc(revision),
    };
  }

  function categoryId(category) {
    return String(firstDefined(category?.id, category?.category_id, ""));
  }

  function selectedCategoryObject() {
    const selected = $("#categorySelect")?.value;
    return app.categories.find((category) => categoryId(category) === String(selected));
  }

  function isUncategorized(category) {
    const id = categoryId(category).toLowerCase();
    const name = String(category?.name || "").toLowerCase();
    return id === "uncategorized" || name === "미분류";
  }

  function categoryName(id) {
    const category = app.categories.find((item) => categoryId(item) === String(id));
    return category?.name || String(id ?? "미분류");
  }

  function setSelectOptions(select, items, { includeAll = false, preserve = true, labeler, valuer } = {}) {
    if (!select) return;
    const previous = preserve ? select.value : "";
    select.replaceChildren();
    if (includeAll) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "전체";
      select.append(option);
    }
    items.forEach((item) => {
      const option = document.createElement("option");
      option.value = String(valuer ? valuer(item) : item.id);
      option.textContent = labeler ? labeler(item) : item.name;
      select.append(option);
    });
    if (previous && Array.from(select.options).some((option) => option.value === previous)) {
      select.value = previous;
    } else if (select.options.length) {
      select.selectedIndex = 0;
    }
  }

  function renderLibrary() {
    renderCategoryOptions();
    renderPoseTable();
    renderWorkflowTable();
    renderWorkflowSteps();
    applyControlLocks();
  }

  function renderCategoryOptions() {
    const sorted = [...app.categories].sort((a, b) => Number(a.order ?? 999) - Number(b.order ?? 999) || String(a.name).localeCompare(String(b.name), "ko"));
    ["#categorySelect", "#poseCategory", "#workflowCategory"].forEach((selector) => {
      setSelectOptions($(selector), sorted, { valuer: categoryId, labeler: (category) => category.name });
    });
    ["#poseCategoryFilter", "#workflowCategoryFilter"].forEach((selector) => {
      setSelectOptions($(selector), sorted, { includeAll: true, valuer: categoryId, labeler: (category) => category.name });
    });
    const selected = selectedCategoryObject();
    if (document.activeElement !== $("#renameCategoryName")) {
      $("#renameCategoryName").value = selected?.name || "";
    }
    setSelectOptions($("#stepPoseId"), [...app.poses].sort(poseSort), {
      valuer: (pose) => pose.id,
      labeler: (pose) => `${pose.id} · ${pose.name}`,
    });
  }

  function poseSort(a, b) {
    const aNumber = Number(a.id);
    const bNumber = Number(b.id);
    if (Number.isFinite(aNumber) && Number.isFinite(bNumber)) return aNumber - bNumber;
    return String(a.id).localeCompare(String(b.id), "ko", { numeric: true });
  }

  function poseMatchesFilters(pose) {
    const categoryFilter = $("#poseCategoryFilter").value;
    const query = $("#poseSearch").value.trim().toLowerCase();
    const poseCategoryId = String(firstDefined(pose.category_id, pose.category, ""));
    if (categoryFilter && poseCategoryId !== categoryFilter) return false;
    return !query || String(pose.id).toLowerCase().includes(query) || String(pose.name).toLowerCase().includes(query);
  }

  function renderPoseTable() {
    const body = $("#poseTableBody");
    body.replaceChildren();
    const poses = [...app.poses].sort(poseSort).filter(poseMatchesFilters);
    if (!poses.length) {
      body.append(emptyTableRow(6, "조건에 맞는 저장 동작이 없습니다."));
      return;
    }
    poses.forEach((pose) => {
      const row = document.createElement("tr");
      const requiresArm = asBoolean(firstDefined(
        pose.arm_enabled, pose.arm?.enabled, pose.arm_positions_deg !== undefined
      )) === true;
      const requiresGripper = asBoolean(firstDefined(
        pose.gripper_enabled, pose.gripper?.enabled, pose.gripper_positions_deg !== undefined
      )) === true;
      row.append(textCell(pose.id), textCell(pose.name), textCell(categoryName(firstDefined(pose.category_id, pose.category))));
      const composition = document.createElement("td");
      if (requiresArm) composition.append(tag("팔 5축"));
      if (requiresGripper) composition.append(tag("그리퍼 9축"));
      if (!composition.childNodes.length) composition.textContent = "—";
      row.append(composition);
      const duration = [
        pose.arm_duration_sec !== undefined ? `팔 ${formatNumber(pose.arm_duration_sec, 1)}s` : "",
        pose.gripper_duration_sec !== undefined ? `그리퍼 ${formatNumber(pose.gripper_duration_sec, 1)}s` : "",
      ].filter(Boolean).join(" · ") || "—";
      row.append(textCell(duration));
      const actionCell = document.createElement("td");
      actionCell.className = "actions-cell";
      actionCell.append(tableActions([
        { label: "실행", kind: "pose", requiresArm, requiresGripper, onClick: () => executePose(pose) },
        { label: "수정", kind: "config", onClick: () => editPose(pose) },
        { label: "복제", kind: "config", onClick: () => duplicatePose(pose) },
        { label: "삭제", kind: "config", danger: true, onClick: () => deletePose(pose) },
      ]));
      row.append(actionCell);
      body.append(row);
    });
    updateTableControlLocks();
  }

  function textCell(value) {
    const cell = document.createElement("td");
    cell.textContent = String(value ?? "—");
    return cell;
  }

  function emptyTableRow(columns, message) {
    const row = document.createElement("tr");
    const cell = document.createElement("td");
    cell.colSpan = columns;
    cell.className = "empty-cell";
    cell.textContent = message;
    row.append(cell);
    return row;
  }

  function tag(text) {
    const element = document.createElement("span");
    element.className = "tag";
    element.textContent = text;
    return element;
  }

  function tableActions(actions) {
    const group = document.createElement("div");
    group.className = "table-actions";
    actions.forEach((action) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = `btn ${action.danger ? "btn-danger-quiet" : "btn-ghost"}`;
      button.textContent = action.label;
      if (action.kind) button.dataset.tableLock = action.kind;
      if (action.requiresArm) button.dataset.requiresArm = "true";
      if (action.requiresGripper) button.dataset.requiresGripper = "true";
      if (action.requiresDrive) button.dataset.requiresDrive = "true";
      if (action.requiresPose) button.dataset.requiresPose = "true";
      if (action.workflowSpecific) button.dataset.workflowSpecific = "true";
      button.addEventListener("click", (event) => {
        event.stopPropagation();
        action.onClick(event);
      });
      group.append(button);
    });
    return group;
  }

  function readPoseForm() {
    const armEnabled = $("#armEnabled").checked;
    const gripperEnabled = $("#gripperEnabled").checked;
    if (!armEnabled && !gripperEnabled) throw new Error("로봇팔 또는 그리퍼 중 하나 이상을 사용해야 합니다.");
    const payload = {
      name: $("#poseName").value.trim(),
      category_id: $("#poseCategory").value,
      arm_enabled: armEnabled,
      arm_positions_deg: $$(".arm-joint").map((input) => Number(input.value)),
      arm_duration_sec: Number($("#armDuration").value),
      gripper_enabled: gripperEnabled,
      gripper_positions_deg: $$(".gripper-joint").map((input) => Number(input.value)),
      gripper_duration_sec: Number($("#gripperDuration").value),
      target_load_raw: Number($("#gripperLoad").value),
      dwell_sec: Number($("#poseDwell").value),
    };
    const manualId = $("#poseId").value.trim();
    if (manualId) payload.id = Number(manualId);
    if (!payload.name) throw new Error("동작 이름을 입력하세요.");
    if (!payload.category_id) throw new Error("카테고리를 선택하세요.");
    const relevantValues = [payload.dwell_sec];
    if (armEnabled) relevantValues.push(...payload.arm_positions_deg, payload.arm_duration_sec);
    if (gripperEnabled) relevantValues.push(...payload.gripper_positions_deg, payload.gripper_duration_sec, payload.target_load_raw);
    if (!relevantValues.every(Number.isFinite)) throw new Error("각도와 시간에는 올바른 숫자를 입력하세요.");
    return payload;
  }

  function poseValue(pose, directKey, nestedKey, fallback) {
    return firstDefined(pose[directKey], pose.arm?.[nestedKey], pose.gripper?.[nestedKey], fallback);
  }

  function editPose(pose, { duplicate = false } = {}) {
    const id = String(pose.id);
    $("#poseEditId").value = duplicate ? "" : id;
    $("#poseId").value = duplicate ? "" : id;
    $("#poseId").disabled = !duplicate;
    $("#poseName").value = `${pose.name}${duplicate ? " 복사본" : ""}`;
    $("#poseCategory").value = String(firstDefined(pose.category_id, pose.category, $("#poseCategory").value));
    $("#poseDwell").value = firstDefined(pose.dwell_sec, pose.dwell, 0.3);

    const armEnabled = asBoolean(firstDefined(pose.arm_enabled, pose.arm?.enabled, pose.arm_positions_deg !== undefined)) === true;
    const armPositions = firstDefined(pose.arm_positions_deg, pose.arm?.positions_deg, [0, 0, 0, 0, 0]);
    $("#armEnabled").checked = armEnabled;
    $$(".arm-joint").forEach((input, index) => { input.value = finiteNumber(armPositions[index]) ?? 0; });
    $("#armDuration").value = firstDefined(pose.arm_duration_sec, pose.arm?.duration_sec, 2);

    const gripperEnabled = asBoolean(firstDefined(pose.gripper_enabled, pose.gripper?.enabled, pose.gripper_positions_deg !== undefined)) === true;
    const gripperPositions = firstDefined(pose.gripper_positions_deg, pose.gripper?.positions_deg, Array(9).fill(0));
    $("#gripperEnabled").checked = gripperEnabled;
    $$(".gripper-joint").forEach((input, index) => { input.value = finiteNumber(gripperPositions[index]) ?? 0; });
    $("#gripperDuration").value = firstDefined(pose.gripper_duration_sec, pose.gripper?.duration_sec, 1);
    $("#gripperLoad").value = firstDefined(pose.target_load_raw, pose.gripper?.target_load_raw, 500);
    updateActuatorDisabledState();
    $("#poseEditorTitle").scrollIntoView({ behavior: "smooth", block: "start" });
    $("#poseName").focus({ preventScroll: true });
  }

  function duplicatePose(pose) {
    editPose(pose, { duplicate: true });
  }

  function resetPoseForm() {
    $("#poseForm").reset();
    $("#poseEditId").value = "";
    $("#poseId").value = "";
    $("#poseId").disabled = false;
    $("#armEnabled").checked = true;
    $("#gripperEnabled").checked = false;
    $("#armDuration").value = "2";
    $("#gripperDuration").value = "1";
    $("#gripperLoad").value = "500";
    $("#poseDwell").value = "0.3";
    updateActuatorDisabledState();
  }

  async function executePose(pose) {
    await mutate(`/poses/${encodeURIComponent(pose.id)}/execute`, {
      successMessage: `동작 '${pose.name}' 실행을 시작했습니다.`,
      context: "동작 실행 실패",
    }).catch(() => {});
  }

  async function deletePose(pose) {
    await mutate(`/poses/${encodeURIComponent(pose.id)}`, {
      method: "DELETE",
      body: withExpectedRevision(),
      successMessage: `동작 '${pose.name}'을 삭제했습니다.`,
      context: "동작 삭제 실패",
    }).catch(() => {});
  }

  function workflowMatchesFilters(workflow) {
    const categoryFilter = $("#workflowCategoryFilter").value;
    const query = $("#workflowSearch").value.trim().toLowerCase();
    const workflowCategoryId = String(firstDefined(workflow.category_id, workflow.category, ""));
    if (categoryFilter && workflowCategoryId !== categoryFilter) return false;
    return !query || String(workflow.id).toLowerCase().includes(query) || String(workflow.name).toLowerCase().includes(query);
  }

  function normalizeWorkflowSteps(steps) {
    if (!Array.isArray(steps)) return [];
    return steps.map((step) => {
      if (typeof step === "string") {
        const [type, value] = step.split(":", 2);
        if (type === "POSE") return { type, pose_id: finiteNumber(value) ?? value };
        if (type === "WAIT_SECONDS" || type === "WAIT") return { type: "WAIT_SECONDS", seconds: finiteNumber(value) ?? 1 };
        return { type: type.toUpperCase() };
      }
      const type = String(firstDefined(step.type, step.command, step.kind, "")).toUpperCase();
      return {
        ...step,
        type,
        pose_id: firstDefined(step.pose_id, step.id),
        seconds: firstDefined(step.seconds, step.duration_sec, step.duration),
      };
    }).filter((step) => STEP_LABELS[step.type]);
  }

  function renderWorkflowTable() {
    const body = $("#workflowTableBody");
    body.replaceChildren();
    const allWorkflows = [...app.workflows].sort((a, b) => String(a.id).localeCompare(String(b.id), "ko", { numeric: true }));
    if (
      app.selectedWorkflowId === null ||
      !allWorkflows.some((workflow) => String(workflow.id) === String(app.selectedWorkflowId))
    ) {
      app.selectedWorkflowId = allWorkflows.length ? String(allWorkflows[0].id) : null;
      app.selectedWorkflowStep = null;
    }
    const workflows = allWorkflows.filter(workflowMatchesFilters);
    if (!workflows.length) {
      body.append(emptyTableRow(5, "조건에 맞는 워크플로우가 없습니다."));
      renderWorkflowProgress();
      return;
    }
    workflows.forEach((workflow) => {
      const row = document.createElement("tr");
      row.className = "workflow-row";
      if (String(workflow.id) === String(app.selectedWorkflowId)) {
        row.classList.add("is-selected");
      }
      row.addEventListener("click", () => selectWorkflowForStages(workflow));
      const steps = normalizeWorkflowSteps(firstDefined(workflow.steps, workflow.workflow, []));
      const requiresDrive = steps.some((step) => ["GO_DROPOFF", "GO_PICKUP"].includes(step.type));
      const poseSteps = steps.filter((step) => step.type === "POSE");
      const referencedPoses = poseSteps
        .map((step) => app.poses.find((pose) => String(pose.id) === String(step.pose_id)))
        .filter(Boolean);
      const requiresArm = referencedPoses.some((pose) => asBoolean(firstDefined(
        pose.arm_enabled, pose.arm?.enabled, pose.arm_positions_deg !== undefined
      )) === true);
      const requiresGripper = referencedPoses.some((pose) => asBoolean(firstDefined(
        pose.gripper_enabled, pose.gripper?.enabled, pose.gripper_positions_deg !== undefined
      )) === true);
      row.append(textCell(workflow.id), textCell(workflow.name), textCell(categoryName(firstDefined(workflow.category_id, workflow.category))), textCell(`${steps.length}단계`));
      const actionCell = document.createElement("td");
      actionCell.className = "actions-cell";
      actionCell.append(tableActions([
        { label: "단계 보기", kind: "view", onClick: () => selectWorkflowForStages(workflow) },
        {
          label: "전체 실행",
          kind: "workflow",
          workflowSpecific: true,
          requiresDrive,
          requiresPose: poseSteps.length > 0,
          requiresArm,
          requiresGripper,
          onClick: () => runWorkflow(workflow),
        },
        { label: "수정", kind: "config", onClick: () => editWorkflow(workflow) },
        { label: "복제", kind: "config", onClick: () => duplicateWorkflow(workflow) },
        { label: "삭제", kind: "config", danger: true, onClick: () => deleteWorkflow(workflow) },
      ]));
      row.append(actionCell);
      body.append(row);
    });
    updateTableControlLocks();
    renderWorkflowProgress();
  }

  function describeStep(step) {
    if (step.type === "POSE") {
      const pose = app.poses.find((item) => String(item.id) === String(step.pose_id));
      return { title: pose ? `${pose.id} · ${pose.name}` : `동작 ID ${step.pose_id}`, detail: "POSE" };
    }
    if (step.type === "WAIT_SECONDS") return { title: `${formatNumber(step.seconds, 1)}초 대기`, detail: "WAIT_SECONDS" };
    return { title: STEP_LABELS[step.type] || step.type, detail: step.type };
  }

  function renderWorkflowSteps() {
    const list = $("#workflowStepList");
    list.replaceChildren();
    if (!app.workflowDraft.length) {
      const empty = document.createElement("li");
      empty.className = "empty-step";
      empty.textContent = "단계를 추가해 워크플로우를 만드세요.";
      list.append(empty);
      return;
    }
    app.workflowDraft.forEach((step, index) => {
      const item = document.createElement("li");
      item.className = "step-item";
      const number = document.createElement("span");
      number.className = "step-index";
      number.textContent = String(index + 1).padStart(2, "0");
      const copy = document.createElement("div");
      copy.className = "step-copy";
      const description = describeStep(step);
      const title = document.createElement("strong");
      title.textContent = description.title;
      const detail = document.createElement("small");
      detail.textContent = description.detail;
      copy.append(title, detail);
      const actions = document.createElement("div");
      actions.className = "step-actions";
      actions.append(
        stepButton("↑", "위로 이동", index === 0, () => moveWorkflowStep(index, -1)),
        stepButton("↓", "아래로 이동", index === app.workflowDraft.length - 1, () => moveWorkflowStep(index, 1)),
        stepButton("×", "단계 삭제", false, () => removeWorkflowStep(index)),
      );
      item.append(number, copy, actions);
      list.append(item);
    });
  }

  function stepButton(text, label, disabled, callback) {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = text;
    button.title = label;
    button.setAttribute("aria-label", label);
    button.dataset.lock = "config";
    button.disabled = disabled || !app.connected || app.active || Boolean(app.checkpoint);
    button.addEventListener("click", callback);
    return button;
  }

  function moveWorkflowStep(index, offset) {
    const nextIndex = index + offset;
    if (nextIndex < 0 || nextIndex >= app.workflowDraft.length) return;
    [app.workflowDraft[index], app.workflowDraft[nextIndex]] = [app.workflowDraft[nextIndex], app.workflowDraft[index]];
    renderWorkflowSteps();
  }

  function removeWorkflowStep(index) {
    app.workflowDraft.splice(index, 1);
    renderWorkflowSteps();
  }

  function addWorkflowStep() {
    const type = $("#stepType").value;
    let step = { type };
    if (type === "POSE") {
      const poseId = $("#stepPoseId").value;
      if (!poseId) {
        showToast("먼저 실행할 저장 동작을 선택하세요.", "warning");
        return;
      }
      step.pose_id = /^\d+$/.test(poseId) ? Number(poseId) : poseId;
    } else if (type === "WAIT_SECONDS") {
      const seconds = Number($("#stepWaitSeconds").value);
      if (!Number.isFinite(seconds) || seconds <= 0) {
        showToast("대기 시간은 0보다 커야 합니다.", "warning");
        return;
      }
      step.seconds = seconds;
    }
    app.workflowDraft.push(step);
    renderWorkflowSteps();
  }

  function updateStepFields() {
    const type = $("#stepType").value;
    $("#stepPoseWrap").hidden = type !== "POSE";
    $("#stepWaitWrap").hidden = type !== "WAIT_SECONDS";
  }

  function editWorkflow(workflow, { duplicate = false } = {}) {
    $("#workflowEditId").value = duplicate ? "" : workflow.id;
    $("#workflowId").value = duplicate ? "" : workflow.id;
    $("#workflowId").disabled = !duplicate;
    $("#workflowName").value = `${workflow.name}${duplicate ? " 복사본" : ""}`;
    $("#workflowCategory").value = String(firstDefined(workflow.category_id, workflow.category, $("#workflowCategory").value));
    app.workflowDraft = normalizeWorkflowSteps(firstDefined(workflow.steps, workflow.workflow, [])).map((step) => ({ ...step }));
    renderWorkflowSteps();
    $("#workflowEditorTitle").scrollIntoView({ behavior: "smooth", block: "start" });
    $("#workflowName").focus({ preventScroll: true });
  }

  function duplicateWorkflow(workflow) {
    editWorkflow(workflow, { duplicate: true });
  }

  function resetWorkflowForm() {
    $("#workflowForm").reset();
    $("#workflowEditId").value = "";
    $("#workflowId").value = "";
    $("#workflowId").disabled = false;
    app.workflowDraft = [];
    updateStepFields();
    renderWorkflowSteps();
  }

  async function runWorkflow(workflow) {
    await mutate(`/workflows/${encodeURIComponent(workflow.id)}/run`, {
      successMessage: `워크플로우 '${workflow.name}'을 시작했습니다.`,
      context: "워크플로우 실행 실패",
    }).catch(() => {});
  }

  async function deleteWorkflow(workflow) {
    await mutate(`/workflows/${encodeURIComponent(workflow.id)}`, {
      method: "DELETE",
      body: withExpectedRevision(),
      successMessage: `워크플로우 '${workflow.name}'을 삭제했습니다.`,
      context: "워크플로우 삭제 실패",
    }).catch(() => {});
  }

  async function issueTurn(degrees, label) {
    const angle = finiteNumber(degrees);
    if (angle === null || angle <= -180 || angle >= 180 || angle === 0) {
      showToast("회전각은 -180°와 180°를 제외한 0이 아닌 값이어야 합니다.", "warning");
      return;
    }
    await mutate("/manual/turn", {
      body: { degrees: angle, exact: true, source: "ui" },
      successMessage: `${label || "수동 회전"} ${formatAngle(angle)} 명령을 시작했습니다.`,
      context: "회전 명령 실패",
    }).catch(() => {});
  }

  async function issueRoute(path, title, _message) {
    await mutate(path, {
      successMessage: `${title} 명령을 시작했습니다.`,
      context: `${title} 실패`,
    }).catch(() => {});
  }

  async function issueArmBoardCommand(command, label) {
    let confirmed = false;
    if (command === "disable") {
      confirmed = await confirmAction(
        "로봇팔 Disable",
        "Disable하면 관리자가 다시 Enable하기 전까지 팔 명령을 받을 수 없습니다.",
        { danger: true, acceptLabel: "Disable" },
      );
      if (!confirmed) return;
    }
    await mutate(`/arm/${command}`, {
      body: command === "disable" ? { confirmed: true } : {},
      successMessage: `로봇팔 ${label} 명령을 실행했습니다.`,
      context: `로봇팔 ${label} 실패`,
    }).catch(() => {});
  }

  function readManualPose(mode) {
    const armPositions = $$(".arm-joint").map((input) => Number(input.value));
    const gripperPositions = $$(".gripper-joint").map((input) => Number(input.value));
    const armDuration = Number($("#armDuration").value);
    const gripperDuration = Number($("#gripperDuration").value);
    const gripperLoad = Number($("#gripperLoad").value);
    const relevant = mode === "arm"
      ? [...armPositions, armDuration]
      : [...gripperPositions, gripperDuration, gripperLoad];
    if (!relevant.every(Number.isFinite)) {
      throw new Error("수동 조종 각도와 Duration에 올바른 숫자를 입력하세요.");
    }
    return {
      name: mode === "arm" ? "수동 로봇팔 조종" : "수동 그리퍼 조종",
      category_id: $("#poseCategory").value || app.categories[0]?.id,
      arm_enabled: mode === "arm",
      arm_positions_deg: armPositions,
      arm_duration_sec: armDuration,
      gripper_enabled: mode === "gripper",
      gripper_positions_deg: gripperPositions,
      gripper_duration_sec: gripperDuration,
      target_load_raw: gripperLoad,
      dwell_sec: 0,
    };
  }

  async function executeManualPose(mode) {
    let payload;
    try {
      payload = readManualPose(mode);
    } catch (error) {
      showToast(error.message, "warning");
      return;
    }
    const label = mode === "arm" ? "팔" : "그리퍼";
    await mutate("/poses/preview", {
      body: payload,
      successMessage: `${label} 수동 조종을 시작했습니다.`,
      context: `${label} 수동 조종 실패`,
    }).catch(() => {});
  }

  function bindEvents() {
    $("#emergencyStop").addEventListener("click", async () => {
      addEvent("전체 중지를 요청했습니다.", "warning", new Date(), "STOP");
      try {
        await api("/stop", { method: "POST", body: { source: "web_ui" } });
        showToast("전체 중지 명령을 전송했습니다.", "warning");
        await pollSnapshot({ immediate: true });
      } catch (error) {
        handleActionError(error, "전체 중지 전송 실패");
      }
    });

    const clearLatchedError = async () => {
      await mutate("/error/clear", { successMessage: "오류 해제를 요청했습니다.", context: "오류 해제 실패" }).catch(() => {});
    };
    $("#clearError").addEventListener("click", clearLatchedError);
    $("#clearErrorTopbar").addEventListener("click", clearLatchedError);

    $("#retryWorkflowStep").addEventListener("click", async () => {
      await mutate("/workflow/retry-step", {
        successMessage: "실패한 워크플로우 단계를 다시 실행합니다.",
        context: "현재 단계 재시도 실패",
      }).catch(() => {});
    });
    $("#skipWorkflowStep").addEventListener("click", async () => {
      await mutate("/workflow/skip-step", {
        successMessage: "실패 단계를 건너뛰고 다음 단계를 실행합니다.",
        context: "다음 단계 진행 실패",
      }).catch(() => {});
    });
    $("#resumeWorkflowAtStep").addEventListener("click", async () => {
      const stepNumber = Math.trunc(finiteNumber(app.selectedWorkflowStep) || 0);
      if (stepNumber < 1) {
        showToast("먼저 다시 시작할 단계를 선택하세요.", "warning");
        return;
      }
      await mutate("/workflow/resume-at-step", {
        body: { step_number: stepNumber },
        successMessage: `${stepNumber}단계부터 워크플로우를 다시 실행합니다.`,
        context: "선택 단계 실행 실패",
      }).catch(() => {});
    });
    $("#abortPausedWorkflow").addEventListener("click", async () => {
      await mutate("/workflow/abort-paused", {
        successMessage: "일시정지된 워크플로우를 종료했습니다.",
        context: "워크플로우 종료 실패",
      }).catch(() => {});
    });

    $("#armEnable").addEventListener("click", () => issueArmBoardCommand("enable", "Enable"));
    $("#armHome").addEventListener("click", () => issueArmBoardCommand("home", "Home"));
    $("#armDisable").addEventListener("click", () => issueArmBoardCommand("disable", "Disable"));
    $("#armClear").addEventListener("click", () => issueArmBoardCommand("clear", "Clear"));
    $("#armEstop").addEventListener("click", () => issueArmBoardCommand("estop", "ESTOP"));
    $("#armStatusRefresh").addEventListener("click", () => issueArmBoardCommand("status", "상태 새로고침"));

    $("#goDropoff").addEventListener("click", () => issueRoute("/route/dropoff", "놓을 위치로 이동", "마커 1에서 마커 3까지 직선으로 전진합니다."));
    $("#goPickup").addEventListener("click", () => issueRoute("/route/pickup", "집기 위치로 원상복귀", "마커 3에서 마커 1까지 직선으로 후진합니다."));
    $("#continueCheckpoint").addEventListener("click", () => issueRoute("/route/continue", "목적지 이동 계속", "남은 목적지 마커 거리 이동을 계속합니다."));
    $("#discardCheckpoint").addEventListener("click", async () => {
      await mutate("/route/discard-checkpoint", { successMessage: "경로 체크포인트를 취소했습니다.", context: "체크포인트 취소 실패" }).catch(() => {});
    });
    $$("[data-turn-degrees]").forEach((button) => button.addEventListener("click", () => issueTurn(button.dataset.turnDegrees, "미세 회전")));
    $$("[data-marker-test]").forEach((button) => button.addEventListener("click", () => {
      const marker = button.dataset.markerTest;
      issueRoute(`/test/marker/${encodeURIComponent(marker)}`, `마커 ${marker} 단일 시험`, "선택한 마커까지 직진하고 거리 정지만 시험합니다.");
    }));
    $("#exactTurnForm").addEventListener("submit", (event) => {
      event.preventDefault();
      issueTurn($("#exactTurnDegrees").value, "직접 지정 회전");
    });

    $("#categorySelect").addEventListener("change", () => {
      const selected = selectedCategoryObject();
      $("#renameCategoryName").value = selected?.name || "";
      if (selected) {
        $("#poseCategory").value = categoryId(selected);
        $("#workflowCategory").value = categoryId(selected);
      }
      applyControlLocks();
    });

    $("#categoryAddForm").addEventListener("submit", async (event) => {
      event.preventDefault();
      const name = $("#newCategoryName").value.trim();
      if (!name) return;
      const result = await mutate("/categories", {
        body: withExpectedRevision({ name }),
        successMessage: `카테고리 '${name}'을 추가했습니다.`,
        context: "카테고리 추가 실패",
      }).catch(() => null);
      if (result !== null) $("#newCategoryName").value = "";
    });

    $("#renameCategory").addEventListener("click", async () => {
      const category = selectedCategoryObject();
      const name = $("#renameCategoryName").value.trim();
      if (!category || !name) {
        showToast("카테고리와 새 이름을 확인하세요.", "warning");
        return;
      }
      await mutate(`/categories/${encodeURIComponent(categoryId(category))}`, {
        method: "PATCH",
        body: withExpectedRevision({ name }),
        successMessage: `카테고리 이름을 '${name}'으로 변경했습니다.`,
        context: "카테고리 이름 변경 실패",
      }).catch(() => {});
    });

    $("#deleteCategory").addEventListener("click", async () => {
      const category = selectedCategoryObject();
      if (!category) return;
      await mutate(`/categories/${encodeURIComponent(categoryId(category))}`, {
        method: "DELETE",
        body: withExpectedRevision(),
        successMessage: `카테고리 '${category.name}'을 삭제했습니다.`,
        context: "카테고리 삭제 실패",
      }).catch(() => {});
    });

    $("#reloadConfig").addEventListener("click", async () => {
      await mutate("/config/reload", { successMessage: "통합 설정을 다시 불러왔습니다.", context: "설정 다시 불러오기 실패" }).catch(() => {});
    });

    $("#armEnabled").addEventListener("change", updateActuatorDisabledState);
    $("#gripperEnabled").addEventListener("change", updateActuatorDisabledState);
    $("#fillCurrentJoints").addEventListener("click", () => {
      if (app.armFeedbackDeg.length !== 5) {
        showToast("5개 로봇팔 관절의 최신 피드백이 아직 없습니다.", "warning");
        return;
      }
      $$(".arm-joint").forEach((input, index) => { input.value = app.armFeedbackDeg[index].toFixed(1); });
      if (app.gripperFeedbackDeg.length === 9) {
        $$(".gripper-joint").forEach((input, index) => { input.value = app.gripperFeedbackDeg[index].toFixed(1); });
      }
      showToast("최신 관절 피드백을 입력란에 채웠습니다.", "success");
    });
    $("#resetPoseForm").addEventListener("click", resetPoseForm);
    $("#manualArmExecute").addEventListener("click", () => executeManualPose("arm"));
    $("#manualGripperExecute").addEventListener("click", () => executeManualPose("gripper"));

    $("#poseForm").addEventListener("submit", async (event) => {
      event.preventDefault();
      let payload;
      try {
        payload = readPoseForm();
      } catch (error) {
        showToast(error.message, "warning");
        return;
      }
      const editId = $("#poseEditId").value;
      const method = editId ? "PATCH" : "POST";
      const path = editId ? `/poses/${encodeURIComponent(editId)}` : "/poses";
      const saved = await mutate(path, {
        method,
        body: withExpectedRevision(payload),
        successMessage: `동작 '${payload.name}'을 저장했습니다.`,
        context: "동작 저장 실패",
      }).catch(() => null);
      if (saved !== null) resetPoseForm();
    });

    $("#testPose").addEventListener("click", async () => {
      let payload;
      try {
        payload = readPoseForm();
      } catch (error) {
        showToast(error.message, "warning");
        return;
      }
      if (payload.arm_enabled && payload.arm_duration_sec > 1.275) {
        showToast("Arm 4 보드에서 duration이 제한될 수 있습니다. 실제 피드백을 확인하세요.", "warning", 5000);
      }
      await mutate("/poses/preview", { body: payload, successMessage: "입력값 시험 실행을 시작했습니다.", context: "시험 실행 실패" }).catch(() => {});
    });

    $("#poseCategoryFilter").addEventListener("change", renderPoseTable);
    $("#poseSearch").addEventListener("input", renderPoseTable);

    $("#stepType").addEventListener("change", updateStepFields);
    $("#addWorkflowStep").addEventListener("click", addWorkflowStep);
    $("#resetWorkflowForm").addEventListener("click", resetWorkflowForm);
    $("#workflowForm").addEventListener("submit", async (event) => {
      event.preventDefault();
      const name = $("#workflowName").value.trim();
      const category = $("#workflowCategory").value;
      if (!name || !category || !app.workflowDraft.length) {
        showToast("이름, 카테고리, 한 개 이상의 단계를 입력하세요.", "warning");
        return;
      }
      const payload = { name, category_id: category, steps: app.workflowDraft.map((step) => ({ ...step })) };
      const manualId = $("#workflowId").value.trim();
      if (manualId) payload.id = Number(manualId);
      const editId = $("#workflowEditId").value;
      const saved = await mutate(editId ? `/workflows/${encodeURIComponent(editId)}` : "/workflows", {
        method: editId ? "PATCH" : "POST",
        body: withExpectedRevision(payload),
        successMessage: `워크플로우 '${name}'을 저장했습니다.`,
        context: "워크플로우 저장 실패",
      }).catch(() => null);
      if (saved !== null) resetWorkflowForm();
    });
    $("#workflowCategoryFilter").addEventListener("change", renderWorkflowTable);
    $("#workflowSearch").addEventListener("input", renderWorkflowTable);

    $("#clearLocalEvents").addEventListener("click", () => {
      app.events = [];
      renderEvents();
    });

    document.addEventListener("visibilitychange", () => {
      if (!document.hidden) pollSnapshot({ immediate: true });
    });
  }

  async function initialize() {
    bindEvents();
    resetPoseForm();
    resetWorkflowForm();
    updateStepFields();
    applyControlLocks();
    await pollSnapshot({ immediate: true });
    const schedulePoll = () => {
      app.pollTimer = window.setTimeout(async () => {
        await pollSnapshot();
        schedulePoll();
      }, app.pollIntervalMs);
    };
    schedulePoll();
  }

  initialize().catch((error) => handleActionError(error, "UI 초기화 실패"));
})();
