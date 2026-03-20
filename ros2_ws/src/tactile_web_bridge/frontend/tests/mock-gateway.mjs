import { mkdirSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { createServer } from "node:http";
import { WebSocketServer } from "ws";

const HOST = "127.0.0.1";
const PORT = Number(process.env.MOCK_GATEWAY_PORT || 8765);
const STATE_WS_PATH = "/ws/live";
const STREAMS = {
  rgb: "/api/streams/rgb.mjpeg",
  detection_overlay: "/api/streams/detection_overlay.mjpeg",
  grasp_overlay: "/api/streams/grasp_overlay.mjpeg",
};
const HISTORY_FILE = resolve(process.cwd(), "test-results", "mock-gateway-history.json");

let eventId = 0;
let dialogMessageId = 0;
let currentScenario = "idle";
let executeOutcome = "completed";
let websocketEnabled = true;
let state = createScenario("idle");
let history = [];
let executeTimer = null;

const clients = new Set();
const server = createServer(handleRequest);
const wss = new WebSocketServer({ noServer: true });

wss.on("connection", (socket) => {
  history.push({ type: "ws_connect", at: nowSec(), payload: { path: STATE_WS_PATH } });
  writeHistoryFile();
  clients.add(socket);
  socket.send(JSON.stringify(state));
  socket.on("close", () => clients.delete(socket));
});

server.on("upgrade", (request, socket, head) => {
  if (request.url !== STATE_WS_PATH || !websocketEnabled) {
    socket.destroy();
    return;
  }
  wss.handleUpgrade(request, socket, head, (ws) => {
    wss.emit("connection", ws, request);
  });
});

server.listen(PORT, HOST, () => {
  writeHistoryFile();
  console.log(`mock gateway ready on http://${HOST}:${PORT}`);
});

for (const signal of ["SIGINT", "SIGTERM"]) {
  process.on(signal, () => {
    for (const socket of clients) socket.close(1001, "shutdown");
    wss.close(() => {
      server.close(() => process.exit(0));
    });
  });
}

async function handleRequest(request, response) {
  const url = new URL(request.url ?? "/", `http://${HOST}:${PORT}`);
  try {
    if (request.method === "GET" && url.pathname === "/api/bootstrap") {
      history.push({ type: "bootstrap", at: nowSec(), payload: { path: url.pathname } });
      writeHistoryFile();
      return json(response, 200, {
        state,
        streams: STREAMS,
        frontend_ready: true,
      });
    }

    if (request.method === "POST" && url.pathname === "/api/dialog/message") {
      const payload = await readJson(request);
      const message = `${payload.message ?? payload.prompt ?? ""}`.trim();
      const mode = normalizeDialogMode(payload.mode);
      history.push({ type: "dialog_message", at: nowSec(), payload: { message, mode } });
      await sleep(120);
      applyDialogMessage(message, mode);
      return json(response, 200, { ok: true, state });
    }

    if (request.method === "POST" && url.pathname === "/api/dialog/mode") {
      const payload = await readJson(request);
      const mode = normalizeDialogMode(payload.mode);
      history.push({ type: "dialog_mode", at: nowSec(), payload: { mode } });
      await sleep(120);
      setDialogMode(mode);
      return json(response, 200, { ok: true, state });
    }

    if (request.method === "POST" && url.pathname === "/api/dialog/reset") {
      history.push({ type: "dialog_reset", at: nowSec(), payload: {} });
      await sleep(120);
      resetDialogSession();
      commitState();
      return json(response, 200, { ok: true, state });
    }

    if (request.method === "POST" && url.pathname === "/api/prompt") {
      const payload = await readJson(request);
      const prompt = `${payload.prompt ?? ""}`.trim();
      history.push({ type: "prompt", at: nowSec(), payload: { prompt } });
      await sleep(120);
      applyPrompt(prompt);
      return json(response, 200, { ok: true, state });
    }

    if (request.method === "POST" && url.pathname === "/api/task/override") {
      const payload = await readJson(request);
      history.push({ type: "override", at: nowSec(), payload });
      await sleep(120);
      applyOverride(payload);
      return json(response, 200, { ok: true, state });
    }

    if (request.method === "POST" && url.pathname === "/api/task/replan") {
      history.push({ type: "replan", at: nowSec(), payload: {} });
      await sleep(120);
      applyReplan();
      return json(response, 200, { ok: true, state });
    }

    if (request.method === "POST" && url.pathname === "/api/execution/execute") {
      history.push({ type: "execute", at: nowSec(), payload: {} });
      await sleep(120);
      startExecuteFlow({ recordHistory: false });
      return json(response, 200, { ok: true, state });
    }

    if (request.method === "GET" && url.pathname.startsWith("/api/streams/")) {
      return svg(response, 200, buildStreamSvg(url.pathname));
    }

    if (request.method === "POST" && url.pathname === "/__test/reset") {
      const payload = await readJson(request);
      resetScenario(payload.scenario ?? "idle", payload.executeOutcome ?? "completed");
      return json(response, 200, { ok: true, scenario: currentScenario, state });
    }

    if (request.method === "GET" && url.pathname === "/__test/history") {
      return json(response, 200, {
        scenario: currentScenario,
        executeOutcome,
        websocketEnabled,
        history,
        state,
      });
    }

    if (request.method === "POST" && url.pathname === "/__test/ws") {
      const payload = await readJson(request);
      handleWsControl(payload);
      return json(response, 200, { ok: true, websocketEnabled, state });
    }

    response.statusCode = 404;
    response.end("not found");
  } catch (error) {
    response.statusCode = 500;
    response.setHeader("Content-Type", "application/json");
    response.end(JSON.stringify({ detail: error instanceof Error ? error.message : "unknown error" }));
  }
}

function resetScenario(scenario, nextOutcome) {
  clearExecuteTimer();
  eventId = 0;
  dialogMessageId = 0;
  currentScenario = `${scenario}`;
  executeOutcome = `${nextOutcome}`;
  websocketEnabled = true;
  history = [];
  state = createScenario(currentScenario);
  commitState();
}

function handleWsControl(payload) {
  const action = `${payload.action ?? ""}`;
  if (action === "disconnect") {
    websocketEnabled = false;
    for (const socket of clients) socket.close(1012, "mock disconnect");
    commitState({ broadcast: false });
    return;
  }

  if (action === "reconnect") {
    websocketEnabled = true;
    commitState({ broadcast: false });
    return;
  }

  if (action === "push_state") {
    if (payload.state && isPlainObject(payload.state)) {
      state = deepMerge(state, payload.state);
    }
    if (payload.feedback && isPlainObject(payload.feedback)) {
      appendEvent({
        category: payload.feedback.category ?? "test",
        level: payload.feedback.level ?? "info",
        message: payload.feedback.message ?? "test feedback",
        data: payload.feedback.data ?? {},
        feedback: true,
      });
    }
    commitState();
    return;
  }

  throw new Error(`unsupported ws action: ${action}`);
}

function applyPrompt(prompt) {
  const mode = state.dialog?.mode ?? "review";
  if (prompt) {
    appendDialogMessage("user", prompt);
  }
  const semantic = updateSemanticFromPrompt(prompt);
  appendDialogMessage(
    "assistant",
    `Structured semantic task updated for ${semantic.target}.`,
    {
      meta: `Action: update_task · Target: ${semantic.target}`,
      semanticTask: structuredClone(state.semantic),
      requestedAction: "update_task",
    },
  );
  setDialogStatus("ready", { mode, pendingAutoExecute: false, lastError: "" });
  commitState();
}

function applyDialogMessage(message, modeInput) {
  const mode = normalizeDialogMode(modeInput);
  if (!message) throw new Error("message is required");
  const hadLock = Boolean(state.execution.target_locked);
  setDialogStatus("thinking", { mode, pendingAutoExecute: false, lastError: "" });
  appendDialogMessage("user", message);

  const semantic = updateSemanticFromPrompt(message);
  const wantsExecute = mode === "auto" && looksLikeExecuteCommand(message);
  appendDialogMessage(
    "assistant",
    wantsExecute
      ? `Structured semantic task updated for ${semantic.target}. Auto mode will continue when the task is ready.`
      : `Structured semantic task updated for ${semantic.target}.`,
    {
      meta: `Action: ${wantsExecute ? "execute" : "update_task"} · Target: ${semantic.target}`,
      semanticTask: structuredClone(state.semantic),
      requestedAction: wantsExecute ? "execute" : "update_task",
    },
  );

  if (!wantsExecute) {
    setDialogStatus("ready", { mode, pendingAutoExecute: false, lastError: "" });
    commitState();
    return;
  }

  if (hadLock) {
    state.execution.target_locked = true;
    state.execution.phase = "waiting_execute";
    state.execution.pick_status = {
      phase: "waiting_execute",
      message: `Auto mode accepted ${semantic.target} and started execution.`,
      updated_at: nowSec(),
    };
    state.logs.stepper_phase = "waiting_execute";
    appendDialogMessage("system", "Auto mode accepted the task and started execution immediately.");
    setDialogStatus("auto_executing", { mode, pendingAutoExecute: false, lastError: "" });
    commitState();
    startExecuteFlow({ recordHistory: false });
    return;
  }

  appendDialogMessage("system", "Auto mode accepted the task and is waiting for target lock before executing.");
  setDialogStatus("awaiting_lock", { mode, pendingAutoExecute: true, lastError: "" });
  commitState();
}

function updateSemanticFromPrompt(prompt) {
  const target = inferTarget(prompt);
  const updatedAt = nowSec();
  state.semantic = {
    task: "pick",
    target_label: target,
    target_hint: target,
    gripper: "parallel_gripper",
    constraints: ["parallel_gripper"],
    excluded_labels: [],
    confidence: 0.93,
    need_human_confirm: false,
    reason: `Structured semantic task updated for ${target}.`,
    prompt_text: prompt,
    raw_json: JSON.stringify({ task: "pick", target_label: target }),
    updated_at: updatedAt,
    result: {
      task: "pick",
      target_label: target,
      target_hint: target,
    },
  };
  state.execution.target_locked = false;
  state.execution.pick_active = false;
  state.execution.phase = "semantic_ready";
  state.execution.pick_status = {
    phase: "semantic_ready",
    message: `Semantic task ready for ${target}`,
    updated_at: updatedAt,
  };
  state.execution.intervention = { active: false, source: "", label: "" };
  state.logs.stepper_phase = "semantic_ready";
  state.logs.intervention_badge = false;
  state.ui_feedback.events = [];
  state.ui_feedback.last_event_id = 0;
  appendEvent({
    category: "prompt",
    level: "info",
    message: `prompt submitted: ${prompt}`,
    data: { prompt },
    feedback: true,
  });
  return { target, updatedAt };
}

function applyOverride(payload) {
  const updatedAt = nowSec();
  const task = `${payload.task ?? state.semantic.task ?? "pick"}`;
  const targetLabel = `${payload.target_label ?? payload.target_hint ?? state.semantic.target_label ?? ""}`.trim();
  const targetHint = `${payload.target_hint ?? targetLabel}`.trim();
  const constraints = Array.isArray(payload.constraints)
    ? payload.constraints.map((item) => `${item}`)
    : ["parallel_gripper"];
  const gripper = `${payload.gripper ?? constraints[0] ?? "parallel_gripper"}`;
  state.semantic = {
    ...state.semantic,
    task,
    target_label: targetLabel,
    target_hint: targetHint,
    gripper,
    constraints,
    excluded_labels: Array.isArray(payload.excluded_labels)
      ? payload.excluded_labels.map((item) => `${item}`)
      : [],
    updated_at: updatedAt,
  };
  state.vision.detection = {
    ...state.vision.detection,
    target_label: targetLabel,
  };
  state.execution.intervention = {
    active: true,
    source: "override_api",
    label: targetLabel || targetHint,
  };
  state.logs.intervention_badge = true;
  if (state.execution.target_locked) {
    state.execution.phase = "waiting_execute";
    state.execution.pick_status = {
      phase: "waiting_execute",
      message: `Override applied for ${targetLabel || targetHint}`,
      updated_at: updatedAt,
    };
    state.logs.stepper_phase = "waiting_execute";
  }
  appendEvent({
    category: "override",
    level: "info",
    message: `override applied: ${targetLabel || targetHint}`,
    data: { label: targetLabel || targetHint },
    feedback: true,
  });
  commitState();
}

function applyReplan() {
  clearExecuteTimer();
  const promptText = `${state.semantic.prompt_text ?? ""}`.trim() || "pick the cup";
  const target = inferTarget(promptText);
  const updatedAt = nowSec();
  state.semantic = {
    ...state.semantic,
    target_label: target,
    target_hint: target,
    updated_at: updatedAt,
  };
  state.execution.phase = "semantic_ready";
  state.execution.target_locked = false;
  state.execution.pick_active = false;
  state.execution.intervention = { active: false, source: "", label: "" };
  state.execution.pick_status = {
    phase: "semantic_ready",
    message: `Re-plan requested for ${target}`,
    updated_at: updatedAt,
  };
  state.logs.stepper_phase = "semantic_ready";
  state.logs.intervention_badge = false;
  appendEvent({
    category: "replan",
    level: "info",
    message: `re-plan requested; replayed prompt: ${promptText}`,
    data: { prompt: promptText },
    feedback: true,
  });
  commitState();
}

function startExecuteFlow(options = {}) {
  const { recordHistory = true } = options;
  clearExecuteTimer();
  if (recordHistory) history.push({ type: "execute", at: nowSec(), payload: {} });
  const updatedAt = nowSec();
  state.execution.phase = "planning";
  state.execution.pick_active = false;
  state.execution.pick_status = {
    phase: "planning",
    message: "Planning grasp execution",
    updated_at: updatedAt,
  };
  state.logs.stepper_phase = "planning";
  appendEvent({
    category: "execution",
    level: "info",
    message: "execute request accepted",
    data: { phase: "planning" },
    feedback: true,
  });
  commitState();

  executeTimer = setTimeout(() => {
    const phaseAt = nowSec();
    if (executeOutcome === "error") {
      state.execution.phase = "error";
      state.execution.pick_active = false;
      state.execution.pick_status = {
        phase: "error",
        message: "Execution failed in mock gateway",
        updated_at: phaseAt,
      };
      state.logs.stepper_phase = "error";
      appendEvent({
        category: "execution",
        level: "error",
        message: "Execution failed in mock gateway",
        data: { phase: "error" },
        feedback: true,
      });
      commitState();
      return;
    }

    state.execution.phase = "executing";
    state.execution.pick_active = true;
    state.execution.pick_status = {
      phase: "executing",
      message: "Executing pick",
      updated_at: phaseAt,
    };
    state.logs.stepper_phase = "executing";
    commitState();

    executeTimer = setTimeout(() => {
      const completedAt = nowSec();
      state.execution.phase = "completed";
      state.execution.pick_active = false;
      state.execution.target_locked = false;
      state.execution.pick_status = {
        phase: "completed",
        message: "Execution completed",
        updated_at: completedAt,
      };
      state.logs.stepper_phase = "completed";
      appendEvent({
        category: "execution",
        level: "info",
        message: "Execution completed",
        data: { phase: "completed" },
        feedback: true,
      });
      commitState();
    }, 180);
  }, 180);
}

function clearExecuteTimer() {
  if (executeTimer !== null) {
    clearTimeout(executeTimer);
    executeTimer = null;
  }
}

function createScenario(name) {
  const base = createBaseState();
  if (name === "control_ready") {
    hydrateControlReady(base);
    return base;
  }
  if (name === "feedback_history") {
    hydrateControlReady(base);
    appendEventToState(base, {
      category: "prompt",
      level: "info",
      message: "historical bootstrap feedback",
      data: { historical: true },
      feedback: true,
    });
    return base;
  }
  if (name === "execute_error") {
    hydrateControlReady(base);
    return base;
  }
  return base;
}

function createBaseState() {
  return {
    connection: {
      backend_ready: true,
      backend_time: nowSec(),
      state_version: 1,
      host: HOST,
      port: PORT,
    },
    semantic: {
      task: "pick",
      target_label: "",
      target_hint: "",
      gripper: "parallel_gripper",
      constraints: ["parallel_gripper"],
      excluded_labels: [],
      updated_at: nowSec(),
      result: {},
    },
    vision: {
      detection: {},
      debug: {},
      debug_candidates: [],
      selected_candidate: null,
      candidate_summary: "",
      image_width: 1280,
      image_height: 720,
      updated_at: nowSec(),
    },
    execution: {
      phase: "idle",
      target_locked: false,
      pick_active: false,
      pick_status: {
        phase: "idle",
        message: "Waiting for semantic or vision updates.",
        updated_at: nowSec(),
      },
      intervention: { active: false, source: "", label: "" },
      grasp_proposals: {
        proposals: [],
        selected_index: 0,
        count: 0,
        updated_at: nowSec(),
      },
      backend_debug: {},
      updated_at: nowSec(),
    },
    tactile: createTactileState(),
    health: {
      healthy: true,
      issues: [],
      latest: [
        {
          node_name: "tactile_web_gateway",
          healthy: true,
          level: 0,
          message: "healthy",
          updated_at: nowSec(),
        },
      ],
      arm_state: {
        connected: true,
        moving: false,
        error: false,
        battery_voltage: 12.3,
      },
      updated_at: nowSec(),
    },
    logs: {
      stepper_phase: "idle",
      intervention_badge: false,
      events: [],
      updated_at: nowSec(),
    },
    ui_feedback: {
      events: [],
      last_event_id: 0,
    },
    dialog: createDialogState(),
  };
}

function hydrateControlReady(nextState) {
  const updatedAt = nowSec();
  nextState.semantic = {
    task: "pick",
    target_label: "cup",
    target_hint: "cup",
    gripper: "parallel_gripper",
    constraints: ["parallel_gripper"],
    excluded_labels: [],
    confidence: 0.96,
    need_human_confirm: false,
    reason: "Structured semantic task updated for cup.",
    prompt_text: "pick the cup on the table",
    raw_json: JSON.stringify({ task: "pick", target_label: "cup" }),
    updated_at: updatedAt,
    result: { task: "pick", target_label: "cup" },
  };
  nextState.vision = {
    detection: {
      accepted: true,
      target_label: "cup",
      confidence: 0.92,
      reason: "mock detection accepted",
      image_width: 1280,
      image_height: 720,
      bbox: {
        x_offset: 120,
        y_offset: 160,
        width: 260,
        height: 280,
        xyxy: [120, 160, 380, 440],
      },
    },
    debug: {
      top_k: 3,
    },
    debug_candidates: [
      {
        index: 0,
        label: "cup",
        confidence: 0.92,
        semantic_bonus: 0.21,
        score: 1.13,
        status: "selectable",
        bbox_xyxy: [120, 160, 380, 440],
        mask_pixels: 4521,
      },
      {
        index: 1,
        label: "bottle",
        confidence: 0.61,
        semantic_bonus: 0.1,
        score: 0.71,
        status: "selectable",
        bbox_xyxy: [500, 180, 660, 520],
        mask_pixels: 2312,
      },
      {
        index: 2,
        label: "remote",
        confidence: 0.42,
        semantic_bonus: 0.02,
        score: 0.44,
        status: "filtered_low_confidence",
        bbox_xyxy: [760, 220, 980, 360],
        mask_pixels: 940,
      },
    ],
    selected_candidate: {
      index: 0,
      label: "cup",
      confidence: 0.92,
      semantic_bonus: 0.21,
      score: 1.13,
      status: "selectable",
      bbox_xyxy: [120, 160, 380, 440],
      mask_pixels: 4521,
    },
    candidate_summary: "cup selected, bottle and remote remain as secondary candidates",
    image_width: 1280,
    image_height: 720,
    updated_at: updatedAt,
  };
  nextState.execution = {
    phase: "waiting_execute",
    target_locked: true,
    pick_active: false,
    pick_status: {
      phase: "waiting_execute",
      message: "Target locked and waiting for Execute",
      updated_at: updatedAt,
    },
    intervention: { active: false, source: "", label: "" },
    grasp_proposals: {
      proposals: [
        { confidence_score: 0.88, candidate_rank: 0 },
        { confidence_score: 0.76, candidate_rank: 1 },
      ],
      selected_index: 0,
      count: 2,
      updated_at: updatedAt,
    },
    backend_debug: {
      backend: "mock_grasp_backend",
    },
    updated_at: updatedAt,
  };
  nextState.logs.stepper_phase = "waiting_execute";
  appendEventToState(nextState, {
    category: "system",
    level: "info",
    message: "mock gateway ready",
    data: {},
    feedback: false,
  });
}

function createTactileState() {
  return {
    rows: 2,
    cols: 4,
    sequence_id: 42,
    frame_id: "mock_frame",
    forces: [0.1, 0.35, 0.65, 0.2, 0.12, 0.48, 0.74, 0.18],
    forces_fx: [0.02, 0.08, 0.11, 0.04, 0.03, 0.09, 0.12, 0.05],
    forces_fy: [0.01, 0.05, 0.09, 0.03, 0.02, 0.06, 0.1, 0.04],
    forces_fz: [0.07, 0.22, 0.45, 0.13, 0.08, 0.3, 0.52, 0.16],
    updated_at: nowSec(),
  };
}

function appendEventToState(targetState, input) {
  const event = createEvent(input);
  targetState.logs.events.push(event);
  targetState.logs.updated_at = event.created_at;
  if (input.feedback) {
    targetState.ui_feedback.events.push(event);
    targetState.ui_feedback.last_event_id = event.id;
  }
  trimEvents(targetState);
  return event;
}

function appendEvent(input) {
  return appendEventToState(state, input);
}

function createDialogState(mode = "review") {
  const normalizedMode = normalizeDialogMode(mode);
  return {
    session_id: `mock-${Math.random().toString(16).slice(2, 10)}`,
    mode: normalizedMode,
    status: "idle",
    status_label: dialogStatusLabel("idle"),
    pending_auto_execute: false,
    last_error: "",
    messages: [],
    updated_at: nowSec(),
  };
}

function normalizeDialogMode(mode) {
  return `${mode}`.toLowerCase() === "auto" ? "auto" : "review";
}

function dialogStatusLabel(status) {
  const labels = {
    idle: "Idle",
    thinking: "Thinking",
    ready: "Ready",
    awaiting_lock: "Awaiting Lock",
    auto_executing: "Auto Executing",
    error: "Error",
  };
  return labels[`${status}`] ?? `${status}`;
}

function ensureDialogState() {
  if (!state.dialog) state.dialog = createDialogState();
  return state.dialog;
}

function setDialogStatus(status, options = {}) {
  const dialog = ensureDialogState();
  dialog.status = `${status}`;
  dialog.status_label = dialogStatusLabel(status);
  if (typeof options.mode === "string") dialog.mode = normalizeDialogMode(options.mode);
  if (typeof options.pendingAutoExecute === "boolean") dialog.pending_auto_execute = options.pendingAutoExecute;
  if (typeof options.lastError === "string") dialog.last_error = options.lastError;
  dialog.updated_at = nowSec();
}

function appendDialogMessage(role, text, options = {}) {
  const dialog = ensureDialogState();
  const message = {
    id: ++dialogMessageId,
    role: `${role}`,
    text: `${text}`,
    meta: `${options.meta ?? ""}`,
    semantic_task: options.semanticTask ?? null,
    requested_action: `${options.requestedAction ?? ""}`,
    created_at: nowSec(),
  };
  dialog.messages.push(message);
  dialog.messages = dialog.messages.slice(-40);
  dialog.updated_at = message.created_at;
  return message;
}

function resetDialogSession() {
  const currentMode = normalizeDialogMode(state.dialog?.mode);
  state.dialog = createDialogState(currentMode);
}

function setDialogMode(modeInput) {
  const mode = normalizeDialogMode(modeInput);
  const dialog = ensureDialogState();
  dialog.mode = mode;
  dialog.pending_auto_execute = false;
  dialog.last_error = "";
  setDialogStatus("idle", { mode, pendingAutoExecute: false, lastError: "" });
  appendDialogMessage("system", `Dialog mode switched to ${mode[0].toUpperCase()}${mode.slice(1)}.`);
  commitState();
}

function looksLikeExecuteCommand(prompt) {
  const lowered = `${prompt}`.toLowerCase();
  return lowered.includes("execute") || lowered.includes("pick") || lowered.includes("grab");
}

function trimEvents(targetState) {
  targetState.logs.events = targetState.logs.events.slice(-40);
  targetState.ui_feedback.events = targetState.ui_feedback.events.slice(-48);
  targetState.ui_feedback.last_event_id = targetState.ui_feedback.events.length > 0
    ? targetState.ui_feedback.events[targetState.ui_feedback.events.length - 1].id
    : 0;
}

function createEvent({ category, level, message, data = {}, feedback = false }) {
  return {
    id: ++eventId,
    created_at: nowSec(),
    category: `${category}`,
    level: `${level}`,
    message: `${message}`,
    data,
    feedback,
  };
}

function commitState({ broadcast = true } = {}) {
  state.connection.backend_time = nowSec();
  state.connection.state_version += 1;
  state.execution.updated_at = nowSec();
  if (state.dialog) state.dialog.status_label = dialogStatusLabel(state.dialog.status);
  trimEvents(state);
  writeHistoryFile();
  if (broadcast && websocketEnabled) broadcastState();
}

function writeHistoryFile() {
  const payload = {
    scenario: currentScenario,
    executeOutcome,
    websocketEnabled,
    history,
    state,
  };
  mkdirSync(dirname(HISTORY_FILE), { recursive: true });
  writeFileSync(HISTORY_FILE, JSON.stringify(payload, null, 2));
}

function broadcastState() {
  const encoded = JSON.stringify(state);
  for (const socket of clients) {
    if (socket.readyState === socket.OPEN) socket.send(encoded);
  }
}

function inferTarget(prompt) {
  const lowered = `${prompt}`.toLowerCase();
  if (lowered.includes("remote")) return "remote";
  if (lowered.includes("bottle")) return "bottle";
  if (lowered.includes("cup")) return "cup";
  if (lowered.includes("mouse")) return "mouse";
  return "target";
}

function deepMerge(base, patch) {
  if (!isPlainObject(base) || !isPlainObject(patch)) return structuredClone(patch);
  const result = { ...base };
  for (const [key, value] of Object.entries(patch)) {
    if (Array.isArray(value)) {
      result[key] = structuredClone(value);
      continue;
    }
    if (isPlainObject(value) && isPlainObject(result[key])) {
      result[key] = deepMerge(result[key], value);
      continue;
    }
    result[key] = structuredClone(value);
  }
  return result;
}

function isPlainObject(value) {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function nowSec() {
  return Date.now() / 1000;
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function readJson(request) {
  const chunks = [];
  for await (const chunk of request) chunks.push(chunk);
  if (chunks.length === 0) return {};
  return JSON.parse(Buffer.concat(chunks).toString("utf8"));
}

function json(response, status, payload) {
  response.statusCode = status;
  response.setHeader("Content-Type", "application/json");
  response.end(JSON.stringify(payload));
}

function svg(response, status, markup) {
  response.statusCode = status;
  response.setHeader("Content-Type", "image/svg+xml");
  response.end(markup);
}

function buildStreamSvg(pathname) {
  const label = pathname.split("/").pop()?.replace(".mjpeg", "").replace(/_/g, " ") ?? "stream";
  return `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
  <rect width="1280" height="720" fill="#0f1625" />
  <rect x="56" y="56" width="1168" height="608" rx="28" fill="#16233a" stroke="#3ec9a7" stroke-width="6" />
  <text x="96" y="150" fill="#f6f7fb" font-family="monospace" font-size="46">mock ${label}</text>
  <text x="96" y="220" fill="#9fb1cb" font-family="monospace" font-size="26">${pathname}</text>
</svg>`;
}
