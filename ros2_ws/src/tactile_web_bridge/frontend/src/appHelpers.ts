import type {
  BootstrapResponse,
  CandidateDebug,
  DialogMessageState,
  SemanticDraft,
  SemanticState,
  TargetInstanceHint,
  UiEvent,
  UiLevel,
  UiState,
} from "./types";

export type StreamMap = BootstrapResponse["streams"];
export type StreamName = keyof StreamMap;
export type BusyAction =
  | "dialog"
  | "execute"
  | "replan"
  | "dialog-reset"
  | "return-home"
  | "scene-reset"
  | "debug-open"
  | "tactile-mode"
  | "tactile-tare"
  | "tactile-clear-tare"
  | null;
export type InterventionState = "idle" | "draft" | "applied";
export type PillTone = UiLevel | "success" | "neutral";

export type ToastItem = {
  id: number;
  level: UiLevel;
  title: string;
  message: string;
};

export type ChatMessage = DialogMessageState;

export type OverlayBox = {
  bbox: number[];
  label: string;
  tone: "selected" | "hovered" | "candidate";
};

export type DisplayEvent = {
  event: UiEvent;
  origin: "backend" | "frontend";
};

export const STEP_PHASES = [
  "idle",
  "semantic_ready",
  "vision_ready",
  "target_locked",
  "waiting_execute",
  "planning",
  "executing",
  "completed",
  "error",
] as const;

export const STREAM_OPTIONS: Array<{ key: StreamName; label: string }> = [
  { key: "rgb", label: "RGB" },
  { key: "detection_overlay", label: "Detection Overlay" },
  { key: "grasp_overlay", label: "Grasp Overlay" },
];

export const DEFAULT_STATE: UiState = {
  connection: { backend_ready: false, backend_time: 0, state_version: 0, host: "", port: 0 },
  semantic: {
    task: "pick",
    target_label: "",
    target_hint: "",
    target_instance: null,
    gripper: "parallel_gripper",
    constraints: ["parallel_gripper"],
    excluded_labels: [],
    updated_at: 0,
    result: {},
  },
  vision: {
    detection: {},
    debug: {},
    debug_candidates: [],
    selected_candidate: null,
    candidate_summary: "",
    image_width: 0,
    image_height: 0,
    updated_at: 0,
  },
  execution: {
    phase: "idle",
    target_locked: false,
    pick_active: false,
    pending_execute: { active: false, source: "", message: "", updated_at: 0 },
    pick_status: {},
    intervention: { active: false, source: "", label: "" },
    grasp_proposals: { proposals: [], selected_index: 0, count: 0, updated_at: 0 },
    backend_debug: {},
    updated_at: 0,
  },
  tactile: {
    rows: 0,
    cols: 0,
    sequence_id: 0,
    frame_id: "",
    forces: [],
    forces_fx: [],
    forces_fy: [],
    forces_fz: [],
    torques_tx: [],
    torques_ty: [],
    torques_tz: [],
    total_fx: 0,
    total_fy: 0,
    total_fz: 0,
    contact_active: false,
    contact_score: 0,
    baseline_ready: false,
    sensor_layout: "",
    requested_mode: "simulation",
    active_mode: "simulation",
    source_name: "",
    source_connected: false,
    device_addr: 0,
    function_code: 0,
    start_addr: 0,
    returned_data_len_hint: 0,
    frame_len_declared: 0,
    payload_len_observed: 0,
    payload_value_count: 0,
    transport_rate_hz: 0,
    publish_rate_hz: 0,
    lrc_ok: false,
    parser_state: "",
    mapping_state: "",
    status_text: "",
    raw_frame_hex: "",
    tare_active: false,
    tare_updated_at: 0,
    tare_message: "",
    freeze_warning: false,
    freeze_duration_sec: 0,
    freeze_repeat_count: 0,
    stamp_sec: 0,
    updated_at: 0,
  },
  gripper_profile: {
    profile_id: "",
    target_label: "",
    object_type: "",
    source: "",
    kp: 0,
    kd: 0,
    target_force: 0,
    contact_threshold: 0,
    safety_max: 0,
    confidence: 0,
    updated_at: 0,
    raw: {},
  },
  health: { healthy: true, issues: [], latest: [], arm_state: {}, updated_at: 0 },
  logs: { stepper_phase: "idle", intervention_badge: false, events: [], updated_at: 0 },
  ui_feedback: { events: [], last_event_id: 0 },
  dialog: {
    session_id: "",
    mode: "auto",
    reply_language: "zh",
    status: "idle",
    status_label: "Idle",
    pending_auto_execute: false,
    last_error: "",
    visual_focus: null,
    messages: [],
    updated_at: 0,
  },
};

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

export function normalizeUiState(next: UiState | Partial<UiState> | null | undefined): UiState {
  const root = isRecord(next) ? next : {};
  const semantic = (isRecord(root.semantic) ? root.semantic : {}) as Partial<UiState["semantic"]>;
  const vision = (isRecord(root.vision) ? root.vision : {}) as Partial<UiState["vision"]>;
  const execution = (isRecord(root.execution) ? root.execution : {}) as Partial<UiState["execution"]>;
  const tactile = (isRecord(root.tactile) ? root.tactile : {}) as Partial<UiState["tactile"]>;
  const health = (isRecord(root.health) ? root.health : {}) as Partial<UiState["health"]>;
  const logs = (isRecord(root.logs) ? root.logs : {}) as Partial<UiState["logs"]>;
  const uiFeedback = (isRecord(root.ui_feedback) ? root.ui_feedback : {}) as Partial<UiState["ui_feedback"]>;
  const gripperProfile = (isRecord(root.gripper_profile) ? root.gripper_profile : {}) as Partial<NonNullable<UiState["gripper_profile"]>>;
  const defaultDialog = DEFAULT_STATE.dialog ?? {
    session_id: "",
    mode: "auto",
    reply_language: "zh",
    status: "idle",
    status_label: "Idle",
    pending_auto_execute: false,
    last_error: "",
    visual_focus: null,
    messages: [],
    updated_at: 0,
  };
  const dialog = (isRecord(root.dialog) ? root.dialog : null) as Partial<NonNullable<UiState["dialog"]>> | null;

  return {
    ...DEFAULT_STATE,
    ...root,
    connection: {
      ...DEFAULT_STATE.connection,
      ...(isRecord(root.connection) ? root.connection : {}),
    },
    semantic: {
      ...DEFAULT_STATE.semantic,
      ...semantic,
      constraints: Array.isArray(semantic.constraints)
        ? semantic.constraints as string[]
        : DEFAULT_STATE.semantic.constraints,
      excluded_labels: Array.isArray(semantic.excluded_labels)
        ? semantic.excluded_labels as string[]
        : DEFAULT_STATE.semantic.excluded_labels,
    },
    vision: {
      ...DEFAULT_STATE.vision,
      ...vision,
      detection: {
        ...DEFAULT_STATE.vision.detection,
        ...(isRecord(vision.detection) ? vision.detection : {}),
      },
      debug: isRecord(vision.debug) ? vision.debug : DEFAULT_STATE.vision.debug,
      debug_candidates: Array.isArray(vision.debug_candidates)
        ? vision.debug_candidates as CandidateDebug[]
        : DEFAULT_STATE.vision.debug_candidates,
      selected_candidate: (vision.selected_candidate as CandidateDebug | null | undefined) ?? DEFAULT_STATE.vision.selected_candidate,
    },
    execution: {
      ...DEFAULT_STATE.execution,
      ...execution,
      pending_execute: {
        ...DEFAULT_STATE.execution.pending_execute,
        ...(isRecord(execution.pending_execute) ? execution.pending_execute : {}),
      },
      pick_status: isRecord(execution.pick_status) ? execution.pick_status : DEFAULT_STATE.execution.pick_status,
      intervention: {
        ...DEFAULT_STATE.execution.intervention,
        ...(isRecord(execution.intervention) ? execution.intervention : {}),
      },
      grasp_proposals: {
        ...DEFAULT_STATE.execution.grasp_proposals,
        ...(isRecord(execution.grasp_proposals) ? execution.grasp_proposals : {}),
        proposals:
          isRecord(execution.grasp_proposals) && Array.isArray(execution.grasp_proposals.proposals)
            ? execution.grasp_proposals.proposals as Record<string, unknown>[]
            : DEFAULT_STATE.execution.grasp_proposals.proposals,
      },
      backend_debug: isRecord(execution.backend_debug) ? execution.backend_debug : DEFAULT_STATE.execution.backend_debug,
      task_goal: isRecord(execution.task_goal) ? execution.task_goal as UiState["execution"]["task_goal"] : undefined,
      task_status: isRecord(execution.task_status) ? execution.task_status as UiState["execution"]["task_status"] : undefined,
    },
    tactile: {
      ...DEFAULT_STATE.tactile,
      ...tactile,
      forces: Array.isArray(tactile.forces)
        ? tactile.forces as number[]
        : DEFAULT_STATE.tactile.forces,
      forces_fx: Array.isArray(tactile.forces_fx)
        ? tactile.forces_fx as number[]
        : DEFAULT_STATE.tactile.forces_fx,
      forces_fy: Array.isArray(tactile.forces_fy)
        ? tactile.forces_fy as number[]
        : DEFAULT_STATE.tactile.forces_fy,
      forces_fz: Array.isArray(tactile.forces_fz)
        ? tactile.forces_fz as number[]
        : DEFAULT_STATE.tactile.forces_fz,
      torques_tx: Array.isArray(tactile.torques_tx)
        ? tactile.torques_tx as number[]
        : DEFAULT_STATE.tactile.torques_tx,
      torques_ty: Array.isArray(tactile.torques_ty)
        ? tactile.torques_ty as number[]
        : DEFAULT_STATE.tactile.torques_ty,
      torques_tz: Array.isArray(tactile.torques_tz)
        ? tactile.torques_tz as number[]
        : DEFAULT_STATE.tactile.torques_tz,
    },
    gripper_profile: {
      ...DEFAULT_STATE.gripper_profile,
      ...gripperProfile,
      raw: isRecord(gripperProfile.raw) ? gripperProfile.raw : DEFAULT_STATE.gripper_profile?.raw,
    },
    health: {
      ...DEFAULT_STATE.health,
      ...health,
      issues: Array.isArray(health.issues) ? health.issues as Record<string, unknown>[] : DEFAULT_STATE.health.issues,
      latest: Array.isArray(health.latest) ? health.latest as Record<string, unknown>[] : DEFAULT_STATE.health.latest,
      arm_state: isRecord(health.arm_state) ? health.arm_state : DEFAULT_STATE.health.arm_state,
    },
    logs: {
      ...DEFAULT_STATE.logs,
      ...logs,
      events: Array.isArray(logs.events) ? logs.events as UiEvent[] : DEFAULT_STATE.logs.events,
    },
    ui_feedback: {
      ...DEFAULT_STATE.ui_feedback,
      ...uiFeedback,
      events: Array.isArray(uiFeedback.events) ? uiFeedback.events as UiEvent[] : DEFAULT_STATE.ui_feedback.events,
    },
    dialog: dialog
      ? {
          ...defaultDialog,
          ...dialog,
          messages: Array.isArray(dialog.messages) ? dialog.messages as DialogMessageState[] : defaultDialog.messages,
        }
      : defaultDialog,
  };
}

export function semanticToDraft(semantic: SemanticState): SemanticDraft {
  const constraints = [...(semantic.constraints ?? [])];
  const gripper = semantic.gripper || constraints[0] || "parallel_gripper";
  if (!constraints.includes(gripper)) constraints.unshift(gripper);
  return {
    task: semantic.task || "pick",
    target_label: semantic.target_label || "",
    target_hint: semantic.target_hint || semantic.target_label || "",
    target_instance: cloneTargetInstance(semantic.target_instance),
    gripper,
    constraints,
    excluded_labels: semantic.excluded_labels ?? [],
  };
}

export function cloneTargetInstance(targetInstance: TargetInstanceHint | null | undefined): TargetInstanceHint | null {
  if (!targetInstance) return null;
  return {
    track_id:
      typeof targetInstance.track_id === "number" && Number.isFinite(targetInstance.track_id)
        ? targetInstance.track_id
        : undefined,
    bbox_xyxy: Array.isArray(targetInstance.bbox_xyxy) ? [...targetInstance.bbox_xyxy] : undefined,
    point_px: Array.isArray(targetInstance.point_px) ? [...targetInstance.point_px] : undefined,
    source: typeof targetInstance.source === "string" ? targetInstance.source : undefined,
  };
}

export function bboxCenterPoint(bbox: number[] | null | undefined): number[] | undefined {
  if (!Array.isArray(bbox) || bbox.length !== 4) return undefined;
  return [
    Math.round((Number(bbox[0]) + Number(bbox[2])) * 0.5),
    Math.round((Number(bbox[1]) + Number(bbox[3])) * 0.5),
  ];
}

export function candidateToTargetInstance(candidate: CandidateDebug | null | undefined): TargetInstanceHint | null {
  if (!candidate) return null;
  const bbox_xyxy = Array.isArray(candidate.bbox_xyxy) ? [...candidate.bbox_xyxy] : undefined;
  const point_px = bboxCenterPoint(bbox_xyxy);
  const track_id =
    typeof candidate.track_id === "number" && Number.isFinite(candidate.track_id) && candidate.track_id > 0
      ? candidate.track_id
      : undefined;
  if (!track_id && !bbox_xyxy?.length) return null;
  return {
    track_id,
    bbox_xyxy,
    point_px,
    source: "vision_candidate",
  };
}

export function nowSec(): number {
  return Date.now() / 1000;
}

export function getErrorMessage(error: unknown, fallback: string): string {
  return error instanceof Error && error.message.trim() ? error.message.trim() : fallback;
}

export function readString(record: Record<string, unknown> | undefined, key: string, fallback = ""): string {
  const value = record?.[key];
  return typeof value === "string" ? value : fallback;
}

export function readNumber(record: Record<string, unknown> | undefined, key: string, fallback = 0): number {
  const value = record?.[key];
  return typeof value === "number" && Number.isFinite(value) ? value : fallback;
}

export function summarizeValue(value: unknown): string {
  if (typeof value === "string") return value.trim() || "--";
  if (typeof value === "number") return Number.isInteger(value) ? `${value}` : value.toFixed(3);
  if (typeof value === "boolean") return value ? "true" : "false";
  if (Array.isArray(value)) return value.length > 0 ? value.slice(0, 6).map(summarizeValue).join(", ") : "--";
  if (value && typeof value === "object") {
    try {
      const serialized = JSON.stringify(value);
      return serialized.length > 120 ? `${serialized.slice(0, 117)}...` : serialized;
    } catch {
      return "[object]";
    }
  }
  return "--";
}

export function humanizeKey(key: string): string {
  return key.replace(/_/g, " ").replace(/\b\w/g, (token) => token.toUpperCase());
}

export function formatTimestamp(seconds: number | undefined): string {
  if (!seconds || !Number.isFinite(seconds)) return "--";
  return new Date(seconds * 1000).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" });
}

export function formatNumber(value: number | undefined, digits = 3): string {
  return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "--";
}

export function parseConstraintsInput(value: string): string[] {
  const seen = new Set<string>();
  const result: string[] = [];
  for (const token of value.split(/[,|\n]/)) {
    const normalized = token.trim();
    if (!normalized || seen.has(normalized)) continue;
    seen.add(normalized);
    result.push(normalized);
  }
  return result;
}

export function createFrontendEvent(
  id: number,
  category: string,
  level: UiLevel,
  message: string,
  data: Record<string, unknown> = {},
): UiEvent {
  return { id, created_at: nowSec(), category, level, message, data };
}

export function phaseLabel(phase: string): string {
  const labels: Record<string, string> = {
    idle: "Idle",
    semantic_ready: "Semantic Ready",
    vision_ready: "Vision Ready",
    target_locked: "Target Locked",
    waiting_execute: "Waiting Execute",
    planning: "Planning",
    executing: "Executing",
    completed: "Completed",
    error: "Error",
  };
  return labels[phase] ?? phase;
}

export function sparklinePoints(values: number[], width = 320, height = 120): string {
  if (values.length === 0) return "";
  const safeValues = values.map((value) => (Number.isFinite(value) ? value : 0));
  const minValue = Math.min(...safeValues);
  const maxValue = Math.max(...safeValues);
  const range = maxValue - minValue || 1;
  return safeValues
    .map((value, index) => {
      const x = safeValues.length === 1 ? width / 2 : (index / (safeValues.length - 1)) * width;
      const y = height - ((value - minValue) / range) * height;
      return `${x.toFixed(2)},${y.toFixed(2)}`;
    })
    .join(" ");
}

export function heatColor(value: number, maxMagnitude: number): string {
  const ratio = Math.min(1, Math.abs(value) / Math.max(maxMagnitude, 1e-6));
  const hue = 200 - ratio * 155;
  const saturation = 74 + ratio * 10;
  const lightness = 14 + ratio * 42;
  return `hsl(${hue} ${saturation}% ${lightness}%)`;
}

export function findCandidate(candidates: CandidateDebug[], index: number | null): CandidateDebug | null {
  return index === null ? null : candidates.find((candidate) => candidate.index === index) ?? null;
}
