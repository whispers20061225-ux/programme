import type {
  BootstrapResponse,
  CandidateDebug,
  DialogMessageState,
  SemanticDraft,
  SemanticState,
  UiEvent,
  UiLevel,
  UiState,
} from "./types";

export type StreamMap = BootstrapResponse["streams"];
export type StreamName = keyof StreamMap;
export type BusyAction = "dialog" | "execute" | "replan" | "dialog-reset" | null;
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
    pick_status: {},
    intervention: { active: false, source: "", label: "" },
    grasp_proposals: { proposals: [], selected_index: 0, count: 0, updated_at: 0 },
    backend_debug: {},
    updated_at: 0,
  },
  tactile: { rows: 0, cols: 0, forces: [], forces_fx: [], forces_fy: [], forces_fz: [], updated_at: 0 },
  health: { healthy: true, issues: [], latest: [], arm_state: {}, updated_at: 0 },
  logs: { stepper_phase: "idle", intervention_badge: false, events: [], updated_at: 0 },
  ui_feedback: { events: [], last_event_id: 0 },
  dialog: {
    session_id: "",
    mode: "review",
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

export function semanticToDraft(semantic: SemanticState): SemanticDraft {
  const constraints = [...(semantic.constraints ?? [])];
  const gripper = semantic.gripper || constraints[0] || "parallel_gripper";
  if (!constraints.includes(gripper)) constraints.unshift(gripper);
  return {
    task: semantic.task || "pick",
    target_label: semantic.target_label || "",
    target_hint: semantic.target_hint || semantic.target_label || "",
    gripper,
    constraints,
    excluded_labels: semantic.excluded_labels ?? [],
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
