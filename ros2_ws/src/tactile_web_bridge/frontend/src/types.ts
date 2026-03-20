export type UiLevel = "info" | "warn" | "error";
export type DialogMode = "review" | "auto";
export type DialogRole = "user" | "assistant" | "system";
export type DialogReplyLanguage = "zh" | "en";

export interface UiEvent {
  id: number;
  created_at: number;
  category: string;
  level: UiLevel;
  message: string;
  data: Record<string, unknown>;
}

export interface SemanticState {
  task?: string;
  target_label?: string;
  target_hint?: string;
  gripper?: string;
  constraints?: string[];
  excluded_labels?: string[];
  confidence?: number;
  need_human_confirm?: boolean;
  reason?: string;
  prompt_text?: string;
  raw_json?: string;
  updated_at?: number;
  result?: Record<string, unknown>;
}

export interface DialogMessageState {
  id: number;
  role: DialogRole;
  text: string;
  meta?: string;
  semantic_task?: SemanticState | null;
  requested_action?: string;
  created_at: number;
}

export interface DialogState {
  session_id: string;
  mode: DialogMode;
  reply_language: DialogReplyLanguage;
  status: string;
  status_label?: string;
  pending_auto_execute: boolean;
  last_error: string;
  visual_focus?: Record<string, unknown> | null;
  messages: DialogMessageState[];
  updated_at: number;
}

export interface CandidateDebug {
  index: number;
  track_id?: number;
  track_new?: boolean;
  track_seen_count?: number;
  track_consecutive_hits?: number;
  track_age_sec?: number;
  track_label_age_sec?: number;
  track_relabel_reason?: string;
  label: string;
  raw_label?: string;
  canonical_label?: string;
  label_zh?: string;
  display_label?: string;
  label_source?: string;
  confidence: number;
  confidence_floor?: number;
  semantic_bonus: number;
  score: number;
  status: string;
  bbox_xyxy?: number[];
  mask_pixels?: number;
}

export interface DetectionState {
  accepted?: boolean;
  target_label?: string;
  confidence?: number;
  reason?: string;
  image_width?: number;
  image_height?: number;
  bbox?: {
    x_offset: number;
    y_offset: number;
    width: number;
    height: number;
    xyxy: number[];
  } | null;
}

export interface UiState {
  connection: {
    backend_ready: boolean;
    backend_time: number;
    state_version: number;
    host: string;
    port: number;
  };
  semantic: SemanticState;
  vision: {
    detection: DetectionState;
    debug: Record<string, unknown>;
    debug_candidates: CandidateDebug[];
    selected_candidate?: CandidateDebug | null;
    candidate_summary?: string;
    image_width: number;
    image_height: number;
    updated_at: number;
  };
  execution: {
    phase: string;
    target_locked: boolean;
    pick_active: boolean;
    pick_status: Record<string, unknown>;
    intervention: {
      active: boolean;
      source: string;
      label: string;
    };
    grasp_proposals: {
      proposals: Record<string, unknown>[];
      selected_index: number;
      count: number;
      updated_at: number;
    };
    backend_debug: Record<string, unknown>;
    updated_at: number;
  };
  tactile: {
    rows?: number;
    cols?: number;
    sequence_id?: number;
    frame_id?: string;
    forces?: number[];
    forces_fx?: number[];
    forces_fy?: number[];
    forces_fz?: number[];
    updated_at?: number;
  };
  health: {
    healthy: boolean;
    issues: Record<string, unknown>[];
    latest: Record<string, unknown>[];
    arm_state: Record<string, unknown>;
    updated_at: number;
  };
  logs: {
    stepper_phase: string;
    intervention_badge: boolean;
    events: UiEvent[];
    updated_at: number;
  };
  ui_feedback: {
    events: UiEvent[];
    last_event_id: number;
  };
  dialog?: DialogState;
}

export interface BootstrapResponse {
  state: UiState;
  streams: {
    rgb: string;
    detection_overlay: string;
    grasp_overlay: string;
  };
  frontend_ready: boolean;
}

export interface SemanticDraft {
  task: string;
  target_label: string;
  target_hint: string;
  gripper: string;
  constraints: string[];
  excluded_labels: string[];
}
