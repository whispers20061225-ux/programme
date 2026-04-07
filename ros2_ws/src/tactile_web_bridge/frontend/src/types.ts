export type UiLevel = "info" | "warn" | "error";
export type DialogMode = "review" | "auto";
export type DialogRole = "user" | "assistant" | "system";
export type DialogReplyLanguage = "zh" | "en";

export interface TargetInstanceHint {
  track_id?: number;
  bbox_xyxy?: number[];
  point_px?: number[];
  source?: string;
}

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
  target_instance?: TargetInstanceHint | null;
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

export interface TaskGoalState {
  schema_version?: string;
  goal_id?: string;
  parent_goal_id?: string;
  step_id?: string;
  step_index?: number;
  goal_type?: string;
  task?: string;
  target_label?: string;
  target_hint?: string;
  target_attributes?: string[];
  target_relations?: string[];
  target_instance?: TargetInstanceHint | null;
  target_part_query?: string;
  preferred_part_tags?: string[];
  forbidden_part_tags?: string[];
  grasp_region?: string;
  preferred_grasp_family?: string;
  placement_label?: string;
  preferred_approach_dirs?: string[];
  forbidden_approach_dirs?: string[];
  transport_constraints?: string[];
  execution_constraints?: string[];
  excluded_labels?: string[];
  reserved_next_skills?: string[];
  confidence?: number;
  min_affordance_score?: number;
  min_grasp_quality?: number;
  max_forbidden_overlap?: number;
  max_target_staleness_sec?: number;
  need_human_confirm?: boolean;
  start_search_sweep?: boolean;
  allow_instance_grounding?: boolean;
  require_target_lock?: boolean;
  allow_replan?: boolean;
  allow_rescan?: boolean;
  strict_part_match?: boolean;
  strict_approach_match?: boolean;
  max_retries?: number;
  planning_timeout_sec?: number;
  execution_timeout_sec?: number;
  success_criteria?: string;
  failure_policy?: string;
  verify_policy?: string;
  recovery_policy?: string;
  handoff_context_json?: string;
  reason?: string;
  prompt_text?: string;
  raw_json?: string;
  updated_at?: number;
}

export interface TaskExecutionStatusState {
  goal_id?: string;
  parent_goal_id?: string;
  step_id?: string;
  phase?: string;
  current_skill?: string;
  skill_status_code?: string;
  message?: string;
  error_code?: string;
  target_label?: string;
  target_hint?: string;
  target_part_query?: string;
  grasp_region?: string;
  active?: boolean;
  success?: boolean;
  requires_confirmation?: boolean;
  target_locked?: boolean;
  target_candidate_visible?: boolean;
  pick_active?: boolean;
  retry_count?: number;
  max_retries?: number;
  grounded_track_id?: number;
  selected_candidate_id?: number;
  progress?: number;
  grounding_confidence?: number;
  best_grasp_quality?: number;
  best_affordance_score?: number;
  verification_score?: number;
  recommended_recovery?: string;
  updated_at?: number;
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
  yolo_confidence?: number;
  semantic_confidence?: number;
  track_stability?: number;
  selection_confidence?: number;
  selection_confidence_smoothed?: number;
  confidence_floor?: number;
  semantic_bonus: number;
  score: number;
  status: string;
  display_status?: string;
  selection_status?: string;
  instance_match?: boolean;
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
    pending_execute: {
      active: boolean;
      source: string;
      message: string;
      updated_at: number;
    };
    task_goal?: TaskGoalState;
    task_status?: TaskExecutionStatusState;
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
  target_instance: TargetInstanceHint | null;
  gripper: string;
  constraints: string[];
  excluded_labels: string[];
}
