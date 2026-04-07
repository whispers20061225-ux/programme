import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { NavLink, Navigate, Route, Routes } from "react-router-dom";
import {
  postDialogMessage,
  postDialogMode,
  postDialogReplyLanguage,
  postDialogReset,
  postExecute,
  postOpenDebugViews,
  postOverride,
  postResetScene,
  postReplan,
  postReturnHome,
} from "./api";
import { ControlPage } from "./ControlPage";
import { LogsPage } from "./LogsPage";
import { TactilePage } from "./TactilePage";
import { VisionPage } from "./VisionPage";
import {
  candidateToTargetInstance,
  createFrontendEvent,
  DEFAULT_STATE,
  type InterventionState,
  getErrorMessage,
  parseConstraintsInput,
  semanticToDraft,
  type ToastItem,
} from "./appHelpers";
import { type BackendConnectionPhase, useBackendState } from "./appUi";
import type {
  CandidateDebug,
  DialogMode,
  DialogReplyLanguage,
  SemanticDraft,
  UiEvent,
  UiLevel,
} from "./types";

function App() {
  const { state, streams, connectionPhase, loading, setState, waitForState } = useBackendState();
  const [draft, setDraft] = useState<SemanticDraft>(() => semanticToDraft(DEFAULT_STATE.semantic));
  const [draftDirty, setDraftDirty] = useState(false);
  const [busyAction, setBusyAction] = useState<
    "dialog" | "execute" | "replan" | "dialog-reset" | "return-home" | "scene-reset" | "debug-open" | null
  >(null);
  const [toasts, setToasts] = useState<ToastItem[]>([]);
  const [frontendEvents, setFrontendEvents] = useState<UiEvent[]>([]);
  const [backendEventFloorId, setBackendEventFloorId] = useState(0);
  const [sessionMeta, setSessionMeta] = useState({ draftSeen: false, applied: false });
  const [draftSource, setDraftSource] = useState("");
  const [draftLabel, setDraftLabel] = useState("");
  const [estopOpen, setEstopOpen] = useState(false);
  const toastIdRef = useRef(0);
  const frontendEventIdRef = useRef(0);
  const lastFeedbackIdRef = useRef(0);
  const bootstrapFeedbackSyncedRef = useRef(false);
  const lastConnectionPhaseRef = useRef<BackendConnectionPhase | null>(null);
  const connectionWasEstablishedRef = useRef(false);
  const draftDirtyRef = useRef(false);

  useEffect(() => {
    draftDirtyRef.current = draftDirty;
  }, [draftDirty]);

  const pushToast = useCallback((level: UiLevel, title: string, message: string) => {
    const id = ++toastIdRef.current;
    setToasts((items) => [...items, { id, level, title, message }].slice(-4));
    const timeoutMs = level === "error" ? 5600 : level === "warn" ? 6200 : 5200;
    window.setTimeout(() => setToasts((items) => items.filter((item) => item.id !== id)), timeoutMs);
  }, []);

  const pushFrontendEvent = useCallback((category: string, level: UiLevel, message: string, data: Record<string, unknown> = {}) => {
    const id = ++frontendEventIdRef.current;
    setFrontendEvents((items) => [...items, createFrontendEvent(id, category, level, message, data)]);
  }, []);

  const resetDraftState = useCallback(() => {
    draftDirtyRef.current = false;
    setDraftDirty(false);
    setDraftSource("");
    setDraftLabel("");
  }, []);

  const markDraftDirty = useCallback((source: string, label: string) => {
    if (!draftDirtyRef.current) pushFrontendEvent("draft", "warn", "manual override draft started", { source, label });
    draftDirtyRef.current = true;
    setDraftDirty(true);
    setDraftSource(source);
    setDraftLabel(label);
    setSessionMeta((current) => ({ ...current, draftSeen: true }));
  }, [pushFrontendEvent]);

  useEffect(() => {
    if (!draftDirty) setDraft(semanticToDraft(state.semantic));
  }, [draftDirty, state.semantic, state.semantic.updated_at]);

  useEffect(() => {
    if (!bootstrapFeedbackSyncedRef.current && !loading && state.connection.state_version > 0) {
      lastFeedbackIdRef.current = state.ui_feedback.last_event_id;
      bootstrapFeedbackSyncedRef.current = true;
    }
  }, [loading, state.connection.state_version, state.ui_feedback.last_event_id]);

  useEffect(() => {
    if (state.ui_feedback.last_event_id <= lastFeedbackIdRef.current) return;
    const freshEvents = state.ui_feedback.events.filter((event) => event.id > lastFeedbackIdRef.current);
    freshEvents.forEach((event) => pushToast(event.level, event.category.toUpperCase(), event.message));
    if (freshEvents.length > 0) lastFeedbackIdRef.current = freshEvents[freshEvents.length - 1].id;
  }, [pushToast, state.ui_feedback]);

  useEffect(() => {
    if (connectionPhase === "connected") {
      if (connectionWasEstablishedRef.current && lastConnectionPhaseRef.current !== "connected") {
        pushToast("info", "Connection", "WebSocket connection restored");
        pushFrontendEvent("connection", "info", "websocket connection restored");
      }
      connectionWasEstablishedRef.current = true;
      lastConnectionPhaseRef.current = "connected";
      return;
    }

    if (connectionPhase === "degraded") {
      if (lastConnectionPhaseRef.current !== "degraded") {
        pushToast("warn", "Connection", "WebSocket dropped, using HTTP fallback");
        pushFrontendEvent("connection", "warn", "websocket degraded, http fallback active");
      }
      lastConnectionPhaseRef.current = "degraded";
      return;
    }

    if (connectionPhase === "reconnecting") {
      if (connectionWasEstablishedRef.current && lastConnectionPhaseRef.current !== "reconnecting") {
        pushToast("warn", "Connection", "WebSocket disconnected, retrying");
        pushFrontendEvent("connection", "warn", "websocket connection lost");
      }
      lastConnectionPhaseRef.current = "reconnecting";
      return;
    }

    lastConnectionPhaseRef.current = connectionPhase;
  }, [connectionPhase, pushFrontendEvent, pushToast]);

  useEffect(() => {
    if (state.execution.intervention.active) setSessionMeta((current) => ({ ...current, applied: true }));
  }, [state.execution.intervention.active]);

  const visibleBackendEvents = useMemo(() => state.logs.events.filter((event) => event.id > backendEventFloorId), [backendEventFloorId, state.logs.events]);
  const interventionState: InterventionState = draftDirty ? "draft" : state.execution.intervention.active ? "applied" : "idle";
  const currentTarget = draft.target_label || draft.target_hint;
  const dialogState = state.dialog ?? DEFAULT_STATE.dialog!;
  const dialogMessages = dialogState.messages ?? [];
  const connectionDisplay = useMemo(() => {
    switch (connectionPhase) {
      case "bootstrapping":
        return { dotClass: "pending", label: "Bootstrapping Gateway" };
      case "ws-connecting":
        return { dotClass: "pending", label: "WebSocket Connecting" };
      case "connected":
        return { dotClass: "online", label: "WebSocket Connected" };
      case "degraded":
        return { dotClass: "pending", label: "HTTP Fallback Active" };
      case "reconnecting":
      default:
        return { dotClass: "offline", label: "Gateway Reconnecting" };
    }
  }, [connectionPhase]);

  const updateTask = useCallback((value: string) => { setDraft((current) => ({ ...current, task: value })); markDraftDirty("control_card", currentTarget); }, [currentTarget, markDraftDirty]);
  const updateTarget = useCallback((value: string) => {
    setDraft((current) => ({ ...current, target_label: value, target_hint: value, target_instance: null }));
    markDraftDirty("control_card", value);
  }, [markDraftDirty]);
  const updateGripper = useCallback((value: string) => { setDraft((current) => ({ ...current, gripper: value })); markDraftDirty("control_card", currentTarget); }, [currentTarget, markDraftDirty]);
  const updateConstraints = useCallback((value: string) => { setDraft((current) => ({ ...current, constraints: parseConstraintsInput(value) })); markDraftDirty("control_card", currentTarget); }, [currentTarget, markDraftDirty]);

  const setVisionOverride = useCallback((candidate: CandidateDebug) => {
    const label = candidate.canonical_label || candidate.label_zh || candidate.label || candidate.display_label || "";
    const overrideDraft: SemanticDraft = {
      ...draft,
      target_label: label,
      target_hint: label,
      target_instance: candidateToTargetInstance(candidate),
    };
    setDraft(overrideDraft);
    pushFrontendEvent("vision", "info", `vision candidate committed as override: ${label}`, {
      label,
      track_id: candidate.track_id,
      bbox_xyxy: candidate.bbox_xyxy,
    });
    pushToast("info", "Vision", `Target switched to ${label}`);
    void (async () => {
      try {
        const next = await postOverride(overrideDraft);
        setState(next);
        resetDraftState();
      } catch (error) {
        pushToast("error", "Vision", getErrorMessage(error, "Failed to switch vision target."));
        pushFrontendEvent("vision", "error", "vision candidate override failed", {
          label,
          track_id: candidate.track_id,
          message: getErrorMessage(error, "vision candidate override failed"),
        });
      }
    })();
  }, [draft, pushFrontendEvent, pushToast, resetDraftState, setState]);

  const handleDialogSubmit = useCallback(async (prompt: string) => {
    const trimmed = prompt.trim();
    if (!trimmed) { pushToast("warn", "Dialog", "Please enter a message."); return false; }
    if (!state.connection.backend_ready) {
      pushToast("warn", "Dialog", "Execution backend is still warming up. Wait a moment and retry.");
      return false;
    }
    setBusyAction("dialog");
    try {
      const next = await postDialogMessage(trimmed, dialogState.mode, dialogState.reply_language);
      setState(next);
      resetDraftState();
      return true;
    } catch (error) {
      pushToast("error", "Dialog", getErrorMessage(error, "Dialog message failed."));
      pushFrontendEvent("dialog", "error", "dialog message failed", { message: getErrorMessage(error, "dialog message failed") });
      return false;
    } finally { setBusyAction(null); }
  }, [dialogState.mode, dialogState.reply_language, pushFrontendEvent, pushToast, resetDraftState, setState, state.connection.backend_ready]);

  const handleDialogModeChange = useCallback(async (mode: DialogMode) => {
    try {
      setState(await postDialogMode(mode));
    } catch (error) {
      pushToast("error", "Dialog", getErrorMessage(error, "Failed to change dialog mode."));
    }
  }, [pushToast, setState]);

  const handleDialogReplyLanguageChange = useCallback(async (replyLanguage: DialogReplyLanguage) => {
    try {
      setState(await postDialogReplyLanguage(replyLanguage));
    } catch (error) {
      pushToast("error", "Dialog", getErrorMessage(error, "Failed to change reply language."));
    }
  }, [pushToast, setState]);

  const handleDialogReset = useCallback(async () => {
    setBusyAction("dialog-reset");
    try {
      setState(await postDialogReset());
      resetDraftState();
    } catch (error) {
      pushToast("error", "Dialog", getErrorMessage(error, "Failed to reset dialog session."));
    } finally {
      setBusyAction(null);
    }
  }, [pushToast, resetDraftState, setState]);

  const handleExecute = useCallback(async () => {
    setBusyAction("execute");
    try {
      if (draftDirtyRef.current) {
        const pendingLabel = draft.target_label || draft.target_hint;
        const overrideState = await postOverride(draft);
        setState(overrideState);
        resetDraftState();
        const overrideApplied = overrideState.execution.intervention.active && overrideState.execution.intervention.label === pendingLabel;
        if (!overrideApplied) await waitForState((next) => next.execution.intervention.active && next.execution.intervention.label === pendingLabel, 2800);
      }
      setState(await postExecute());
    } catch (error) {
      pushToast("error", "Execute", getErrorMessage(error, "Execute request failed."));
      pushFrontendEvent("execution", "error", "execute request failed", { message: getErrorMessage(error, "execute request failed") });
    } finally { setBusyAction(null); }
  }, [draft, pushFrontendEvent, pushToast, resetDraftState, setState, waitForState]);

  const handleReplan = useCallback(async () => {
    setBusyAction("replan");
    try { setState(await postReplan()); resetDraftState(); }
    catch (error) {
      pushToast("error", "Re-plan", getErrorMessage(error, "Re-plan failed."));
      pushFrontendEvent("replan", "error", "replan request failed", { message: getErrorMessage(error, "replan request failed") });
    } finally { setBusyAction(null); }
  }, [pushFrontendEvent, pushToast, resetDraftState, setState]);

  const handleReturnHome = useCallback(async () => {
    setBusyAction("return-home");
    try {
      setState(await postReturnHome());
      pushToast("info", "Execution", "Return-home requested.");
      pushFrontendEvent("execution", "info", "return home requested");
    } catch (error) {
      pushToast("error", "Execution", getErrorMessage(error, "Return-home failed."));
      pushFrontendEvent("execution", "error", "return home failed", { message: getErrorMessage(error, "return home failed") });
    } finally {
      setBusyAction(null);
    }
  }, [pushFrontendEvent, pushToast, setState]);

  const handleResetScene = useCallback(async () => {
    setBusyAction("scene-reset");
    try {
      setState(await postResetScene());
      resetDraftState();
      pushToast("info", "Execution", "Scene reset requested.");
      pushFrontendEvent("execution", "info", "scene reset requested");
    } catch (error) {
      pushToast("error", "Execution", getErrorMessage(error, "Scene reset failed."));
      pushFrontendEvent("execution", "error", "scene reset failed", { message: getErrorMessage(error, "scene reset failed") });
    } finally {
      setBusyAction(null);
    }
  }, [pushFrontendEvent, pushToast, resetDraftState, setState]);

  const handleOpenDebugViews = useCallback(async () => {
    setBusyAction("debug-open");
    try {
      setState(await postOpenDebugViews());
      pushToast("info", "Debug", "Debug views requested.");
      pushFrontendEvent("execution", "info", "debug views requested");
    } catch (error) {
      pushToast("error", "Debug", getErrorMessage(error, "Open debug views failed."));
      pushFrontendEvent("execution", "error", "open debug views failed", {
        message: getErrorMessage(error, "open debug views failed"),
      });
    } finally {
      setBusyAction(null);
    }
  }, [pushFrontendEvent, pushToast, setState]);

  const handleClearLogs = useCallback(() => {
    const latestBackendId = state.logs.events.length > 0 ? state.logs.events[state.logs.events.length - 1].id : 0;
    setBackendEventFloorId(latestBackendId);
    setFrontendEvents([]);
    setSessionMeta({ draftSeen: draftDirtyRef.current, applied: state.execution.intervention.active });
    pushToast("info", "Logs", "Local log view cleared.");
  }, [pushToast, state.execution.intervention.active, state.logs.events]);

  const handleExportLogs = useCallback(() => {
    const payload = {
      exported_at: new Date().toISOString(),
      ui_state: state,
      backend_events: visibleBackendEvents,
      frontend_session_events: frontendEvents,
      manual_intervention: {
        draft_seen: sessionMeta.draftSeen || draftDirty,
        applied: sessionMeta.applied || state.execution.intervention.active,
        current_label: (draftDirty ? draft.target_label || draft.target_hint : state.execution.intervention.label) || draftLabel,
        source: draftDirty ? draftSource || "local_draft" : state.execution.intervention.source || "",
      },
    };
    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = `programme-ui-logs-${new Date().toISOString().replace(/[:.]/g, "-")}.json`;
    anchor.click();
    window.setTimeout(() => URL.revokeObjectURL(url), 0);
    pushToast("info", "Logs", "JSON export ready.");
  }, [draft.target_hint, draft.target_label, draftDirty, draftLabel, draftSource, frontendEvents, pushToast, sessionMeta.applied, sessionMeta.draftSeen, state, visibleBackendEvents]);

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand-block">
          <div className="brand-mark">P</div>
          <div>
            <div className="brand-title">Programme UI</div>
            <div className="brand-subtitle" data-testid="connection-status">
              <span className={`signal-dot ${connectionDisplay.dotClass}`} />
              {connectionDisplay.label}
            </div>
          </div>
        </div>
        <nav className="nav-tabs">
          <NavLink to="/control" data-testid="nav-control" className={({ isActive }) => (isActive ? "tab active" : "tab")}>Control</NavLink>
          <NavLink to="/vision" data-testid="nav-vision" className={({ isActive }) => (isActive ? "tab active" : "tab")}>Vision</NavLink>
          <NavLink to="/tactile" data-testid="nav-tactile" className={({ isActive }) => (isActive ? "tab active" : "tab")}>Tactile</NavLink>
          <NavLink to="/logs" data-testid="nav-logs" className={({ isActive }) => (isActive ? "tab active" : "tab")}>Logs</NavLink>
        </nav>
        <button className="estop-button" data-testid="estop-button" onClick={() => setEstopOpen(true)}>E-Stop <span>Not Wired</span></button>
      </header>

      {connectionPhase === "reconnecting" ? <div className="connection-overlay" data-testid="connection-overlay">Gateway connection lost. Reconnecting...</div> : null}

      <main className="page-shell">
        <Routes>
          <Route path="/" element={<Navigate to="/control" replace />} />
          <Route path="/control" element={<ControlPage state={state} streams={streams} draft={draft} draftDirty={draftDirty} busyAction={busyAction} chatMessages={dialogMessages} dialogMode={dialogState.mode} dialogReplyLanguage={dialogState.reply_language} dialogStatusLabel={dialogState.status_label || dialogState.status} dialogPendingAutoExecute={Boolean(dialogState.pending_auto_execute)} onTaskChange={updateTask} onTargetChange={updateTarget} onGripperChange={updateGripper} onConstraintsChange={updateConstraints} onDialogSubmit={handleDialogSubmit} onDialogModeChange={handleDialogModeChange} onDialogReplyLanguageChange={handleDialogReplyLanguageChange} onDialogReset={handleDialogReset} onExecute={handleExecute} onReplan={handleReplan} onReturnHome={handleReturnHome} onResetScene={handleResetScene} onOpenDebugViews={handleOpenDebugViews} />} />
        <Route path="/vision" element={<VisionPage state={state} streams={streams} onChooseCandidate={setVisionOverride} />} />
          <Route path="/tactile" element={<TactilePage state={state} />} />
          <Route path="/logs" element={<LogsPage state={state} interventionState={interventionState} visibleBackendEvents={visibleBackendEvents} frontendEvents={frontendEvents} onClear={handleClearLogs} onExport={handleExportLogs} />} />
        </Routes>
      </main>

      {estopOpen ? (
        <div className="modal-scrim" data-testid="estop-modal" onClick={() => setEstopOpen(false)}>
          <div className="modal-card" onClick={(event) => event.stopPropagation()}>
            <div className="modal-title">E-Stop Not Connected</div>
            <p>This v1 build keeps the emergency stop button as an explicit placeholder only. No ROS emergency-stop control path is wired behind it.</p>
            <button className="ghost-button" onClick={() => setEstopOpen(false)}>Close</button>
          </div>
        </div>
      ) : null}

      <div className="toast-stack" data-testid="toast-stack">
        {toasts.map((toast) => (
          <div key={toast.id} className={`toast-item ${toast.level}`} data-testid="toast-item">
            <div className="toast-title">{toast.title}</div>
            <div className="toast-message">{toast.message}</div>
          </div>
        ))}
      </div>
    </div>
  );
}

export default App;
