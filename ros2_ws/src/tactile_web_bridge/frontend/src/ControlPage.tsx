import { type FormEvent, useMemo, useState } from "react";
import type { BusyAction, ChatMessage, StreamMap } from "./appHelpers";
import { phaseLabel, readString } from "./appHelpers";
import { EmptyState, Panel, StatusPill, StreamThumbnail } from "./appUi";
import type { DialogMode, DialogReplyLanguage, SemanticDraft, UiState } from "./types";

type ControlPageProps = {
  state: UiState;
  streams: StreamMap;
  draft: SemanticDraft;
  draftDirty: boolean;
  busyAction: BusyAction;
  chatMessages: ChatMessage[];
  dialogMode: DialogMode;
  dialogReplyLanguage: DialogReplyLanguage;
  dialogStatusLabel: string;
  dialogPendingAutoExecute: boolean;
  onTaskChange: (value: string) => void;
  onTargetChange: (value: string) => void;
  onGripperChange: (value: string) => void;
  onConstraintsChange: (value: string) => void;
  onDialogSubmit: (prompt: string) => Promise<boolean>;
  onDialogModeChange: (mode: DialogMode) => Promise<void>;
  onDialogReplyLanguageChange: (replyLanguage: DialogReplyLanguage) => Promise<void>;
  onDialogReset: () => Promise<void>;
  onExecute: () => Promise<void>;
  onReplan: () => Promise<void>;
  onReturnHome: () => Promise<void>;
  onResetScene: () => Promise<void>;
  onOpenDebugViews: () => Promise<void>;
};

export function ControlPage(props: ControlPageProps) {
  const [promptInput, setPromptInput] = useState("");
  const targetValue = props.draft.target_label || props.draft.target_hint;
  const constraintsValue = props.draft.constraints.join(", ");
  const pendingExecute = props.state.execution.pending_execute ?? {
    active: false,
    source: "",
    message: "",
    updated_at: 0,
  };
  const currentMessage = pendingExecute.active
    ? pendingExecute.message || "Execute queued while waiting for target lock."
    : readString(props.state.execution.pick_status, "message", "Waiting for semantic or vision updates.");
  const executionPin = props.draft.target_instance?.track_id
    ? `T${props.draft.target_instance.track_id}`
    : props.draft.target_instance?.bbox_xyxy?.length === 4
      ? "BBox Hint"
      : "Semantic Only";
  const executeDisabled =
    props.busyAction !== null ||
    !props.state.connection.backend_ready ||
    !targetValue.trim() ||
    pendingExecute.active ||
    props.state.execution.pick_active;
  const detectionPreviewBoxes = useMemo(() => {
    const selected = props.state.vision.selected_candidate;
    const selectedLabel =
      selected?.display_label || selected?.label_zh || selected?.canonical_label || selected?.label || "target";
    if (selected?.bbox_xyxy && selected.bbox_xyxy.length === 4) {
      return [{ bbox: selected.bbox_xyxy, label: `Selected: ${selectedLabel}`, tone: "selected" as const }];
    }
    if (props.state.vision.detection.bbox?.xyxy) {
      return [
        {
          bbox: props.state.vision.detection.bbox.xyxy,
          label: `Detection: ${props.state.vision.detection.target_label || "target"}`,
          tone: "selected" as const,
        },
      ];
    }
    return [];
  }, [props.state.vision.detection.bbox, props.state.vision.detection.target_label, props.state.vision.selected_candidate]);

  const handleSubmit = async (event: FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    const ok = await props.onDialogSubmit(promptInput);
    if (ok) setPromptInput("");
  };

  return (
    <div className="page-grid control-grid">
      <Panel
        title="Dialog Control"
        subtitle="Multi-turn dialog updates the structured task in the backend session. Review mode stops at task drafting; Auto mode immediately starts execution once the instruction is clear."
        className="panel-tall"
      >
        <div className="status-row">
          <StatusPill tone={props.state.execution.target_locked ? "success" : "neutral"}>
            {props.state.execution.target_locked ? "Target Locked" : "Waiting for Lock"}
          </StatusPill>
          <StatusPill tone={props.state.execution.pick_active ? "warn" : "neutral"}>
            {props.state.execution.pick_active ? "Executing" : phaseLabel(props.state.execution.phase)}
          </StatusPill>
          <StatusPill tone={props.dialogMode === "auto" ? "info" : "neutral"}>
            {props.dialogMode === "auto" ? "Auto Mode" : "Review Mode"}
          </StatusPill>
          <StatusPill tone={props.dialogPendingAutoExecute ? "warn" : "neutral"}>
            {props.dialogPendingAutoExecute ? "Auto Queued" : props.dialogStatusLabel}
          </StatusPill>
          <StatusPill tone={pendingExecute.active ? "info" : "neutral"}>
            {pendingExecute.active ? "Execute Queued" : "Manual Ready"}
          </StatusPill>
          {props.draftDirty ? <span data-testid="intervention-badge-draft"><StatusPill tone="warn">Manual Draft</StatusPill></span> : null}
          {!props.draftDirty && props.state.execution.intervention.active ? <span data-testid="intervention-badge-applied"><StatusPill tone="info">Override Applied</StatusPill></span> : null}
        </div>

        <div className="button-row compact">
          <button
            type="button"
            className={props.dialogMode === "review" ? "toggle-chip active" : "toggle-chip"}
            onClick={() => void props.onDialogModeChange("review")}
            disabled={props.busyAction === "dialog"}
          >
            Review
          </button>
          <button
            type="button"
            className={props.dialogMode === "auto" ? "toggle-chip active" : "toggle-chip"}
            onClick={() => void props.onDialogModeChange("auto")}
            disabled={props.busyAction === "dialog"}
          >
            Auto
          </button>
          <button
            type="button"
            className="ghost-button"
            onClick={() => void props.onDialogReset()}
            disabled={props.busyAction !== null}
          >
            {props.busyAction === "dialog-reset" ? "Resetting..." : "New Session"}
          </button>
        </div>

        <div className="button-row compact">
          <span className="inline-note">Assistant Reply Language</span>
          <button
            type="button"
            className={props.dialogReplyLanguage === "zh" ? "toggle-chip active" : "toggle-chip"}
            onClick={() => void props.onDialogReplyLanguageChange("zh")}
            disabled={props.busyAction === "dialog"}
          >
            中文
          </button>
          <button
            type="button"
            className={props.dialogReplyLanguage === "en" ? "toggle-chip active" : "toggle-chip"}
            onClick={() => void props.onDialogReplyLanguageChange("en")}
            disabled={props.busyAction === "dialog"}
          >
            English
          </button>
        </div>

        <form className="prompt-form" onSubmit={handleSubmit}>
          <label className="field-label" htmlFor="prompt-input">Operator Prompt</label>
          <textarea
            id="prompt-input"
            className="prompt-textarea"
            data-testid="prompt-input"
            value={promptInput}
            onChange={(event) => setPromptInput(event.currentTarget.value)}
            placeholder="Example: pick up the blue cup on the left with the parallel gripper. Then say: use the leftmost bottle instead."
            disabled={props.busyAction !== null || !props.state.connection.backend_ready}
          />
          <div className="button-row">
            <button className="primary-button" data-testid="prompt-submit" type="submit" disabled={props.busyAction !== null || !props.state.connection.backend_ready || !promptInput.trim()}>
              {props.busyAction === "dialog" ? "Sending..." : "Send Message"}
            </button>
            <span className="inline-note">
              {!props.state.connection.backend_ready
                ? "Execution backend is still warming up. Wait for backend ready before sending the first command."
                : props.dialogMode === "auto"
                ? "Auto mode updates the task and immediately starts execution once the target and instruction are clear."
                : "Review mode updates the task only. Execution still requires the Execute button."}
            </span>
          </div>
        </form>

        <div className="chat-list" data-testid="chat-list">
          {props.chatMessages.length === 0 ? (
            <EmptyState title="No conversation yet" message="After the first message, this feed will show the dialog history, assistant replies, and task summaries stored in the backend session." />
          ) : (
            props.chatMessages.map((message) => (
              <article key={message.id} className={`chat-bubble ${message.role}`} data-testid={`chat-message-${message.role}`}>
                <div className="chat-role">
                  {message.role === "user" ? "User" : message.role === "assistant" ? "Assistant" : "System"}
                </div>
                <div className="chat-text">{message.text}</div>
                {message.meta ? <div className="chat-meta">{message.meta}</div> : null}
              </article>
            ))
          )}
        </div>
      </Panel>

      <div className="stack-column">
        <Panel title="Confirmation Card" subtitle="v1 keeps a fixed schema: task / target / gripper / constraints.">
          <div className="input-grid">
            <label className="field-group"><span className="field-label">Task</span><input className="text-input" data-testid="confirm-task-input" value={props.draft.task} onChange={(event) => props.onTaskChange(event.currentTarget.value)} disabled={props.busyAction !== null} /></label>
            <label className="field-group"><span className="field-label">Target</span><input className="text-input" data-testid="confirm-target-input" value={targetValue} onChange={(event) => props.onTargetChange(event.currentTarget.value)} disabled={props.busyAction !== null} /></label>
            <label className="field-group"><span className="field-label">Gripper</span><input className="text-input" data-testid="confirm-gripper-input" value={props.draft.gripper} onChange={(event) => props.onGripperChange(event.currentTarget.value)} disabled={props.busyAction !== null} /></label>
            <label className="field-group"><span className="field-label">Constraints</span><input className="text-input" data-testid="confirm-constraints-input" value={constraintsValue} onChange={(event) => props.onConstraintsChange(event.currentTarget.value)} disabled={props.busyAction !== null} /></label>
          </div>

          {props.draft.excluded_labels.length > 0 ? (
            <div className="muted-block">
              <div className="field-label">Excluded Labels</div>
              <div className="chip-row">{props.draft.excluded_labels.map((item) => <span key={item} className="chip">{item}</span>)}</div>
            </div>
          ) : null}

          <div className="metric-grid compact">
            <div className="metric-card"><span className="metric-label">Current Phase</span><strong>{phaseLabel(props.state.execution.phase)}</strong></div>
            <div className="metric-card"><span className="metric-label">Semantic Target</span><strong>{props.state.semantic.target_label || props.state.semantic.target_hint || "--"}</strong></div>
            <div className="metric-card"><span className="metric-label">Execution Pin</span><strong>{executionPin}</strong></div>
            <div className="metric-card"><span className="metric-label">Execution Hint</span><strong>{currentMessage}</strong></div>
          </div>

          <div className="button-row">
            <button className="primary-button" data-testid="execute-button" onClick={props.onExecute} disabled={executeDisabled}>
              {props.busyAction === "execute" ? "Submitting..." : pendingExecute.active ? "Queued..." : "Execute"}
            </button>
            <button className="ghost-button" data-testid="replan-button" onClick={props.onReplan} disabled={props.busyAction !== null}>
              {props.busyAction === "replan" ? "Re-planning..." : "Re-plan"}
            </button>
            <button className="ghost-button" data-testid="return-home-button" onClick={props.onReturnHome} disabled={props.busyAction !== null}>
              {props.busyAction === "return-home" ? "Returning..." : "Return Home"}
            </button>
            <button className="ghost-button" data-testid="reset-scene-button" onClick={props.onResetScene} disabled={props.busyAction !== null}>
              {props.busyAction === "scene-reset" ? "Resetting..." : "Reset Scene"}
            </button>
            <button className="ghost-button" data-testid="open-debug-button" onClick={props.onOpenDebugViews} disabled={props.busyAction !== null}>
              {props.busyAction === "debug-open" ? "Opening..." : "Open Debug"}
            </button>
          </div>
          <div className="inline-note">Execute submits the draft override first, then starts search and either arms immediately or queues the pick until target lock arrives.</div>
        </Panel>

        <Panel title="Preview Streams" subtitle="Control keeps only small monitoring thumbnails, not the main vision workspace.">
          <div className="thumbnail-grid">
            <StreamThumbnail title="RGB" src={props.streams.rgb} imageWidth={props.state.vision.image_width} imageHeight={props.state.vision.image_height} />
            <StreamThumbnail
              title="Detection Overlay"
              src={props.streams.rgb}
              imageWidth={props.state.vision.image_width}
              imageHeight={props.state.vision.image_height}
              boxes={detectionPreviewBoxes}
            />
          </div>
        </Panel>
      </div>
    </div>
  );
}
