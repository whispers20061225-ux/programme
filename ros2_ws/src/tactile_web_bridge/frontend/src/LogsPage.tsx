import { useMemo } from "react";
import type { DisplayEvent, InterventionState } from "./appHelpers";
import { formatTimestamp, phaseLabel, STEP_PHASES } from "./appHelpers";
import { EmptyState, Panel, StatusPill } from "./appUi";
import type { UiEvent, UiState } from "./types";

type LogsPageProps = {
  state: UiState;
  interventionState: InterventionState;
  visibleBackendEvents: UiEvent[];
  frontendEvents: UiEvent[];
  onClear: () => void;
  onExport: () => void;
};

export function LogsPage(props: LogsPageProps) {
  const currentIndex = STEP_PHASES.indexOf(props.state.logs.stepper_phase as (typeof STEP_PHASES)[number]);
  const mergedEvents = useMemo<DisplayEvent[]>(
    () =>
      [
        ...props.visibleBackendEvents.map((event) => ({ event, origin: "backend" as const })),
        ...props.frontendEvents.map((event) => ({ event, origin: "frontend" as const })),
      ].sort((left, right) => right.event.created_at - left.event.created_at),
    [props.frontendEvents, props.visibleBackendEvents],
  );

  return (
    <div className="page-grid logs-grid">
      <Panel
        title="Task Stepper"
        subtitle="The main flow stays linear. Manual intervention is highlighted as a separate badge."
        actions={
          <div className="button-row compact">
            <button className="ghost-button" data-testid="logs-clear-button" onClick={props.onClear}>Clear</button>
            <button className="primary-button" data-testid="logs-export-button" onClick={props.onExport}>Export JSON</button>
          </div>
        }
      >
        <div className="stepper" data-testid="logs-stepper" data-current-phase={props.state.logs.stepper_phase}>
          {STEP_PHASES.map((phase, index) => {
            const reached = currentIndex >= index;
            const current = props.state.logs.stepper_phase === phase;
            return (
              <div
                key={phase}
                className={`stepper-item ${reached ? "reached" : ""} ${current ? "current" : ""}`}
                data-testid={`stepper-phase-${phase}`}
              >
                <div className="stepper-dot">{index + 1}</div>
                <div className="stepper-label">{phaseLabel(phase)}</div>
              </div>
            );
          })}
        </div>

        <div className="status-row">
          <StatusPill tone={props.interventionState === "idle" ? "neutral" : "warn"}>
            {props.interventionState === "draft" ? "Manual Intervention / Draft" : props.interventionState === "applied" ? "Manual Intervention / Applied" : "Manual Intervention / None"}
          </StatusPill>
          <StatusPill tone={props.state.execution.pick_active ? "warn" : "info"}>
            pick_active {props.state.execution.pick_active ? "true" : "false"}
          </StatusPill>
          <StatusPill tone={props.state.connection.backend_ready ? "success" : "error"}>
            backend {props.state.connection.backend_ready ? "ready" : "down"}
          </StatusPill>
        </div>
      </Panel>

      <Panel title="Aggregated Event Log" subtitle="Backend high-value events plus frontend session events. Clear affects only the browser view." className="panel-tall">
        <div className="metric-grid compact">
          <div className="metric-card"><span className="metric-label">Backend Events</span><strong data-testid="logs-backend-count">{props.visibleBackendEvents.length}</strong></div>
          <div className="metric-card"><span className="metric-label">Frontend Session Events</span><strong data-testid="logs-frontend-count">{props.frontendEvents.length}</strong></div>
          <div className="metric-card"><span className="metric-label">Latest Update</span><strong>{formatTimestamp(props.state.logs.updated_at)}</strong></div>
        </div>

        <div className="logs-terminal" data-testid="logs-terminal">
          {mergedEvents.length === 0 ? (
            <EmptyState title="No logs yet" message="Waiting for gateway events or frontend session events." />
          ) : (
            mergedEvents.map(({ event, origin }) => (
              <article key={`${origin}-${event.id}-${event.created_at}`} className="log-entry" data-testid="log-entry">
                <div className="log-entry-header">
                  <span className={`log-origin ${origin}`}>{origin}</span>
                  <span className={`log-level ${event.level}`}>{event.level}</span>
                  <span className="log-category">{event.category}</span>
                  <span className="log-time">{formatTimestamp(event.created_at)}</span>
                </div>
                <div className="log-message">{event.message}</div>
                {Object.keys(event.data).length > 0 ? <pre className="log-payload">{JSON.stringify(event.data, null, 2)}</pre> : null}
              </article>
            ))
          )}
        </div>
      </Panel>
    </div>
  );
}
