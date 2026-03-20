import { useMemo } from "react";
import { formatNumber, formatTimestamp, heatColor, humanizeKey, readNumber, readString, sparklinePoints, summarizeValue } from "./appHelpers";
import { EmptyState, Panel, SparklineCard, StatusPill } from "./appUi";
import type { UiState } from "./types";

export function TactilePage(props: { state: UiState }) {
  const rows = props.state.tactile.rows ?? 0;
  const cols = props.state.tactile.cols ?? 0;
  const cellCount = rows > 0 && cols > 0 ? rows * cols : 0;
  const rawForces = props.state.tactile.forces ?? [];
  const cells = useMemo(() => (cellCount === 0 ? [] : Array.from({ length: cellCount }, (_, index) => rawForces[index] ?? 0)), [cellCount, rawForces]);
  const maxMagnitude = useMemo(() => Math.max(1, ...cells.map((value) => Math.abs(value))), [cells]);

  return (
    <div className="page-grid tactile-grid">
      <div className="stack-column">
        <Panel title="Taxel Heatmap" subtitle="A single tactile grid with a parallel-gripper outline reference. No fake dual-finger telemetry is shown.">
          {cellCount === 0 ? (
            <EmptyState title="No tactile grid yet" message="Waiting for /tactile/raw rows, cols, and forces." />
          ) : (
            <div className="heatmap-shell" data-testid="tactile-heatmap">
              <svg className="gripper-outline" viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true">
                <rect x="20" y="10" width="18" height="80" rx="8" />
                <rect x="62" y="10" width="18" height="80" rx="8" />
                <path d="M38 18 H62" />
                <path d="M38 82 H62" />
              </svg>
              <div className="heatmap-grid" style={{ gridTemplateColumns: `repeat(${cols}, minmax(0, 1fr))` }}>
                {cells.map((value, index) => (
                  <div key={index} className="taxel-cell" data-testid="tactile-taxel-cell" style={{ backgroundColor: heatColor(value, maxMagnitude) }} title={`taxel ${index}: ${formatNumber(value, 4)}`}>
                    <span>{formatNumber(value, 2)}</span>
                  </div>
                ))}
              </div>
            </div>
          )}

          <div className="metric-grid compact">
            <div className="metric-card"><span className="metric-label">Rows x Cols</span><strong>{rows > 0 && cols > 0 ? `${rows} x ${cols}` : "--"}</strong></div>
            <div className="metric-card"><span className="metric-label">Frame</span><strong>{props.state.tactile.frame_id || "--"}</strong></div>
            <div className="metric-card"><span className="metric-label">Sequence</span><strong>{props.state.tactile.sequence_id ?? "--"}</strong></div>
            <div className="metric-card"><span className="metric-label">Updated</span><strong>{formatTimestamp(props.state.tactile.updated_at)}</strong></div>
          </div>
        </Panel>

        <Panel title="Force Series" subtitle="forces, fx, fy, and fz are rendered with lightweight SVG charts only.">
          <div className="sparkline-grid">
            <div data-testid="tactile-sparkline-forces"><SparklineCard title="forces" values={props.state.tactile.forces ?? []} points={sparklinePoints(props.state.tactile.forces ?? [])} accentClass="teal" /></div>
            <div data-testid="tactile-sparkline-fx"><SparklineCard title="fx" values={props.state.tactile.forces_fx ?? []} points={sparklinePoints(props.state.tactile.forces_fx ?? [])} accentClass="blue" /></div>
            <div data-testid="tactile-sparkline-fy"><SparklineCard title="fy" values={props.state.tactile.forces_fy ?? []} points={sparklinePoints(props.state.tactile.forces_fy ?? [])} accentClass="gold" /></div>
            <div data-testid="tactile-sparkline-fz"><SparklineCard title="fz" values={props.state.tactile.forces_fz ?? []} points={sparklinePoints(props.state.tactile.forces_fz ?? [])} accentClass="coral" /></div>
          </div>
        </Panel>
      </div>

      <div className="stack-column">
        <Panel title="Arm State" subtitle="Current arm status derived from the stable /arm/state publisher.">
          <div className="record-grid">
            {Object.entries(props.state.health.arm_state).length === 0 ? (
              <EmptyState title="No arm state yet" message="Waiting for /arm/state." />
            ) : (
              Object.entries(props.state.health.arm_state).map(([key, value]) => (
                <div key={key} className="record-row"><span>{humanizeKey(key)}</span><strong>{summarizeValue(value)}</strong></div>
              ))
            )}
          </div>
        </Panel>

        <Panel title="System Health" subtitle="Issues are shown first. If there are no issues, the latest healthy snapshots are shown.">
          <div className="health-list">
            {(props.state.health.issues.length > 0 ? props.state.health.issues : props.state.health.latest).map((item, index) => {
              const name = readString(item, "node_name", `node-${index + 1}`);
              const message = readString(item, "message", "--");
              const level = readNumber(item, "level", 0);
              const healthy = Boolean(item.healthy);
              return (
                <article key={`${name}-${index}`} className="health-item">
                  <div className="health-item-header">
                    <strong>{name}</strong>
                    <StatusPill tone={level >= 2 ? "error" : healthy ? "success" : "warn"}>level {level}</StatusPill>
                  </div>
                  <div className="health-item-message">{message}</div>
                </article>
              );
            })}
            {props.state.health.issues.length === 0 && props.state.health.latest.length === 0 ? (
              <EmptyState title="No health state yet" message="Waiting for /system/health data." />
            ) : null}
          </div>
        </Panel>

        <Panel title="Gripper State" subtitle="The current project has no stable publisher for this, so the card stays explicit placeholder-only.">
          <EmptyState title="No real gripper telemetry" message="This placeholder avoids pretending that stable gripper state already exists." />
        </Panel>
      </div>
    </div>
  );
}
