import { useMemo } from "react";
import {
  formatNumber,
  formatTimestamp,
  heatColor,
  humanizeKey,
  readNumber,
  readString,
  summarizeValue,
} from "./appHelpers";
import { EmptyState, Panel, StatusPill } from "./appUi";
import type { UiState } from "./types";

export function TactilePage(props: {
  state: UiState;
  modeBusy: boolean;
  tareBusy: boolean;
  onSetMode: (useHardware: boolean) => void;
  onTare: () => void;
  onClearTare: () => void;
}) {
  const tactile = props.state.tactile;
  const profile = props.state.gripper_profile ?? {};
  const rows = tactile.rows ?? 0;
  const cols = tactile.cols ?? 0;
  const cellCount = rows > 0 && cols > 0 ? rows * cols : 0;
  const heatValues = tactile.forces ?? [];
  const fxValues = tactile.forces_fx ?? [];
  const fyValues = tactile.forces_fy ?? [];
  const fzValues = tactile.forces_fz ?? [];
  const cells = useMemo(
    () => (
      cellCount === 0
        ? []
        : Array.from({ length: cellCount }, (_, index) => ({
            index,
            heat: heatValues[index] ?? 0,
            fx: fxValues[index] ?? 0,
            fy: fyValues[index] ?? 0,
            fz: fzValues[index] ?? 0,
          }))
    ),
    [cellCount, heatValues, fxValues, fyValues, fzValues],
  );
  const maxMagnitude = useMemo(
    () => Math.max(1, ...cells.map((value) => Math.abs(value.heat))),
    [cells],
  );
  const requestedMode = (tactile.requested_mode || "simulation").toLowerCase();
  const activeMode = (tactile.active_mode || tactile.requested_mode || "simulation").toLowerCase();
  const sourceName = tactile.source_name || "--";
  const rawFrameHex = tactile.raw_frame_hex || "";
  const statusText = tactile.status_text || "waiting for tactile data";
  const hardwareRequested = requestedMode === "hardware";
  const hardwareActive = activeMode === "hardware";
  const realButtonClass = hardwareRequested ? "primary-button" : "ghost-button";
  const virtualButtonClass = hardwareRequested ? "ghost-button" : "primary-button";
  const contactActive = Boolean(tactile.contact_active);
  const baselineReady = Boolean(tactile.baseline_ready);
  const tareActive = Boolean(tactile.tare_active);
  const tareMessage = tactile.tare_message || "";
  const freezeWarning = Boolean(tactile.freeze_warning);
  const freezeDurationSec = Number(tactile.freeze_duration_sec ?? 0);
  const freezeRepeatCount = Number(tactile.freeze_repeat_count ?? 0);

  return (
    <div className="page-grid tactile-grid">
      <div className="stack-column">
        <Panel
          title="Source & Diagnostics"
          subtitle="Mode selection only switches the tactile source. Frame parsing, baseline handling, and filtering stay below the web layer."
          actions={(
            <div className="stack-column">
              <div className="button-row">
                <button
                  className={realButtonClass}
                  disabled={props.modeBusy || (hardwareRequested && hardwareActive)}
                  onClick={() => props.onSetMode(true)}
                  type="button"
                >
                  Real
                </button>
                <button
                  className={virtualButtonClass}
                  disabled={props.modeBusy || (!hardwareRequested && !hardwareActive)}
                  onClick={() => props.onSetMode(false)}
                  type="button"
                >
                  Virtual
                </button>
              </div>
              <div className="button-row compact">
                <button
                  className="ghost-button"
                  disabled={props.tareBusy || !hardwareActive || !tactile.source_connected}
                  onClick={props.onTare}
                  type="button"
                >
                  Tare
                </button>
                <button
                  className="ghost-button"
                  disabled={props.tareBusy || !tareActive}
                  onClick={props.onClearTare}
                  type="button"
                >
                  Clear Tare
                </button>
              </div>
            </div>
          )}
        >
          <div className="status-row">
            <StatusPill tone={hardwareRequested ? "info" : "neutral"}>
              Requested {tactile.requested_mode || "simulation"}
            </StatusPill>
            <StatusPill tone={hardwareActive ? (tactile.source_connected ? "success" : "warn") : "neutral"}>
              Active {tactile.active_mode || "simulation"}
            </StatusPill>
            <StatusPill tone={tactile.source_connected ? "success" : (hardwareActive ? "warn" : "neutral")}>
              Link {tactile.source_connected ? "up" : "down"}
            </StatusPill>
            <StatusPill tone={contactActive ? "success" : "neutral"}>
              Contact {contactActive ? "on" : "off"}
            </StatusPill>
            <StatusPill tone={baselineReady ? "success" : "warn"}>
              Baseline {baselineReady ? "ready" : "learning"}
            </StatusPill>
            <StatusPill tone={tareActive ? "info" : "neutral"}>
              Tare {tareActive ? "on" : "off"}
            </StatusPill>
            <StatusPill tone={freezeWarning ? "warn" : "neutral"}>
              Raw {freezeWarning ? "static" : "live"}
            </StatusPill>
            <StatusPill tone={rawFrameHex ? (tactile.lrc_ok ? "success" : "error") : "neutral"}>
              LRC {rawFrameHex ? (tactile.lrc_ok ? "ok" : "bad") : "--"}
            </StatusPill>
          </div>

          <div className="metric-grid">
            <div className="metric-card">
              <span className="metric-label">Source</span>
              <strong>{sourceName}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Layout</span>
              <strong>{tactile.sensor_layout || "--"}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Device Addr</span>
              <strong>{tactile.device_addr ?? 0}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Start Addr</span>
              <strong>{tactile.start_addr ?? 0}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Observed Payload</span>
              <strong>{tactile.payload_len_observed ?? 0}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Mapped Values</span>
              <strong>{tactile.payload_value_count ?? 0}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Transport Hz</span>
              <strong>{formatNumber(tactile.transport_rate_hz, 1)}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Publish Hz</span>
              <strong>{formatNumber(tactile.publish_rate_hz, 1)}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Frame Stamp</span>
              <strong>{formatTimestamp(tactile.stamp_sec)}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Updated</span>
              <strong>{formatTimestamp(tactile.updated_at)}</strong>
            </div>
          </div>

          <div className="muted-block">
            {freezeWarning ? (
              <div className="record-row">
                <span>Warning</span>
                <strong>{`Raw frame unchanged for ${formatNumber(freezeDurationSec, 1)}s (${freezeRepeatCount} repeats)`}</strong>
              </div>
            ) : null}
            {tareActive || tareMessage ? (
              <div className="record-row">
                <span>Tare</span>
                <strong>{tareActive ? (tareMessage || "manual tactile zero active") : "manual tactile zero cleared"}</strong>
              </div>
            ) : null}
            <div className="record-row">
              <span>Status</span>
              <strong>{statusText}</strong>
            </div>
            <div>
              <div className="field-label">Raw Frames</div>
              {rawFrameHex ? (
                <pre className="log-payload">{rawFrameHex}</pre>
              ) : (
                <div className="inline-note">No raw frames buffered yet.</div>
              )}
            </div>
          </div>
        </Panel>

        <Panel title="Total Force & Contact" subtitle="These values come from address 1008-1010 after hardware-side decode, baseline, and filtering.">
          <div className="metric-grid">
            <div className="metric-card">
              <span className="metric-label">Fx Total</span>
              <strong>{formatNumber(tactile.total_fx, 2)}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Fy Total</span>
              <strong>{formatNumber(tactile.total_fy, 2)}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Fz Total</span>
              <strong>{formatNumber(tactile.total_fz, 2)}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Contact Score</span>
              <strong>{formatNumber(tactile.contact_score, 2)}</strong>
            </div>
          </div>

          <div className="metric-grid compact">
            <div className="metric-card">
              <span className="metric-label">Rows x Cols</span>
              <strong>{rows > 0 && cols > 0 ? `${rows} x ${cols}` : "--"}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Frame</span>
              <strong>{tactile.frame_id || "--"}</strong>
            </div>
            <div className="metric-card">
              <span className="metric-label">Sequence</span>
              <strong>{tactile.sequence_id ?? "--"}</strong>
            </div>
          </div>
        </Panel>

        <Panel title="Taxel Heatmap" subtitle="Heatmap shows the current 3x3 taxel Fz field. Side panels below keep the signed Fx/Fy per taxel visible.">
          {cellCount === 0 ? (
            <EmptyState
              title="No mapped taxel grid yet"
              message={
                rawFrameHex
                  ? "Frames are arriving, but the current cycle did not decode a valid 3x3 taxel grid."
                  : "Waiting for /tactile/raw rows, cols, and taxel forces."
              }
            />
          ) : (
            <div className="heatmap-shell" data-testid="tactile-heatmap">
              <svg
                className="gripper-outline"
                viewBox="0 0 100 100"
                preserveAspectRatio="none"
                aria-hidden="true"
              >
                <rect x="20" y="10" width="18" height="80" rx="8" />
                <rect x="62" y="10" width="18" height="80" rx="8" />
                <path d="M38 18 H62" />
                <path d="M38 82 H62" />
              </svg>
              <div
                className="heatmap-grid"
                style={{ gridTemplateColumns: `repeat(${cols}, minmax(0, 1fr))` }}
              >
                {cells.map((cell) => (
                  <div
                    key={cell.index}
                    className="taxel-cell"
                    data-testid="tactile-taxel-cell"
                    style={{ backgroundColor: heatColor(cell.heat, maxMagnitude) }}
                    title={`taxel ${cell.index + 1}: fx=${formatNumber(cell.fx, 2)} fy=${formatNumber(cell.fy, 2)} fz=${formatNumber(cell.fz, 2)}`}
                  >
                    <span>{formatNumber(cell.heat, 1)}</span>
                  </div>
                ))}
              </div>
            </div>
          )}
        </Panel>
      </div>

      <div className="stack-column">
        <Panel title="Per-Taxel Components" subtitle="Each slot shows decoded Fx, Fy, and Fz after hardware-side baseline and filtering.">
          {cells.length === 0 ? (
            <EmptyState title="No taxel details yet" message="Waiting for valid 3x3 tactile output." />
          ) : (
            <div className="record-grid">
              {cells.map((cell) => (
                <div key={cell.index} className="record-row">
                  <span>{`P${cell.index + 1}`}</span>
                  <strong>{`Fx ${formatNumber(cell.fx, 2)} | Fy ${formatNumber(cell.fy, 2)} | Fz ${formatNumber(cell.fz, 2)}`}</strong>
                </div>
              ))}
            </div>
          )}
        </Panel>

        <Panel title="Gripper Profile" subtitle="This is the current host-side grasp profile selected from task handoff metadata. Real-time PID still belongs on STM32.">
          {profile.profile_id ? (
            <div className="metric-grid">
              <div className="metric-card">
                <span className="metric-label">Profile</span>
                <strong>{profile.profile_id}</strong>
              </div>
              <div className="metric-card">
                <span className="metric-label">Target</span>
                <strong>{profile.target_label || profile.object_type || "--"}</strong>
              </div>
              <div className="metric-card">
                <span className="metric-label">Kp</span>
                <strong>{formatNumber(profile.kp, 3)}</strong>
              </div>
              <div className="metric-card">
                <span className="metric-label">Kd</span>
                <strong>{formatNumber(profile.kd, 3)}</strong>
              </div>
              <div className="metric-card">
                <span className="metric-label">Target Force</span>
                <strong>{formatNumber(profile.target_force, 2)}</strong>
              </div>
              <div className="metric-card">
                <span className="metric-label">Contact Th</span>
                <strong>{formatNumber(profile.contact_threshold, 2)}</strong>
              </div>
              <div className="metric-card">
                <span className="metric-label">Safety Max</span>
                <strong>{formatNumber(profile.safety_max, 2)}</strong>
              </div>
              <div className="metric-card">
                <span className="metric-label">Confidence</span>
                <strong>{formatNumber(profile.confidence, 2)}</strong>
              </div>
            </div>
          ) : (
            <EmptyState
              title="No host profile yet"
              message="Waiting for task handoff metadata or profile selection."
            />
          )}
        </Panel>

        <Panel title="Arm State" subtitle="Current arm status derived from the stable /arm/state publisher.">
          <div className="record-grid">
            {Object.entries(props.state.health.arm_state).length === 0 ? (
              <EmptyState title="No arm state yet" message="Waiting for /arm/state." />
            ) : (
              Object.entries(props.state.health.arm_state).map(([key, value]) => (
                <div key={key} className="record-row">
                  <span>{humanizeKey(key)}</span>
                  <strong>{summarizeValue(value)}</strong>
                </div>
              ))
            )}
          </div>
        </Panel>

        <Panel title="System Health" subtitle="Issues are shown first. If there are no issues, the latest healthy snapshots are shown.">
          <div className="health-list">
            {(props.state.health.issues.length > 0
              ? props.state.health.issues
              : props.state.health.latest).map((item, index) => {
              const name = readString(item, "node_name", `node-${index + 1}`);
              const message = readString(item, "message", "--");
              const level = readNumber(item, "level", 0);
              const healthy = Boolean(item.healthy);
              return (
                <article key={`${name}-${index}`} className="health-item">
                  <div className="health-item-header">
                    <strong>{name}</strong>
                    <StatusPill tone={level >= 2 ? "error" : healthy ? "success" : "warn"}>
                      level {level}
                    </StatusPill>
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
      </div>
    </div>
  );
}
