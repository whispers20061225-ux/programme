import { useMemo, useState } from "react";
import type { OverlayBox, StreamMap } from "./appHelpers";
import { findCandidate, formatNumber, STREAM_OPTIONS } from "./appHelpers";
import { EmptyState, OverlayStage, Panel, StatusPill } from "./appUi";
import type { CandidateDebug, UiState } from "./types";

type VisionPageProps = {
  state: UiState;
  streams: StreamMap;
  onChooseLabel: (label: string) => void;
};

const OVERLAY_CONFIDENCE_FLOOR = 0.07;

function candidateDisplayLabel(candidate: CandidateDebug | null | undefined): string {
  if (!candidate) return "--";
  return candidate.display_label || candidate.label_zh || candidate.canonical_label || candidate.label || "--";
}

function candidatePickLabel(candidate: CandidateDebug | null | undefined): string {
  if (!candidate) return "";
  return candidate.canonical_label || candidate.label_zh || candidate.label || "";
}

export function VisionPage(props: VisionPageProps) {
  const [activeStream, setActiveStream] = useState<keyof StreamMap>("detection_overlay");
  const [hoveredIndex, setHoveredIndex] = useState<number | null>(null);
  const visionCandidates = useMemo<CandidateDebug[]>(() => {
    if (props.state.vision.debug_candidates.length > 0) return props.state.vision.debug_candidates;
    const fallback = (props.state.vision.debug as { top_candidates?: CandidateDebug[] }).top_candidates;
    return Array.isArray(fallback) ? fallback : [];
  }, [props.state.vision.debug, props.state.vision.debug_candidates]);
  const visibleCandidates = useMemo<CandidateDebug[]>(
    () => visionCandidates.filter((candidate) => candidate.confidence >= OVERLAY_CONFIDENCE_FLOOR),
    [visionCandidates],
  );
  const hoveredCandidate = useMemo(
    () => findCandidate(visibleCandidates, hoveredIndex),
    [hoveredIndex, visibleCandidates],
  );

  const overlayBoxes = useMemo<OverlayBox[]>(() => {
    const boxes = new Map<string, OverlayBox>();
    const selected = props.state.vision.selected_candidate;
    const addBox = (bbox: number[], label: string, tone: OverlayBox["tone"]) => {
      if (bbox.length !== 4) return;
      boxes.set(bbox.join("-"), { bbox, label, tone });
    };

    for (const candidate of visibleCandidates) {
      if (!candidate.bbox_xyxy || candidate.bbox_xyxy.length !== 4) continue;
      addBox(
        candidate.bbox_xyxy,
        `${candidateDisplayLabel(candidate)} ${formatNumber(candidate.confidence, 2)}`,
        selected?.index === candidate.index ? "selected" : "candidate",
      );
    }

    if (selected?.bbox_xyxy && selected.bbox_xyxy.length === 4 && selected.confidence >= OVERLAY_CONFIDENCE_FLOOR) {
      addBox(
        selected.bbox_xyxy,
        `Selected: ${candidateDisplayLabel(selected)} ${formatNumber(selected.confidence, 2)}`,
        "selected",
      );
    } else if (boxes.size === 0 && props.state.vision.detection.bbox?.xyxy && (props.state.vision.detection.confidence ?? 0) >= OVERLAY_CONFIDENCE_FLOOR) {
      addBox(
        props.state.vision.detection.bbox.xyxy,
        `Detection: ${props.state.vision.detection.target_label || "target"} ${formatNumber(props.state.vision.detection.confidence, 2)}`,
        "selected",
      );
    }

    if (
      hoveredCandidate?.bbox_xyxy &&
      hoveredCandidate.bbox_xyxy.length === 4 &&
      hoveredCandidate.confidence >= OVERLAY_CONFIDENCE_FLOOR
    ) {
      addBox(
        hoveredCandidate.bbox_xyxy,
        `Hover: ${candidateDisplayLabel(hoveredCandidate)} ${formatNumber(hoveredCandidate.confidence, 2)}`,
        "hovered",
      );
    }
    return Array.from(boxes.values());
  }, [hoveredCandidate, props.state.vision.detection, props.state.vision.selected_candidate, visibleCandidates]);

  const stageSrc = activeStream === "detection_overlay" ? props.streams.rgb : props.streams[activeStream];
  const stageBoxes = overlayBoxes;

  return (
    <div className="page-grid vision-grid">
      <Panel
        title="Vision Monitor"
        subtitle="Detection overlay draws candidate boxes on top of the live RGB stream. All candidates with confidence >= 0.07 are shown, and hovering a candidate highlights its bounding box."
        className="panel-tall"
        actions={
          <div className="toggle-row">
            {STREAM_OPTIONS.map((item) => (
              <button
                key={item.key}
                type="button"
                data-testid={`vision-toggle-${item.key}`}
                aria-pressed={activeStream === item.key}
                className={activeStream === item.key ? "toggle-chip active" : "toggle-chip"}
                onClick={() => setActiveStream(item.key)}
              >
                {item.label}
              </button>
            ))}
          </div>
        }
      >
        <OverlayStage
          src={stageSrc}
          alt={STREAM_OPTIONS.find((item) => item.key === activeStream)?.label ?? "Vision Stream"}
          imageWidth={props.state.vision.image_width}
          imageHeight={props.state.vision.image_height}
          boxes={stageBoxes}
          testId="vision-stage"
        />

        <div className="metric-grid">
          <div className="metric-card"><span className="metric-label">Accepted</span><strong>{props.state.vision.detection.accepted ? "yes" : "no"}</strong></div>
          <div className="metric-card"><span className="metric-label">Target Label</span><strong>{candidateDisplayLabel(props.state.vision.selected_candidate) !== "--" ? candidateDisplayLabel(props.state.vision.selected_candidate) : (props.state.vision.detection.target_label || "--")}</strong></div>
          <div className="metric-card"><span className="metric-label">Confidence</span><strong>{formatNumber(props.state.vision.detection.confidence, 4)}</strong></div>
          <div className="metric-card"><span className="metric-label">Image Size</span><strong>{props.state.vision.image_width > 0 && props.state.vision.image_height > 0 ? `${props.state.vision.image_width} x ${props.state.vision.image_height}` : "--"}</strong></div>
        </div>

        <div className="summary-block">
          <div className="field-label">Candidate Summary</div>
          <div className="summary-text">{props.state.vision.candidate_summary || "Waiting for detection_debug top-k data."}</div>
        </div>
      </Panel>

      <Panel title="Top-K Candidates" subtitle="Clicking a candidate only stages a label-level override. There is no instance pinning in v1.">
        {visibleCandidates.length === 0 ? (
          <EmptyState title="No candidates yet" message="Waiting for detection_debug top-k candidates and bounding boxes." />
        ) : (
          <div className="candidate-list">
            {visibleCandidates.map((candidate) => {
              const isHovered = hoveredIndex === candidate.index;
              const isSelected = props.state.vision.selected_candidate?.index === candidate.index;
              return (
                <button
                  key={candidate.index}
                  type="button"
                  data-testid={`vision-candidate-${candidate.index}`}
                  className={["candidate-item", isHovered ? "hovered" : "", isSelected ? "selected" : ""].filter(Boolean).join(" ")}
                  onMouseEnter={() => setHoveredIndex(candidate.index)}
                  onMouseLeave={() => setHoveredIndex((current) => (current === candidate.index ? null : current))}
                  onClick={() => props.onChooseLabel(candidatePickLabel(candidate))}
                >
                  <div className="candidate-header">
                    <strong>{candidateDisplayLabel(candidate)}</strong>
                    <StatusPill tone={candidate.status === "selectable" ? "success" : "warn"}>{candidate.status}</StatusPill>
                  </div>
                  <div className="candidate-meta">
                    <span>index {candidate.index}</span>
                    <span>track {typeof candidate.track_id === "number" && candidate.track_id >= 0 ? `T${candidate.track_id}` : "--"}</span>
                    <span>conf {formatNumber(candidate.confidence, 4)}</span>
                    <span>score {formatNumber(candidate.score, 4)}</span>
                    <span>bonus {formatNumber(candidate.semantic_bonus, 3)}</span>
                  </div>
                  <div className="candidate-meta">
                    <span>target {candidatePickLabel(candidate) || "--"}</span>
                    <span>source {candidate.label_source || "yolo"}</span>
                    <span>relabel {candidate.track_relabel_reason || "--"}</span>
                  </div>
                  <div className="candidate-meta">
                    <span>age {formatNumber(candidate.track_age_sec, 2)}</span>
                    <span>hits {typeof candidate.track_seen_count === "number" ? candidate.track_seen_count : "--"}</span>
                    <span>raw {candidate.raw_label || candidate.label || "--"}</span>
                    <span>bbox {candidate.bbox_xyxy ? candidate.bbox_xyxy.join(", ") : "--"}</span>
                    <span>mask {candidate.mask_pixels ?? 0}</span>
                  </div>
                </button>
              );
            })}
          </div>
        )}
      </Panel>
    </div>
  );
}
