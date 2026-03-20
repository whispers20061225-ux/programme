import { type ReactNode, useCallback, useEffect, useRef, useState } from "react";
import type { UiState } from "./types";
import type { OverlayBox, PillTone, StreamMap } from "./appHelpers";
import { DEFAULT_STATE } from "./appHelpers";
import { STATE_WS_PATH, backendAssetUrl, backendWsUrl, fetchBootstrap } from "./api";

export type BackendConnectionPhase = "bootstrapping" | "ws-connecting" | "connected" | "degraded" | "reconnecting";

export function useBackendState() {
  const [state, setState] = useState<UiState>(DEFAULT_STATE);
  const [streams, setStreams] = useState<StreamMap>({
    rgb: backendAssetUrl("/api/streams/rgb.mjpeg"),
    detection_overlay: backendAssetUrl("/api/streams/detection_overlay.mjpeg"),
    grasp_overlay: backendAssetUrl("/api/streams/grasp_overlay.mjpeg"),
  });
  const [connectionPhase, setConnectionPhase] = useState<BackendConnectionPhase>("bootstrapping");
  const [loading, setLoading] = useState(true);
  const stateRef = useRef(state);
  const reconnectTimerRef = useRef<number | null>(null);
  const fallbackPollRef = useRef<number | null>(null);
  const bootstrapSettledRef = useRef(false);
  const socketEverOpenedRef = useRef(false);
  const httpHealthyRef = useRef(false);

  useEffect(() => {
    stateRef.current = state;
  }, [state]);

  const waitForState = useCallback(
    (predicate: (next: UiState) => boolean, timeoutMs = 3000) =>
      new Promise<UiState>((resolve, reject) => {
        const start = Date.now();
        const interval = window.setInterval(() => {
          const current = stateRef.current;
          if (predicate(current)) {
            clearInterval(interval);
            resolve(current);
            return;
          }
          if (Date.now() - start > timeoutMs) {
            clearInterval(interval);
            reject(new Error("Timed out waiting for UI state"));
          }
        }, 120);
      }),
    [],
  );

  useEffect(() => {
    let cancelled = false;
    let socket: WebSocket | null = null;

    const applyBootstrap = (nextState: UiState, nextStreams: StreamMap) => {
      setState(nextState);
      setStreams(nextStreams);
      bootstrapSettledRef.current = true;
      httpHealthyRef.current = true;
      setLoading(false);
    };

    const refreshBootstrap = async (): Promise<boolean> => {
      try {
        const bootstrap = await fetchBootstrap();
        if (cancelled) return false;
        applyBootstrap(bootstrap.state, bootstrap.streams);
        return true;
      } catch {
        if (cancelled) return false;
        bootstrapSettledRef.current = true;
        httpHealthyRef.current = false;
        setLoading(false);
        return false;
      }
    };

    const connect = () => {
      if (cancelled) return;
      setConnectionPhase((current) => {
        if (current === "connected") return current;
        if (!bootstrapSettledRef.current) return "bootstrapping";
        if (socketEverOpenedRef.current && httpHealthyRef.current) return "degraded";
        return socketEverOpenedRef.current ? "reconnecting" : "ws-connecting";
      });
      socket = new WebSocket(backendWsUrl(STATE_WS_PATH));
      socket.onopen = () => {
        socketEverOpenedRef.current = true;
        httpHealthyRef.current = true;
        setConnectionPhase("connected");
      };
      socket.onmessage = (event) => setState(JSON.parse(event.data) as UiState);
      socket.onerror = () => socket?.close();
      socket.onclose = () => {
        if (cancelled) return;
        if (!bootstrapSettledRef.current) setConnectionPhase("bootstrapping");
        else if (httpHealthyRef.current) setConnectionPhase("degraded");
        else setConnectionPhase("reconnecting");
        reconnectTimerRef.current = window.setTimeout(connect, 1200);
      };
    };

    void refreshBootstrap().then((ok) => {
      if (cancelled) return;
      setConnectionPhase(ok ? "ws-connecting" : "reconnecting");
      connect();
    });

    fallbackPollRef.current = window.setInterval(() => {
      if (cancelled) return;
      if (socket?.readyState === WebSocket.OPEN) return;
      void refreshBootstrap().then((ok) => {
        if (cancelled || socket?.readyState === WebSocket.OPEN) return;
        if (!bootstrapSettledRef.current) {
          setConnectionPhase("bootstrapping");
          return;
        }
        if (ok) setConnectionPhase(socketEverOpenedRef.current ? "degraded" : "ws-connecting");
        else setConnectionPhase("reconnecting");
      });
    }, 2500);

    return () => {
      cancelled = true;
      socket?.close();
      if (reconnectTimerRef.current !== null) window.clearTimeout(reconnectTimerRef.current);
      if (fallbackPollRef.current !== null) window.clearInterval(fallbackPollRef.current);
    };
  }, []);

  return {
    state,
    streams,
    connected: connectionPhase === "connected",
    connectionPhase,
    loading,
    setState,
    waitForState,
  };
}

export function Panel(props: { title: string; subtitle?: string; actions?: ReactNode; className?: string; children: ReactNode }) {
  return (
    <section className={props.className ? `panel ${props.className}` : "panel"}>
      <div className="panel-header">
        <div>
          <h2 className="panel-title">{props.title}</h2>
          {props.subtitle ? <p className="panel-subtitle">{props.subtitle}</p> : null}
        </div>
        {props.actions ? <div className="panel-actions">{props.actions}</div> : null}
      </div>
      {props.children}
    </section>
  );
}

export function StatusPill(props: { tone: PillTone; children: ReactNode }) {
  return <span className={`status-pill tone-${props.tone}`}>{props.children}</span>;
}

export function EmptyState(props: { title: string; message: string }) {
  return (
    <div className="empty-state">
      <div className="empty-title">{props.title}</div>
      <div className="empty-message">{props.message}</div>
    </div>
  );
}

export function OverlayStage(props: {
  src: string;
  alt: string;
  imageWidth: number;
  imageHeight: number;
  boxes: OverlayBox[];
  testId?: string;
}) {
  const width = props.imageWidth > 0 ? props.imageWidth : 1280;
  const height = props.imageHeight > 0 ? props.imageHeight : 720;
  const aspectRatio = props.imageWidth > 0 && props.imageHeight > 0 ? `${props.imageWidth} / ${props.imageHeight}` : "16 / 9";
  return (
    <div className="live-stage" style={{ aspectRatio }} data-testid={props.testId}>
      <img className="live-image" src={props.src} alt={props.alt} />
      <OverlayBoxes width={width} height={height} boxes={props.boxes} />
    </div>
  );
}

function OverlayBoxes(props: { width: number; height: number; boxes: OverlayBox[] }) {
  return (
    <svg className="overlay-svg" viewBox={`0 0 ${props.width} ${props.height}`} preserveAspectRatio="none" aria-hidden="true">
      {props.boxes.map((box) => {
        const [x1, y1, x2, y2] = box.bbox;
        return (
          <g
            key={`${box.tone}-${box.label}-${box.bbox.join("-")}`}
            className={`overlay-box ${box.tone}`}
            data-testid={`overlay-box-${box.tone}`}
          >
            <rect x={x1} y={y1} width={Math.max(1, x2 - x1)} height={Math.max(1, y2 - y1)} rx={8} />
            <text x={x1 + 10} y={Math.max(24, y1 + 22)} className="overlay-label">{box.label}</text>
          </g>
        );
      })}
    </svg>
  );
}

export function StreamThumbnail(props: {
  title: string;
  src: string;
  imageWidth: number;
  imageHeight: number;
  boxes?: OverlayBox[];
}) {
  const width = props.imageWidth > 0 ? props.imageWidth : 1280;
  const height = props.imageHeight > 0 ? props.imageHeight : 720;
  const aspectRatio = props.imageWidth > 0 && props.imageHeight > 0 ? `${props.imageWidth} / ${props.imageHeight}` : "16 / 9";
  return (
    <div className="thumbnail-card">
      <div className="thumbnail-title">{props.title}</div>
      <div className="thumbnail-frame" style={{ aspectRatio }}>
        <img className="thumbnail-image" src={props.src} alt={props.title} />
        {props.boxes && props.boxes.length > 0 ? <OverlayBoxes width={width} height={height} boxes={props.boxes} /> : null}
      </div>
    </div>
  );
}

export function SparklineCard(props: { title: string; values: number[]; points: string; accentClass: string }) {
  return (
    <div className="sparkline-card">
      <div className="sparkline-header">
        <span>{props.title}</span>
        <strong>{props.values.length > 0 ? `${props.values.length} samples` : "--"}</strong>
      </div>
      {props.points ? (
        <svg className="sparkline-svg" viewBox="0 0 320 120" preserveAspectRatio="none" aria-hidden="true">
          <polyline className={`sparkline-polyline ${props.accentClass}`} points={props.points} />
        </svg>
      ) : (
        <EmptyState title="No data yet" message="Waiting for tactile samples." />
      )}
    </div>
  );
}
