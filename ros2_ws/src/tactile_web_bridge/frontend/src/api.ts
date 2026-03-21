import type {
  BootstrapResponse,
  DialogMode,
  DialogReplyLanguage,
  SemanticDraft,
  UiState,
} from "./types";

const DEV_BACKEND_HTTP_ORIGIN = "http://127.0.0.1:8765";
const DEV_BACKEND_WS_ORIGIN = "ws://127.0.0.1:8765";
export const STATE_WS_PATH = "/ws/live";

function joinUrl(origin: string, path: string): string {
  const normalizedPath = path.startsWith("/") ? path : `/${path}`;
  return `${origin}${normalizedPath}`;
}

export function backendHttpOrigin(): string {
  if (typeof window === "undefined") return DEV_BACKEND_HTTP_ORIGIN;
  return window.location.origin;
}

export function backendWsOrigin(): string {
  if (typeof window === "undefined") return DEV_BACKEND_WS_ORIGIN;
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.host}`;
}

export function backendWsUrl(path = STATE_WS_PATH): string {
  return joinUrl(backendWsOrigin(), path);
}

export function backendAssetUrl(path: string): string {
  if (/^https?:\/\//.test(path) || /^wss?:\/\//.test(path)) return path;
  return joinUrl(backendHttpOrigin(), path);
}

export function resolveStreamMap(streams: BootstrapResponse["streams"]): BootstrapResponse["streams"] {
  return {
    rgb: backendAssetUrl(streams.rgb),
    detection_overlay: backendAssetUrl(streams.detection_overlay),
    grasp_overlay: backendAssetUrl(streams.grasp_overlay),
  };
}

async function requestJson<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(joinUrl(backendHttpOrigin(), path), {
    headers: {
      "Content-Type": "application/json",
      ...(init?.headers ?? {}),
    },
    ...init,
  });
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.detail ?? payload.message ?? `Request failed: ${response.status}`);
  }
  return payload as T;
}

export async function fetchBootstrap(): Promise<BootstrapResponse> {
  const response = await requestJson<BootstrapResponse>("/api/bootstrap");
  return {
    ...response,
    streams: resolveStreamMap(response.streams),
  };
}

export async function postPrompt(prompt: string): Promise<UiState> {
  const response = await requestJson<{ ok: boolean; state: UiState }>("/api/prompt", {
    method: "POST",
    body: JSON.stringify({ prompt }),
  });
  return response.state;
}

export async function postDialogMessage(
  message: string,
  mode: DialogMode,
  replyLanguage: DialogReplyLanguage,
): Promise<UiState> {
  const response = await requestJson<{ ok: boolean; state: UiState }>("/api/dialog/message", {
    method: "POST",
    body: JSON.stringify({ message, mode, reply_language: replyLanguage }),
  });
  return response.state;
}

export async function postDialogMode(mode: DialogMode): Promise<UiState> {
  const response = await requestJson<{ ok: boolean; state: UiState }>("/api/dialog/mode", {
    method: "POST",
    body: JSON.stringify({ mode }),
  });
  return response.state;
}

export async function postDialogReset(): Promise<UiState> {
  const response = await requestJson<{ ok: boolean; state: UiState }>("/api/dialog/reset", {
    method: "POST",
  });
  return response.state;
}

export async function postDialogReplyLanguage(
  replyLanguage: DialogReplyLanguage,
): Promise<UiState> {
  const response = await requestJson<{ ok: boolean; state: UiState }>(
    "/api/dialog/reply-language",
    {
      method: "POST",
      body: JSON.stringify({ reply_language: replyLanguage }),
    },
  );
  return response.state;
}

export async function postOverride(draft: SemanticDraft): Promise<UiState> {
  const response = await requestJson<{ ok: boolean; state: UiState }>("/api/task/override", {
    method: "POST",
    body: JSON.stringify({
      task: draft.task,
      target_label: draft.target_label,
      target_hint: draft.target_hint,
      gripper: draft.gripper,
      constraints: draft.constraints,
      excluded_labels: draft.excluded_labels,
    }),
  });
  return response.state;
}

export async function postReplan(): Promise<UiState> {
  const response = await requestJson<{ ok: boolean; state: UiState }>("/api/task/replan", {
    method: "POST",
  });
  return response.state;
}

export async function postExecute(): Promise<UiState> {
  const response = await requestJson<{ ok: boolean; state: UiState }>("/api/execution/execute", {
    method: "POST",
  });
  return response.state;
}

export async function postReturnHome(): Promise<UiState> {
  const response = await requestJson<{ ok: boolean; state: UiState }>("/api/execution/return-home", {
    method: "POST",
  });
  return response.state;
}
