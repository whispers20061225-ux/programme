import { expect, type APIRequestContext, type Page } from "@playwright/test";

const gatewayBase = process.env.PLAYWRIGHT_GATEWAY_BASE || "http://127.0.0.1:8765";

export async function resetScenario(
  request: APIRequestContext,
  scenario: string,
  executeOutcome = "completed",
) {
  const response = await request.post(`${gatewayBase}/__test/reset`, {
    data: { scenario, executeOutcome },
  });
  expect(response.ok()).toBeTruthy();
}

export async function getHistory(request: APIRequestContext) {
  const response = await request.get(`${gatewayBase}/__test/history`);
  expect(response.ok()).toBeTruthy();
  return response.json();
}

export async function wsAction(
  request: APIRequestContext,
  action: "disconnect" | "reconnect" | "push_state",
  payload: Record<string, unknown> = {},
) {
  const response = await request.post(`${gatewayBase}/__test/ws`, {
    data: { action, ...payload },
  });
  expect(response.ok()).toBeTruthy();
}

export async function gotoControl(page: Page) {
  await page.goto("/");
  await expect(page).toHaveURL(/\/control$/);
}
