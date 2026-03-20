import { expect, test } from "@playwright/test";
import { gotoControl, resetScenario } from "./helpers";

test.describe("tactile and logs", () => {
  test("renders tactile heatmap and sparkline cards from mock state", async ({ page, request }) => {
    await resetScenario(request, "control_ready");
    await page.goto("/tactile");

    await expect(page.getByTestId("tactile-heatmap")).toBeVisible();
    await expect(page.getByTestId("tactile-taxel-cell")).toHaveCount(8);
    await expect(page.getByTestId("tactile-sparkline-forces")).toBeVisible();
    await expect(page.getByTestId("tactile-sparkline-fx")).toBeVisible();
    await expect(page.getByTestId("tactile-sparkline-fy")).toBeVisible();
    await expect(page.getByTestId("tactile-sparkline-fz")).toBeVisible();
  });

  test("exports logs with manual intervention payload and clear only affects browser view", async ({ page, request }) => {
    await resetScenario(request, "control_ready");
    await gotoControl(page);

    await page.getByTestId("confirm-target-input").fill("bottle");
    await page.getByTestId("execute-button").click();
    await expect(page.getByText("Execution completed", { exact: true }).first()).toBeVisible();

    await page.getByTestId("nav-logs").click();
    await expect(page.getByTestId("logs-backend-count")).not.toHaveText("0");

    const [download] = await Promise.all([
      page.waitForEvent("download"),
      page.getByTestId("logs-export-button").click(),
    ]);
    const downloadPath = await download.path();
    expect(downloadPath).not.toBeNull();
    const fs = await import("node:fs/promises");
    const payload = JSON.parse(await fs.readFile(downloadPath!, "utf8"));
    expect(Object.keys(payload)).toEqual(
      expect.arrayContaining([
        "exported_at",
        "ui_state",
        "backend_events",
        "frontend_session_events",
        "manual_intervention",
      ]),
    );
    expect(payload.manual_intervention.applied).toBeTruthy();

    await page.getByTestId("logs-clear-button").click();
    await expect(page.getByTestId("logs-backend-count")).toHaveText("0");
    await expect(page.getByTestId("logs-frontend-count")).toHaveText("0");
  });
});
