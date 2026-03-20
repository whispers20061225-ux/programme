import { expect, test } from "@playwright/test";
import { gotoControl, resetScenario, wsAction } from "./helpers";

test.describe("connection and feedback", () => {
  test("does not replay bootstrap feedback as fresh toast", async ({ page, request }) => {
    await resetScenario(request, "feedback_history");
    await gotoControl(page);

    await expect(page.getByTestId("toast-item")).toHaveCount(0);
  });

  test("uses http fallback on websocket disconnect and recovers cleanly", async ({ page, request }) => {
    await resetScenario(request, "control_ready");
    await gotoControl(page);

    await wsAction(request, "disconnect");
    await expect(page.getByTestId("connection-overlay")).toHaveCount(0);
    await expect(page.getByTestId("connection-status")).toContainText("HTTP Fallback Active");
    await expect(page.getByTestId("toast-stack")).toContainText("using HTTP fallback");

    await wsAction(request, "reconnect");
    await expect(page.getByTestId("connection-status")).toContainText("WebSocket Connected");
  });

  test("caps the toast queue at four items", async ({ page, request }) => {
    await resetScenario(request, "control_ready");
    await gotoControl(page);

    for (let index = 1; index <= 5; index += 1) {
      await wsAction(request, "push_state", {
        feedback: {
          category: "test",
          level: "info",
          message: `feedback-${index}`,
        },
      });
    }

    await expect(page.getByTestId("toast-item")).toHaveCount(4);
    await expect(page.getByTestId("toast-stack")).not.toContainText("feedback-1");
    await expect(page.getByTestId("toast-stack")).toContainText("feedback-5");
  });
});
