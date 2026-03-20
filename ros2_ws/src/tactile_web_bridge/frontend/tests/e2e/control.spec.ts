import { expect, test } from "@playwright/test";
import { getHistory, gotoControl, resetScenario } from "./helpers";

test.describe("control flow", () => {
  test("submits prompt and updates chat plus confirmation card", async ({ page, request }) => {
    await resetScenario(request, "idle");
    await gotoControl(page);

    const promptInput = page.getByTestId("prompt-input");
    const submitButton = page.getByTestId("prompt-submit");

    await promptInput.fill("pick the blue cup on the table");
    await submitButton.click();

    await expect(submitButton).toBeDisabled();
    await expect(page.getByTestId("chat-message-user")).toContainText("pick the blue cup on the table");
    await expect(page.getByTestId("chat-message-assistant")).toContainText("Structured semantic task updated");
    await expect(page.getByTestId("confirm-target-input")).toHaveValue("cup");
    await expect(page.getByTestId("confirm-task-input")).toHaveValue("pick");
  });

  test("marks local draft, executes override first, then executes pick", async ({ page, request }) => {
    await resetScenario(request, "control_ready");
    await gotoControl(page);

    const targetInput = page.getByTestId("confirm-target-input");
    await targetInput.fill("bottle");
    await expect(page.getByTestId("intervention-badge-draft")).toBeVisible();

    await page.getByTestId("execute-button").click();

    await expect(page.getByTestId("execute-button")).toBeDisabled();
    await expect(page.getByTestId("intervention-badge-applied")).toBeVisible();
    await expect(page.getByTestId("toast-stack")).toContainText("override applied");

    const { history } = await getHistory(request);
    const actionHistory = history.filter((entry: { type: string }) => ["override", "execute"].includes(entry.type));
    expect(actionHistory.map((entry: { type: string }) => entry.type)).toEqual(["override", "execute"]);
    expect(actionHistory[0].payload.target_label).toBe("bottle");
    await expect(page.getByText("Execution completed", { exact: true }).first()).toBeVisible();
  });

  test("replan clears draft and preserves chat history", async ({ page, request }) => {
    await resetScenario(request, "idle");
    await gotoControl(page);

    await page.getByTestId("prompt-input").fill("pick the cup near the edge");
    await page.getByTestId("prompt-submit").click();
    await expect(page.getByTestId("chat-message-user")).toContainText("pick the cup near the edge");

    await page.getByTestId("confirm-target-input").fill("remote");
    await expect(page.getByTestId("intervention-badge-draft")).toBeVisible();

    await page.getByTestId("replan-button").click();

    await expect(page.getByTestId("intervention-badge-draft")).toHaveCount(0);
    await expect(page.getByTestId("confirm-target-input")).toHaveValue("cup");
    await expect(page.getByTestId("chat-list")).toContainText("pick the cup near the edge");
  });
});
