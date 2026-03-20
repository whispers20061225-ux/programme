import { expect, test } from "@playwright/test";
import { getHistory, gotoControl, resetScenario } from "./helpers";

test.describe("e-stop placeholder", () => {
  test("opens the placeholder modal without issuing API calls", async ({ page, request }) => {
    await resetScenario(request, "idle");
    await gotoControl(page);

    await page.getByTestId("estop-button").click();
    await expect(page.getByTestId("estop-modal")).toBeVisible();
    await expect(page.getByTestId("estop-modal")).toContainText("E-Stop Not Connected");

    const { history } = await getHistory(request);
    const mutatingActions = history.filter((entry: { type: string }) =>
      ["dialog_message", "dialog_mode", "dialog_reset", "prompt", "override", "replan", "execute"].includes(entry.type),
    );
    expect(mutatingActions).toEqual([]);
  });
});
