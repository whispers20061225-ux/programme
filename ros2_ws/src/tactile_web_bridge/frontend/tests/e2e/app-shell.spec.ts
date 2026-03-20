import { expect, test } from "@playwright/test";
import { gotoControl, resetScenario } from "./helpers";

test.describe("app shell", () => {
  test.beforeEach(async ({ request }) => {
    await resetScenario(request, "idle");
  });

  test("redirects root to /control and switches between the four routes", async ({ page }) => {
    await gotoControl(page);

    await expect(page.getByTestId("nav-control")).toHaveClass(/active/);
    await expect(page.getByText("Dialog Control")).toBeVisible();

    await page.getByTestId("nav-vision").click();
    await expect(page).toHaveURL(/\/vision$/);
    await expect(page.getByText("Vision Monitor")).toBeVisible();

    await page.getByTestId("nav-tactile").click();
    await expect(page).toHaveURL(/\/tactile$/);
    await expect(page.getByText("Taxel Heatmap")).toBeVisible();

    await page.getByTestId("nav-logs").click();
    await expect(page).toHaveURL(/\/logs$/);
    await expect(page.getByText("Task Stepper")).toBeVisible();
  });
});
