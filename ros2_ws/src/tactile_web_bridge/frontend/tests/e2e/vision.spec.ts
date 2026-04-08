import { expect, test } from "@playwright/test";
import { resetScenario } from "./helpers";

test.describe("vision flow", () => {
  test.beforeEach(async ({ request }) => {
    await resetScenario(request, "control_ready");
  });

  test("switches streams and highlights hovered candidate boxes", async ({ page }) => {
    await page.goto("/vision");

    await expect(page.getByTestId("vision-toggle-detection_overlay")).toHaveAttribute("aria-pressed", "true");
    await page.getByTestId("vision-toggle-rgb").click();
    await expect(page.getByTestId("vision-toggle-rgb")).toHaveAttribute("aria-pressed", "true");
    await page.getByTestId("vision-toggle-grasp_overlay").click();
    await expect(page.getByTestId("vision-toggle-grasp_overlay")).toHaveAttribute("aria-pressed", "true");

    await page.getByTestId("vision-candidate-1").hover();
    await expect(page.getByTestId("overlay-box-hovered")).toBeVisible();
  });

  test("clicking a candidate applies target override and syncs back to control", async ({ page }) => {
    await page.goto("/vision");

    await page.getByTestId("vision-candidate-1").click();
    await expect(page.getByTestId("toast-stack")).toContainText("Target switched to bottle");

    await page.getByTestId("nav-control").click();
    await expect(page.getByTestId("confirm-target-input")).toHaveValue("bottle");
    await expect(page.getByTestId("intervention-badge-applied")).toBeVisible();
  });
});
