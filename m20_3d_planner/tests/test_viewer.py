"""Browser integration: offline opening, actual poses and camera interaction.

Requires Playwright and an existing exported run; no extra browser download is
needed when system Chrome is available. The numerical tests run independently.
"""

import shutil
import unittest

import numpy as np

from m20_3d_planner import core, settings
from m20_3d_planner.visualization.web import registered_runs

try:
    from playwright.sync_api import sync_playwright
except ImportError:
    sync_playwright = None

ENTRY = settings.DEFAULT_RUN / "index.html"
CHROME = shutil.which("google-chrome") or shutil.which("chromium")


@unittest.skipUnless(
    sync_playwright and CHROME and ENTRY.exists(),
    "Requires Playwright, Chrome and an exported run",
)
class ViewerChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.playwright = sync_playwright().start()
        cls.browser = cls.playwright.chromium.launch(
            executable_path=CHROME,
            headless=True,
            args=[
                "--no-sandbox",
                "--enable-unsafe-swiftshader",
                "--use-angle=swiftshader",
                "--disable-dev-shm-usage",
            ],
        )

    @classmethod
    def tearDownClass(cls):
        cls.browser.close()
        cls.playwright.stop()

    def setUp(self):
        self.page = self.browser.new_page(viewport={"width": 1536, "height": 1024})
        self.errors = []
        self.requests = []
        self.page.on("pageerror", lambda error: self.errors.append(str(error)))
        self.page.on("request", lambda request: self.requests.append(request.url))
        self.page.goto(ENTRY.as_uri(), wait_until="load")
        self.page.wait_for_function("window.plannerViewer?.getState().ready")

    def tearDown(self):
        self.page.close()
        self.assertEqual(self.errors, [])

    def state(self):
        return self.page.evaluate("window.plannerViewer.getState()")

    def test_all_scenes_and_modes_match_recorded_body_transforms(self):
        entries = self.page.evaluate("window.PLANNER_MANIFEST.scenes")
        for entry in entries:
            name = entry["id"]
            self.page.evaluate("name => window.plannerViewer.setScene(name)", name)
            for mode in entry.get("modes", settings.MODES):
                self.page.evaluate("mode => window.plannerViewer.setMode(mode)", mode)
                current = self.state()
                with np.load(
                    settings.DEFAULT_RUN
                    / entry["directory"]
                    / "trials"
                    / f"{entry['sourceScene']}__{mode}.validation.npz"
                ) as data:
                    expected = data["dense_poses"][current["frame"]]
                np.testing.assert_allclose(
                    current["pose"], expected, rtol=0, atol=5.1e-8
                )
                body_matrix = np.asarray(current["bodyMatrix"]).reshape(
                    (4, 4), order="F"
                )
                np.testing.assert_allclose(
                    body_matrix[:3, :3],
                    core.rpy_matrix(np.array(current["pose"][3:])),
                    atol=1e-12,
                )
        self.assertFalse(
            any(url.startswith(("http://", "https://")) for url in self.requests)
        )

    def test_navigation_layers_and_progress_preserve_camera(self):
        canvas = self.page.locator("#viewport > canvas")
        box = canvas.bounding_box()
        self.page.mouse.move(
            box["x"] + box["width"] * 0.6, box["y"] + box["height"] * 0.6
        )
        before = self.state()
        self.page.mouse.wheel(0, -300)
        self.page.wait_for_timeout(150)
        self.assertGreater(self.state()["zoom"], before["zoom"])
        self.page.mouse.down()
        self.page.mouse.move(
            box["x"] + box["width"] * 0.7, box["y"] + box["height"] * 0.5, steps=6
        )
        self.page.mouse.up()
        self.page.wait_for_timeout(500)
        self.assertFalse(np.allclose(self.state()["camera"], before["camera"]))
        target = self.state()["target"]
        self.page.mouse.down(button="right")
        self.page.mouse.move(
            box["x"] + box["width"] * 0.65, box["y"] + box["height"] * 0.55, steps=4
        )
        self.page.mouse.up(button="right")
        self.assertFalse(np.allclose(self.state()["target"], target))
        camera = self.state()
        self.page.click('[data-phase="optimize"]')
        self.page.evaluate("window.plannerViewer.setProgress(150)")
        np.testing.assert_allclose(self.state()["camera"], camera["camera"], atol=1e-5)
        self.assertEqual(self.state()["zoom"], camera["zoom"])
        self.page.uncheck("#show-cloud")
        self.page.uncheck("#show-initial")
        self.assertFalse(self.state()["cloudVisible"])
        self.assertFalse(self.state()["initialVisible"])
        self.page.check("#show-cloud")
        self.page.check("#show-initial")
        self.page.click("#play")
        self.page.wait_for_timeout(300)
        self.assertGreater(self.state()["frame"], 150)
        self.page.click("#play")

    def test_real_initial_and_final_optimizer_snapshots_and_video(self):
        self.page.evaluate("window.plannerViewer.setScene('tilted_corridor_30deg')")
        self.page.click('[data-phase="optimize"]')
        np.testing.assert_array_equal(self.state()["pose"][3:], np.zeros(3))
        self.page.evaluate("window.plannerViewer.setProgress(300)")
        self.assertAlmostEqual(self.state()["pose"][3] * 180 / np.pi, 29.2, delta=0.1)
        self.page.click('[data-view="front"]')
        self.page.click("#focus-body")
        self.page.wait_for_timeout(200)
        self.page.screenshot(path=str(settings.DEFAULT_RUN / "report/viewer_front.png"))
        self.page.click("#video-open")
        self.page.wait_for_function(
            "document.getElementById('video-player').readyState >= 1"
        )
        self.assertTrue(self.page.locator("#videos").is_visible())
        self.page.click("#video-close")
        self.assertFalse(self.page.locator("#videos").is_visible())

    def test_nonparallel_cross_sections_and_native_optimizer_snapshots(self):
        run = settings.HERE / "outputs/nonparallel_30deg"
        if not (run / "results.json").exists():
            self.skipTest("Requires the nonparallel experiment")
        for name in (
            "flat_ceiling_bent_floor_30deg",
            "flat_floor_sloped_ceiling_30deg",
        ):
            self.page.evaluate("name => window.plannerViewer.setScene(name)", name)
            self.assertEqual(len(self.state()["crossSection"]), 2)
            self.assertTrue(self.page.locator("#scene-description").is_visible())
            for mode in settings.MODES:
                self.page.evaluate("mode => window.plannerViewer.setMode(mode)", mode)
                self.page.evaluate("window.plannerViewer.setPhase('optimize')")
                np.testing.assert_array_equal(self.state()["pose"][3:], np.zeros(3))
                with np.load(run / "trials" / f"{name}__{mode}.npz") as data:
                    self.page.evaluate("window.plannerViewer.setProgress(99999)")
                    np.testing.assert_allclose(
                        self.state()["pose"],
                        data["iteration_poses"][-1, 11],
                        atol=5.1e-8,
                        rtol=0,
                    )
                self.page.uncheck("#show-section")
                self.assertFalse(self.state()["sectionVisible"])
                self.page.check("#show-section")
        self.page.evaluate(
            "window.plannerViewer.setScene('flat_ceiling_bent_floor_30deg')"
        )
        self.page.evaluate("window.plannerViewer.setMode('full')")
        self.page.evaluate("window.plannerViewer.setPhase('compare')")
        self.page.wait_for_timeout(200)
        self.page.screenshot(path=str(run / "report/viewer_front.png"))

    def test_imports_persist_and_report_and_video_follow_selected_scene(self):
        sources = registered_runs(settings.DEFAULT_RUN)
        expected = len({name for source in sources for name in source.scenes})
        self.assertEqual(self.page.locator("#scene option").count(), expected)
        for entry in self.page.evaluate("window.PLANNER_MANIFEST.scenes"):
            self.page.evaluate(
                "name => window.plannerViewer.setScene(name)", entry["id"]
            )
            self.assertEqual(
                self.page.locator("#run-name").text_content(), entry["run"]
            )
            self.assertEqual(
                self.page.locator("#report-link").get_attribute("href"), entry["report"]
            )
            self.assertTrue((settings.DEFAULT_RUN / entry["report"]).is_file())
        for video in self.page.evaluate("window.PLANNER_MANIFEST.videos"):
            self.assertTrue((settings.DEFAULT_RUN / video["file"]).is_file())
            self.page.evaluate(
                "name => window.plannerViewer.setScene(name)", video["scene"]
            )
            self.page.click("#video-open")
            self.assertEqual(
                self.page.locator("#video-select").input_value(), video["file"]
            )
            self.page.wait_for_function(
                "document.getElementById('video-player').readyState >= 1"
            )
            self.page.click("#video-close")

    def test_selection_datasets_share_one_scene_and_disable_missing_modes(self):
        entries = self.page.evaluate("window.PLANNER_MANIFEST.scenes")
        name = "open_entry_flat_floor_channel_30deg"
        variants = {
            entry["run"]: entry for entry in entries if entry["sourceScene"] == name
        }
        if (
            not {"baseline_full_cloud", "radius_060", "rays_4deg", "rays_2deg"}
            <= variants.keys()
        ):
            self.skipTest("Point-selection comparison has not been exported")
        self.assertEqual(self.page.locator(f'#scene option[value="{name}"]').count(), 1)
        self.page.evaluate("name => window.plannerViewer.setScene(name)", name)
        self.page.select_option("#method", "translation")
        poses = []
        for run in ("baseline_full_cloud", "radius_060", "rays_4deg", "rays_2deg"):
            entry = variants[run]
            self.page.select_option("#dataset", entry["id"])
            self.page.wait_for_function(
                "id => window.plannerViewer.getState().scene === id", arg=entry["id"]
            )
            self.assertEqual(self.page.locator("#scene").input_value(), name)
            self.assertEqual(self.state()["mode"], "full")
            self.assertTrue(
                self.page.locator('#method option[value="translation"]').is_disabled()
            )
            self.assertTrue(
                self.page.locator(
                    '#method option[value="translation_yaw"]'
                ).is_disabled()
            )
            poses.append(self.state()["pose"])
        np.testing.assert_array_equal(poses[0], poses[1])


if __name__ == "__main__":
    unittest.main()
