"""KITTI coordinates and segment metric, cross-checked with the official C++ devkit."""

from pathlib import Path
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parent


def calibration(path):
    for line in Path(path).read_text().splitlines():
        if line.startswith("Tr:"):
            pose = np.eye(4)
            pose[:3] = np.fromstring(line.split(":", 1)[1], sep=" ").reshape(3, 4)
            return pose
    raise ValueError("missing KITTI Velodyne-to-camera Tr")


def ground_truth(data, sequence):
    camera = np.loadtxt(Path(data) / "poses" / f"{sequence}.txt").reshape(-1, 3, 4)
    poses = np.tile(np.eye(4), (len(camera), 1, 1))
    poses[:, :3] = camera
    tr = calibration(Path(data) / "sequences" / sequence / "calib.txt")
    camera_relative = np.linalg.inv(poses[0])@poses
    return np.linalg.inv(tr)@camera_relative@tr, camera_relative, tr


def write_poses(path, poses):
    np.savetxt(path, np.asarray(poses)[:, :3].reshape(-1, 12), fmt="%.12e")


def segment_errors(ground, predicted):
    traveled = np.r_[0, np.cumsum(np.linalg.norm(np.diff(ground[:, :3, 3], axis=0), axis=1))]
    rows = []
    for start in range(0, len(ground), 10):
        for length in range(100, 801, 100):
            end = np.searchsorted(traveled, traveled[start]+length, side="right")
            if end >= len(ground):
                continue
            a = np.linalg.inv(ground[start])@ground[end]
            b = np.linalg.inv(predicted[start])@predicted[end]
            error = np.linalg.inv(b)@a
            rotation = np.arccos(np.clip((np.trace(error[:3, :3])-1)/2, -1, 1))
            translation = np.linalg.norm(error[:3, 3])
            rows.append({"start_frame": start, "end_frame": int(end), "length_m": length,
                         "translation_error_percent": float(100*translation/length),
                         "rotation_error_deg_per_100m": float(np.degrees(rotation)*100/length)})
    return rows


def official_errors(data, gt_path, result_path, build):
    """Run only loadPoses/calcSequenceErrors, never the devkit server/mail entry."""
    build = Path(build)
    build.mkdir(parents=True, exist_ok=True)
    source = build / "official_metric_driver.cpp"
    devkit = (Path(data) / "devkit/cpp").resolve()
    # The public devkit bundles an incomplete evaluation-server Mail stub.
    # Compile its unchanged metric functions only, without server/plot/mail code.
    original = (devkit / "evaluate_odometry.cpp").read_text()
    metric_functions = original.split("void saveSequenceErrors", 1)[0].replace('#include "mail.h"', '')
    (build / "official_metric_functions.hpp").write_text(metric_functions)
    driver = '''#include "official_metric_functions.hpp"
int main(int argc,char** argv) {
  if(argc!=3) return 2;
  auto gt=loadPoses(argv[1]); auto result=loadPoses(argv[2]);
  if(gt.empty() || gt.size()!=result.size()) return 3;
  auto errors=calcSequenceErrors(gt,result);
  for(const auto &e:errors) printf("%d %.12g %.12g %.12g\\n",e.first_frame,e.r_err,e.t_err,e.len);
  return 0;
}
'''
    source.write_text(driver)
    binary = build / "official_metric"
    if not binary.exists() or binary.stat().st_mtime < source.stat().st_mtime:
        subprocess.run(["g++", "-O2", "-std=c++17", "-I", str(devkit), str(source),
                        str(devkit / "matrix.cpp"), "-o", str(binary)], check=True, capture_output=True)
    output = subprocess.run([str(binary), str(gt_path), str(result_path)], check=True,
                            capture_output=True, text=True).stdout
    return np.array([[float(v) for v in line.split()] for line in output.splitlines()])


def summarize(ground, predicted, camera_ground, camera_predicted, data, gt_path, pred_path, build):
    rows = segment_errors(camera_ground, camera_predicted)
    official = official_errors(data, gt_path, pred_path, build)
    assert len(official) == len(rows)
    python_t = np.array([r["translation_error_percent"] for r in rows])
    python_r = np.array([r["rotation_error_deg_per_100m"] for r in rows])
    # The reference devkit casts intermediate rotation trace to float32.
    np.testing.assert_allclose(python_t, official[:, 2]*100, atol=2e-4, rtol=1e-5)
    np.testing.assert_allclose(python_r, np.degrees(official[:, 1])*100, atol=.005, rtol=.02)
    delta = np.linalg.norm(predicted[:, :3, 3]-ground[:, :3, 3], axis=1)
    return {"kitti_translation_error_percent": float(np.mean(official[:, 2])*100),
            "kitti_rotation_error_deg_per_100m": float(np.degrees(np.mean(official[:, 1]))*100),
            "kitti_segments": len(rows), "official_devkit_crosscheck": True,
            "ape_rmse_m_no_alignment": float(np.sqrt(np.mean(delta*delta))),
            "final_position_error_m": float(delta[-1])}, rows
