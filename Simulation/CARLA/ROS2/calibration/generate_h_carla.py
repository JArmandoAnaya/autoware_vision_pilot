#!/usr/bin/env python3
"""Derive the CARLA ground homography (H_carla.yaml) from the camera's actual
intrinsics + extrinsics — NOT hand-typed matrix values.

Single source of truth: the `main_cam` sensor block in
`Simulation/CARLA/ROS2/config/VisionPilot_carla10.json`. This tool reads that
camera's field-of-view, image size and mounting pose and emits the
full-frame ground-plane homography that VisionPilot bakes into its preprocess
warp `C` at build time (`-DVISIONPILOT_GROUND_HOMOGRAPHY`).

Because H is *derived* from the same JSON CARLA spawns the camera from, the
homography can never silently disagree with the real camera (the mismatch that
produced distorted perception). Re-run this whenever the camera config changes:

    python Simulation/CARLA/ROS2/calibration/generate_h_carla.py

Geometry (pinhole, flat ground, world X=forward, Y=left, camera at height h):
    f  = (W/2) / tan(fov_h/2)            focal length in pixels
    cx, cy = W/2, H/2                    principal point (image centre)
    For a forward-looking camera (pitch 0) the horizon is the image centre; a
    pixel (u,v) below it back-projects onto the ground at
        X = f*h / (v - cy)               forward distance
        Y = (cx - u) * h / (v - cy)      lateral offset (+ = left)
    which is the homography
        H = [[0,        0,        1     ],
             [-1/f,      0,        cx/f  ],
             [0,         1/(f*h),  -cy/(f*h)]]
    mapping [u,v,1] -> [X,Y,1] (projective). pitch != 0 is rejected here
    (the model-view V it must compose with is itself ~pitch 0); extend with a
    full K[r1 r2 t] build if a tilted camera is ever needed.
"""
import argparse
import json
import math
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_JSON = os.path.join(_HERE, "..", "config", "VisionPilot_carla10.json")
_DEFAULT_OUT = os.path.join(_HERE, "..", "..", "..", "..", "VisionPilot", "config", "H_carla.yaml")


def _main_cam(cfg_path):
    with open(cfg_path) as f:
        cfg = json.load(f)
    for s in cfg.get("sensors", []):
        if s.get("id") == "main_cam":
            return s
    raise SystemExit("ERROR: no sensor with id 'main_cam' in %s" % cfg_path)


def derive_homography(fov_deg, width, height, cam_height_m, pitch_deg):
    if abs(pitch_deg) > 1e-6:
        raise SystemExit(
            "ERROR: pitch=%.3f deg unsupported by the closed form; main_cam must "
            "be pitch 0 to compose with the model-view V. Extend the tool for "
            "tilted cameras." % pitch_deg)
    f = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    cx, cy = width / 2.0, height / 2.0
    fh = f * cam_height_m
    return [0.0, 0.0, 1.0,
            -1.0 / f, 0.0, cx / f,
            0.0, 1.0 / fh, -cy / fh], f, cx, cy


def write_yaml(out_path, data):
    body = ("%%YAML:1.0\n---\nH: !!opencv-matrix\n  rows: 3\n  cols: 3\n  dt: d\n"
            "  data: [ %s ]\n" % ", ".join("%.10e" % v for v in data))
    with open(out_path, "w") as f:
        f.write(body)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-c", "--config", default=_DEFAULT_JSON,
                    help="CARLA sensor JSON (default: VisionPilot_carla10.json)")
    ap.add_argument("-o", "--out", default=_DEFAULT_OUT,
                    help="output H_carla.yaml path")
    args = ap.parse_args()

    cam = _main_cam(os.path.abspath(args.config))
    attrs, pose = cam["attributes"], cam["spawn_point"]
    fov = float(attrs["fov"])
    w = int(attrs["image_size_x"])
    h_img = int(attrs["image_size_y"])
    cam_h = float(pose["z"])
    pitch = float(pose.get("pitch", 0.0))

    data, f, cx, cy = derive_homography(fov, w, h_img, cam_h, pitch)
    write_yaml(os.path.abspath(args.out), data)

    print("main_cam: fov=%.3f deg  %dx%d  height=%.3f m  pitch=%.3f deg"
          % (fov, w, h_img, cam_h, pitch))
    print("derived : f=%.4f px  cx=%.1f  cy=%.1f  (covers %.1f m at v=%d .. %.1f m at v=%d)"
          % (f, cx, cy, f * cam_h / (h_img - 1 - cy), h_img - 1,
             f * cam_h / 1.0, int(cy) + 1))
    print("wrote   : %s" % os.path.abspath(args.out))


if __name__ == "__main__":
    main()
