#!/usr/bin/env python3
"""Convert a LIO-SAM .pcd map (XYZ or XYZI, ascii or binary) to a .usd point cloud.

Usage:
    python3 pcd_to_usd.py <input.pcd> <output.usd> [--point-size 0.05]

Requires: usd-core (pxr), numpy, and the PCL CLI tool pcl_convert_pcd_ascii_binary
(all already present in this environment).
"""
import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, Vt


def read_ascii_pcd(path):
    fields = None
    data_start = None
    with open(path, "r") as f:
        lines = f.readlines()
    for i, line in enumerate(lines):
        if line.startswith("FIELDS"):
            fields = line.split()[1:]
        elif line.startswith("DATA"):
            data_start = i + 1
            break
    if fields is None or data_start is None:
        raise ValueError(f"Could not parse PCD header: {path}")

    values = np.loadtxt(lines[data_start:], dtype=np.float32)
    if values.ndim == 1:
        values = values.reshape(1, -1)

    xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
    points = values[:, [xi, yi, zi]]

    intensity = None
    if "intensity" in fields:
        intensity = values[:, fields.index("intensity")]
    return points, intensity


def pcd_to_ascii(input_pcd, tmp_dir):
    converter = shutil.which("pcl_convert_pcd_ascii_binary")
    if converter is None:
        sys.exit("pcl_convert_pcd_ascii_binary not found (install ros-humble-perception-pcl / pcl-tools)")
    ascii_pcd = str(Path(tmp_dir) / "ascii.pcd")
    subprocess.run([converter, input_pcd, ascii_pcd, "0"], check=True)
    return ascii_pcd


def write_usd(points, intensity, output_usd, point_size):
    stage = Usd.Stage.CreateNew(output_usd)
    prim = UsdGeom.Points.Define(stage, "/PointCloud")
    prim.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    prim.CreateWidthsAttr(Vt.FloatArray([point_size] * len(points)))

    if intensity is not None:
        norm = (intensity - intensity.min()) / max(np.ptp(intensity), 1e-6)
        colors = np.repeat(norm.reshape(-1, 1), 3, axis=1).astype(np.float32)
        prim.CreateDisplayColorAttr(Vt.Vec3fArray.FromNumpy(colors))

    stage.SetDefaultPrim(prim.GetPrim())
    stage.GetRootLayer().Save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_pcd")
    parser.add_argument("output_usd")
    parser.add_argument("--point-size", type=float, default=0.05)
    args = parser.parse_args()

    with tempfile.TemporaryDirectory() as tmp_dir:
        ascii_pcd = pcd_to_ascii(args.input_pcd, tmp_dir)
        points, intensity = read_ascii_pcd(ascii_pcd)

    write_usd(points, intensity, args.output_usd, args.point_size)
    print(f"Saved {len(points)} points -> {args.output_usd}")


if __name__ == "__main__":
    main()
