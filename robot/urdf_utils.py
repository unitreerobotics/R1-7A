"""URDF preprocessing helpers for R1-A7.

The stock A7.urdf caps the elbow lower limit at -0.97564 rad, but the real arm
can bend to about -1.30 rad. These helpers relax that limit and make mesh paths
absolute so a patched copy can be loaded from a temp directory.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


def patch_urdf_elbow_lower(urdf_path: str | Path, elbow_lower: float) -> str:
    """Relax the elbow lower limit in a URDF and absolutize mesh paths.

    Patching lets the IK solver and the post-solve clamp use the full reachable
    range (mirrors teleop_r1a7.py --elbow-lower-override). The patched copy is
    written to a temp dir, so mesh paths are made absolute.

    Args:
        urdf_path: Path to the source URDF.
        elbow_lower: New lower limit (rad) for both elbow joints.

    Returns:
        Path to the patched URDF.
    """
    source = Path(urdf_path).expanduser().resolve()
    if not source.exists():
        raise FileNotFoundError(f"URDF does not exist: {source}")

    tree = ET.parse(source)
    root = tree.getroot()
    for joint_name in ("left_elbow_joint", "right_elbow_joint"):
        joint = root.find(f".//joint[@name='{joint_name}']")
        if joint is None:
            raise RuntimeError(f"Cannot find {joint_name} in {source}")
        limit = joint.find("limit")
        if limit is None:
            raise RuntimeError(f"Cannot find limit for {joint_name} in {source}")
        limit.set("lower", f"{float(elbow_lower):.6f}")

    urdf_dir = source.parent
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename or os.path.isabs(filename) or "://" in filename:
            continue
        mesh.set("filename", str((urdf_dir / filename).resolve()))

    patch_dir = Path(tempfile.gettempdir()) / "r1a7_urdf_patch"
    patch_dir.mkdir(parents=True, exist_ok=True)
    tag = f"{elbow_lower:.4f}".replace("-", "m").replace(".", "p")
    patched_path = patch_dir / f"{source.stem}_elbow_lower_{tag}{source.suffix}"
    tree.write(patched_path, encoding="utf-8", xml_declaration=True)
    return str(patched_path)
