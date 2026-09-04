"""Meshcat visualization for R1-A7.

This module provides two visualization backends:

1. **Placo backend** (preferred, same as teleop_r1a7.py): uses
   ``placo_utils.robot_viz`` which directly wraps the meshcat server and
   correctly syncs all joint states without manual FK computation.

2. **Pinocchio/STL backend** (fallback): loads STL meshes directly and
   updates transforms via Pinocchio FK.  Used when Placo is unavailable.

The Placo backend is automatically selected when placo is importable.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin


class RobotVisualizer:
    """Wrapper for meshcat.Visualizer to hold robot visualization state."""

    def __init__(self, model, data, geom_model, geom_data, mesh_info, meshcat_viz):
        self.model = model
        self.data = data
        self.geom_model = geom_model
        self.geom_data = geom_data
        self.mesh_info = mesh_info
        self.viz = meshcat_viz


class PlacoVisualizer:
    """Placo-native visualizer: wraps placo_utils.robot_viz (same as teleop_r1a7.py)."""

    def __init__(self, placo_robot, robot_viz):
        self.placo_robot = placo_robot
        self.robot_viz = robot_viz


def create_robot_visualization(model, urdf_path: Path) -> PlacoVisualizer | RobotVisualizer:
    """Create robot visualization in meshcat.

    Tries the Placo backend first (same path as teleop_r1a7.py), which
    correctly handles all joint states.  Falls back to the STL+Pinocchio
    approach if Placo cannot be imported.
    """
    urdf_path = Path(urdf_path)

    # Primary: Placo backend (matches teleop_r1a7.py visualization)
    try:
        import placo
        from placo_utils.visualization import robot_viz
        import webbrowser

        placo_robot = placo.RobotWrapper(str(urdf_path))
        viz = robot_viz(placo_robot)
        webbrowser.open(viz.viewer.url())
        print(f"Placo meshcat visualizer: {viz.viewer.url()}")
        return PlacoVisualizer(placo_robot=placo_robot, robot_viz=viz)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        print(f"Warning: Placo visualization unavailable ({exc}); using STL fallback")

    # Fallback: STL + Pinocchio
    meshcat_viz = __import__("meshcat").Visualizer()
    meshcat_viz.open()
    assets_dir = urdf_path.parent

    geom_model = pin.buildGeomFromUrdf(
        model,
        str(urdf_path),
        pin.GeometryType.VISUAL,
        package_dirs=[str(assets_dir)],
    )

    data = model.createData()
    geom_data = pin.GeometryData(geom_model)

    meshcat_viz_impl = __import__("meshcat")
    import meshcat.geometry as g

    meshcat_viz_impl = meshcat_viz  # alias to avoid shadowing

    mesh_info = []
    for geom_id, geom_obj in enumerate(geom_model.geometryObjects):
        mesh_path = geom_obj.meshPath

        if not mesh_path or not mesh_path.strip():
            continue

        try:
            full_mesh_path = Path(mesh_path)
            if not full_mesh_path.exists():
                full_mesh_path = assets_dir / "meshes" / Path(mesh_path).name

            if not full_mesh_path.exists():
                print(f"Warning: mesh not found: {full_mesh_path}")
                continue

            stl_geom = g.StlMeshGeometry.from_file(str(full_mesh_path))
            mat = g.MeshLambertMaterial(color=0x808080)
            node_path = geom_obj.name + "/link"
            meshcat_viz[node_path].set_object(stl_geom, mat)

            mesh_scale = np.asarray(geom_obj.meshScale, dtype=float).reshape(3)
            mesh_info.append({
                "path": node_path,
                "geom_id": geom_id,
                "mesh_scale": mesh_scale,
            })

        except Exception as e:
            print(f"Warning: Could not load {mesh_path}: {e}")

    return RobotVisualizer(
        model=model,
        data=data,
        geom_model=geom_model,
        geom_data=geom_data,
        mesh_info=mesh_info,
        meshcat_viz=meshcat_viz,
    )


def apply_joint_configuration(viz: PlacoVisualizer | RobotVisualizer | None, q: np.ndarray) -> None:
    """Update robot visualization with current configuration.

    Handles both backends:
    - Placo: q is the 21-element URDF joint vector; the function copies it into
      placo_robot.state.q[7:] (7 floating-base elements stay at identity) and
      calls display() with the full 28-element state.
    - STL: q is the 21-element URDF joint vector used directly with Pinocchio FK.
    """
    if viz is None:
        return

    if isinstance(viz, PlacoVisualizer):
        # Placo uses a 28-element state: 7 floating base + 21 URDF joints.
        # Our q is 21 elements; copy into [7:] so the floating base stays at identity.
        viz.placo_robot.state.q[7:] = np.asarray(q, dtype=float).flatten()
        viz.placo_robot.update_kinematics()
        viz.robot_viz.display(viz.placo_robot.state.q)
        return

    if not isinstance(viz, RobotVisualizer):
        return

    model = viz.model
    data = viz.data
    geom_model = viz.geom_model
    geom_data = viz.geom_data
    meshcat_viz = viz.viz

    # Update joint world placements from q
    pin.forwardKinematics(model, data, q)
    pin.updateGeometryPlacements(model, data, geom_model, geom_data)

    for info in viz.mesh_info:
        try:
            M = geom_data.oMg[info["geom_id"]]
            T = np.array(M.homogeneous, dtype=float, copy=True)

            scale = info["mesh_scale"]
            S = np.eye(4)
            S[0, 0], S[1, 1], S[2, 2] = scale
            T = T @ S

            meshcat_viz[info["path"]].set_transform(T)
        except Exception as e:
            print(f"Warning: visualization update failed for {info['path']}: {e}")
