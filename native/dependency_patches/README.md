# Official dependency pins and local baseline patches

- `r1_arm_sdk.patch`: applies to official `unitreerobotics/r1-7a_sdk` commit `2c89fe4f60f0c98f1bc06795cb2477cc73d0c92c`. Preserves the existing local bus diagnostics, calibration tooling and event-driven serial implementation.
- `dex1_1_service.patch`: applies to official `unitreerobotics/dex1_1_service` commit `ced3216290bb22dae356213993786beeaa7143ab`. Preserves explicit port selection, motor ID / DDS side separation, command initialization, diagnostics and local SDK2 build integration.

`setup.sh` checks whether each patch is already applied before applying it. Conflicts stop installation instead of overwriting changes. These patches contain source changes, not local calibration data or compiled runtime libraries.

The active workstation still uses the external SDK/calibration and existing bridge selected by `scripts/real_env.sh`. Updating the submodule Git metadata alone does not replace those runtime files.

- `xr_pybind.patch`: applies to `YanjieZe/XRoboToolkit-PC-Service-Pybind` commit `75cb1130ac63e76d8f7e7788049be415e8be44f2`; preserves atomic frame snapshots, per-device metadata, button counts and callback timing.
- `xr_pc_service.patch`: applies to `XR-Robotics/XRoboToolkit-PC-Service` commit `85bac4dbc1fd5cef42c74a160d9c30aa3491f122`; preserves the latest-frame mailbox and forwarding changes.

Setup applies the XR patches before compiling the native client SDK and Python binding. Building/installing the complete desktop PC Service remains a separate step.
