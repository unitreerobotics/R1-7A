#!/usr/bin/env bash
# R1-7A teleoperation environment setup.
# 默认使用清华 Conda / PyPI 镜像；不配置也不依赖 HTTP 代理。
#
# 用法:
#   bash setup.sh
#   bash setup.sh --skip-apt          # 跳过系统依赖
#   bash setup.sh --skip-xrt          # 跳过 XRoboToolkit（无 Pico 也可先跑仿真）
#   ENV_NAME=r1_teleop bash setup.sh  # 自定义环境名
#
# 装好后:
#   conda activate r1_teleop
#   export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
#   python scripts/teleop.py --sim --right --mock-xr --debug
#
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_NAME="${ENV_NAME:-r1_teleop}"
PYTHON_VERSION="${PYTHON_VERSION:-3.10}"

CONDA_FORGE="${CONDA_FORGE:-https://conda.anaconda.org/conda-forge}"
PIP_INDEX="${PIP_INDEX:-https://mirrors.aliyun.com/pypi/simple}"

SKIP_APT=0
SKIP_XRT=0
for arg in "$@"; do
  case "$arg" in
    --skip-apt) SKIP_APT=1 ;;
    --skip-xrt) SKIP_XRT=1 ;;
    -h|--help)
      sed -n '2,16p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown option: $arg" >&2
      exit 1
      ;;
  esac
done

log() { printf '\n==> %s\n' "$*"; }

# Idempotent source adaptation; preserve user edits and fail on patch conflicts.
apply_dependency_patch() {
  local dependency_dir="$PROJECT_ROOT/third_party/$1"
  local dependency_patch="$PROJECT_ROOT/native/dependency_patches/$2"
  if git -C "$dependency_dir" apply --reverse --check "$dependency_patch" 2>/dev/null; then
    log "$1: 本地基线补丁已应用"
  elif git -C "$dependency_dir" apply --check "$dependency_patch"; then
    git -C "$dependency_dir" apply "$dependency_patch"
  else
    echo "$1: 补丁与当前工作树冲突；保留现有文件，停止安装" >&2
    exit 1
  fi
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "缺少命令: $1" >&2
    exit 1
  }
}

# ---------------------------------------------------------------------------
# 0. 前置检查
# ---------------------------------------------------------------------------
require_cmd conda
require_cmd git
require_cmd python3

# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"

# ---------------------------------------------------------------------------
# 1. 系统依赖
# ---------------------------------------------------------------------------
if [[ "$SKIP_APT" -eq 0 ]]; then
  log "安装系统依赖 (apt)"
  sudo apt update
  sudo apt install -y \
    g++ make build-essential cmake ffmpeg \
    libserialport-dev libboost-program-options-dev libyaml-cpp-dev libfmt-dev libspdlog-dev \
    libxcb-randr0-dev libxcb-xtest0-dev libxcb-xinerama0-dev \
    libxcb-shape0-dev libxcb-xkb-dev libxcb-cursor0 \
    libx11-xcb1 libqt5gui5
else
  log "跳过 apt (--skip-apt)"
fi

# ---------------------------------------------------------------------------
# 2. Conda 环境
# ---------------------------------------------------------------------------
if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  log "Conda 环境已存在: $ENV_NAME"
else
  log "创建 Conda 环境: $ENV_NAME (python=$PYTHON_VERSION)"
  conda create -y -n "$ENV_NAME" \
    --override-channels \
    -c "$CONDA_FORGE" \
    "python=${PYTHON_VERSION}" libstdcxx-ng pybind11
fi

conda activate "$ENV_NAME"
export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
log "已激活: $CONDA_PREFIX"

# ---------------------------------------------------------------------------
# 3. Placo / Pinocchio（已验证版本组合，勿与 requirements.txt 混装）
# ---------------------------------------------------------------------------
log "安装 Placo / Pinocchio"
python -m pip install --no-cache-dir \
  -i "$PIP_INDEX" \
  "placo==0.9.23" \
  "pin==3.8.0" \
  "coal==3.0.2" \
  "libcoal==3.0.2" \
  "libpinocchio==3.8.0" \
  "cmeel-boost==1.89.0" \
  "cmeel-urdfdom==4.0.1" \
  "cmeel-tinyxml2==10.0.0" \
  "eigenpy==3.12.0" \
  "numpy==2.2.6"

SITE_PKGS="$(python -c 'import site; print(site.getsitepackages()[0])')"
printf '%s\n' \
  'try:' \
  '    import coal  # noqa: F401' \
  'except ImportError:' \
  '    pass' \
  > "$SITE_PKGS/sitecustomize.py"
log "已写入 $SITE_PKGS/sitecustomize.py"

# ---------------------------------------------------------------------------
# 4. 通用 Python 依赖
# ---------------------------------------------------------------------------
log "安装通用 Python 依赖"
python -m pip install \
  -i "$PIP_INDEX" \
  "matplotlib>=3.9" \
  "rerun-sdk>=0.22" \
  "meshcat>=0.3.2" \
  "mujoco>=3.1" \
  "sshkeyboard>=2.3.1" \
  tyro scipy mcap pyzmq opencv-python pyrealsense2 pytest

# ---------------------------------------------------------------------------
# 5. Unitree SDK2 Python
# ---------------------------------------------------------------------------
log "初始化固定版本的机械臂和 Unitree SDK submodule"
git -C "$PROJECT_ROOT" submodule update --init --recursive -- \
  third_party/r1_arm_sdk third_party/unitree_sdk2_python third_party/unitree_sdk2 third_party/dex1_1_service
mkdir -p "$PROJECT_ROOT/third_party"
cd "$PROJECT_ROOT/third_party"

cd unitree_sdk2_python
python -m pip install -e .
python -c "import unitree_sdk2py; print('Unitree SDK 安装成功')"

# ---------------------------------------------------------------------------
# 6. XRoboToolkit Python SDK
# ---------------------------------------------------------------------------
if [[ "$SKIP_XRT" -eq 0 ]]; then
  log "安装 XRoboToolkit Python SDK"
  git -C "$PROJECT_ROOT" submodule update --init --recursive -- \
    third_party/XRoboToolkit-PC-Service-Pybind third_party/XRoboToolkit-PC-Service

  apply_dependency_patch XRoboToolkit-PC-Service xr_pc_service.patch
  apply_dependency_patch XRoboToolkit-PC-Service-Pybind xr_pybind.patch
  PYBIND_ROOT="$PROJECT_ROOT/third_party/XRoboToolkit-PC-Service-Pybind"
  SDK_ROOT="$PROJECT_ROOT/third_party/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK"

  cd "$SDK_ROOT"
  # 丢掉可能从别处拷来的、带绝对路径的旧 CMake 缓存
  if [[ -d build ]]; then
    if rm -rf build 2>/dev/null; then
      :
    else
      echo "警告: 无法删除旧 build/（可能属主是 root）。若编译失败请手动: sudo rm -rf \"$SDK_ROOT/build\"" >&2
    fi
  fi
  bash build.sh

  cd "$PYBIND_ROOT"
  mkdir -p lib include
  cp "$SDK_ROOT/PXREARobotSDK.h" include/
  cp -r "$SDK_ROOT/nlohmann" include/nlohmann/
  cp "$SDK_ROOT/build/libPXREARobotSDK.so" lib/

  python -m pip uninstall -y xrobotoolkit_sdk || true
  python setup.py install
  python -c "import xrobotoolkit_sdk; print('XR SDK 安装成功')"
else
  log "跳过 XRoboToolkit (--skip-xrt)"
fi

# ---------------------------------------------------------------------------
# 7. 原生机械臂 SDK、bridge 和夹爪服务（仅构建，不连接硬件）
# ---------------------------------------------------------------------------
log "构建机械臂 SDK / bridge / Dex1 服务"
cd "$PROJECT_ROOT"
apply_dependency_patch r1_arm_sdk r1_arm_sdk.patch
apply_dependency_patch dex1_1_service dex1_1_service.patch
R1ARM_NATIVE_ROOT="${R1ARM_SDK_ROOT:-${ARM:-$PROJECT_ROOT/third_party/r1_arm_sdk}}"
UNITREE_NATIVE_ROOT="${UNITREE_SDK2_ROOT:-$PROJECT_ROOT/third_party/unitree_sdk2}"
cmake -S "$R1ARM_NATIVE_ROOT" -B "$PROJECT_ROOT/build/r1_arm_sdk" -DCMAKE_BUILD_TYPE=Release
cmake --build "$PROJECT_ROOT/build/r1_arm_sdk" -j"$(nproc)"
python scripts/build_r1arm_sdk_bridge.py --sdk-root "$R1ARM_NATIVE_ROOT"
cmake -S "$PROJECT_ROOT/third_party/dex1_1_service" -B "$PROJECT_ROOT/build/dex1_1_service" \
  -DUNITREE_SDK2_ROOT="$UNITREE_NATIVE_ROOT" -DCMAKE_BUILD_TYPE=Release
cmake --build "$PROJECT_ROOT/build/dex1_1_service" -j"$(nproc)"

# ---------------------------------------------------------------------------
# 8. 自检
# ---------------------------------------------------------------------------
log "依赖自检"
cd "$PROJECT_ROOT"
export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

python - <<'PY'
import numpy
import coal
import pinocchio
import placo
import unitree_sdk2py
import scipy
import meshcat
import cv2
import pyrealsense2
import zmq
import mcap

print("NumPy:", numpy.__version__)
print("Pinocchio:", pinocchio.__version__)

try:
    import xrobotoolkit_sdk
    print("xrobotoolkit_sdk: OK")
except ImportError:
    print("xrobotoolkit_sdk: 未安装（若用了 --skip-xrt 属正常）")

print("核心依赖检查通过")
PY

log "安装完成"
cat <<EOF

下一步:
  conda activate ${ENV_NAME}
  export LD_LIBRARY_PATH="\$CONDA_PREFIX/lib:\$LD_LIBRARY_PATH"
  cd ${PROJECT_ROOT}
  python scripts/teleop.py --sim --right --mock-xr --debug

说明:
  - 若 meshcat / sqlite3 报 CXXABI 错误，确认已把 \$CONDA_PREFIX/lib 放在 LD_LIBRARY_PATH 最前
  - 真机使用前请先阅读 README.md 的设备检查和安全步骤。
EOF
