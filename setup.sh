#!/usr/bin/env bash
# R1Arm teleoperation environment setup.
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
#   python scripts/teleop_r1arm_sim.py --mock-xr --debug
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
  tyro scipy mcap pyzmq opencv-python pytest

# ---------------------------------------------------------------------------
# 5. Unitree SDK2 Python
# ---------------------------------------------------------------------------
log "安装 Unitree SDK2 Python"
mkdir -p "$PROJECT_ROOT/third_party"
cd "$PROJECT_ROOT/third_party"

if [[ ! -d unitree_sdk2_python/.git ]]; then
  git -c http.version=HTTP/1.1 clone --depth 1 \
    https://github.com/unitreerobotics/unitree_sdk2_python.git
fi

cd unitree_sdk2_python
python -m pip install -e .
python -c "import unitree_sdk2py; print('Unitree SDK 安装成功')"

# ---------------------------------------------------------------------------
# 6. XRoboToolkit Python SDK
# ---------------------------------------------------------------------------
if [[ "$SKIP_XRT" -eq 0 ]]; then
  log "安装 XRoboToolkit Python SDK"
  cd "$PROJECT_ROOT/third_party"

  if [[ ! -d XRoboToolkit-PC-Service-Pybind/.git ]]; then
    git -c http.version=HTTP/1.1 clone --depth 1 \
      https://github.com/YanjieZe/XRoboToolkit-PC-Service-Pybind.git
  fi

  PYBIND_ROOT="$PROJECT_ROOT/third_party/XRoboToolkit-PC-Service-Pybind"
  SDK_ROOT="$PYBIND_ROOT/tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK"
  mkdir -p "$PYBIND_ROOT/tmp"
  cd "$PYBIND_ROOT/tmp"

  if [[ ! -d XRoboToolkit-PC-Service/.git ]]; then
    git -c http.version=HTTP/1.1 clone --depth 1 --single-branch \
      https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
  fi

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
# 7. 自检
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
  python scripts/teleop_r1arm_sim.py --mock-xr --debug

说明:
  - 若 meshcat / sqlite3 报 CXXABI 错误，确认已把 \$CONDA_PREFIX/lib 放在 LD_LIBRARY_PATH 最前
  - 真机使用前请先阅读 README.md 的设备检查和安全步骤。
EOF
