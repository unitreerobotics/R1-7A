# R1-A7 环境安装指南

完整依赖安装请运行仓库根目录的脚本（默认清华 Conda/PyPI 镜像，不配置代理）：

```bash
cd /path/to/r1a7_teleop
bash setup.sh
```

常用选项：

```bash
bash setup.sh --skip-apt   # 系统依赖已装好时跳过 apt
bash setup.sh --skip-xrt   # 暂不装 XRoboToolkit（无 Pico 也可先跑仿真）
```

装好后每次激活环境建议加上：

```bash
conda activate r1_teleop
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
```

> 不要在安装完成后再次运行 `pip install -r requirements.txt`，会与脚本里已验证的
> Placo/Pinocchio 版本组合冲突。

---

## 1. 网络和代理

清华 Conda/PyPI 镜像可以直连，GitHub 需要使用本机 `7890` HTTP 代理。
首先确认代理能够访问 GitHub：

```bash
curl -x http://127.0.0.1:7890 \
  --http1.1 -I --connect-timeout 15 \
  https://github.com
```

出现第二个 `HTTP/1.1 200 OK`（或 `301/302`）后，配置当前终端：

```bash
export HTTP_PROXY=http://127.0.0.1:7897
export HTTPS_PROXY=http://127.0.0.1:7897
export http_proxy=http://127.0.0.1:7897
export https_proxy=http://127.0.0.1:7897
unset ALL_PROXY all_proxy

# 国内镜像绕过代理
export NO_PROXY="${NO_PROXY:-localhost,127.0.0.1},mirrors.tuna.tsinghua.edu.cn,pypi.tuna.tsinghua.edu.cn"
export no_proxy="$NO_PROXY"
```

`setup.sh` 拉 GitHub（Unitree SDK / XRoboToolkit）前，若直连失败，请先在当前终端
按上面配置好代理再执行脚本。

---

## 9. 仿真测试

不连接机器人和 Pico，先运行 Mock XR 测试（Meshcat 默认开启）：

```bash
cd /path/to/r1a7_teleop
conda activate r1_teleop
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
python scripts/teleop_sim.py --mock-xr --debug
```

关掉可视化：`python scripts/teleop_sim.py --mock-xr --no-is-vis`

仿真测试通过后，再根据机器人实际网卡名称运行真机程序。不要在未确认网络、
急停和关节初始状态之前直接启动真机控制。

---

## 常见问题

### Conda 访问 `pkgs/r` 返回 404

旧的 `~/.condarc` 包含失效渠道。可以将其移除：

```bash
conda config --remove default_channels \
  https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r
```

`setup.sh` 中的 Conda 命令带有 `--override-channels`，不会依赖该失效渠道。

### GitHub 连接超时

检查本地代理：

```bash
curl -x http://127.0.0.1:7890 \
  --http1.1 -I --connect-timeout 15 \
  https://github.com
```

如果只出现 `200 Connection established`，随后报 TLS EOF，说明代理节点或代理
内核异常，需要在代理客户端中更换节点或重启内核。

也可换节点后重试，或对本机已有的完整克隆做本地拷贝（避免反复 git clone）。

### `meshcat` / `sqlite3` 报 `CXXABI_1.3.15` not found

当前 shell 的 `LD_LIBRARY_PATH` 抢先加载了系统 `libstdc++`。激活环境后执行：

```bash
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
```

### XRoboToolkit `build.sh` 报 CMakeCache 路径不匹配

`build/` 从别的机器/目录拷来时会带旧绝对路径。清掉后重编：

```bash
SDK=third_party/XRoboToolkit-PC-Service-Pybind/tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK
sudo rm -rf "$SDK/build"   # 若属主是 root 才需要 sudo
cd "$SDK" && bash build.sh
```

### 不小心在 `third_party` 中运行 `pip install -e .`

如果提示该目录没有 `setup.py` 或 `pyproject.toml`，说明前面的 `cd` 失败了。
这不会损坏环境；进入正确的 SDK 子目录后重新安装即可。
