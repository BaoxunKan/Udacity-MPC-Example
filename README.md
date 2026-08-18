## 用 Cursor 打开 Dev Container

本仓库已包含 `.devcontainer/`。推荐用 [Cursor](https://cursor.com/) 在容器里开发：cmake、g++、Ipopt、CppAD、uWebSockets 等依赖都在镜像里，打开后会自动编译。

### 前置条件

- 已安装并启动 [Docker](https://docs.docker.com/get-docker/)
- 已安装 [Cursor](https://cursor.com/)
- 已安装 Dev Containers 扩展：在 Extensions 中搜索 `Dev Containers` 并启用

### 打开步骤

1. 在 Cursor 中打开本仓库：`File` → `Open Folder…`
2. 出现 **Reopen in Container** 提示时点确认。
   若没有提示：按 `Ctrl+Shift+P`（macOS：`Cmd+Shift+P`），输入并选择 **Dev Containers: Reopen in Container**
3. 首次会构建镜像，需要几分钟。完成后左下角会显示容器名 `CarND MPC`
4. 容器创建后会自动执行 `cmake` + `make`，可执行文件在 `build/mpc`

之后在 Cursor 容器终端里编译、运行即可，不必在宿主机安装 Ipopt / uWebSockets。

### 修改代码后重新编译

```bash
cmake --build build -j$(nproc)
```

## 运行效果

湖区赛道上的 MPC 跟踪录屏见 [`result/mpc_result.mkv`](./result/mpc_result.mkv)：

<video src="./result/mpc_result.mkv" controls width="720"></video>

若 Markdown 预览无法播放，用系统播放器直接打开该文件即可。

## 和 Simulator 一起运行

`mpc` 在 **Dev Container 内** 监听 WebSocket 端口 `4567`；Udacity Term 2 Simulator 是带图形界面的 Unity 程序，需要在 **宿主机** 上运行（不要放进容器）。容器已把 `4567` 映射到本机（`forwardPorts` 与 `-p 4567:4567`），Simulator 连接 `localhost:4567` 即可。

```
宿主机 Simulator  --ws://localhost:4567-->  Dev Container 里的 ./mpc
```

Simulator 与控制器之间的数据格式见 [DATA.md](./DATA.md)。

### 1. 解压仓库里的 Simulator（宿主机）

Linux 版已经放在仓库根目录：[`term2_sim_linux.zip`](./term2_sim_linux.zip)，不必再从 GitHub Releases 下载。在**宿主机**终端（有图形界面的 Linux 桌面，不是 Dev Container）执行：

```bash
unzip term2_sim_linux.zip
chmod +x term2_sim_linux/term2_sim.x86_64
```

解压后目录结构为：

```
term2_sim_linux/
  term2_sim.x86_64   # 64 位可执行文件（推荐）
  term2_sim.x86      # 32 位
  term2_sim_Data/    # Unity 资源，需与可执行文件放在一起
```

只需解压一次。解压出的 `term2_sim_linux/` 不要提交进 git。

### 2. 在容器里启动 MPC

在 Cursor 的容器终端中：

```bash
cd build && ./mpc
```

看到 `Listening to port 4567` 表示就绪。默认读取当前目录的 `mpc_config.json`（CMake 会把它复制到 `build/`）。也可以指定仓库根目录的配置：

```bash
./mpc ../mpc_config.json
```

### 3. 在宿主机启动 Simulator

先确保上一步的 `./mpc` 已在监听，再在宿主机运行：

```bash
./term2_sim_linux/term2_sim.x86_64
```

1. 在 Simulator 菜单中选择 **Project 5: MPC**（或 MPC 场景）并开始
2. 容器终端应打印 `Connected!!!`，车辆开始由控制器驾驶

断开后终端会打印 `Disconnected`；重新在 Simulator 里点开始即可再次连接。

其他系统若没有用仓库里的 zip，可从 [self-driving-car-sim releases](https://github.com/udacity/self-driving-car-sim/releases) 下载对应的 Term 2 Simulator。

### 常见问题

- **`Failed to listen to port`**：本机 `4567` 已被占用。关掉之前的 `mpc`，或执行 `docker-compose down`。
- **Simulator 一直 Connecting / 连不上**：确认容器在跑且日志已有 `Listening to port 4567`；打开 Cursor 的 **PORTS** 面板，确认 `4567` 已转发到本机。
- **改代码后行为没变**：先 `cmake --build build`，再重启 `./mpc`。
- **控制效果变差、延迟明显**：容器 / VM 额外延迟会影响 MPC。尽量让 Docker 跑在本地 Linux 引擎上，而不是嵌套虚拟机。

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. Linux 版已放在仓库根目录 [`term2_sim_linux.zip`](./term2_sim_linux.zip)；其他系统见 [releases tab](https://github.com/udacity/self-driving-car-sim/releases)。解压与联调见上文 **和 Simulator 一起运行**。
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
