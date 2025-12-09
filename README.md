# Monte02 + Track-On 关键点跟踪测试说明

本次使用 **Ubuntu 22.04 + ROS2 Humble** 完成了两项测试：

* **笔记本 + Orbbec 336 + Track-on 关键点跟踪**
* **笔记本 + Monte02 真机 + Track-on 关键点跟踪**

项目代码链接：
（*TODO：在此处补充仓库 URL*）

---

## 1. Monte02 视频流获取（Head + Left / Right hand）

要进行后续测试，需要首先完成 **Monte02 的视频流获取（有线连接）**。

视频流获取方式有两种：

1. **方案一：使用珊瑚的可视化网页方式获取视频流**（软件组件更新，需要维护，暂不采用）
2. **方案二：使用 robot_video_client，通过 RTSP 协议获取视频流** ✔（推荐）

RTSP 协议包含 **Server（机器人）** 与 **Client（本机）**，Server 已配置好，只需配置 Client。

robot_video_client 配置可参考 @赵斌斌 的文档，如有疑问可联系他。

---

## 1.1 源码拉取与编译

```bash
# 拉取
mkdir -p 01_ws_robot_video_client/src
cd 01_ws_robot_video_client/src
git clone https://github.com/JackZhaobin/robot_video_client.git
cd robot_video_client/
git checkout feature/ir_rtsp_release_7rtsp
cd ../../

# 编译
colcon build --packages-select robot_video_client
```

> 运行部分可跳转至 **1.2.3 节**

---

## 1.2 配置文件

### 1.2.1 图像传参配置

打开：

```
robot_video_client/config/video_config.json
```

该文件定义各个摄像头及其 RTSP 视频流地址，默认 IP 为 **192.168.11.2**（有线连接 IP）。

> Client 端无需修改此文件，其 Server 端对应路径为：
> `/home/corenetic/robot_workspace/install/robot_video_server/r/share/robot_video_server/r/config/camera_config.json`

（此处为你原文中的两张图片，可在 README 中自行插入）

---

### 1.2.2 局域网 DDS 传输配置

编辑：

```
robot_video_client/config/dds_config.json
```

需要注意的是：

* `ethName` 必须与 **本机连接 AGX 的网口名称一致**
* 必须确保本机网段为 **192.168.11.x**

查看网卡名称：

```bash
ifconfig
```

示例配置：

```json
{
  "domain": {
    "id": 66,
    "multicastIp": "224.5.6.7",
    "multicastPort": 8888,
    "period": 3
  },
  "network": {
    "ethName": "eno1"
  },
  "transport": {
    "portRange": "10050-10100",
    "excludePort": [],
    "domainSockRange": "10050-10100",
    "shmKeyRange": "10050-10100"
  },
  "mempool": {
    "maxSize": 50000000
  }
}
```

---

### 1.2.3 运行 Client

#### 1. 添加到组播

```bash
sudo route add -net 224.5.6.7 netmask 255.255.255.255 dev eth0
```

（若需删除组播）

```bash
sudo route del -net 224.5.6.7 netmask 255.255.255.255 dev eth1
```

#### 2. 初始化环境

```bash
source install/setup.bash
```

#### 3. 启动 client

```bash
ros2 launch robot_video_client robot_video_client.launch.py
```

运行后，新开终端查看话题：

```bash
ros2 topic list
```

在 **rviz2** 中订阅视频流，若深度图无法获取说明 DDS 配置有误。

预期话题包括：

```
/camera_head_front/color/video
/camera_head_front/depth/stream
/left/color/video
/left/depth/stream
/left/left_ir/video
/left/right_ir/video
/right/color/video
/right/depth/stream
/right/left_ir/video
/right/right_ir/video
```

---

## 2. Monte02 头部关键点跟踪

---

## 2.1 环境配置

### 2.1.1 Track-On 推理环境

Track-On 使用 **Python 3.8**，但 ROS2 Humble 使用 **Python 3.10+**。
因此运行 ROS 时必须退出 Track-On 的 conda 环境。

模型权重：

```
https://huggingface.co/gaydemir/track_on/resolve/main/track_on_checkpoint.pt?download=true
```

无需参考官方 track_on 仓库的环境配置步骤。

#### 查看系统 Python

```bash
which python3
python3 -V
```

#### 退出所有 conda 环境并确保使用系统 Python

```bash
conda deactivate
export PATH=/usr/bin:$PATH
unset PYTHONPATH PYTHONHOME

which python3
python3 -V
```

#### 安装依赖

```bash
sudo apt-get update
sudo apt-get install -y python3-pip
```

安装 torch 与 mmcv：

```bash
python3 -m pip install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
python3 -m pip install --no-cache-dir mmcv==2.2.0 -f https://download.openmmlab.com/mmcv/dist/cu121/torch2.4/index.html
```

安装 Track-on 依赖：

```bash
cd path/to/trackon
python3 -m pip install --no-cache-dir -r src/track_on/requirements.txt
```

编译：

```bash
cd path/to/work_space/tracking_with_cameara_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

### 2.1.2 Orbbec 相机配置（可选）

如果未使用 Orbbec 336 测试，可跳过。
但 **colcon build** 时需忽略 OrbbecSDK_ROS2。

参考：**OrbbecSDK_ROS2**

---

### 2.1.3 robot_video_client 环境配置

参考 **第一节**。

---

## 2.2 编译与运行

执行：

```bash
colcon build
```

编译成功后即可运行脚本。本工程包含三个脚本：

| 脚本名                                 | 功能                                              |
| ----------------------------------- | ----------------------------------------------- |
| `tracking_with_cam0.sh`             | 使用本机摄像头测试 Track-on 环境                           |
| `tracking_with_orbbec.sh`           | 使用 Orbbec 336 进行关键点跟踪                           |
| `tracking_with_robot_front_head.sh` | 使用 robot_video_client 的高帧率 RTSP 视频流并运行 Track-on |

---

## 2.3 运行结果示例

随意点击若干点后按 **空格键** 即可开始跟踪。
推理帧率约 **15–20 FPS**。

---

**Finish on 2025.12.2**
Any question please contact **zhaoxian.lin**

---

如果你希望我把“图片”部分替换成占位符、加目录、自动加入脚本示例、或渲染成更专业的技术文档风格，我也可以继续优化。
