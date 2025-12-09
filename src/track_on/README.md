# Track-On: Transformer-based Online Point Tracking with Memory  

[**arXiv**](https://arxiv.org/abs/2501.18487) | [**Webpage**](https://kuis-ai.github.io/track_on)

This repository is the official implementation of the paper:

> **Track-On: Transformer-based Online Point Tracking with Memory**  
>
> [Görkay Aydemir](https://gorkaydemir.github.io), Xiongyi Cai, [Weidi Xie](https://weidixie.github.io), [Fatma Guney](https://mysite.ku.edu.tr/fguney/)
>
> *International Conference on Learning Representations (ICLR), 2025*  
## Overview

**Track-On** is an efficient, **online point tracking** model that tracks points in a **frame-by-frame** manner using memory. It leverages a transformer-based architecture to maintain a compact yet effective memory of previously tracked points.

<p align="center">
  <img src="media/method_overview.png" alt="Track-On Overview" width="800" />
</p>

---

## Installation

### 1. Clone the repository
```bash
git clone https://github.com/gorkaydemir/track_on.git 
cd track_on
```

### 2. Set up the environment
```bash
conda create -n trackon python=3.8 -y
conda activate trackon
conda install pytorch==2.4.1 torchvision==0.19.1 torchaudio==2.4.1 pytorch-cuda=12.1 -c pytorch -c nvidia
pip install mmcv==2.2.0 -f https://download.openmmlab.com/mmcv/dist/cu121/torch2.4/index.html
pip install -r requirements.txt
```

### 3. Download Datasets  

To obtain the necessary datasets, follow the instructions provided in the [TAP-Vid repository](https://github.com/google-deepmind/tapnet/tree/main/tapnet/tapvid):  

- **Evaluation Datasets**:  
  - TAP-Vid Benchmark (DAVIS, RGB-Stacking, Kinetics)
  - Robo-TAP  

- **Training Dataset**:  
  - MOVi-F – Refer to this [GitHub issue](https://github.com/facebookresearch/co-tracker/issues/8) for additional guidance.

---

## Quick Demo

Check out the [demo notebook](demo.ipynb) for a quick start with the model.

### Usage Options
Track-On provides two practical usage modes, both handling frames online but differing in input format:

#### 1. **Frame-by-frame input (for streaming videos)**
```python
from model.track_on_ff import TrackOnFF

model = TrackOnFF(args)
model.init_queries_and_memory(queries, first_frame)

while True:
    out = model.ff_forward(new_frame)
```

#### 2. **Video input (for benchmarking)**
```python
from model.track_on import TrackOn

model = TrackOn(args)
out = model.inference(video, queries)
```

---

## Evaluation

### 1. Download Pretrained Weights
Download the pre-trained checkpoint from [Hugging Face](https://huggingface.co/gaydemir/track_on/resolve/main/track_on_checkpoint.pt?download=true).

### 2. Run Evaluation
Given:
- `evaluation_dataset`: The dataset to evaluate on
- `tapvid_root`: Path to evaluation dataset
- `checkpoint_path`: Path to the downloaded checkpoint

Run the following command:
```bash
torchrun --master_port=12345 --nproc_per_node=1 main.py \
    --eval_dataset evaluation_dataset \
    --tapvid_root /path/to/eval/data \
    --checkpoint_path /path/to/checkpoint \
    --online_validation
```
This should reproduce the exact results reported in the paper when configured correctly.

---

## Training

### 1. Prepare datasets
- **Movi-f dataset**: Located at `/root/to/movi_f`
- **TAP-Vid evaluation dataset**:
  - Dataset name: `eval_dataset`
  - Path: `/root/to/tap_vid`
- **Training name**: `training_name`

### 2. Run Training
A multi-node training script is provided in [`train.sh`](scripts/train.sh). Default training arguments are set within the script.

---

## 📖 Citation
If you find our work useful, please cite:
```bibtex
@InProceedings{Aydemir2025ICLR,
    author    = {Aydemir, G\"orkay and Cai, Xiongyi and Xie, Weidi and G\"uney, Fatma},
    title     = {{Track-On}: Transformer-based Online Point Tracking with Memory},
    booktitle = {The Thirteenth International Conference on Learning Representations},
    year      = {2025}
}
```

---

## Acknowledgments
This repository incorporates code from several public works, including [CoTracker](https://github.com/facebookresearch/co-tracker), [TAPNet](https://github.com/google-deepmind/tapnet), [DINOv2](https://github.com/facebookresearch/dinov2), [ViTAdapter](https://github.com/czczup/ViT-Adapter), and [SPINO](https://github.com/robot-learning-freiburg/SPINO). Special thanks to the authors of these projects for making their code available.

# H5视频提取工具

这个工具用于从特定格式的H5文件中提取双摄像头视频数据并转换为MP4格式。

## 依赖项安装

在运行脚本前，请确保安装以下依赖：

```bash
pip install h5py numpy opencv-python tqdm
```

## 使用方法

1. 将H5文件放在指定目录下
2. 打开`extract_videos.py`文件，根据需要修改以下配置：
   - `h5_file_path`: H5文件的路径
   - `output_dir`: 提取的视频输出目录
   - `fps`: 视频帧率（默认30fps）
3. 运行脚本：

```bash
python extract_videos.py
```

4. 脚本会在指定的输出目录中生成两个MP4视频文件，分别对应两个摄像头。

## 数据结构说明

该脚本专门针对包含如下结构的H5文件：
- 多个名为"step_X"的组，其中X是帧序号
- 每个step组中包含"obs/image"数据集
- 图像数据集形状为(1, 2, 3, 480, 640)，分别代表：
  - 批次维度 (1)
  - 摄像头数量 (2)
  - 颜色通道 (3) - RGB
  - 图像高度 (480)
  - 图像宽度 (640)

# 珊瑚数据track_on处理指南

本指南说明如何使用track_on工具处理珊瑚数据集。

## 处理流程

### 1. 从源数据提取视频
使用`convert_h5_to_mp4.py`从原始H5文件中提取出MP4视频和第一帧图片(PNG)：

```bash
python3 convert_h5_to_mp4.py /home/nexus/workspaces/tracking/track_on/media/0509
```

### 2. 标注关键点
使用labelme的create point功能对提取出的第一帧图片进行标注，生成JSON格式的标注文件。

### 3. 关键点追踪
使用`track_on_videos.py`进行关键点追踪：

```bash
python track_on_videos.py --input_dir /home/nexus/workspaces/tracking/dataset/0510/0509_cuihu/logs_pm --output_dir /home/nexus/workspaces/tracking/track_on/out/coral_dataset_0510_pm --checkpoint_path /home/nexus/workspaces/tracking/track_on/checkpoints/track_on_checkpoint.pt
```

### 4. 检查追踪效果
在输出的每条数据文件夹中检查追踪效果：
- 如果效果不好，则删除该数据（建议记录未采用的数据）
- 如果效果好，则将原始H5文件也复制到该输出文件夹内

### 5. 添加关键点信息
最后，运行`add_keypoint_to_h5.py`将追踪好的关键点信息添加到H5文件中：

```bash
python add_keypoint_to_h5.py --input_dir media/coral_0509/20250809_1 --output_dir media/coral_0509/20250809_1_added_kp
```

