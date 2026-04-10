# Kinect-Skeleton-Extraction

## 基于 Kinect 深度信息的骨骼数据提取算法的改进与实现

**Improvement and Implementation of Skeleton Data Extraction Algorithm Based on Kinect Depth Information**

[English](#english) | [中文](#中文)

---

<a id="中文"></a>

## 中文

### 项目简介

本项目针对 Kinect V2 深度相机在**手臂自遮挡**场景下骨骼关节点定位不准确的问题，提出了一种基于深度图像特性的改进算法。

**核心贡献：**

1. **深度图像前景人体提取** — 结合 Kinect 用户索引图像（IBodyIndexFrame）与阈值分割，从深度图像中精确提取人体区域，并通过中值滤波和数学形态学操作进行图像预处理。

2. **自遮挡手臂关节点提取** — 利用手臂与躯干之间的深度值差异提取自遮挡手臂，通过 Guo-Hall 图像细化算法获得手臂骨架，再借助肘部关节点的位置稳定性定位手部关节点。支持左手、右手及双手同时自遮挡三种情况。

### 算法流程

```
Kinect 采集深度图像 + 用户索引图像
        ↓
基于用户索引的人体前景提取 → 二值化 → 图像预处理
        ↓
截取人体中段 → 计算平均深度 → 阈值分割提取手臂
        ↓
基于躯干矩形判断是否存在自遮挡
        ↓
  是 → Guo-Hall 细化 → 骨架端点定位 → 手部关节点
  否 → 直接使用 Kinect 原始骨骼数据
        ↓
合并关节点 → 完整人体骨架
```

### 文件说明

| 文件 | 说明 |
|------|------|
| `main.cpp` | **主程序**，包含完整的改进算法（人体提取、手臂分离、自遮挡判断、关节点定位、性能计时） |
| `Depth_Information.cpp` | 在彩色图像上绘制 Kinect 原生骨架的参考程序 |
| `markedCoordinate.cpp` | 人工标注工具，用于在保存的图像上手动点击标注关节点坐标 |
| `norm_L2.cpp` | 误差计算工具，计算人工标注点与算法定位点之间的 L2 范数误差 |

### 环境配置

**硬件要求：**
- Kinect V2 传感器 + Kinect Adapter for Windows
- 支持 USB 3.0 的 Windows PC

**软件依赖：**

| 依赖 | 版本 | 说明 |
|------|------|------|
| Visual Studio | 2019+ | C++ 开发环境 |
| Kinect for Windows SDK 2.0 | v2.0 | [下载地址](https://www.microsoft.com/en-us/download/details.aspx?id=44561) |
| OpenCV | 4.x | 核心图像处理 |
| opencv_contrib | 与 OpenCV 同版本 | 提供 `ximgproc::thinning`（Guo-Hall 细化算法） |

**VS 项目配置：**

安装 Kinect SDK 后，系统环境变量 `KINECTSDK20_DIR` 会自动设置。在 VS 项目属性中配置：

- **C/C++ → 附加包含目录：** `$(KINECTSDK20_DIR)\inc;` + OpenCV 的 include 路径
- **链接器 → 附加库目录：** `$(KINECTSDK20_DIR)\lib\x64;` + OpenCV 的 lib 路径
- **链接器 → 附加依赖项：** `kinect20.lib` + OpenCV 相关 lib

> **注意：** opencv_contrib 需要单独下载并用 CMake 与 OpenCV 一起重新编译。

### 物理约束条件

实验时需满足以下条件以获得最佳效果：
- Kinect 传感器离地高度：约 1.1～1.15 米
- 人体与传感器距离：2.35～2.5 米
- 传感器水平放置，人体正面朝向传感器

### 实验结果

改进算法在手臂自遮挡场景下，手部关节点定位误差平均约 3.2 像素（Kinect 原算法平均约 7.8 像素），定位准确率显著提升。详细实验数据参见论文第四章。

---

<a id="english"></a>

## English

### Overview

This project addresses the inaccurate joint localization of Kinect V2's built-in skeleton tracking when **arm self-occlusion** occurs. An improved algorithm based on depth image characteristics is proposed.

**Key Contributions:**

1. **Foreground Human Body Extraction from Depth Images** — Combines Kinect's user index image (IBodyIndexFrame) with threshold segmentation to extract the human body region, followed by median filtering and morphological preprocessing.

2. **Self-Occluded Arm Joint Extraction** — Leverages the depth difference between the arm and torso to isolate occluded arms, applies Guo-Hall image thinning to obtain the arm skeleton, and locates hand joints using the positional stability of elbow joints. Supports left-arm, right-arm, and both-arm occlusion scenarios.

### Algorithm Pipeline

```
Kinect captures depth image + body index image
        ↓
Body extraction via user index → Binarization → Preprocessing
        ↓
Crop torso region → Compute average depth → Threshold to extract arm
        ↓
Check if hand joint is inside torso rectangle (occlusion detection)
        ↓
  Yes → Guo-Hall thinning → Skeleton endpoints → Hand joint localization
  No  → Use Kinect's original skeleton data
        ↓
Merge joints → Complete body skeleton
```

### File Description

| File | Description |
|------|-------------|
| `main.cpp` | **Main program** — full pipeline: body extraction, arm separation, occlusion detection, joint localization, performance timing |
| `Depth_Information.cpp` | Reference program for drawing Kinect's native skeleton on color images |
| `markedCoordinate.cpp` | Manual annotation tool for clicking joint positions on saved images |
| `norm_L2.cpp` | Error calculation tool — computes L2 norm between manual annotations and algorithm outputs |

### Environment Setup

**Hardware:**
- Kinect V2 sensor + Kinect Adapter for Windows
- Windows PC with USB 3.0

**Software Dependencies:**

| Dependency | Version | Notes |
|------------|---------|-------|
| Visual Studio | 2019+ | C++ development |
| Kinect for Windows SDK 2.0 | v2.0 | [Download](https://www.microsoft.com/en-us/download/details.aspx?id=44561) |
| OpenCV | 4.x | Core image processing |
| opencv_contrib | Same as OpenCV | Provides `ximgproc::thinning` (Guo-Hall algorithm) |

**VS Project Configuration:**

After installing Kinect SDK, the environment variable `KINECTSDK20_DIR` is set automatically. Configure in VS project properties:

- **C/C++ → Additional Include Directories:** `$(KINECTSDK20_DIR)\inc;` + OpenCV include path
- **Linker → Additional Library Directories:** `$(KINECTSDK20_DIR)\lib\x64;` + OpenCV lib path
- **Linker → Additional Dependencies:** `kinect20.lib` + OpenCV libs

> **Note:** opencv_contrib must be downloaded separately and compiled together with OpenCV using CMake.

### Physical Constraints

For optimal results, the following conditions should be met:
- Kinect sensor height: ~1.1–1.15 m above ground
- Subject distance: 2.35–2.5 m from the sensor
- Sensor placed horizontally, subject facing the sensor

### Results

Under arm self-occlusion, the improved algorithm achieves an average hand joint localization error of ~3.2 pixels, compared to ~7.8 pixels with Kinect's built-in algorithm. See Chapter 4 of the thesis for detailed experimental data.

---

### License

This project is for academic and educational purposes.

