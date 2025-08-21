# COLMAP 动态相机轨迹可视化 / COLMAP Dynamic Camera Trajectory Visualization

本工具用于将 **COLMAP** 重建结果中的相机位姿和点云进行三维可视化，并支持动态播放相机轨迹。
This tool visualizes **COLMAP** reconstruction results including camera poses and 3D points in an interactive 3D viewer, with support for dynamic playback of the camera trajectory.

你可以选择在播放过程中逐步显示相机锥体，也可以一次性显示所有相机。
You can choose to show camera frustums progressively during playback, or display all at once.

---

## 功能特性 / Features

* 解析 **COLMAP** 的 `cameras.txt`、`images.txt`、`points3D.txt` 文件
  Parse **COLMAP** model files: `cameras.txt`, `images.txt`, `points3D.txt`

  * 若输入目录仅包含 `.bin` 文件，脚本会调用 `colmap model_converter` 自动转换为 `.txt`
    If the input directory only contains `.bin` files, the script will call `colmap model_converter` to generate `.txt` files
* 动态播放相机轨迹 / Dynamic playback of the camera trajectory
* 可选择播放过程中逐步显示相机（锥体、坐标轴、中心点）
  Optionally show cameras progressively (frustum, axes, camera centers) during playback
* 支持点云可视化（可选采样点数限制，避免过大点云卡顿）
  Visualize point cloud (with optional random downsampling for large datasets)
* 支持轨迹按 **image\_id** 或 **文件名顺序** 播放
  Sort trajectory by **image\_id** or **filename order**
* 支持交互控制（键盘热键）
  Interactive keyboard controls

---

## 安装依赖 / Installation

```bash
pip install open3d numpy
```

若需自动从 `.bin` 转 `.txt`，需确保系统安装了 `colmap` 并在 PATH 中。
If you want automatic conversion from `.bin` to `.txt`, ensure `colmap` is installed and in PATH.

---

## 使用方法 / Usage

```bash
python main.py --model_dir /path/to/sparse/0 [options]
```

### 常用参数 / Key Arguments

* `--model_dir`：COLMAP 模型目录（通常为 `sparse/0`）
  Path to the COLMAP model (usually `sparse/0`)
* `--fps`：播放速度（默认 `5`）
  Playback speed (default `5`)
* `--traj_sort {id,name}`：轨迹排序方式
  Trajectory sorting method:

  * `id` → 按 `image_id` 升序 / ascending by `image_id`
  * `name` → 按文件名自然顺序 / natural order by filename
* `--scale`：相机锥体大小（默认 `0.1`）
  Camera frustum size (default `0.1`)
* `--max_points`：最大点数，超出将随机采样（默认 `200000`）
  Maximum number of points to render (default `200000`, random sampling if exceeded)
* `--show_only_current_frustum`：启用后，播放过程中逐步显示相机锥体，而不是一次性全部显示
  If enabled, frustums will appear progressively instead of all at once
* `--use_colmap_converter`：若输入为 `.bin`，是否调用 `colmap model_converter`（默认开启）
  Enable `colmap model_converter` for `.bin` input (default enabled)

---

## 键盘交互 / Keyboard Controls

* **空格 / Space**：播放 / 暂停
  Play / Pause
* **n / N**：单步到下一帧
  Next frame (step)
* **p / P**：单步到上一帧
  Previous frame (step)
* **+ / =**：提高播放速度
  Increase playback speed
* **-**：降低播放速度
  Decrease playback speed
* **r / R**：回到第一帧（并清除已显示的相机，如果启用了 `--show_only_current_frustum`）
  Reset to first frame (also clears accumulated frustums if progressive mode enabled)
* **q / Q / Esc**：退出可视化
  Quit visualization

---

## 示例 / Examples

```bash
# 按文件名顺序播放轨迹，逐步显示相机
# Play trajectory in filename order, show frustums progressively
python main.py --model_dir ./sparse/0 --traj_sort name --show_only_current_frustum

# 一次性显示所有相机，每秒 8 帧
# Show all frustums at once, play at 8 FPS
python main.py --model_dir ./sparse/0 --fps 8
```

---

## 注意事项 / Notes

* 初次运行可能较慢，尤其在点云很大时；可调小 `--max_points` 加快可视化速度
  Initial load may be slow for large point clouds; reduce `--max_points` for faster rendering
* 使用 `--show_only_current_frustum` 时，已播放的相机会保持在场景中，直到重置或退出
  With `--show_only_current_frustum`, frustums remain visible once played until reset or exit
* 如果需要在相机锥体上贴原始图像，需要提供图像路径，功能可扩展
  Extension: To render original images onto frustums, image file paths are required
