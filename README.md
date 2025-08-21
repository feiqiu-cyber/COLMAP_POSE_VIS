# COLMAP Dynamic Camera Trajectory Visualization

This tool visualizes **COLMAP** reconstruction results including camera poses and 3D points in an interactive 3D viewer, with support for dynamic playback of the camera trajectory.
You can choose to show camera frustums progressively during playback, or display all at once.

👉 For the Chinese version, see [README\_CN](./README_CN.md)

## Features

* Parse **COLMAP** model files: `cameras.txt`, `images.txt`, `points3D.txt`

  * If the input directory only contains `.bin` files, the script will call `colmap model_converter` to automatically generate `.txt` files
* Dynamic playback of the camera trajectory
* Optionally show cameras progressively (frustum, axes, camera centers) during playback
* Visualize point cloud (with optional random downsampling for large datasets)
* Sort trajectory by **image\_id** or **filename order**
* Interactive keyboard controls

## Installation

```bash
pip install open3d numpy
```

If you want automatic conversion from `.bin` to `.txt`, ensure that `colmap` is installed and accessible in your PATH.

## Usage

```bash
python main.py --model_dir /path/to/sparse/0 [options]
```

### Key Arguments

* `--model_dir` : Path to the COLMAP model (usually `sparse/0`)
* `--fps` : Playback speed (default `5`)
* `--traj_sort {id,name}` : Trajectory sorting method

  * `id` → ascending by `image_id`
  * `name` → natural order by filename
* `--scale` : Camera frustum size (default `0.1`)
* `--max_points` : Maximum number of points to render (default `200000`, random sampling if exceeded)
* `--show_only_current_frustum` : If enabled, frustums will appear progressively along playback instead of showing all at once
* `--use_colmap_converter` : Enable use of `colmap model_converter` when input files are `.bin` (default enabled)

## Keyboard Controls

* **Space** : Play / Pause
* **n / N** : Next frame (single step)
* **p / P** : Previous frame (single step)
* **+ / =** : Increase playback speed
* **-** : Decrease playback speed
* **r / R** : Reset to the first frame (also clears accumulated frustums if `--show_only_current_frustum` is enabled)
* **q / Q / Esc** : Quit visualization

## Examples

```bash
# Play trajectory in filename order, show frustums progressively
python main.py --model_dir ./sparse/0 --traj_sort name --show_only_current_frustum

# Show all frustums at once, play at 8 FPS
python main.py --model_dir ./sparse/0 --fps 8
```

---

## Notes

* Initial load may take time for large point clouds; reduce `--max_points` for faster visualization
* With `--show_only_current_frustum`, cameras remain visible once played until reset or exit
* Extension: If you want to render original images onto the frustums, you need to provide image file paths
