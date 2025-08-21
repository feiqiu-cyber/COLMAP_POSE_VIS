#!/usr/bin/env python3
"""
visualize_colmap_dynamic.py

Interactive / dynamic visualization of COLMAP poses + trajectory.

Features:
 - Parse COLMAP TXT (or auto-convert from .bin if `colmap` is available)
 - Display point cloud, camera frustums and frames
 - Dynamically animate camera trajectory: play/pause, step, change speed
 - Highlight current camera (moving marker + colored frustum)
 - Optionally show only frustums up to the current camera while playing (cumulative)
 - Keyboard controls shown in the console

Usage:
  pip install open3d numpy
  python visualize_colmap_dynamic.py --model_dir /path/to/sparse/0 --fps 5 --traj_sort name --show_only_current_frustum

Keyboard controls (when Open3D window active):
  Space      : toggle play / pause
  n or N     : step to next frame (when paused)
  p or P     : step to previous frame (when paused)
  + / =      : increase playback speed (fps)
  -          : decrease playback speed (fps)
  r or R     : reset to first camera (clears cumulative frustums if --show_only_current_frustum used)
  q or Q / Esc: quit

Note: requires Open3D with VisualizerWithKeyCallback support.
"""

import argparse
import os
import subprocess
import time
import math
import re
from typing import Dict, List, Tuple

import numpy as np
import open3d as o3d

# -------------------------
# Utilities: COLMAP TXT parsing (cameras.txt, images.txt, points3D.txt)
# -------------------------

def read_cameras_txt(path: str) -> Dict[int, Dict]:
    cameras = {}
    with open(path, 'r') as f:
        for line in f:
            if line.startswith('#') or line.strip() == '':
                continue
            parts = line.strip().split()
            cam_id = int(parts[0])
            model = parts[1]
            width = int(parts[2]); height = int(parts[3])
            params = list(map(float, parts[4:]))
            cameras[cam_id] = {'model': model, 'width': width, 'height': height, 'params': params}
    return cameras


def read_images_txt(path: str) -> Dict[int, Dict]:
    images = {}
    with open(path, 'r') as f:
        lines = [l.strip() for l in f if not l.startswith('#') and l.strip() != '']
    i = 0
    while i < len(lines):
        parts = lines[i].split()
        if len(parts) < 9:
            i += 1
            continue
        image_id = int(parts[0])
        qvec = list(map(float, parts[1:5]))
        tvec = list(map(float, parts[5:8]))
        camera_id = int(parts[8])
        name = parts[9] if len(parts) > 9 else ''
        images[image_id] = {'qvec': qvec, 'tvec': tvec, 'camera_id': camera_id, 'name': name}
        i += 1
        if i < len(lines):
            i += 1
    return images


def read_points3D_txt(path: str) -> Dict[int, Dict]:
    pts = {}
    with open(path, 'r') as f:
        for line in f:
            if line.startswith('#') or line.strip() == '':
                continue
            parts = line.split()
            pid = int(parts[0])
            x, y, z = map(float, parts[1:4])
            r, g, b = map(int, parts[4:7])
            error = float(parts[7])
            pts[pid] = {'xyz': np.array([x, y, z], dtype=float), 'rgb': np.array([r,g,b], dtype=np.uint8), 'error': error}
    return pts

# -------------------------
# Math helpers
# -------------------------

def qvec2rotmat(qvec: List[float]) -> np.ndarray:
    # qvec: [qw, qx, qy, qz]
    qw, qx, qy, qz = qvec
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n == 0:
        return np.eye(3)
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    R = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=float)
    return R


def build_K_from_camera(camera: Dict) -> np.ndarray:
    model = camera['model'].upper()
    params = camera['params']
    if model == 'SIMPLE_PINHOLE':
        f, cx, cy = params[0], params[1], params[2]
        K = np.array([[f,0,cx],[0,f,cy],[0,0,1]], dtype=float)
    elif model == 'PINHOLE':
        fx, fy, cx, cy = params[0], params[1], params[2], params[3]
        K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=float)
    else:
        if len(params) >= 4:
            fx, fy, cx, cy = params[0], params[1], params[2], params[3]
            K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=float)
        elif len(params) >= 3:
            f, cx, cy = params[0], params[1], params[2]
            K = np.array([[f,0,cx],[0,f,cy],[0,0,1]], dtype=float)
        else:
            raise ValueError(f"Unknown camera params: {camera['model']} {params}")
    return K

# natural sort for filenames
def natural_key(s: str):
    parts = re.split('([0-9]+)', s)
    key = []
    for p in parts:
        if p.isdigit():
            key.append(int(p))
        else:
            key.append(p.lower())
    return key

# -------------------------
# Build frustum and local geometries
# -------------------------

def create_camera_frustum(R: np.ndarray, t: np.ndarray, K: np.ndarray, width: int, height: int, scale: float = 0.5):
    # Camera center
    C = -R.T.dot(t)
    corners_px = np.array([[0,0],[width,0],[width,height],[0,height]], dtype=float)
    z = scale
    Kinv = np.linalg.inv(K)
    corners_cam = (Kinv.dot(np.column_stack((corners_px, np.ones((4,)))).T) * z).T
    corners_world = (R.T.dot(corners_cam.T)).T + C.reshape((1,3))
    points = [C] + corners_world.tolist()
    lines = []
    for i in range(1,5):
        lines.append([0,i])
    lines += [[1,2],[2,3],[3,4],[4,1]]
    ls = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(points),
                              lines=o3d.utility.Vector2iVector(lines))
    ls.colors = o3d.utility.Vector3dVector([[0.0, 1.0, 0.0]] * len(lines))
    # small coordinate frame
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale*0.2)
    T = np.eye(4)
    T[:3,:3] = R.T
    T[:3,3] = C
    frame.transform(T)
    return ls, frame, C

# -------------------------
# Convert BIN -> TXT using colmap if available
# -------------------------

def convert_bin_to_txt(model_dir: str, out_dir: str):
    cmd = ['colmap', 'model_converter', '--input_path', model_dir, '--output_path', out_dir, '--output_type', 'TXT']
    print('Running:', ' '.join(cmd))
    subprocess.check_call(cmd)

# -------------------------
# Main: prepare geometry + start interactive loop
# -------------------------

def visualize_dynamic(txt_dir: str,
                      scale: float = 0.8,
                      max_points: int = 200000,
                      traj_sort: str = 'name',
                      fps: float = 5.0,
                      start_playing: bool = True,
                      show_only_current_frustum: bool = True):

    cameras = read_cameras_txt(os.path.join(txt_dir, 'cameras.txt'))
    images = read_images_txt(os.path.join(txt_dir, 'images.txt'))
    points3D = read_points3D_txt(os.path.join(txt_dir, 'points3D.txt'))

    # build point cloud
    pts = []
    cols = []
    for pid, d in points3D.items():
        pts.append(d['xyz'])
        cols.append(d['rgb']/255.0)
    pts = np.array(pts, dtype=float)
    cols = np.array(cols, dtype=float)
    if pts.shape[0] > max_points:
        idx = np.random.choice(pts.shape[0], max_points, replace=False)
        pts = pts[idx]; cols = cols[idx]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(cols)

    # order images for trajectory
    items = list(images.items())
    if traj_sort == 'id':
        items.sort(key=lambda x: x[0])
    else:
        items.sort(key=lambda x: natural_key(x[1].get('name','')))

    cam_frustums = []
    cam_frames = []
    cam_centers = []
    cam_center_spheres = []

    for img_id, m in items:
        cam_id = m['camera_id']
        if cam_id not in cameras:
            continue
        cam = cameras[cam_id]
        K = build_K_from_camera(cam)
        R = qvec2rotmat(m['qvec'])
        t = np.array(m['tvec'], dtype=float)
        ls, frame, C = create_camera_frustum(R, t, K, cam['width'], cam['height'], scale=scale)
        cam_frustums.append(ls)
        cam_frames.append(frame)
        cam_centers.append(C)
        s = o3d.geometry.TriangleMesh.create_sphere(radius=scale*0.03)
        s.compute_vertex_normals()
        s.paint_uniform_color([1.0, 0.5, 0.0])
        s.translate(C)
        cam_center_spheres.append(s)

    # create full trajectory lineset (static - will be partially shown if desired)
    centers_np = np.array(cam_centers)
    traj_ls = None
    if len(centers_np) >= 2:
        traj_pts = centers_np.tolist()
        traj_lines = [[i, i+1] for i in range(len(traj_pts)-1)]
        traj_ls = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(traj_pts),
                                       lines=o3d.utility.Vector2iVector(traj_lines))
        traj_ls.colors = o3d.utility.Vector3dVector([[1.0, 0.0, 0.0]] * len(traj_lines))

    # moving highlight marker (a bigger sphere that will move along centers)
    highlight_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=scale*0.05)
    highlight_sphere.compute_vertex_normals()
    highlight_sphere.paint_uniform_color([0.0, 0.8, 1.0])
    if len(cam_centers) > 0:
        highlight_sphere.translate(cam_centers[0])

    # build geometries list (only static/common ones added initially)
    geoms = [pcd]
    if traj_ls is not None:
        geoms.append(traj_ls)
    geoms.append(highlight_sphere)
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale))

    # state variables
    N = len(cam_centers)
    state = {
        'idx': 0,
        'playing': start_playing,
        'fps': float(fps),
        'last_step_time': time.time(),
        'prev_idx': None
    }

    # track which indices have been added to the visualizer when using cumulative mode
    added_indices = set()

    # Create visualizer and add geometries (only the common ones and the initial current camera if requested)
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name='COLMAP dynamic trajectory', width=1600, height=900)
    # add common geometries
    for g in geoms:
        vis.add_geometry(g)

    # helper to ensure frustums up to idx are visible (cumulative behavior)
    def ensure_cumulative_until(vis_obj, idx):
        if idx is None or idx < 0:
            return
        # add any indices from 0..idx that haven't been added yet
        for i in range(0, idx + 1):
            if i in added_indices:
                continue
            try:
                vis_obj.add_geometry(cam_frustums[i], reset_bounding_box=False)
                vis_obj.add_geometry(cam_frames[i], reset_bounding_box=False)
                vis_obj.add_geometry(cam_center_spheres[i], reset_bounding_box=False)
                added_indices.add(i)
            except Exception:
                pass

    # helper to set coloring: highlight current index, dim previous
    def recolor_cumulative(vis_obj, current_idx):
        # for indices that have been added, color previous ones dim and current one highlighted
        for i in range(0, N):
            if i not in added_indices:
                continue
            try:
                if i == current_idx:
                    cam_frustums[i].colors = o3d.utility.Vector3dVector([[0.0, 1.0, 0.0]] * len(cam_frustums[i].lines))
                    cam_frames[i].paint_uniform_color([0.0, 1.0, 0.0])
                    cam_center_spheres[i].paint_uniform_color([0.0, 0.8, 1.0])
                else:
                    cam_frustums[i].colors = o3d.utility.Vector3dVector([[0.2, 0.2, 0.2]] * len(cam_frustums[i].lines))
                    cam_frames[i].paint_uniform_color([0.5, 0.5, 0.5])
                    cam_center_spheres[i].paint_uniform_color([1.0, 0.5, 0.0])
                vis_obj.update_geometry(cam_frustums[i])
                vis_obj.update_geometry(cam_frames[i])
                vis_obj.update_geometry(cam_center_spheres[i])
            except Exception:
                pass

    # helper to clear cumulative geometries (used on reset)
    def clear_cumulative(vis_obj):
        # remove all added geometries from the visualizer
        for i in sorted(list(added_indices), reverse=True):
            try:
                vis_obj.remove_geometry(cam_frustums[i], reset_bounding_box=False)
                vis_obj.remove_geometry(cam_frames[i], reset_bounding_box=False)
                vis_obj.remove_geometry(cam_center_spheres[i], reset_bounding_box=False)
            except Exception:
                pass
        added_indices.clear()

    # helper: update highlight + partial trajectory
    def update_visuals(vis_obj):
        idx = state['idx']
        # move highlight sphere to current center
        if N > 0:
            try:
                verts = np.asarray(highlight_sphere.vertices)
                centroid = verts.mean(axis=0)
                target = cam_centers[idx]
                delta = target - centroid
                highlight_sphere.translate(delta)
                vis_obj.update_geometry(highlight_sphere)
            except Exception:
                pass

        # manage cumulative frustums
        if show_only_current_frustum:
            # cumulative: ensure all frustums up to idx are present
            ensure_cumulative_until(vis_obj, idx)
            recolor_cumulative(vis_obj, idx)
        else:
            # non-cumulative: on first call add all frustums/frames/spheres if not added
            if state['prev_idx'] is None:
                for i in range(N):
                    try:
                        vis_obj.add_geometry(cam_frustums[i], reset_bounding_box=False)
                        vis_obj.add_geometry(cam_frames[i], reset_bounding_box=False)
                        vis_obj.add_geometry(cam_center_spheres[i], reset_bounding_box=False)
                    except Exception:
                        pass
                # recolor all (so current is highlighted)
                for i in range(N):
                    try:
                        if i == idx:
                            cam_frustums[i].colors = o3d.utility.Vector3dVector([[0.0, 1.0, 0.0]] * len(cam_frustums[i].lines))
                            cam_frames[i].paint_uniform_color([0.0, 1.0, 0.0])
                            cam_center_spheres[i].paint_uniform_color([0.0, 0.8, 1.0])
                        else:
                            cam_frustums[i].colors = o3d.utility.Vector3dVector([[0.2, 0.2, 0.2]] * len(cam_frustums[i].lines))
                            cam_frames[i].paint_uniform_color([0.5, 0.5, 0.5])
                            cam_center_spheres[i].paint_uniform_color([1.0, 0.5, 0.0])
                        vis_obj.update_geometry(cam_frustums[i])
                        vis_obj.update_geometry(cam_frames[i])
                        vis_obj.update_geometry(cam_center_spheres[i])
                    except Exception:
                        pass

        # update partial trajectory: keep lines up to idx
        if traj_ls is not None and N >= 2:
            if idx >= 1:
                new_lines = [[i, i+1] for i in range(min(idx, N-1))]
            else:
                new_lines = []
            traj_ls.lines = o3d.utility.Vector2iVector(new_lines)
            traj_ls.colors = o3d.utility.Vector3dVector([[1.0, 0.0, 0.0]] * len(new_lines))
            vis_obj.update_geometry(traj_ls)

        state['prev_idx'] = idx

    # register key callbacks
    def kb_toggle_play(vis_obj):
        state['playing'] = not state['playing']
        print('Playing ->', state['playing'])
        return False
    vis.register_key_callback(ord(' '), kb_toggle_play)
    def kb_next(vis_obj):
        state['idx'] = (state['idx'] + 1) % max(1, N)
        state['last_step_time'] = time.time()
        update_visuals(vis_obj)
        return False
    vis.register_key_callback(ord('n'), kb_next)
    vis.register_key_callback(ord('N'), kb_next)
    def kb_prev(vis_obj):
        state['idx'] = (state['idx'] - 1) % max(1, N)
        state['last_step_time'] = time.time()
        update_visuals(vis_obj)
        return False
    vis.register_key_callback(ord('p'), kb_prev)
    vis.register_key_callback(ord('P'), kb_prev)
    def kb_plus(vis_obj):
        state['fps'] = min(state['fps'] * 1.5, 60.0)
        print('FPS ->', state['fps'])
        return False
    vis.register_key_callback(ord('+'), kb_plus)
    vis.register_key_callback(ord('='), kb_plus)
    def kb_minus(vis_obj):
        state['fps'] = max(state['fps'] / 1.5, 0.1)
        print('FPS ->', state['fps'])
        return False
    vis.register_key_callback(ord('-'), kb_minus)
    def kb_reset(vis_obj):
        # reset index and clear cumulative if in that mode
        state['idx'] = 0
        state['last_step_time'] = time.time()
        if show_only_current_frustum:
            clear_cumulative(vis_obj)
        update_visuals(vis_obj)
        return False
    vis.register_key_callback(ord('r'), kb_reset)
    vis.register_key_callback(ord('R'), kb_reset)
    def kb_quit(vis_obj):
        print('Quitting...')
        vis_obj.close()
        return False
    vis.register_key_callback(ord('q'), kb_quit)
    vis.register_key_callback(ord('Q'), kb_quit)
    # ESC
    vis.register_key_callback(256, kb_quit)

    # initial visuals (if cumulative mode, show index 0 only)
    if show_only_current_frustum:
        ensure_cumulative_until(vis, 0)
        recolor_cumulative(vis, 0)
    else:
        # other case will add all on first update
        pass
    update_visuals(vis)

    try:
        # main loop: poll events and update highlight per fps
        while vis.poll_events():
            now = time.time()
            if state['playing'] and N > 0:
                interval = 1.0 / max(1e-6, state['fps'])
                if now - state['last_step_time'] >= interval:
                    state['idx'] = (state['idx'] + 1) % N
                    state['last_step_time'] = now
                    update_visuals(vis)
            vis.update_renderer()
            time.sleep(0.01)
    except Exception:
        pass
    finally:
        vis.destroy_window()


# -------------------------
# CLI
# -------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_dir', type=str, required=True, help='COLMAP model dir (contains cameras.txt/images.txt/points3D.txt or .bin files)')
    parser.add_argument('--use_colmap_converter', type=bool, default=True, help='If bin files present, try to use colmap model_converter')
    parser.add_argument('--traj_sort', type=str, choices=['name','id'], default='name', help='Order for trajectory')
    parser.add_argument('--scale', type=float, default=0.8, help='Frustum scale')
    parser.add_argument('--max_points', type=int, default=200000, help='Max points to visualize')
    parser.add_argument('--fps', type=float, default=5.0, help='Playback frames-per-second')
    parser.add_argument('--start_playing', action='store_true', help='Start playing immediately')
    parser.add_argument('--show_only_current_frustum', action='store_true', help='If set, only show frustums up to the current camera (cumulative) while playing')
    args = parser.parse_args()

    model_dir = args.model_dir
    txt_dir = model_dir
    cameras_txt = os.path.join(txt_dir, 'cameras.txt')
    images_txt = os.path.join(txt_dir, 'images.txt')
    points_txt = os.path.join(txt_dir, 'points3D.txt')
    cameras_bin = os.path.join(model_dir, 'cameras.bin')
    images_bin = os.path.join(model_dir, 'images.bin')
    points_bin = os.path.join(model_dir, 'points3D.bin')

    if not (os.path.exists(cameras_txt) and os.path.exists(images_txt) and os.path.exists(points_txt)):
        if args.use_colmap_converter and (os.path.exists(cameras_bin) and os.path.exists(images_bin) and os.path.exists(points_bin)):
            out_dir = os.path.join(model_dir, 'txt_export')
            os.makedirs(out_dir, exist_ok=True)
            convert_bin_to_txt(model_dir, out_dir)
            txt_dir = out_dir
        else:
            raise RuntimeError('TXT files not found and conversion not possible. Provide cameras.txt/images.txt/points3D.txt or enable colmap converter.')

    print('Controls: Space=play/pause, n=next, p=prev, +/- change speed, r=reset, q=quit')
    print('If window is not focused, click it then use keys.')

    visualize_dynamic(txt_dir, scale=args.scale, max_points=args.max_points, traj_sort=args.traj_sort, fps=args.fps, start_playing=args.start_playing, show_only_current_frustum=args.show_only_current_frustum)

if __name__ == '__main__':
    main()
