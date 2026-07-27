"""Perception helpers for the Gemini Live brain node.

Stateless vision utilities: turning a camera frame + a prompt into bounding boxes
(Gemini), refining a box into a mask (SAM2), and lifting a pixel into a 3D base-frame
point (depth + TF). Every function takes its dependencies (Gemini client, SAM2
predictor, camera_info, transforms) as explicit arguments so this module has no
knowledge of the ROS node and can be tested in isolation.
"""

import asyncio
import json
import re

import cv2
import numpy as np
import torch
from PIL import ImageFont
from scipy.spatial.transform import Rotation as R
from google.genai import types


# ---- Geometry ----

def transform_to_matrix(t_msg):
    """geometry_msgs/TransformStamped -> 4x4 homogeneous transform matrix."""
    mat = np.eye(4)
    q = [t_msg.transform.rotation.x, t_msg.transform.rotation.y,
         t_msg.transform.rotation.z, t_msg.transform.rotation.w]
    mat[:3, :3] = R.from_quat(q).as_matrix()
    mat[:3,  3] = [t_msg.transform.translation.x,
                   t_msg.transform.translation.y,
                   t_msg.transform.translation.z]
    return mat


def get_3d_point_from_pixel(x_pixel, y_pixel, depth, camera_info):
    """Converts a 2D pixel + depth (meters) to a 3D point in the camera frame."""
    fx = camera_info.k[0]
    cx = camera_info.k[2]
    fy = camera_info.k[4]
    cy = camera_info.k[5]
    z = depth
    x = (x_pixel - cx) * z / fx
    y = (y_pixel - cy) * z / fy
    return x, y, z


def transform_point(point_x, point_y, point_z, transform_stamped):
    """Transforms a 3D point through a geometry_msgs/TransformStamped."""
    tx = transform_stamped.transform.translation.x
    ty = transform_stamped.transform.translation.y
    tz = transform_stamped.transform.translation.z
    rx = transform_stamped.transform.rotation.x
    ry = transform_stamped.transform.rotation.y
    rz = transform_stamped.transform.rotation.z
    rw = transform_stamped.transform.rotation.w

    v = np.array([point_x, point_y, point_z])
    u = np.array([rx, ry, rz])
    s = rw
    # Quaternion rotation of a vector, then translation.
    v_prime = v + 2.0 * np.cross(u, np.cross(u, v) + s * v)
    v_prime += np.array([tx, ty, tz])
    return float(v_prime[0]), float(v_prime[1]), float(v_prime[2])


# ---- Gemini JSON parsing (tolerant of malformed model output) ----

async def query_gemini_json(client, model_id, prompt, pil_image, logger=None):
    """Run a synchronous Gemini vision call in a worker thread so the event loop keeps moving.
    Falls back to a salvage parser when Gemini emits malformed JSON (it does, sometimes).
    """
    def _call():
        return client.models.generate_content(
            model=model_id,
            contents=[pil_image, prompt],
            config=types.GenerateContentConfig(
                response_mime_type="application/json", temperature=0.0,
            ),
        )
    response = await asyncio.to_thread(_call)
    if logger:
        logger.info(f"Vision response: {response.text}")
    try:
        return json.loads(response.text)
    except json.JSONDecodeError as e:
        if logger:
            logger.warn(f"Gemini returned invalid JSON ({e}); salvaging items via regex.")
        return salvage_json_objects(response.text)


def salvage_json_objects(text):
    """Pull individual {...} objects out of possibly-malformed JSON.
    Tries strict parse per chunk first; if that fails, extracts box + last-quoted-string as label.
    """
    items = []
    for match in re.finditer(r'\{[^{}]*\}', text):
        chunk = match.group(0)
        try:
            items.append(json.loads(chunk))
            continue
        except json.JSONDecodeError:
            pass
        # Field-level extraction as a last resort
        box_match = re.search(r'"box(?:_2d)?"\s*:\s*\[([^\]]+)\]', chunk)
        if not box_match:
            continue
        try:
            box = [int(float(x.strip())) for x in box_match.group(1).split(',')]
        except ValueError:
            continue
        if len(box) != 4:
            continue
        # Last quoted string in the chunk is reliably the semantic name,
        # even when the model emits malformed pairs like `"label": "item name": "cup"`.
        labels = re.findall(r'"([^"]+)"', chunk)
        label = labels[-1] if labels else 'object'
        items.append({'box_2d': box, 'label': label})
    return items


def normalize_box_label(item):
    """Return (box, label) from one Gemini-shaped dict, tolerating common key variants.
    Returns (None, None) if no usable box is present.
    """
    if not isinstance(item, dict):
        return None, None
    box = item.get('box_2d') or item.get('box')
    label = (item.get('label') or item.get('item name') or item.get('part name')
             or item.get('item_name') or item.get('part_name') or 'object')
    if isinstance(box, list) and len(box) == 4:
        return box, label
    return None, None


def iter_box_label_pairs(value):
    """Yield (box_2d, label) pairs out of any reasonable Gemini response shape:
    bare list, single dict, dict-wrapped list, list of dicts, etc.
    """
    if isinstance(value, list):
        for item in value:
            yield from iter_box_label_pairs(item)
    elif isinstance(value, dict):
        box, label = normalize_box_label(value)
        if box is not None:
            yield box, label
        else:
            for v in value.values():
                if isinstance(v, (list, dict)):
                    yield from iter_box_label_pairs(v)


# ---- SAM2 segmentation ----

async def run_sam2(predictor, rgb_image, box):
    """Box-prompt SAM2 to refine a Gemini bounding box into a binary mask.
    rgb_image: HxWx3 RGB; box: (xmin, ymin, xmax, ymax). Returns HxW bool mask, or None if predictor is None.
    """
    if not predictor:
        return None
    h, w = rgb_image.shape[:2]
    xmin, ymin, xmax, ymax = box

    def _predict():
        with torch.inference_mode(), torch.autocast(device_type="cuda", dtype=torch.bfloat16):
            predictor.set_image(rgb_image)
            masks, _, _ = predictor.predict(
                box=np.array([xmin, ymin, xmax, ymax]),
                multimask_output=False,
            )
        return masks

    masks = await asyncio.to_thread(_predict)
    mask = masks[0].cpu().numpy() > 0 if torch.is_tensor(masks) else masks[0] > 0

    # Clip the mask to inside its own bounding box (SAM2 sometimes spills slightly)
    box_mask = np.zeros((h, w), dtype=bool)
    box_mask[max(0, ymin):min(h, ymax), max(0, xmin):min(w, xmax)] = True
    mask = mask & box_mask

    torch.set_default_dtype(torch.float32)
    torch.cuda.empty_cache()
    return mask


def mask_centroid(mask, fallback_box):
    """Pixel centroid of a binary mask, falling back to the box center if the mask is empty."""
    M = cv2.moments(mask.astype(np.uint8))
    if M["m00"] != 0:
        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    xmin, ymin, xmax, ymax = fallback_box
    return int((xmin + xmax) / 2), int((ymin + ymax) / 2)


# ---- Annotation + measurement ----

def load_annotation_font(size=18):
    for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                 "DejaVuSans-Bold.ttf"):
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    return ImageFont.load_default()


def estimate_object_width(mask, points_3d, transform):
    """Estimates a graspable width (meters) for one object: the smaller of its two horizontal
    footprint extents, after PCA-aligning the footprint (so a rotated object still gets a tight
    estimate rather than an axis-aligned bounding box's inflated diagonal extent). Returns None
    if there aren't enough valid 3D points in the mask to form a reliable estimate.
    """
    obj_points_cam = points_3d[mask].reshape(-1, 3)
    obj_points_cam = obj_points_cam[(obj_points_cam[:, 2] > 0.1) & (obj_points_cam[:, 2] < 1.0)]
    if obj_points_cam.shape[0] < 10:
        return None

    T_base_cam = transform_to_matrix(transform)
    ones = np.ones((obj_points_cam.shape[0], 1))
    obj_points_base = (T_base_cam @ np.hstack([obj_points_cam, ones]).T).T[:, :3]

    # PCA on the XY footprint -> principal yaw angle, so extents are measured along the
    # object's own axes rather than the world's (a diagonally-oriented box would otherwise
    # look wider than it really is).
    xy = obj_points_base[:, :2]
    xy_centered = xy - xy.mean(axis=0)
    cov = (xy_centered.T @ xy_centered) / max(len(xy_centered) - 1, 1)
    _, vecs = np.linalg.eigh(cov)
    primary = vecs[:, -1]
    yaw = float(np.arctan2(primary[1], primary[0]))

    c_yaw, s_yaw = np.cos(-yaw), np.sin(-yaw)
    rot2d = np.array([[c_yaw, -s_yaw], [s_yaw, c_yaw]])
    pts_obb = (rot2d @ xy.T).T
    obb_min = pts_obb.min(axis=0)
    obb_max = pts_obb.max(axis=0)
    extents = obb_max - obb_min
    return float(min(extents))
