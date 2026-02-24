import json
import base64
import numpy as np
from PIL import Image, ImageColor, ImageDraw, ImageFont
from io import BytesIO
import dataclasses
from typing import Tuple

def parse_json(json_output):
    """Parses JSON output from Gemini, handling markdown fencing."""
    # Parsing out the markdown fencing
    lines = json_output.splitlines()
    for i, line in enumerate(lines):
        if line == "```json":
            # Remove everything before "```json"
            json_output = "\n".join(lines[i + 1 :])
            # Remove everything after the closing "```"
            json_output = json_output.split("```")[0]
            break  # Exit the loop once "```json" is found
    return json_output

def get_image_resized(img_path):
    """Resizes an image to a max width of 800px while maintaining aspect ratio."""
    img = Image.open(img_path)
    img = img.resize(
        (800, int(800 * img.size[1] / img.size[0])), Image.Resampling.LANCZOS
    )
    return img

def plot_bounding_boxes(img, bounding_boxes_json):
    """Plots bounding boxes on an image and displays it.
    
    Args:
        img: The PIL Image object.
        bounding_boxes_json: JSON string containing list of bounding boxes 
                             with 'box_2d' [y1, x1, y2, x2] and optional 'label'.
    """
    # Load the image
    width, height = img.size
    
    # Create a drawing object
    draw = ImageDraw.Draw(img)

    # Define a list of colors
    additional_colors = [
        colorname for (colorname, colorcode) in ImageColor.colormap.items()
    ]
    colors = [
        "red", "green", "blue", "yellow", "orange", "pink", "purple", "brown",
        "gray", "beige", "turquoise", "cyan", "magenta", "lime", "navy", 
        "maroon", "teal", "olive", "coral", "lavender", "violet", "gold", "silver"
    ] + additional_colors

    # Parsing out the markdown fencing
    bounding_boxes_str = parse_json(bounding_boxes_json)
    
    try:
        bounding_boxes = json.loads(bounding_boxes_str)
    except json.JSONDecodeError:
        print(f"Error decoding JSON: {bounding_boxes_str}")
        return

    # Try to load a font, otherwise use default
    try:
        font = ImageFont.truetype("LiberationSans-Regular.ttf", size=14)
    except IOError:
        font = ImageFont.load_default()

    # Iterate over the bounding boxes
    for i, bounding_box in enumerate(bounding_boxes):
        # Select a color from the list
        color = colors[i % len(colors)]

        # Handle different response formats (box_2d vs point)
        if "box_2d" in bounding_box:
            # Convert normalized coordinates to absolute coordinates
            abs_y1 = int(bounding_box["box_2d"][0] / 1000 * height)
            abs_x1 = int(bounding_box["box_2d"][1] / 1000 * width)
            abs_y2 = int(bounding_box["box_2d"][2] / 1000 * height)
            abs_x2 = int(bounding_box["box_2d"][3] / 1000 * width)

            if abs_x1 > abs_x2:
                abs_x1, abs_x2 = abs_x2, abs_x1

            if abs_y1 > abs_y2:
                abs_y1, abs_y2 = abs_y2, abs_y1

            # Draw the bounding box
            draw.rectangle(((abs_x1, abs_y1), (abs_x2, abs_y2)), outline=color, width=4)
            
            # Draw center point
            center_x = (abs_x1 + abs_x2) // 2
            center_y = (abs_y1 + abs_y2) // 2
            draw.ellipse((center_x - 5, center_y - 5, center_x + 5, center_y + 5), fill=color)

        elif "point" in bounding_box:
             # Convert normalized coordinates to absolute coordinates
            abs_y = int(bounding_box["point"][0] / 1000 * height)
            abs_x = int(bounding_box["point"][1] / 1000 * width)
            
            # Draw point
            draw.ellipse((abs_x - 5, abs_y - 5, abs_x + 5, abs_y + 5), fill=color, outline=color)
            
            abs_x1 = abs_x
            abs_y1 = abs_y # For text placement

        # Draw the text
        if "label" in bounding_box:
            draw.text(
                (abs_x1 + 8, abs_y1 + 6), bounding_box["label"], fill=color, font=font
            )

    # Display the image
    img.show()

@dataclasses.dataclass(frozen=True)
class SegmentationMask:
  # bounding box pixel coordinates (not normalized)
  y0: int  # in [0..height - 1]
  x0: int  # in [0..width - 1]
  y1: int  # in [0..height - 1]
  x1: int  # in [0..width - 1]
  mask: np.array  # [img_height, img_width] with values 0..255
  label: str


def parse_segmentation_masks(
    predicted_str: str, *, img_height: int, img_width: int
) -> list[SegmentationMask]:
  items = json.loads(parse_json(predicted_str))
  masks = []
  for item in items:
    # raw_box = item["box_2d"] # Unused
    abs_y0 = int(item["box_2d"][0] / 1000 * img_height)
    abs_x0 = int(item["box_2d"][1] / 1000 * img_width)
    abs_y1 = int(item["box_2d"][2] / 1000 * img_height)
    abs_x1 = int(item["box_2d"][3] / 1000 * img_width)
    if abs_y0 >= abs_y1 or abs_x0 >= abs_x1:
      print("Invalid bounding box", item["box_2d"])
      continue
    label = item["label"]
    png_str = item["mask"]
    if not png_str.startswith("data:image/png;base64,"):
      print("Invalid mask")
      continue
    png_str = png_str.removeprefix("data:image/png;base64,")
    png_str = base64.b64decode(png_str)
    mask = Image.open(BytesIO(png_str))
    bbox_height = abs_y1 - abs_y0
    bbox_width = abs_x1 - abs_x0
    if bbox_height < 1 or bbox_width < 1:
      print("Invalid bounding box")
      continue
    mask = mask.resize(
        (bbox_width, bbox_height), resample=Image.Resampling.BILINEAR
    )
    np_mask = np.zeros((img_height, img_width), dtype=np.uint8)
    np_mask[abs_y0:abs_y1, abs_x0:abs_x1] = mask
    masks.append(
        SegmentationMask(abs_y0, abs_x0, abs_y1, abs_x1, np_mask, label)
    )
  return masks


def overlay_mask_on_img(
    img: Image, mask: np.ndarray, color: str, alpha: float = 0.7
) -> Image.Image:
  """Overlays a single mask onto a PIL Image using a named color."""
  if not (0.0 <= alpha <= 1.0):
    raise ValueError("Alpha must be between 0.0 and 1.0")

  # Convert the color name string to an RGB tuple
  try:
    color_rgb: Tuple[int, int, int] = ImageColor.getrgb(color)
  except ValueError as e:
    # Re-raise with a more informative message if color name is invalid
    raise ValueError(
        f"Invalid color name '{color}'. Supported names are typically HTML/CSS "
        f"color names. Error: {e}"
    )

  # Prepare the base image for alpha compositing
  img_rgba = img.convert("RGBA")
  width, height = img_rgba.size

  # Create the colored overlay layer
  # Calculate the RGBA tuple for the overlay color
  alpha_int = int(alpha * 255)
  overlay_color_rgba = color_rgb + (alpha_int,)

  # Create an RGBA layer (all zeros = transparent black)
  colored_mask_layer_np = np.zeros((height, width, 4), dtype=np.uint8)

  # Mask has values between 0 and 255, threshold at 127 to get binary mask.
  mask_np_logical = mask > 127

  # Apply the overlay color RGBA tuple where the mask is True
  colored_mask_layer_np[mask_np_logical] = overlay_color_rgba

  # Convert the NumPy layer back to a PIL Image
  colored_mask_layer_pil = Image.fromarray(colored_mask_layer_np, "RGBA")

  # Composite the colored mask layer onto the base image
  result_img = Image.alpha_composite(img_rgba, colored_mask_layer_pil)

  return result_img


def plot_segmentation_masks(
    img: Image, segmentation_masks: list[SegmentationMask]
):
  """Plots segmentation masks on an image."""
  # Define a list of colors
  additional_colors = [
        colorname for (colorname, colorcode) in ImageColor.colormap.items()
  ]
  colors = [
      "red", "green", "blue", "yellow", "orange", "pink", "purple", "brown",
      "gray", "beige", "turquoise", "cyan", "magenta", "lime", "navy", 
      "maroon", "teal", "olive", "coral", "lavender", "violet", "gold", "silver"
  ] + additional_colors

  try:
    font = ImageFont.truetype("LiberationSans-Regular.ttf", size=14)
  except IOError:
    font = ImageFont.load_default()

  # Do this in 3 passes to make sure the boxes and text are always visible.

  # Overlay the mask
  for i, mask in enumerate(segmentation_masks):
    color = colors[i % len(colors)]
    img = overlay_mask_on_img(img, mask.mask, color)

  # Create a drawing object
  draw = ImageDraw.Draw(img)

  # Draw the bounding boxes
  for i, mask in enumerate(segmentation_masks):
    color = colors[i % len(colors)]
    draw.rectangle(
        ((mask.x0, mask.y0), (mask.x1, mask.y1)), outline=color, width=4
    )

  # Draw the text labels
  for i, mask in enumerate(segmentation_masks):
    color = colors[i % len(colors)]
    if mask.label != "":
      draw.text((mask.x0 + 8, mask.y0 - 20), mask.label, fill=color, font=font)
  
  img.show()
  return img
