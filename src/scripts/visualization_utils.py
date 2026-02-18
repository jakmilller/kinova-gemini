import json
import textwrap
import time
import base64
import dataclasses
from io import BytesIO
import numpy as np
from PIL import Image, ImageColor, ImageDraw, ImageFont
from typing import Tuple
import IPython
from IPython import display

# from https://github.com/google-gemini/cookbook/blob/main/quickstarts/gemini-robotics-er.ipynb

def parse_json(json_output):
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
    img = Image.open(img_path)
    img = img.resize(
        (800, int(800 * img.size[1] / img.size[0])), Image.Resampling.LANCZOS
    )
    return img

def generate_point_html(pil_image, points_json):
  buffered = BytesIO()
  pil_image.save(buffered, format="PNG")
  img_str = base64.b64encode(buffered.getvalue()).decode()
  points_json = parse_json(points_json)

  return f"""



    Point Visualization
    


    
        
        
    

    


"""



additional_colors = [
    colorname for (colorname, colorcode) in ImageColor.colormap.items()
]


def plot_bounding_boxes(img, bounding_boxes):
  """Plots bounding boxes on an image.

  Plots bounding boxes on an image with markers for each a name, using PIL,
  normalized coordinates, and different colors.

  Args:
      img_path: The path to the image file.
      bounding_boxes: A list of bounding boxes containing the name of the object
        and their positions in normalized [y1 x1 y2 x2] format.
  """

  # Load the image
  width, height = img.size
  print(img.size)
  # Create a drawing object
  draw = ImageDraw.Draw(img)

  # Define a list of colors
  colors = [
      "red",
      "green",
      "blue",
      "yellow",
      "orange",
      "pink",
      "purple",
      "brown",
      "gray",
      "beige",
      "turquoise",
      "cyan",
      "magenta",
      "lime",
      "navy",
      "maroon",
      "teal",
      "olive",
      "coral",
      "lavender",
      "violet",
      "gold",
      "silver",
  ] + additional_colors

  # Parsing out the markdown fencing
  bounding_boxes = parse_json(bounding_boxes)

  font = ImageFont.truetype("LiberationSans-Regular.ttf", size=14)

  # Iterate over the bounding boxes
  for i, bounding_box in enumerate(json.loads(bounding_boxes)):
    # Select a color from the list
    color = colors[i % len(colors)]

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
    raw_box = item["box_2d"]
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
  """Overlays a single mask onto a PIL Image using a named color.

  The mask image defines the area to be colored. Non-zero pixels in the
  mask image are considered part of the area to overlay.

  Args:
      img: The base PIL Image object.
      mask: A PIL Image object representing the mask. Should have the same
        height and width as the img. Modes '1' (binary) or 'L' (grayscale) are
        typical, where non-zero pixels indicate the masked area.
      color: A standard color name string (e.g., 'red', 'blue', 'yellow').
      alpha: The alpha transparency level for the overlay (0.0 fully
        transparent, 1.0 fully opaque). Default is 0.7 (70%).

  Returns:
      A new PIL Image object (in RGBA mode) with the mask overlaid.

  Raises:
      ValueError: If color name is invalid, mask dimensions mismatch img
                  dimensions, or alpha is outside the 0.0-1.0 range.
  """
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
  """Plots bounding boxes on an image.

  Plots bounding boxes on an image with markers for each a name, using PIL,
  normalized coordinates, and different colors.

  Args:
      img: The PIL.Image.
      segmentation_masks: A string encoding as JSON a list of segmentation masks
        containing the name of the object, their positions in normalized [y1 x1
        y2 x2] format, and the png encoded segmentation mask.
  """
  # Define a list of colors
  colors = [
      "red",
      "green",
      "blue",
      "yellow",
      "orange",
      "pink",
      "purple",
      "brown",
      "gray",
      "beige",
      "turquoise",
      "cyan",
      "magenta",
      "lime",
      "navy",
      "maroon",
      "teal",
      "olive",
      "coral",
      "lavender",
      "violet",
      "gold",
      "silver",
  ] + additional_colors

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
  return img


def overlay_points_on_frames(original_frames, points_data_per_frame):
  """Overlays points on original frames and returns the modified frames."""
  modified_frames = []

  # Define colors for drawing points (using a consistent color per label for clarity)
  label_colors = {}
  current_color_index = 0
  available_colors = [
      "red",
      "green",
      "blue",
      "yellow",
      "orange",
      "pink",
      "purple",
      "brown",
      "gray",
      "beige",
      "turquoise",
      "cyan",
      "magenta",
      "lime",
      "navy",
      "maroon",
      "teal",
      "olive",
      "coral",
      "lavender",
      "violet",
      "gold",
      "silver",
  ]

  font = ImageFont.load_default()

  # Check if the number of original frames matches the number of processed data entries
  if len(original_frames) != len(points_data_per_frame):
    print(
        f"Error: Number of original frames ({len(original_frames)}) does not "
        "match the number of processed point data entries"
        f" ({len(points_data_per_frame)}). Cannot overlay points accurately."
    )
    return original_frames  # Return original frames if data doesn't match
  else:
    # Iterate through the frames and draw points
    for i, frame_pil in enumerate(original_frames):
      # Ensure frame is in RGB mode for drawing
      img = frame_pil.convert("RGB")
      draw = ImageDraw.Draw(img)
      width, height = img.size

      frame_points = points_data_per_frame[i]

      # Draw points on the frame
      for point_info in frame_points:
        if "point" in point_info and "label" in point_info:
          y_norm, x_norm = point_info["point"]
          label = point_info["label"]

          # Get color for the label
          if label not in label_colors:
            label_colors[label] = available_colors[
                current_color_index % len(available_colors)
            ]
            current_color_index += 1
          color = label_colors[label]

          # Convert normalized coordinates to absolute pixel coordinates
          abs_x = int(x_norm / 1000.0 * width)
          abs_y = int(y_norm / 1000.0 * height)

          # Draw a circle at the point
          point_radius = 5
          draw.ellipse(
              (
                  abs_x - point_radius,
                  abs_y - point_radius,
                  abs_x + point_radius,
                  abs_y + point_radius,
              ),
              fill=color,
              outline=color,
          )

          # Draw the label
          # Adjust label position to avoid going out of bounds
          label_pos_x = abs_x + point_radius + 2
          label_pos_y = (
              abs_y - point_radius - 10
              if abs_y - point_radius - 10 > 0
              else abs_y + point_radius + 2
          )
          draw.text((label_pos_x, label_pos_y), label, fill=color, font=font)

      # Append the modified PIL Image
      modified_frames.append(img)

    print(f"Processed and drew points on {len(modified_frames)} frames.")
    return modified_frames


def display_gif(frames_to_display):
  """Saves and displays a list of PIL Images as a GIF."""
  if frames_to_display:
    try:
      # Save the modified frames as a new GIF
      output_gif_path = "/tmp/annotated_aloha_pen.gif"
      # Duration per frame in milliseconds (adjust as needed, 40ms is 25fps)
      duration_ms = 40
      # Ensure all frames are in RGB mode before saving as GIF
      rgb_frames = [frame.convert("RGB") for frame in frames_to_display]
      if rgb_frames:
        rgb_frames[0].save(
            output_gif_path,
            save_all=True,
            append_images=rgb_frames[1:],
            duration=duration_ms,
            loop=0,
        )

        # Display the GIF in Colab
        display.display(display.Image(output_gif_path))
        print(f"Displayed annotated GIF: {output_gif_path}")
      else:
        print("No frames to create GIF.")

    except Exception as e:
      print(f"Error creating or displaying annotated GIF: {e}")
  else:
    print("No frames to display.")


def extract_frames(gif):
  """Extracts frames from a GIF and returns a list of PIL Image objects."""
  frames = []
  try:
    while True:
      # Convert each frame to RGB to ensure compatibility with drawing
      frame = gif.convert("RGB")
      frames.append(frame)
      gif.seek(gif.tell() + 1)  # Move to the next frame
  except EOFError:
    pass  # End of sequence

  print(f"Extracted {len(frames)} frames from the GIF.")

  return frames


def populate_points_for_all_frames(total_frames, step, analyzed_data):
  """Populates point data for all frames based on analyzed frames."""
  points_data_all_frames = []
  analyzed_data_index = 0
  for i in range(total_frames):
    if i % step == 0 and analyzed_data_index < len(analyzed_data):
      points_data_all_frames.append(analyzed_data[analyzed_data_index])
      analyzed_data_index += 1
    else:
      # For frames that were not analyzed, use the data from the last analyzed
      # frame or append an empty list if no frame has been analyzed yet
      if analyzed_data_index > 0:
        points_data_all_frames.append(analyzed_data[analyzed_data_index - 1])
      else:
        # Should not happen if frames list is not empty
        points_data_all_frames.append([])
  return points_data_all_frames
