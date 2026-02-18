## this is directly based on the Gemini Robotics ER example notebook, made to run locally
from google import genai
from google.genai import types
import os
from dotenv import load_dotenv
import visualization_utils as vis_utils
import textwrap
import time
import IPython
import json
from robot_controller_ros2 import KinovaRobotControllerROS2 as Controller

workspace_path = os.path.expanduser('~/kinova-gemini')
load_dotenv(os.path.join(workspace_path, '.env'))

# Initialize Client for Gemini Robotics ER
client = genai.Client(api_key=os.getenv('gemini_api_key'))

MODEL_ID = "gemini-robotics-er-1.5-preview"

def call_gemini_robotics_er(img, prompt, config=None):
    default_config = types.GenerateContentConfig(
        temperature=0.5,
        thinking_config=types.ThinkingConfig(thinking_budget=0)
    )

    if config is None:
        config = default_config

    image_response = client.models.generate_content(
          model=MODEL_ID,
          contents=[img, prompt],
          config=config,
    )

    print(image_response.text)
    return vis_utils.parse_json(image_response.text)

img = vis_utils.get_image_resized("/home/mcrr-lab/kinova-gemini/src/pics/plate_Color.png")

with open(os.path.join(workspace_path, 'prompt.txt'), 'r') as f:
    prompt = f.read()

start_time = time.time()
json_output = call_gemini_robotics_er(img, prompt)

print(f"\nTotal processing time: {(time.time() - start_time):.4f} seconds")
try:
  segmentation_masks = vis_utils.parse_segmentation_masks(
      json_output, img_height=img.size[1], img_width=img.size[0]
  )
  print(f"Successfully parsed {len(segmentation_masks)} segmentation masks.")

  annotated_img = vis_utils.plot_segmentation_masks(
      img.convert("RGBA"), segmentation_masks
  )
  annotated_img.show()

#   vis_utils.display.display(annotated_img)


except json.JSONDecodeError as e:
  print(f"Error decoding JSON response: {e}")
except Exception as e:
  print(f"An error occurred during mask processing or plotting: {e}")
     