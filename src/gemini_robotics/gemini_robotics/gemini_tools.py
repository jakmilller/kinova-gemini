from google.genai import types

# --- Inspection Tools ---

inspect_scene_tool = types.FunctionDeclaration(
    name="inspect_scene",
    description="Uses an intelligent robotics model to perform an in-depth analysis of the scene. It identifies all items, provides their semantic descriptions, and determines their 3D poses relative to the robot base.",
    parameters=types.Schema(
        type="OBJECT",
        properties={},
    )
)

# --- Low-level Tools ---

move_to_home_tool = types.FunctionDeclaration(
    name="move_to_home",
    description="Moves the robot arm to pre-defined home position. From the home position, the robot's gripper is looking down at objects on the table.",
    parameters=types.Schema(
        type="OBJECT",
        properties={},
    )
)

move_to_user_tool = types.FunctionDeclaration(
    name="move_to_user",
    description="Moves the robot arm to a pre-defined position near the user. From the user position, the robot is looking horizontally at the user.",
    parameters=types.Schema(
        type="OBJECT",
        properties={},
    )
)

grasp_object_tool = types.FunctionDeclaration(
    name="grasp_object",
    description="Closes the gripper until it detects contact with an object.",
    parameters=types.Schema(
        type="OBJECT",
        properties={},
    )
)

open_gripper_tool = types.FunctionDeclaration(
    name="open_gripper",
    description="Opens the gripper fully.",
    parameters=types.Schema(
        type="OBJECT",
        properties={},
    )
)

move_to_position_tool = types.FunctionDeclaration(
    name="move_to_position",
    description="Call when you want to move the gripper to a specific Cartesian position. Arguments are absolute coordinates in meters relative to the robot base.",
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "x": types.Schema(type="NUMBER", description="Target x coordinate in meters"),
            "y": types.Schema(type="NUMBER", description="Target y coordinate in meters"),
            "z": types.Schema(type="NUMBER", description="Target z coordinate in meters"),
        },
        required=["x", "y", "z"]
    )
)

move_to_pose_tool = types.FunctionDeclaration(
    name="move_to_pose",
    description="Call when you want to execute a complex grasp on a specific object. The system will autonomously determine the best 6D grasp pose and execute it.",
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "object_label": types.Schema(type="STRING", description="The label or semantic description of the target object to grasp (e.g., 'red block', 'cup handle')."),
        },
        required=["object_label"]
    )
)

adjust_joints_tool = types.FunctionDeclaration(
    name="adjust_joints",
    description="Call when user requests adjustment to the current joint position (e.g., 'rotate last joint 30 degrees'). Arguments are relative adjustments in degrees to current joint angles.",
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "joint number": types.Schema(type="NUMBER", description="The joint number to adjust (1-7)."),
            "amount": types.Schema(type="NUMBER", description="adjustment in degrees"),
        },
        required=["joint number", "amount"]
    )
)

# --- Helper Tools ---

task_complete_tool = types.FunctionDeclaration(
    name="task_complete",
    description="Call when you have successfully completed the user's requested goal.",
    parameters=types.Schema(
        type="OBJECT",
        properties={},
    )
)

# --- Tool Collections ---

INSPECTION_TOOLS = [inspect_scene_tool]
LOW_LEVEL_TOOLS = [move_to_home_tool, move_to_user_tool, grasp_object_tool, open_gripper_tool, move_to_position_tool, move_to_pose_tool, adjust_joints_tool]
HELPER_TOOLS = [task_complete_tool]

ALL_TOOLS = INSPECTION_TOOLS + LOW_LEVEL_TOOLS + HELPER_TOOLS