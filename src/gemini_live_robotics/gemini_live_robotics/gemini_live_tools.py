from google.genai import types

# --- Inspection Tools ---

inspect_scene_tool = types.FunctionDeclaration(
    name="inspect_scene",
    description=(
        "Uses an intelligent robotics model to perform an in-depth analysis of the scene. "
        "It identifies all items, provides their semantic descriptions, and determines their 3D poses "
        "relative to the robot base. Optionally, also identifies a specific spatial region described "
        "in natural language (e.g. a chess square, a compartment of a box, a spot relative to other objects)."
    ),
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "target_location": types.Schema(
                type="STRING",
                description=(
                    "OPTIONAL natural-language description of a SPECIFIC SPATIAL LOCATION the user wants "
                    "identified separately from the general scene. Use this when the user's request involves "
                    "moving something TO a region that is not itself a discrete object. "
                    "Examples: '3 spaces forward from the queen', 'the first compartment of the box', "
                    "'the spot to the left of the red cup'. Leave empty for a general scene inspection."
                ),
            ),
        },
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
    description=(
        "Executes a complete pick sequence on a target object: pre-shapes the gripper, "
        "moves to a pre-grasp position, approaches the grasp pose, and closes the gripper until contact. "
        "Two mutually exclusive modes — provide exactly one:\n"
        "  • object_label: autonomously determines the best 6D grasp pose using AnyGrasp vision (preferred "
        "for objects with handles, rims, or complex geometry).\n"
        "  • x/y/z: approaches the given TCP contact-point coordinates using the arm's current orientation "
        "(suitable for simple flat objects when you already know the position from inspect_scene).\n"
        "After this tool succeeds the gripper is closed around the object. Use move_to_position to carry "
        "it somewhere, then open_gripper to release."
    ),
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "object_label": types.Schema(
                type="STRING",
                description="Semantic label of the target object (e.g. 'red cup', 'bottle handle'). Use this mode for complex objects — AnyGrasp will find the best 6D grasp pose autonomously.",
            ),
            "x": types.Schema(type="NUMBER", description="TCP contact-point x coordinate in meters (base frame). Use with y and z when you already know the object position from inspect_scene."),
            "y": types.Schema(type="NUMBER", description="TCP contact-point y coordinate in meters (base frame)."),
            "z": types.Schema(type="NUMBER", description="TCP contact-point z coordinate in meters (base frame)."),
        },
    )
)

open_gripper_tool = types.FunctionDeclaration(
    name="open_gripper",
    description="Opens the gripper fully, releasing any held object.",
    parameters=types.Schema(
        type="OBJECT",
        properties={},
    )
)

move_to_position_tool = types.FunctionDeclaration(
    name="move_to_position",
    description=(
        "Moves the gripper TCP to the specified Cartesian position, keeping the current wrist orientation. "
        "Works the same whether the gripper is empty or holding a grasped object — use this to carry an "
        "object to its destination after grasp_object. "
        "Arguments are absolute coordinates in meters relative to the robot base."
    ),
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

stop_robot_tool = types.FunctionDeclaration(
    name="stop_robot",
    description="Immediately stops the robot from moving.",
    parameters=types.Schema(
        type="OBJECT",
        properties={},
    )
)

# --- Tool Collections ---

INSPECTION_TOOLS = [inspect_scene_tool]
LOW_LEVEL_TOOLS = [move_to_home_tool, move_to_user_tool, grasp_object_tool, open_gripper_tool, move_to_position_tool, adjust_joints_tool, stop_robot_tool]

ALL_TOOLS = INSPECTION_TOOLS + LOW_LEVEL_TOOLS
