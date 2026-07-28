from google.genai import types

# v2 tool set — a small, orthogonal collection. Each tool does exactly one thing:
# perception, an absolute move, a relative move, an autonomous grasp, or a gripper/joint action.
# There is no multi-mode grasp and no coordinate-grasp tool: the model composes a manual pick
# from move_to_position + close_gripper, and uses grasp_object only for the AnyGrasp path.

# --- Perception ---

inspect_scene_tool = types.FunctionDeclaration(
    name="inspect_scene",
    description=(
        "Uses an intelligent robotics model to perform an in-depth analysis of the scene. "
        "It identifies all items, provides their semantic descriptions, and determines their 3D poses "
        "relative to the robot base. Optionally, also identifies a specific spatial region described "
        "in natural language (e.g. a chess square, a compartment of a box, a spot relative to other objects). "
        "Does NOT move the robot."
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

# --- Motion ---

move_to_position_tool = types.FunctionDeclaration(
    name="move_to_position",
    description=(
        "Moves the gripper to the specified absolute Cartesian position, keeping the current wrist "
        "orientation. Works the same whether the gripper is empty or holding a grasped object — use it "
        "to carry an object to its destination. Coordinates are in meters relative to the robot base "
        "(+X forward, +Y left, +Z up). Ground the coordinates in object positions from inspect_scene."
    ),
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "x": types.Schema(type="NUMBER", description="Target x coordinate in meters (base frame)."),
            "y": types.Schema(type="NUMBER", description="Target y coordinate in meters (base frame)."),
            "z": types.Schema(type="NUMBER", description="Target z coordinate in meters (base frame)."),
        },
        required=["x", "y", "z"],
    )
)

move_relative_tool = types.FunctionDeclaration(
    name="move_relative",
    description=(
        "Moves the gripper by a relative offset from its current position, keeping the current wrist "
        "orientation. Offsets are in meters along the robot base axes: +X forward, +Y left, +Z up. "
        "Use this for incremental adjustments where you do not have (or do not need) absolute "
        "coordinates — e.g. 'move up 2 cm' is dz=0.02, 'back off 5 cm' is dx=-0.05. Any unspecified "
        "offset defaults to 0."
    ),
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "dx": types.Schema(type="NUMBER", description="Relative offset along base +X (forward) in meters. Default 0."),
            "dy": types.Schema(type="NUMBER", description="Relative offset along base +Y (left) in meters. Default 0."),
            "dz": types.Schema(type="NUMBER", description="Relative offset along base +Z (up) in meters. Default 0."),
        },
    )
)

adjust_joints_tool = types.FunctionDeclaration(
    name="adjust_joints",
    description=(
        "Adjusts a single joint by a relative amount in degrees. Most useful for joint 7 (the wrist roll) "
        "to spin the gripper or a held object, or to line the fingers up across a long/narrow object "
        "before a manual grasp."
    ),
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "joint": types.Schema(type="NUMBER", description="The joint number to adjust (1-7)."),
            "amount": types.Schema(type="NUMBER", description="Relative adjustment in degrees."),
        },
        required=["joint", "amount"],
    )
)

move_to_home_tool = types.FunctionDeclaration(
    name="move_to_home",
    description="Moves the robot arm to the pre-defined home position, where the gripper looks down at objects on the table.",
    parameters=types.Schema(type="OBJECT", properties={}),
)

move_to_user_tool = types.FunctionDeclaration(
    name="move_to_user",
    description="Moves the robot arm to a pre-defined position near the user, where the gripper faces the user horizontally.",
    parameters=types.Schema(type="OBJECT", properties={}),
)

stop_robot_tool = types.FunctionDeclaration(
    name="stop_robot",
    description="Immediately stops the robot from moving. Call this if the user changes their mind mid-task.",
    parameters=types.Schema(type="OBJECT", properties={}),
)

# --- Grasp ---

grasp_simple_object_tool = types.FunctionDeclaration(
    name="grasp_simple_object",
    description=(
        "Picks up an object at a known position with a single call. It pre-shapes the gripper to the "
        "object's width, backs off to a pre-grasp pose a few centimeters in front of the object, moves "
        "straight in along the gripper's axis, and closes. Use this for ordinary top-down picks. You "
        "MUST call inspect_scene first to get the object's x/y/z, width and depth. "
        "After it succeeds the gripper is closed around the object; use move_to_position to carry it "
        "and adjust_gripper(open) to release. For objects with handles, rims, or complex shapes that "
        "need a side/angled grasp, use grasp_complex_object instead."
    ),
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "x": types.Schema(type="NUMBER", description="Object x coordinate in meters (base frame), from inspect_scene."),
            "y": types.Schema(type="NUMBER", description="Object y coordinate in meters (base frame), from inspect_scene."),
            "z": types.Schema(type="NUMBER", description="Object z coordinate in meters (base frame), from inspect_scene."),
            "width": types.Schema(
                type="NUMBER",
                description="OPTIONAL. The object's estimated width in meters (from inspect_scene's Width, "
                            "converted from cm). The gripper pre-shapes to this before approaching, reducing "
                            "the chance of bumping neighbors. Omit only if no width estimate is available.",
            ),
            "depth": types.Schema(
                type="NUMBER",
                description="OPTIONAL. The object's thickness front-to-back in meters (from inspect_scene's "
                            "Depth, converted from cm). Sets how far the fingers travel past the object's "
                            "surface before closing, so they end up around the middle of it rather than "
                            "pinching the near edge. Omit only if no depth estimate is available.",
            ),
        },
        required=["x", "y", "z"],
    )
)

grasp_complex_object_tool = types.FunctionDeclaration(
    name="grasp_complex_object",
    description=(
        "Autonomously picks up a named object using AnyGrasp vision: it finds the best 6D grasp pose and "
        "approach angle on its own, moves directly to that pose, and closes the gripper. This is the mode "
        "that can grasp from the side or at an angle — use it for objects with handles, rims, or complex "
        "geometry. For a simple top-down pick, use grasp_simple_object instead. After this succeeds the "
        "gripper is closed around the object; use move_to_position to carry it and adjust_gripper(open) to release."
    ),
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "object_label": types.Schema(
                type="STRING",
                description="Semantic label of the target object (e.g. 'coffee mug', 'jug handle').",
            ),
        },
        required=["object_label"],
    )
)

# --- Gripper ---

adjust_gripper_tool = types.FunctionDeclaration(
    name="adjust_gripper",
    description=(
        "Controls the gripper. Pass action='open' to open fully and release any held object; "
        "action='close' to close the gripper); or action='custom' with a width in "
        "meters to set a specific opening (e.g. to pre-shape before sliding between two items). "
        "For an ordinary pick you do not need this — grasp_simple_object and grasp_complex_object handle "
        "their own gripper shaping and closing."
    ),
    parameters=types.Schema(
        type="OBJECT",
        properties={
            "action": types.Schema(
                type="STRING",
                enum=["open", "close", "custom"],
                description="'open' = fully open, 'close' = close until contact, 'custom' = open to a specific width.",
            ),
            "width": types.Schema(
                type="NUMBER",
                description="REQUIRED only when action='custom': the desired opening width in meters (e.g. 0.05 for 5 cm).",
            ),
        },
        required=["action"],
    )
)

# --- Tool Collection ---

ALL_TOOLS = [
    inspect_scene_tool,
    move_to_position_tool,
    move_relative_tool,
    grasp_simple_object_tool,
    grasp_complex_object_tool,
    adjust_gripper_tool,
    adjust_joints_tool,
    move_to_home_tool,
    move_to_user_tool,
    stop_robot_tool,
]
