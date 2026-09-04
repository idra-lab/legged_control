#!/usr/bin/env python3
"""Aliengo Gazebo bring-up, with a boolean flag choosing which controller drives the robot.

SINGLE ENTRY POINT (Task 14, revised design): earlier iterations of this task created a second,
opti-pessi-only launch file that duplicated ~95% of what a Gazebo + legged_controller bring-up
already needs (xacro -> robot_state_publisher -> gz_sim -> spawn -> controller spawners -> RViz).
That file is gone; this is the one launch file for Aliengo-in-Gazebo, and the `optipessi` argument
picks the controller:

    ros2 launch legged_controllers aliengo_gazebo.launch.py                   # legged/LeggedController (default, RTF 1.0)
    ros2 launch legged_controllers aliengo_gazebo.launch.py optipessi:=true   # legged/OptiPessiController (RTF 0.2)

WHY THIS ISN'T load_controller_launch.xml + empty_world_launch.xml, EXTENDED IN PLACE: those two
existing XML launch files are two independently-invoked commands (see README.md) with no argument
coupling between them today. Satisfying "one flag drives both the physics rate and the controller
choice, decided in one place" means the process that resolves `optipessi` has to own both the world
selection AND the controller-manager spawner calls at once. XML launch has no ternary/expression
substitution to pick between two world file constants from one boolean without writing the same
two-branch condition pair Python needs anyway -- so folding this into either XML file would still
mean writing the equivalent of this file's logic, just in XML, while now ALSO being responsible for
threading the flag across an <include> boundary between two files. Doing it in one Python file was
the smaller change. legged_unitree_description/launch/empty_world_launch.xml and
legged_controllers/launch/load_controller_launch.xml are both left exactly as they were --
a1/go1/legacy aliengo users are unaffected.

CONTROLLER ACTIVATION: ros2_control refuses to activate two controllers that claim the same command
interfaces, so exactly one of {legged_controller, opti_pessi_controller} may be `active`. Both are
still loaded here -- one `active`, one `--inactive`/configured (registered together in
config/controllers.yaml) -- so `ros2 control switch_controllers --deactivate <one> --activate
<other>` can swap stacks at runtime for A/B comparison without restarting Gazebo.

KNOWN LIMITATION, READ BEFORE JUDGING WHETHER THE OPTI-PESSI STANCE "STANDS STABLY":
opti_pessi_wbc_bridge has no static four-foot stance in its model -- the LIP/contact model
underlying the Opti-Pessi OCP is always one diagonal pair standing, one swinging (see
LegIndexing.h's contact-order comment and OptiPessiController's seeded standing plan). So even
before the background solver (Task 13) has produced a real plan, the seeded standing plan already
makes two feet hop slightly off-position instead of a dead-still four-foot stance. That is expected
behavior given the current model, not a bring-up bug -- it is called out in the Task 14 spec as a
deliberate gap (a contact-force blend across phase boundaries, added later only if the bring-up
shows force chatter at phase boundaries).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Prefix of the pinocchio build the whole stack is COMPILED against (pinocchio_DIR is pointed here
# for every package in this workspace).
_PINOCCHIO_PREFIX = "/opt/openrobots"

# The two pinocchio libraries that end up duplicated; see _pinocchio_abi_preload() below.
_PINOCCHIO_CONFLICTING_LIBS = ("libpinocchio_default.so", "libpinocchio_parsers.so")


def _pinocchio_abi_preload():
    """
    Builds the LD_PRELOAD value that pins pinocchio to a single ABI at runtime.

    This workspace has two pinocchio installs: ROS jazzy's 4.0.0 and /opt/openrobots' 3.9.0.
    `pinocchioConfig.cmake` resolves its legacy library list with `find_library(... HINTS ...)` and
    no NO_DEFAULT_PATH, so when those hints miss it falls through to CMAKE_PREFIX_PATH and silently
    returns jazzy's copy. Because legged_interface, legged_wbc and legged_estimation all
    `ament_export_dependencies(... pinocchio)`, that faulty resolution reaches
    liblegged_controllers.so, which therefore carries DT_NEEDED entries for BOTH
    libpinocchio_default.so.4.0.0 and libpinocchio_default.so.3.9.0. Two ABI-incompatible copies of
    the same C++ library in one process segfault inside pinocchio's URDF parser as soon as a
    controller builds a model.

    LD_LIBRARY_PATH cannot fix this: the two copies have DIFFERENT SONAMEs, so the loader maps both
    regardless of search order -- LD_LIBRARY_PATH only decides where each SONAME is found, not
    whether it is loaded. LD_PRELOAD does fix it, because a preloaded object is placed at the front
    of the global symbol lookup scope, so every duplicated symbol resolves to the preloaded copy.

    Preloading the openrobots build is the semantically correct choice, not just the convenient one:
    every package here is COMPILED against openrobots' headers, so openrobots is the ABI the object
    code actually expects. The jazzy copy is on the link line by accident.

    This is a launch-time containment, not a cure. The real fix is to stop the wrong copy reaching
    the link line at all -- see the diagnosis notes for this workspace.
    """
    libs = [
        os.path.join(_PINOCCHIO_PREFIX, "lib", name)
        for name in _PINOCCHIO_CONFLICTING_LIBS
        if os.path.exists(os.path.join(_PINOCCHIO_PREFIX, "lib", name))
    ]
    if not libs:
        return None

    # Never clobber a preload the caller already set.
    existing = os.environ.get("LD_PRELOAD", "").strip()
    if existing:
        libs.append(existing)
    return ":".join(libs)


def generate_launch_description():
    # ------------------------------------------------------------------------------------
    # Arguments
    # ------------------------------------------------------------------------------------
    optipessi_arg = DeclareLaunchArgument(
        "optipessi",
        default_value="false",
        description=(
            "false (default): legged/LeggedController (OCS2 MPC), RTF 1.0 -- the existing, proven "
            "stack. true: legged/OptiPessiController (Task 12/13, via opti_pessi_wbc_bridge), RTF "
            "0.2 -- the opti-pessi solver averages ~0.97s wall-clock per solve against a "
            "~0.2-0.25s contact phase, so it needs slowed-down sim time to keep up."
        ),
    )
    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="aliengo",
        description=(
            "Robot type passed to robot.xacro. NOTE: when optipessi:=true, the Opti-Pessi config "
            "(opti_pessi_interface/config, legged_controllers/config/aliengo, and "
            "opti_pessi_wbc_bridge's aliengoLegGeometry()) is Aliengo-specific -- changing this "
            "argument alone does not retarget the opti-pessi path to another robot."
        ),
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="S1",
        description="(optipessi:=true only) Opti-Pessi scenario id: opti_pessi_interface/config/scenario_<id>.info",
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz2")
    recompile_libraries_arg = DeclareLaunchArgument(
        "optiPessiRecompileLibraries",
        default_value="false",
        description="(optipessi:=true only) Force CppAD library regeneration for the Opti-Pessi OCP.",
    )

    optipessi = LaunchConfiguration("optipessi")
    robot_type = LaunchConfiguration("robot_type")
    scenario = LaunchConfiguration("scenario")
    use_sim_time = LaunchConfiguration("use_sim_time")
    recompile_libraries = LaunchConfiguration("optiPessiRecompileLibraries")

    # ------------------------------------------------------------------------------------
    # Config file paths
    # ------------------------------------------------------------------------------------
    xacro_file = PathJoinSubstitution([FindPackageShare("legged_unitree_description"), "urdf", "robot.xacro"])
    # config/controllers.yaml now registers BOTH legged_controller and opti_pessi_controller (see
    # that file's Task 14 comment) -- one merged file, matching gazebo.xacro's own default for
    # `controllers_yaml`, passed explicitly here only for clarity.
    controllers_yaml = PathJoinSubstitution([FindPackageShare("legged_controllers"), "config", "controllers.yaml"])
    # Same fixed tmp path load_controller_launch.xml/generate_urdf.sh already use for
    # legged_controller -- generate_urdf.sh writes to /tmp/legged_control/<robot_type>.urdf.
    urdf_file = [TextSubstitution(text="/tmp/legged_control/"), robot_type, TextSubstitution(text=".urdf")]

    task_file = PathJoinSubstitution([FindPackageShare("legged_controllers"), "config", "aliengo", "task.info"])
    reference_file = PathJoinSubstitution([FindPackageShare("legged_controllers"), "config", "aliengo", "reference.info"])
    gait_command_file = PathJoinSubstitution([FindPackageShare("legged_controllers"), "config", "aliengo", "gait.info"])
    opti_pessi_task_file = PathJoinSubstitution([FindPackageShare("opti_pessi_interface"), "config", "task.info"])
    opti_pessi_scenario_file = [
        FindPackageShare("opti_pessi_interface"),
        TextSubstitution(text="/config/scenario_"),
        scenario,
        TextSubstitution(text=".info"),
    ]
    # RTF 0.2 vs RTF 1.0, coupled to the same `optipessi` flag that picks the controller -- see
    # legged_gazebo/worlds/opti_pessi_aliengo_world.world's header comment for the physics numbers.
    opti_pessi_world_file = PathJoinSubstitution([FindPackageShare("legged_gazebo"), "worlds", "opti_pessi_aliengo_world.world"])
    default_world_file = PathJoinSubstitution([FindPackageShare("legged_gazebo"), "worlds", "empty_world.world"])
    rviz_config = PathJoinSubstitution([FindPackageShare("legged_controllers"), "config", "config.rviz"])

    # ------------------------------------------------------------------------------------
    # Robot description (xacro -> robot_state_publisher). Same for both controllers -- the
    # gz_ros2_control plugin loads the merged controllers.yaml regardless of `optipessi`.
    # ------------------------------------------------------------------------------------
    robot_description_content = Command(
        ["xacro ", xacro_file, " robot_type:=", robot_type, " controllers_yaml:=", controllers_yaml]
    )

    generate_urdf_node = Node(
        package="legged_common",
        executable="generate_urdf.sh",
        name="generate_urdf",
        output="screen",
        arguments=[xacro_file, robot_type],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": use_sim_time}],
    )

    # ------------------------------------------------------------------------------------
    # Gazebo (gz_sim) -- exactly one of these two runs, chosen by `optipessi`, which is the one
    # place the real-time factor is decided (do not hardcode RTF 0.2 anywhere else).
    # ------------------------------------------------------------------------------------
    gz_sim_optipessi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": ["-r ", opti_pessi_world_file]}.items(),
        condition=IfCondition(optipessi),
    )
    gz_sim_default = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": ["-r ", default_world_file]}.items(),
        condition=UnlessCondition(optipessi),
    )

    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_urdf",
        output="screen",
        # -z 0.5 matches empty_world_launch.xml's existing spawn height (comfortably above the
        # nominal standing height so nothing clips the ground plane on spawn); it is unrelated to
        # the comHeight investigated for Step 4, which governs the WBC's height REFERENCE, not the
        # one-shot spawn drop height.
        arguments=["-z", "0.5", "-topic", "robot_description", "-name", robot_type],
    )

    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # ------------------------------------------------------------------------------------
    # Broadcasters -- needed by both controllers.
    # ------------------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--service-call-timeout", "120.0"],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="imu_sensor_broadcaster_spawner",
        output="screen",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager", "--service-call-timeout", "120.0"],
    )

    # ------------------------------------------------------------------------------------
    # Controller spawners -- BOTH controllers are always loaded (--inactive for the one not
    # selected), exactly one is activated. Each spawner performs load+configure(+activate) in one
    # call -- the existing idiom in load_controller_launch.xml -- so there is no separate
    # "activate after load" step to sequence.
    # ------------------------------------------------------------------------------------
    legged_controller_active_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="legged_controller_spawner",
        output="screen",
        condition=UnlessCondition(optipessi),
        arguments=["legged_controller", "--controller-manager", "/controller_manager", "--service-call-timeout", "120.0"],
        parameters=[{"urdfFile": urdf_file, "taskFile": task_file, "referenceFile": reference_file, "gaitCommandFile": gait_command_file}],
    )
    legged_controller_inactive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="legged_controller_spawner",
        output="screen",
        condition=IfCondition(optipessi),
        arguments=[
            "legged_controller",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
            "--service-call-timeout",
            "120.0",
        ],
        parameters=[{"urdfFile": urdf_file, "taskFile": task_file, "referenceFile": reference_file, "gaitCommandFile": gait_command_file}],
    )

    opti_pessi_controller_active_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="opti_pessi_controller_spawner",
        output="screen",
        condition=IfCondition(optipessi),
        arguments=["opti_pessi_controller", "--controller-manager", "/controller_manager", "--service-call-timeout", "120.0"],
        parameters=[
            {
                "urdfFile": urdf_file,
                "taskFile": task_file,
                "referenceFile": reference_file,
                "optiPessiTaskFile": opti_pessi_task_file,
                "optiPessiScenarioFile": opti_pessi_scenario_file,
                "optiPessiRecompileLibraries": recompile_libraries,
            }
        ],
    )
    opti_pessi_controller_inactive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="opti_pessi_controller_spawner",
        output="screen",
        condition=UnlessCondition(optipessi),
        arguments=[
            "opti_pessi_controller",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
            "--service-call-timeout",
            "120.0",
        ],
        parameters=[
            {
                "urdfFile": urdf_file,
                "taskFile": task_file,
                "referenceFile": reference_file,
                "optiPessiTaskFile": opti_pessi_task_file,
                "optiPessiScenarioFile": opti_pessi_scenario_file,
                "optiPessiRecompileLibraries": recompile_libraries,
            }
        ],
    )

    # ------------------------------------------------------------------------------------
    # legged/LeggedController's companions (target trajectories publisher, gait command) -- only
    # meaningful when it is the active controller. OptiPessiController does not use OCS2 MPC target
    # trajectories or the gait scheduler; it seeds/solves its own plan internally.
    # ------------------------------------------------------------------------------------
    gait_command_node = Node(
        package="ocs2_legged_robot_ros",
        executable="legged_robot_gait_command",
        name="legged_robot_gait_command",
        output="screen",
        prefix="xterm -e",
        condition=UnlessCondition(optipessi),
        parameters=[{"gaitCommandFile": gait_command_file}],
    )

    target_trajectories_publisher_node = Node(
        package="legged_controllers",
        executable="legged_target_trajectories_publisher",
        name="legged_robot_target",
        output="screen",
        condition=UnlessCondition(optipessi),
        parameters=[{"taskFile": task_file, "referenceFile": reference_file}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Pin pinocchio to one ABI for every process this launch starts -- notably gz_sim, which is
    # where gz_ros2_control (and therefore the controller manager, and therefore the URDF parse)
    # actually runs. Must come first so it applies to everything below it.
    actions = []
    pinocchio_preload = _pinocchio_abi_preload()
    if pinocchio_preload is not None:
        actions.append(SetEnvironmentVariable("LD_PRELOAD", pinocchio_preload))

    return LaunchDescription(
        actions
        + [
            optipessi_arg,
            robot_type_arg,
            scenario_arg,
            use_sim_time_arg,
            rviz_arg,
            recompile_libraries_arg,
            generate_urdf_node,
            robot_state_publisher_node,
            gz_sim_optipessi,
            gz_sim_default,
            spawn_urdf_node,
            clock_bridge_node,
            joint_state_broadcaster_spawner,
            imu_sensor_broadcaster_spawner,
            legged_controller_active_spawner,
            legged_controller_inactive_spawner,
            opti_pessi_controller_active_spawner,
            opti_pessi_controller_inactive_spawner,
            gait_command_node,
            target_trajectories_publisher_node,
            rviz_node,
        ]
    )
