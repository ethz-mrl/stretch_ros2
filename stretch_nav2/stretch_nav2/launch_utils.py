import math
import os
import tempfile

import yaml


def write_okvis_nav_params(source_params, bt_nav_to_pose, bt_nav_through_poses,
                           rolling_global_costmap=False, use_rpp_controller=False,
                           max_linear_vel=0.26, max_angular_vel=0.4):
    """Produce the OKVIS-tuned nav2 params file WITHOUT touching the shared
    nav2_params.yaml. Read the pristine params (used as-is by the wheel-odometry
    navigation.launch.py) and apply every OKVIS-navigation-specific override here,
    so all tuning lives in this launch file. Returns the path to a generated temp
    file. RewrittenYaml can only rewrite pre-existing keys and struggles with list
    values and adding the (Humble) BT-xml keys, so we edit the parsed YAML directly.

    `rolling_global_costmap`: the stock global_costmap is a STATIC window fixed at
    origin (0, 0) with a ~5m extent, sized for the case where map_server publishes a
    real map to size/position it against. Launches with no map_server (e.g. the
    OKVIS explore launch, reloc:=none with map==odom==session start) have nothing to
    size it from, so the robot walks straight out of that fixed 5m box the moment it
    drives negative in x or y -- Nav2 then aborts every goal instantly ("Robot is out
    of bounds of the costmap!"). Set this to drop static_layer (nothing would ever
    populate it anyway with no /map) and make the costmap follow the robot instead.

    `use_rpp_controller`: replace DWB with
    nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController for
    FollowPath. DWB's RotateToGoal critic switches to pure in-place rotation the
    moment the robot is within FollowPath.xy_goal_tolerance, which combined with
    SmacPlannerHybrid's curved final approach arcs (needed to also match the goal
    orientation) can look like the robot "spinning" near the goal. RPP instead
    steers continuously off a lookahead point on the path -- no discrete rotate-only
    mode -- which tracks those curved approaches far more smoothly.

    `max_linear_vel`: caps top driving speed (m/s), applied to both the RPP
    controller's desired_linear_vel and the velocity_smoother's linear limits.

    `max_angular_vel`: caps top turning speed (rad/s), applied to the
    velocity_smoother's angular limits (this is what actually clips the commanded
    angular velocity downstream of whichever controller is in use).
    """
    with open(source_params) as f:
        cfg = yaml.safe_load(f)

    cs = cfg['controller_server']['ros__parameters']
    # Arrival tolerance (general_goal_checker) = when Nav2 declares the goal reached.
    cs['general_goal_checker']['xy_goal_tolerance'] = 0.1
    cs['general_goal_checker']['yaw_goal_tolerance'] = math.radians(10)

    if use_rpp_controller:
        cs['controller_plugins'] = ['FollowPath']
        cs['FollowPath'] = {
            'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            'desired_linear_vel': max_linear_vel,
            'lookahead_dist': 0.4,
            'min_lookahead_dist': 0.3,
            'max_lookahead_dist': 0.9,
            'lookahead_time': 1.5,
            'transform_tolerance': 0.2,
            'use_velocity_scaled_lookahead_dist': False,
            'min_approach_linear_velocity': 0.03,
            'approach_velocity_scaling_dist': 0.6,
            'use_collision_detection': True,
            'max_allowed_time_to_collision_up_to_carrot': 1.0,
            'use_regulated_linear_velocity_scaling': True,
            'use_cost_regulated_linear_velocity_scaling': False,
            'regulated_linear_scaling_min_radius': 0.9,
            'regulated_linear_scaling_min_speed': 0.25,
            'use_fixed_curvature_lookahead': False,
            'curvature_lookahead_dist': 0.25,
            # use_rotate_to_heading puts RPP into a discrete "pure rotate in place"
            # mode whenever the bearing to the lookahead point exceeds
            # rotate_to_heading_min_angle. SmacPlannerHybrid's paths curve/loop
            # (forced nonzero minimum_turning_radius even though Stretch can pivot
            # in place), so that bearing keeps swinging as the path curls, and RPP
            # keeps re-entering rotate mode instead of settling -- this IS the
            # observed continuous spinning. Disabled: RPP always steers via
            # continuous curvature (angular velocity proportional to path
            # curvature) instead, which can't get stuck sustaining a spin the way
            # the discrete mode can. It also avoids rotate-to-heading's default
            # 1.8 rad/s command getting clamped to this launch's 0.05 rad/s
            # velocity-smoother cap, which made the final in-place rotation behave
            # unpredictably.
            'use_rotate_to_heading': True,
            'max_angular_accel': 0.4,
            'max_robot_pose_search_dist': 10.0,
            'use_interpolation': True,
            # RPP refuses to command reverse velocity at all unless this is set,
            # even though SmacPlannerHybrid's REEDS_SHEPP model plans occasional
            # (heavily reverse_penalty'd) reverse segments -- without this the
            # controller just can't follow those segments and the robot is stuck
            # going forward-only.
            'allow_reversing': False,
        }
    else:
        # DWB's RotateToGoal critic forces PURE ROTATION once within
        # FollowPath.xy_goal_tolerance of the goal. This MUST be <= the goal checker's
        # xy tolerance; otherwise the robot flips to rotate-only while still too far in
        # xy to ever satisfy completion, so it spins forever and never drives the last
        # stretch (the "controller replans, base doesn't move" deadlock). Keep it equal
        # so rotate-to-goal begins exactly at the arrival radius.
        cs['FollowPath']['xy_goal_tolerance'] = cs['general_goal_checker']['xy_goal_tolerance']
        # Must nearly stop before the final rotate-to-goal, so it settles precisely.
        cs['FollowPath']['trans_stopped_velocity'] = 0.05
        # Gentler rotation dynamics: the stock 1.0 rad/s @ 3.2 rad/s^2 overshoots the
        # yaw window at 20 Hz and hunts back and forth. 0.4 rad/s @ 0.8 rad/s^2 stops
        # within ~0.1 rad from full rate, so the final alignment settles first try.
        cs['FollowPath']['max_vel_theta'] = max_angular_vel
        cs['FollowPath']['acc_lim_theta'] = 0.8
        cs['FollowPath']['decel_lim_theta'] = -0.8
    # Keep the velocity smoother's theta limits in lockstep with the controller's, or it
    # re-clips/re-shapes the commands it already planned for.
    vs = cfg['velocity_smoother']['ros__parameters']
    vs['max_velocity'] = [max_linear_vel, 0.0, max_angular_vel]
    vs['min_velocity'] = [-max_linear_vel, 0.0, -max_angular_vel]
    vs['max_accel'] = [2.5, 0.0, 0.4]
    vs['max_decel'] = [-2.5, 0.0, -0.4]

    # Global planner: Smac Hybrid-A* instead of NavFn. NavFn paths carry no
    # orientation, so the goal yaw is only reachable by spinning in place at the
    # end (RotateToGoal). Hybrid-A* plans kinematically feasible curves that END
    # IN THE GOAL ORIENTATION, so PathAlign steers the robot through a final arc
    # and it arrives already facing the right way.
    cfg['planner_server']['ros__parameters']['GridBased'] = {
        'plugin': 'nav2_smac_planner/SmacPlannerHybrid',
        'tolerance': 0.25,
        'allow_unknown': True,
        'downsample_costmap': False,
        'downsampling_factor': 1,
        'max_iterations': 1000000,
        'max_on_approach_iterations': 1000,
        'max_planning_time': 5.0,
        # REEDS_SHEPP permits short reverse segments so plans still exist when
        # the robot starts nose-in to a wall (DUBIN = forward-only would fail
        # there). Reverse is heavily penalized and DWB caps it at 0.05 m/s.
        'motion_model_for_search': 'REEDS_SHEPP',
        'angle_quantization_bins': 72,
        # Stretch turns in place, but Hybrid-A* needs a radius > 0; 0.35 m keeps
        # approach arcs tight without exploding the search.
        'minimum_turning_radius': 0.35,
        'reverse_penalty': 3.0,
        'change_penalty': 0.0,
        'non_straight_penalty': 1.2,
        'cost_penalty': 2.0,
        'retrospective_penalty': 0.015,
        'analytic_expansion_ratio': 3.5,
        'analytic_expansion_max_length': 3.0,
        'lookup_table_size': 20.0,
        'cache_obstacle_heuristic': False,
        'smooth_path': True,
        'smoother': {
            'max_iterations': 1000,
            'w_smooth': 0.3,
            'w_data': 0.2,
            'tolerance': 1.0e-10,
        },
    }

    for scope in ('local_costmap', 'global_costmap'):
        p = cfg[scope][scope]['ros__parameters']
        # Inflation kept > the 0.22 m inscribed radius but well below the 0.55 default
        # so DWB can use lab passages that are physically safe for Stretch.
        p['inflation_layer']['inflation_radius'] = 0.25
        # OKVIS's 6-DoF pose can put the floor lidar a few cm below odom z=0; start the
        # voxel column below zero so the scan still raytraces and clears stale cells.
        p['voxel_layer']['origin_z'] = -0.10

    bs = cfg['behavior_server']['ros__parameters']
    # Wait-only recovery: spin/backup are disruptive on Stretch. Drop them (and their
    # now-unused plugin blocks) so only Wait remains.
    for plug in ('spin', 'backup', 'drive_on_heading', 'assisted_teleop'):
        bs.pop(plug, None)
    bs['behavior_plugins'] = ['wait']
    bs['wait'] = {'plugin': 'nav2_behaviors/Wait'}

    bn = cfg['bt_navigator']['ros__parameters']
    # Humble param names (`default_bt_xml_filename` is silently ignored). Point at the
    # wait-only-recovery trees. BOTH must be set: bt_navigator loads the nav-to-pose AND
    # nav-through-poses defaults at configure time, and the upstream trees invoke Spin/
    # BackUp whose (disabled) servers would abort bt_navigator startup.
    bn.pop('default_bt_xml_filename', None)
    bn['default_nav_to_pose_bt_xml'] = bt_nav_to_pose
    bn['default_nav_through_poses_bt_xml'] = bt_nav_through_poses

    if rolling_global_costmap:
        gc = cfg['global_costmap']['global_costmap']['ros__parameters']
        gc['plugins'] = [p for p in gc['plugins'] if p != 'static_layer']
        gc.pop('static_layer', None)
        gc['rolling_window'] = True
        gc['width'] = 10
        gc['height'] = 10

    fd, path = tempfile.mkstemp(prefix='okvis_nav2_params_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False)
    return path


def write_okvis_vio_config(source_cfg):
    """OKVIS's config uses do_loop_closures:true (full VI-SLAM). Loop closures re-
    optimize the pose graph and make world->base_link JUMP discretely -- fine for
    building a map, but nav2/AMCL assume odom->base_link is SMOOTH, so a jumping
    odometry makes the robot pose lurch, AMCL over-corrects, and the goal appears to
    jump. For navigation we want pure VIO (smooth odometry; AMCL supplies the global
    map correction). Flip that one flag via a text substitution -- the file is OpenCV
    `%YAML:1.0`, not PyYAML-parseable -- and write a temp config. No okvis fork edit.
    """
    with open(source_cfg) as f:
        text = f.read()
    if 'do_loop_closures: false' in text:
        # Already pure VIO (e.g. the source config was edited directly) -- nothing
        # to patch, use it as-is.
        return source_cfg
    patched = text.replace('do_loop_closures: true', 'do_loop_closures: false')
    if patched == text:
        raise RuntimeError('did not find `do_loop_closures: true` in ' + source_cfg)
    fd, path = tempfile.mkstemp(prefix='okvis_vio_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        f.write(patched)
    return path
