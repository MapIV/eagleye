import os

import ament_index_python.packages
import launch
import launch_ros.actions
import yaml

def generate_launch_description():
  # share_dir = ament_index_python.packages.get_package_share_directory('eagleye_rt')
  # params_file = os.path.join(share_dir, 'config', 'eagleye_config.yaml')
  # with open(params_file, 'r') as f:
  #   params = yaml.safe_load(f)['eagleye']['ros__parameters']
  velocity_scale_factor = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='velocity_scale_factor',
                                      parameters=[params])
  yawrate_offset_stop_node = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='yawrate_offset_stop_node',
                                      parameters=[params])
  yawrate_offset_node_1st = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='yawrate_offset_node_1st',
                                      parameters=[params])
  yawrate_offset_node_2nd = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='yawrate_offset_node_2nd',
                                      parameters=[params])
  heading_node_1st = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='heading_node_1st',
                                      parameters=[params])
  heading_node_2nd = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='heading_node_2nd',
                                      parameters=[params])
  heading_node_3rd = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='heading_node_3rd',
                                      parameters=[params])
  heading_interpolate_node_1st = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='heading_interpolate_node_1st',
                                      parameters=[params])
  heading_interpolate_node_2nd = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='heading_interpolate_node_2nd',
                                      parameters=[params])
  heading_interpolate_node_3rd = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='heading_interpolate_node_3rd',
                                      parameters=[params])
  slip_angle_node = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='slip_angle_node',
                                      parameters=[params])
  distance_node = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='distance_node',
                                      parameters=[params])
  trajectory_node = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='trajectory_node',
                                      parameters=[params])
  position_node = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='position_node',
                                      parameters=[params])
  position_interpolate_node = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='position_interpolate_node',
                                      parameters=[params])
  smoothing_node = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='smoothing_node',
                                      parameters=[params])
  height_node = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='height_node',
                                      parameters=[params])
  slip_coefficient_node = launch_ros.actions.Node(package='eagleye_rt',
                                      executable='slip_coefficient_node',
                                      parameters=[params])


  return launch.LaunchDescription([velocity_scale_factor,
                                   yawrate_offset_stop_node
                                   yawrate_offset_node_1st
                                   yawrate_offset_node_2nd
                                   heading_node_1st
                                   heading_node_2nd
                                   heading_node_3rd
                                   heading_interpolate_node_1st
                                   heading_interpolate_node_2nd
                                   heading_interpolate_node_3rd
                                   slip_angle_node
                                   distance_node
                                   trajectory_node
                                   position_node
                                   position_interpolate_node
                                   smoothing_node
                                   height_node
                                   slip_coefficient_node
                                      launch.actions.RegisterEventHandler(
                                      event_handler=launch.event_handlers.OnProcessExit(
                                          target_action=slip_coefficient_node,
                                          on_exit=[launch.actions.EmitEvent(
                                              event=launch.events.Shutdown())],
                                      )),
                                  ])


