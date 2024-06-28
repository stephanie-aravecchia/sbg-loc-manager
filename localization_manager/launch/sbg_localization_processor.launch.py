
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true')

    loc_processor = Node(
            package='localization_manager',
            executable='sbg_localization_processor',
            name='sbg_localization_processor',
            namespace="hus",
            parameters=[
                {'use_sim_time': use_sim_time},
                {'reference_frame': 'utm'},
                {'local_reference_frame': 'utm_local_gte'},
                {'target_frame': 'base_link'},
                {'imu_frame': 'sbg'},
                {'local_ref_latitude': 49.10222222},
                {'local_ref_longitude': 6.21555556},
                {'local_ref_altitude': 201.838511}
            ],
            remappings=[
                ('sbg_ekf_nav', '/hus/sbg/ekf_nav'),
                ('sbg_imu_data', '/hus/sbg/imu_data'),
                ('sbg_ekf_quat', '/hus/sbg/ekf_quat'),
                ('sbg_ekf_euler', '/hus/sbg/ekf_euler'),
                ('/tf','/hus/tf'),
                ('/tf_static', '/hus/tf_static'),
            ],
            output="screen"
        )
    ld = LaunchDescription([loc_processor])
    return ld