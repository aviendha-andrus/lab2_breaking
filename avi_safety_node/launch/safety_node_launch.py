from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
	student_arg = DeclareLaunchArgument('student', default_value ='avi')
	mode_arg = DeclareLaunchArgument('mode', default_value ='sim')
	ttc_arg = DeclareLaunchArgument('ttc', default_value='0.0')

	mode = LaunchConfiguration('mode')
	student = LaunchConfiguration('student')
	ttc = LaunchConfiguration('ttc')

	package_name =[student,TextSubstitution(text='_safety_node')]
	executable_name = [student,TextSubstitution(text='_safety_node.py')]

	student_node = Node(
		package=package_name,
		executable=executable_name,
		name=LaunchConfiguration('student'),
		output='screen',
		parameters=[
			{'mode': mode},
			{'ttc': ttc}
		]
	)

	return LaunchDescription([
		mode_arg,
		ttc_arg,
		student_arg,
		student_node
	])

# ros2 launch safety_node_launch.py mode:=sim student:=bree ttc:=2.0