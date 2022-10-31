from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

package_path = Path(__file__).resolve().parent.parent

def generate_launch_description():
    ld = LaunchDescription()

    dca1000evm_cli_node = Node(
        package='dca1000evm_cli',
        executable='dca1000evm_cli_node',
        name='dca1000evm_cli_node',
        emulate_tty=True,
        parameters = [
                ParameterFile(
                    param_file = package_path / 'config/params.yaml',
                    allow_substs = True)],
        arguments=['--ros-args', '--log-level', 'DEBUG'] 
    )

    ld.add_action(dca1000evm_cli_node)

    return ld