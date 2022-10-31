import rclpy
import shlex
import subprocess
from rclpy.node import Node
from dca1000evm_interface.srv import (StartRecord, StopRecord, FpgaConfig,
                                                RecordConfig, ResetArDevice)
from ament_index_python.packages import get_package_share_directory

class dca1000evm_cli(Node):
    def __init__(self) -> None:
        super().__init__('dca1000evm_cli_node')

        # create services
        self.start_record_srv = self.create_service(StartRecord,
                                        "start_record", self.start_record_cb)
        self.stop_record_srv = self.create_service(StopRecord,
                                        "stop_record", self.stop_record_cb)
        self.fpga_config_srv = self.create_service(FpgaConfig,
                                        "fpga_config", self.fpga_config_cb)
        self.record_config_srv = self.create_service(RecordConfig,
                                        "record_config", self.record_config_cb)
        self.reset_ar_srv = self.create_service(ResetArDevice,
                                        "reset_ar_device", self.reset_ar_cb)

        # load parameters
        self.declare_parameter('config_file', 'None')
        self.declare_parameter('cli_timeout', 10)
        self.config_file = self.get_parameter(
                    'config_file').get_parameter_value().string_value
        self.cli_timeout = self.get_parameter(
                    'cli_timeout').get_parameter_value().integer_value
        
        # cli directory
        package_share_directory = get_package_share_directory(
                                                    'dca1000evm_cli')
        self.cli_path = package_share_directory + '/cli/'

        # valid responses from the board
        self.valid_responses = set(['Reset AR Device command : Success',
                                    'FPGA Configuration command : Success',
                                    'Configure Record command : Success',
                                    'Start Record command : Success',
                                    'Stop Record command : Success',
                                    'Record stop is done successfully'])

    def start_record_cb(self, request, response):
        """
        Start recording data
        """
        if request.data:
            proc = subprocess.Popen(shlex.split(
                f'./DCA1000EVM_CLI_Control start_record {self.config_file}'),
                                    cwd=self.cli_path, stdout=subprocess.PIPE)
            out, err = proc.communicate(timeout=self.cli_timeout)
            for line in out.splitlines():
                self.get_logger().debug(line.decode('utf-8'))
                if line.decode('utf-8') in self.valid_responses:
                    response.success = True
                    break
                response.success = False
            if not response.success:
                self.get_logger().error("Failed to start recording.")
        return response

    def stop_record_cb(self, request, response):
        """
        Stop recording data
        """
        if request.data:
            proc = subprocess.Popen(shlex.split(
                f'./DCA1000EVM_CLI_Control stop_record {self.config_file}'),
                                    cwd=self.cli_path, stdout=subprocess.PIPE)
            out, err = proc.communicate(timeout=self.cli_timeout)
            for line in out.splitlines():
                self.get_logger().debug(line.decode('utf-8'))
                if line.decode('utf-8') in self.valid_responses:
                    response.success = True
                    break
                response.success = False
            if not response.success:
                self.get_logger().error("Failed to stop recording.")
        return response

    def fpga_config_cb(self, request, response):
        """
        Send FPGA configuration
        """
        if request.data:
            proc = subprocess.Popen(shlex.split(
                f'./DCA1000EVM_CLI_Control fpga {self.config_file}'),
                                    cwd=self.cli_path, stdout=subprocess.PIPE)
            out, err = proc.communicate(timeout=self.cli_timeout)
            for line in out.splitlines():
                self.get_logger().debug(line.decode('utf-8'))
                if line.decode('utf-8') in self.valid_responses:
                    response.success = True
                    break
                response.success = False
            if not response.success:
                self.get_logger().error("Failed to configure FPGA.")
        return response

    def record_config_cb(self, request, response):
        """
        Send record configuration
        """
        if request.data:
            proc = subprocess.Popen(shlex.split(
                f'./DCA1000EVM_CLI_Control record {self.config_file}'),
                                    cwd=self.cli_path, stdout=subprocess.PIPE)
            out, err = proc.communicate(timeout=self.cli_timeout)
            for line in out.splitlines():
                self.get_logger().debug(line.decode('utf-8'))
                if line.decode('utf-8') in self.valid_responses:
                    response.success = True
                    break
                response.success = False
            if not response.success:
                self.get_logger().error("Failed to configure record.")
        return response

    def reset_ar_cb(self, request, response):
        """
        Reset the AR device
        """
        if request.data:
            proc = subprocess.Popen(shlex.split(
              f'./DCA1000EVM_CLI_Control reset_ar_device {self.config_file}'),
                                    cwd=self.cli_path, stdout=subprocess.PIPE)
            out, err = proc.communicate(timeout=self.cli_timeout)
            for line in out.splitlines():
                self.get_logger().debug(line.decode('utf-8'))
                if line.decode('utf-8') in self.valid_responses:
                    response.success = True
                    break
                response.success = False
            if not response.success:
                self.get_logger().error("Failed to reset AR device.")
        return response

def main():
    rclpy.init()
    node = dca1000evm_cli()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
