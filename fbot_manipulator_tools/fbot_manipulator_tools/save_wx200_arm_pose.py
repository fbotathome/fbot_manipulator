import rclpy
from rclpy.wait_for_message import wait_for_message
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

import yaml
import os

from interbotix_xs_msgs.srv import TorqueEnable
from sensor_msgs.msg import JointState

from collections import OrderedDict

class OrderedDumper(yaml.SafeDumper):
    '''
    @brief Custom YAML dumper to handle OrderedDict.
    '''
    def representOrderedDictionary(self, data):
        return self.represent_dict(data.items())
    
class OrderedLoader(yaml.SafeLoader):
    pass

def ConstructorOrderedDictionary(loader, node):
    '''
    @brief Constructor to OrderDictionary
    '''
    return OrderedDict(loader.construct_pairs(node))

OrderedLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    ConstructorOrderedDictionary
)

class ArmJointStateSaver(Node):
    '''
    @class ArmJointStateSaver
    @brief A ROS 2 node that saves the current joint state of a robot arm to a YAML file.
    This node subscribes to a specified joint state topic and allows the user to save the current pose of the arm
    by entering a name. The saved joint states are stored in a YAML file for later use.
    '''
    def __init__(self, node_name):
        '''
        @brief Initializes the ArmJointStateSaver node.
        @param node_name: The name of the node.
        @return: None
        '''
        super().__init__(node_name='wx200ArmPoseSaver')
        self.client = self.create_client(TorqueEnable, '/wx200/torque_enable')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Service not found')

        self.initializeRequisitions()
        self.poses = {'poses':{}}
        ws_dir = os.path.abspath(os.path.join(get_package_share_directory('fbot_manipulator_tools'), '../../../..'))
        self.config_path = os.path.join(ws_dir, "src", "fbot_manipulator", "config")
        while True:
            check_sleep = input("The torque will be disabled. The Arm is in the sleep pose? (y/n): ").lower()            
            if check_sleep == 'n':
                continue
            elif check_sleep == 'y':
                self.get_logger().warning('Disabling the torque')
                self.torqueControl(self.req_disabled)
                break
            else:
                self.get_logger().warning("Invalid input. Please enter 'y' or 'n'. ")

        while True:
            self.yaml_file = input("Enter the name of the file to save the joint states (e.g., arm_poses.yaml): ")
            if self.yaml_file.endswith('.yaml'):
                break
            else:
                self.get_logger().warning("The file extension must be a .yaml (e.g., arm_poses.yaml): ")
        self.yaml_path = self.config_path + '/' + self.yaml_file
        self.savePose()

    def initializeRequisitions(self):
        '''
        @brief Initializes the service requests for enabling and disabling torque.
        @return: None
        '''
        self.req_disabled = TorqueEnable.Request()
        self.req_disabled.cmd_type = 'group'
        self.req_disabled.name = 'all'
        self.req_disabled.enable = False
        self.req_enabled = TorqueEnable.Request()
        self.req_enabled.cmd_type = 'group'
        self.req_enabled.name = 'all'
        self.req_enabled.enable = True

    def torqueControl(self, data):
        '''
        @brief Sends a request to enable or disable the arm torque.
        @param data: self.req_disable(Disable arm torque) self.req_enabled(Enable arm torque)
        @return: Returns the client call
        '''
        return self.client.call_async(data)
    
    def savePose(self) -> None:
        '''
        @brief This method waits for a message on the '/wx200/joint_states' topic, retrieves the joint names and positions, and allows the user to name the pose. The pose is then saved in a YAML file. The method will keep asking for poses until the user decides to stop saving. The torque is disabled before saving poses and enabled after saving is complete.
        @return: None
        '''
        keys_to_remove = {'gripper', 'left_finger', 'right_finger'}
        keep_saving = True
        while rclpy.ok():
            arm_name = input("Move the arm to the desired pose and enter its name (e.g., 'PrePickup', 'LookToGarbage'): ")
            success, message = wait_for_message(msg_type= JointState, node=self, topic='/wx200/joint_states', time_to_wait=2)
            joints = message.name
            values = message.position
            self.joints_values = tuple(zip(joints,values))
            filtered_joints_values = [(k, v) for k, v in self.joints_values if k not in keys_to_remove]
            self.get_logger().info(f"Received msg: {filtered_joints_values}")
            self.current_pose = filtered_joints_values

            if not success:
                self.get_logger().warning("No pose received from topic yet.")
                continue
    
            if not arm_name: 
                self.get_logger().warning("No name provided, skipping pose.")
                continue
            self.poses['poses'][arm_name] = OrderedDict(filtered_joints_values)

            self.get_logger().info(f"Pose '{arm_name}' saved.")
            while keep_saving == True:
                save_more = input("Do you want to add more poses? (y/n): ").lower()
                if save_more == 'n':
                    keep_saving = False
                    self.writeToYaml()
                    self.get_logger().warning('Enabling the torque')
                    self.get_logger().info(f"Poses saved to {self.yaml_file}. Shutting down node.")

                    self.torqueControl(self.req_enabled)
                    return               
                elif save_more == 'y':
                    break
                else:
                    self.get_logger().warning("Invalid input. Please enter 'y' or 'n'.")

    def writeToYaml(self):
        ''' 
        @brief Writes the saved poses to a YAML file.
        @return: None
        '''
        OrderedDumper.add_representer(OrderedDict, OrderedDumper.representOrderedDictionary)

        if os.path.exists(self.yaml_path):
            self.get_logger().warning(f"{self.yaml_file} already exists. The new poses will be appended to the existing data.")

            with open(self.yaml_path, 'r') as yaml_file:
                try:
                    existing_data = yaml.load(yaml_file, Loader=OrderedLoader) or OrderedDict()
                except yaml.YAMLError as e:
                    self.get_logger().error(f"Error reading {self.yaml_file}: {e}")
                    existing_data = OrderedDict()
        else:
            self.get_logger().warning(f"{self.yaml_file} does not exist. Creating a new file.")
            existing_data = OrderedDict()

        if 'poses' not in existing_data:
            existing_data['poses']= OrderedDict(OrderedDict())

        existing_data['poses'].update(self.poses['poses'])
        self.get_logger().info(self.yaml_path)
        with open(self.yaml_path, 'w') as yaml_file:
             yaml.dump(existing_data, yaml_file, default_flow_style=False, Dumper=OrderedDumper)
        return self.get_logger().info('Saved Poses to the .yaml file')


def main(args=None) -> None:
    rclpy.init(args=args)
    saver = ArmJointStateSaver(node_name = 'arm')
    rclpy.spin_once(saver)
    saver.destroy_node()
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()