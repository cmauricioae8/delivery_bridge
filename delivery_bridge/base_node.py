# import os, copy

# ROS2 imports
import rclpy
from rclpy.node import Node
# from rclpy.qos import qos_profile_sensor_data

# from delivery_bridge.webapp.socket_io import emitEvent
# from delivery_bridge.webapp.settings import settings

from delivery_bridge.topics.battery_subscriber import BatterySubscriber
# from delivery_bridge.topics.pose_subscriber import PoseSubscriber
# from delivery_bridge.topics.pose_publisher import PosePublisher
# from delivery_bridge.topics.map_subscriber import MapSubscriber
from delivery_bridge.topics.cmd_vel_publisher import CmdVelPublisher
# from delivery_bridge.topics.mode_status_publisher import ModeStatusPublisher


class BaseNode(Node):
    def __init__(self):
        # Initialize ROS 2 client library
        rclpy.init()
        # Create the node
        super().__init__("server_node")
        self.logger = self.get_logger()
        self.logger.info("Starting node BaseNode ...")

        # emitEvent(
        #     "on_status_change",
        #     {
        #         "data": {
        #             "general": {
        #                 "on_ros": True,
        #             }
        #         }
        #     },
        # )

        ################################################################################

        # environ = os.environ.copy()
        """
            - LIDAR_MODEL='rplidar' or 'pacecat', pacecat_cr
            - ROS_DOMAIN_ID=2
        """

        # if "LIDAR_MODEL" in environ:
        #     self.logger.info(f"LIDAR_MODEL found = {environ['LIDAR_MODEL']}")
        #     if environ["LIDAR_MODEL"] == "rplidar":
        #         lidar_rotation_angle = 3.14159


        self.battery_subscriber = BatterySubscriber(
            self,
            "/battery", #settings.BATTERY.TOPIC_NAME,
            "std_msgs/Float64", # settings.BATTERY.TOPIC_TYPE,
            10.0, # settings.BATTERY.PERCENTAGE_ZERO_SAFE,
            95.0, # settings.BATTERY.PERCENTAGE_FULL_SAFE,
            20.0, # settings.BATTERY.PERCENTAGE_LOW,
            20.0, # settings.BATTERY.VOLTAGE_ZERO,
            25.0, # settings.BATTERY.VOLTAGE_FULL,
            1, # settings.BATTERY.MAX_EMIT_RATE,
        )
        # ros2 topic pub --rate 8 /battery std_msgs/msg/Float64 data:\ 24.1

        self.cmd_vel_publisher = CmdVelPublisher(
            self,
            "/cmd_vel", # settings.CMD_VEL.TOPIC_NAME,
            "geometry_msgs/Twist", # settings.CMD_VEL.TOPIC_TYPE,
        )

        self.logger.info("... BaseNode initialized")

        # emitEvent(
        #     "on_status_change",
        #     {
        #         "data": {
        #             "general": {
        #                 "ready": True,
        #             }
        #         }
        #     },
        # )

    def init_topics(self):
        # Try to subscribe and create the publishers for the topics
        self.logger.info("Initializing topics ...")
        self.battery_subscriber.try_subscribe()
        self.cmd_vel_publisher.try_create_publisher()
        # self.navigation_client.try_create_client()
        self.logger.info("Topics initialized")


base_node = BaseNode()
