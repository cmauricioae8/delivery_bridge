#!/usr/bin/env python

# Third apps
import cv2 as cv
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

# Local apps
from .base_topics import BaseSubscriber
# from ..utils import image_to_base64_str
# from delivery_bridge.webapp.socket_io import emitEvent

"""
class MapData:
    def __init__(self):
        self.image: np.ndarray = None
        self.data: list = []
        self.width: int = 0
        self.height: int = 0
        self.resolution: float = 0
        self.origin_x: float = 0
        self.origin_y: float = 0
        self.origin_rad: float = 0

    def update(
        self,
        data: list,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_rad: float,
    ):
        self.data = data
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_rad = origin_rad

    def image_to_base64(self) -> str:
        return image_to_base64_str(self.image)

    def to_dict(self) -> dict:
        return {
            "image": self.image_to_base64(),
            "width": self.width,  # in pixels
            "height": self.height,  # in pixels
            "resolution": self.resolution,  # m/pixel
            "origin_x": self.origin_x,  # m
            "origin_y": self.origin_y,  # m
            "origin_rad": self.origin_rad,  # rad
        }
"""

class MapSubscriber(BaseSubscriber):
    def __init__(
        self,
        node: Node,
        topic_name: str,
        # colors: MapColors,
        max_rate: int = -1,
        # event_name: str = "map",
    ):
        super().__init__(node, topic_name, "nav_msgs/OccupancyGrid", max_rate)

        # self.event_name = event_name

        # state for map topic
        # self.map_data: MapData = MapData()
        self.map_available = False

        # assign message class
        self.message_class = OccupancyGrid

    def clear_map(self):
        self.map_available = False

    def safe_callback(self, msg: OccupancyGrid):
        # self.node.logger.info(f"Map received from '{self.topic_name}' topic")
        # print("Received a map of size %d x %d" % (msg.info.width, msg.info.height))
        # print("Resolution: %.2f m/pix" % (msg.info.resolution))
        # print("Origin: %.2f, %.2f, %.2f" % (
        #  msg.info.origin.position.x,
        #  msg.info.origin.position.y,
        #  msg.info.origin.position.z))

        # The origin of the map [m, m, rad].  This is the real-world pose of the
        # cell (0,0) in the map.
        # That is the coordinate of the lower left corner of your map in the reference
        # frame
        origin_x = abs(msg.info.origin.position.x)
        origin_y = msg.info.height * msg.info.resolution - abs(
            msg.info.origin.position.y
        )
        origin_rad = msg.info.origin.position.z

        self.map_available = True
