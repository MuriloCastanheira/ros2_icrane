# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import math

import argparse

import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

import omni
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

import time

# Note that this is not the system level rclpy, but one compiled for omniverse
import numpy as np
import rclpy
from rclpy.node import Node
from icrane_msgs.msg import OmniverseInfo
import omni.kit.commands
from math import pi
import math
from pxr import Usd, Sdf



class Subscriber(Node):
    def __init__(self):
        super().__init__("tutorial_subscriber")

        # setting up the world with a cube
        assets_root_path = get_assets_root_path()
        asset_path = "/root/colcon_ws/Aquatec_isaac/Grua/Grua_Modelagem/New_Grua.usd"

        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=1.0)
        self.ros_world.scene.add_default_ground_plane()
        add_reference_to_stage(usd_path=asset_path, prim_path="/World")
        
        self.people = 10
        self.boxes = 4
        self.points_info = 0
        self.joint_states = 0
        self.load_position = 0
        self.people_info = 0
        self.boxes_info = 0
        self.key = 0
        self.path_x = 0
        self.path_y = 0

        self.old_position = 0
        self.old_ghost_position = 0
        self.position_direction = 0
        self.visibility = 1

        self.altura = 18

        self.ros_sub = self.create_subscription(OmniverseInfo, "compress", self.integration_callback, 1)
        self.ros_world.reset()

    def integration_callback(self, msg):

        if self.ros_world.is_playing():           
            self.joint_states = msg.joint_state
            self.people_info = msg.people_info
            self.boxes_info = msg.boxes_info
            self.points_info = msg.points
            if msg.load_position.poses:
                self.path_x = msg.load_position.poses[0].pose.position.x
                self.path_y = msg.load_position.poses[0].pose.position.y
                self.path_z = msg.load_position.poses[0].pose.position.z
            self.key = 1           
            
            
            
    def run_simulation(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.ros_world.is_playing():
                if self.ros_world.current_time_step_index == 0:
                    self.ros_world.reset()

                if self.key > 0:
                    

                    


                    ###################### Guindaste
                    # isso tem que virar uma função em separado
                    if self.joint_states.position:
                        ## Ta indiu
                        if self.old_position < (self.joint_states.position[0] * 180/pi):
                            self.position_direction = 1
                        ## Ta vindiu
                        elif self.old_position > (self.joint_states.position[0] * 180/pi):
                            self.position_direction = 2
                        
                        self.old_position = self.joint_states.position[0] * 180/pi

                        omni.kit.commands.execute('ChangeProperty',
                            prop_path=Sdf.Path('/World/_dof_spacial/base/joint2.drive:angular:physics:targetPosition'),
                            value= self.joint_states.position[0] * 180/pi,
                            prev=23.600000381469727)
                        
                    ###################### trajeto ###########################################################################################
                    print(self.visibility)
                    ## Se tiver indiu
                    if self.position_direction == 1:
                        #if (math.atan2(0, 0) - math.atan2(self.points_info.predict_x, self.points_info.predict_y))* 180/math.pi + 90 > self.joint_states.position[0] * 180/pi:
                         #   if self.old_ghost_position < (math.atan2(0, 0) - math.atan2(self.points_info.predict_x, self.points_info.predict_y))* 180/math.pi + 90 and self.old_ghost_position < 98:
                        for i in range(30):
                            omni.kit.commands.execute('TransformMultiPrimsSRTCpp',
                            count=1,
                            paths=[f'/World/Trajeto/ball_{i+1}'],
                            new_translations=[-20 + self.points_info.predict_x[i] * 3.63,-18 + self.points_info.predict_y[i] * 3.63, self.points_info.predict_z[i]* 3.63],
                            new_rotation_eulers=[90, 0.0, 0.0],
                            new_rotation_orders=[0, 1, 2],
                            new_scales=[1, 1, 1],
                            old_translations=[0.0, 0.0, 0.0],
                            old_rotation_eulers=[0.0, 0.0, 0.0],
                            old_rotation_orders=[0, 1, 2],
                            old_scales=[1.0, 1.0, 1.0],
                            usd_context_name='',
                            time_code=0.0)
                            
                    if self.position_direction == 2:
                        pass


                if self.altura < 0:    
                    for i in range(18):
                        omni.kit.commands.execute('TransformMultiPrimsSRTCpp',
                        count=1,
                        paths=[f'/World/WoodenCrate_A1'],
                        new_translations=[0, -18.0, 18-i],
                        new_rotation_eulers=[90, 0.0, 0.0],
                        new_rotation_orders=[0, 1, 2],
                        new_scales=[2.0, 2.0, 2.0],
                        old_translations=[0.0, 0.0, 0.0],
                        old_rotation_eulers=[0.0, 0.0, 0.0],
                        old_rotation_orders=[0, 1, 2],
                        old_scales=[1.0, 1.0, 1.0],
                        usd_context_name='',
                        time_code=0.0) 

                        self.altura = i

                if self.altura > 0:    
                    for i in range(18):
                        omni.kit.commands.execute('TransformMultiPrimsSRTCpp',
                        count=1,
                        paths=[f'/World/WoodenCrate_A1'],
                        new_translations=[0, -18.0, i],
                        new_rotation_eulers=[90, 0.0, 0.0],
                        new_rotation_orders=[0, 1, 2],
                        new_scales=[2.0, 2.0, 2.0],
                        old_translations=[0.0, 0.0, 0.0],
                        old_rotation_eulers=[0.0, 0.0, 0.0],
                        old_rotation_orders=[0, 1, 2],
                        old_scales=[1.0, 1.0, 1.0],
                        usd_context_name='',
                        time_code=0.0)  

                        self.altura = i                 
                                

        # Cleanup
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    subscriber = Subscriber()
    subscriber.run_simulation()
