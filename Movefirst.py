!/usr/bin/env python3

# Copyright (c) 2016 Anki, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''Tell Cozmo to drive to the specified pose and orientation.

Define a destination pose for Cozmo. If relative_to_robot is set to true,
the given pose will assume the robot's pose as its origin.
'''

import sys

import cozmo
from cozmo.util import degrees, Pose, distance_mm, speed_mmps
import numpy as np

class explore:
    
    def __init__(self,length,width,robot):
        self.states= np.zeros((length,width))
        self.cozmo_state=(0,0)
	self.robot=robot
    
    def in_state(state):
        self.cozmo_state=state  

    def get_state():
        return self.robot.pose

    def move_to_state():
        return 0
    
def run(sdk_conn):
    '''The run method runs once Cozmo is connected.'''
    robot = sdk_conn.wait_for_robot()

    #robot.go_to_pose(Pose(200, 200, 0,angle_z=degrees(90)), relative_to_robot=False).wait_for_completed()
    #print (robot.pose)
    
    tasks = [0,0,90,0,270,0,0]

    for i in tasks:
	robot.turn_in_place(degrees(i)).wait_for_completed()
	robot.drive_straight(distance_mm(100),speed_mmps(50)).wait_for_completed()


if __name__ == '__main__':
    cozmo.setup_basic_logging()
    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
