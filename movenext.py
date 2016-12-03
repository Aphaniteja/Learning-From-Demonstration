
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
import math

    
def run(sdk_conn):
    '''The run method runs once Cozmo is connected.'''
    robot = sdk_conn.wait_for_robot()

    #robot.go_to_pose(Pose(200, 200, 0,angle_z=degrees(90)), relative_to_robot=False).wait_for_completed()
    #print (robot.pose)
    
    tasks = [0,0,90,0,270,0,0]
    cozmo.objects.OBJECT_VISIBILITY_TIMEOUT=0.2
    while(True):
        pin=robot.world.visible_object_count()
        print (pin)
        obj=robot.world.visible_objects
        if(pin>0):
            cube=robot.world.wait_until_observe_num_objects(num=1, object_type=cozmo.objects.LightCube, timeout=60)
            k=cube[-1].pose
            translation = robot.pose - k
            if(math.fabs(float(translation.position.x)))<=5:
               break
            print(translation)
            

        robot.drive_straight(distance_mm(10),speed_mmps(50)).wait_for_completed()
        
          
if __name__ == '__main__':
    cozmo.setup_basic_logging()
    try:
        cozmo.connect_with_tkviewer(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
