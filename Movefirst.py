
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
import tkinter as tk

class Environment:
    def __init__(self,height,width):
        self.states = np.zeros((height,width))
        self.source = (0,0)
        self.goal = (3,3)
        self.reward = 1
    
    def set_goal(self,location):
        self.goal = location

    def set_reward(self,reward_val):
        self.reward = reward_val


class Agent:
    def __init__(self,start_state,environment):
        self.current_state = start_state
        self.orientation = 0
        self.map = environment.states
        self.height = self.map.shape[0]
        self.width = self.map.shape[1]
        self.map[0][0]=1
        self.move_count=0

        self.allowable = [['w','d','s','a'],['s','a','w','d'],['d','s','a','w'],['a','w','d','s']]

    def remap_update(self,current_state,dir):
        if(dir==0):
            current_state[0]=current_state[0]+1
        elif((dir%360)==180):
            current_state[0]=current_state[0]-1
        elif((dir%360)==90):
            current_state[1]=current_state[1]+1
        elif((dir%360)==270):
            current_state[1]=current_state[1]-1

        return current_state


    def goto_state(self,state):
        current_state=self.current_state
        xdiff=state[0]-current_state[0]
        ydiff=state[1]-current_state[1]
        while(xdiff>0):
           robot.turn_in_place(degrees(0)).wait_for_completed()
           robot.drive_straight(distance_mm(50),speed_mmps(200)).wait_for_completed()
           update_map("w")
           current_state=self.current_state
           xdiff=state[0]-current_state[0]
           ydiff=state[1]-current_state[1]
        while(xdiff<0):
           robot.turn_in_place(degrees(180)).wait_for_completed()
           robot.drive_straight(distance_mm(50),speed_mmps(200)).wait_for_completed()
           update_map("s")
           current_state=self.current_state
           xdiff=state[0]-current_state[0]
           ydiff=state[1]-current_state[1]           
        while(ydiff>0):
           robot.turn_in_place(degrees(90)).wait_for_completed()
           robot.drive_straight(distance_mm(50),speed_mmps(200)).wait_for_completed()
           update_map("d")

           current_state=self.current_state
           xdiff=state[0]-current_state[0]
           ydiff=state[1]-current_state[1]
        while(ydiff<0):
           robot.turn_in_place(degrees(270)).wait_for_completed()
           robot.drive_straight(distance_mm(50),speed_mmps(200)).wait_for_completed
           update_map("a")
           current_state=self.current_state
           xdiff=state[0]-current_state[0]
           ydiff=state[1]-current_state[1]           

    def update_map(self,move_pos):
        current_state=list(self.current_state)
        if(move_pos=='w'):
            self.orientation=(self.orientation+0)%360
            current_state=self.remap_update(current_state,self.orientation)
            self.map[current_state[0]][current_state[1]]+=1
        elif(move_pos=='s'):
            self.orientation=(self.orientation+180)%360
            current_state=self.remap_update(current_state,self.orientation)
            self.map[current_state[0]][current_state[1]]+=1
        elif(move_pos=='d'):
            self.orientation=(self.orientation-90)%360
            current_state=self.remap_update(current_state,self.orientation)
            self.map[current_state[0]][current_state[1]]+=1
        elif(move_pos=='a'):
            self.orientation=(self.orientation+90)%360
            current_state=self.remap_update(current_state,self.orientation)
            self.map[current_state[0]][current_state[1]]+=1

        self.current_state=tuple(current_state)

    def print_map(self):
        print(self.map)

    def allowed_grids(self):
        legal_moves=['up','down','left','right']
        if(self.current_state[1]==0):
            legal_moves.remove('right')

        if(self.current_state[1]==self.height-1):
            legal_moves.remove('left')

        if(self.current_state[0]==0):
            legal_moves.remove('down')

        if(self.current_state[0]==self.width-1):
            legal_moves.remove('up')

        return legal_moves

    def get_legal_moves(self):
        allowed = self.allowed_grids()

        moves=[] 

        for dir in allowed:
            if(dir=='up'):
                orien=(self.orientation%360)/90
                moves.append(self.allowable[0][int(orien)])
            if(dir=='down'):
                orien=(self.orientation%360)/90
                moves.append(self.allowable[1][int(orien)])
            if(dir=='right'):
                orien=(self.orientation%360)/90
                moves.append(self.allowable[2][int(orien)])
            if(dir=='left'):
                orien=(self.orientation%360)/90
                moves.append(self.allowable[3][int(orien)])

        return moves




    
def run(sdk_conn):
    robot = sdk_conn.wait_for_robot()
    poser=robot.pose

    #robot.go_to_pose(Pose(200, 200, 0,angle_z=degrees(90)), relative_to_robot=False).wait_for_completed()
    
    tasks = [0,0,90,0,270,0,0]
    field = Environment(5,5)
    bot = Agent((0,0),field)
    movesr=[]
    executed_moves=[]

    while(True):
        
    #for i in tasks:  
        moves=bot.get_legal_moves()


        #task=input()
        task=np.random.choice(moves,1)

        task=task[0]

        executed_moves.append(task)

        moves.extend(['p','l','o','c'])

        if task not in moves:
            print('Illegal Move')
            robot.play_anim_trigger(cozmo.anim.Triggers.DriveEndAngry).wait_for_completed()
            print('Try',moves)
            continue
        bot.move_count+=1
        print(task)
        

        if(task=='w'):
            i=0
            bot.update_map(task)

        elif(task=='a'):
            i=90
            bot.update_map(task)

        elif(task=='d'):
            i=270
            bot.update_map(task)

        elif(task=='s'):
            i=180
            bot.update_map(task)

        elif(task=='p'):
            bot.print_map()
            continue


        robot.turn_in_place(degrees(i)).wait_for_completed()
        robot.drive_straight(distance_mm(100),speed_mmps(200)).wait_for_completed()
        bot.print_map()

        if(bot.current_state[0]==1 and bot.current_state[1]==1):
            robot.set_all_backpack_lights(cozmo.lights.blue_light)
            robot.play_anim_trigger(cozmo.anim.Triggers.MajorWin).wait_for_completed()
            bot.print_map()
            robot.go_to_pose(poser, relative_to_robot=False).wait_for_completed()
            #print(poser)
            #print(robot.pose)
            bot.current_state=(0,0)
            bot.orientation=0
            print(executed_moves)
            movesr.append(bot.move_count)
            bot.moves_count=0
            print(movesr[-1],"latest moves")
            

        
        #bot.print_map()
        #print(bot.get_legal_moves())
        #print(bot.orientation)
        #print(bot.current_state)



if __name__ == '__main__':
     cozmo.setup_basic_logging()
     try:
         cozmo.connect(run)
     except cozmo.ConnectionError as e:
         sys.exit("A connection error occurred: %s" % e)

     except KeyboardInterrupt as e:
        sys.exit("Interrupted")
