import sys

import cozmo
from cozmo.util import degrees, Pose, distance_mm, speed_mmps
import numpy as np
import matplotlib.pyplot as plt
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
        self.prev_state=start_state
        self.orientation = 0
        self.map = environment.states
        self.height = self.map.shape[0]
        self.width = self.map.shape[1]
        self.map[0][0]=1
        self.move_count=0
        self.environment=environment
        
        self.mapper = {}
        
        i=0
        for a in range(5):
            for b in range(5):
                self.mapper[(a,b)]=i
                i=i+1
        

        self.alpha = 0.5
        self.gamma = 0.9
        self.Q = np.zeros((25,4)) #up, down, Right, Left

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



    def update_map(self,move_pos):
        self.prev_state=self.current_state
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
    
    def get_legal_dirs(self):
        legal_moves=['u','j','h','k']
        if(self.current_state[1]==0):
            legal_moves.remove('h')

        if(self.current_state[1]==self.height-1):
            legal_moves.remove('k')

        if(self.current_state[0]==0):
            legal_moves.remove('j')

        if(self.current_state[0]==self.width-1):
            legal_moves.remove('u')

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

    def actionvalues (self, s):
        return self.Q[s,:]
    def statevalue (self, s):
        if s == None:
            return 0
        elif s == 'terminal':
            return 0
        else:
            return np.max(self.actionvalues(s))

    def agentChoose(self):    # epsilon greedy
        s=self.mapper[self.current_state]
        if(np.random.rand()<0.9):
            ind=np.argmax(self.Q[s,:])
            action_map_inv={0:'u',1:'j',2:'h',3:'k'}
            
            if(action_map_inv[ind] not in self.get_legal_dirs()):
                dirs=np.random.choice(self.get_legal_dirs(),1)
               
                action_map={'u':0,'j':1,'h':2,'k':3}
                return action_map[dirs[0]]
            
            return ind
            
            
        
        else:
            print('Exploring Off policy')
            dirs=np.random.choice(self.get_legal_dirs(),1)
            
            action_map={'u':0,'j':1,'h':2,'k':3}
            
            return action_map[dirs[0]]
    
    def policy (self, state):
        return self.egreedy(self.epsilon, self.numactions, self.actionvalues(state))

    def agentLearn (self, a, r): #default is one step q
        s=self.mapper[self.prev_state]
        sprime=self.mapper[self.current_state]
        self.Q[s,a] += self.alpha * (r + (self.gamma * self.statevalue(sprime)) - self.Q[s,a]) 

    def get_task(self,dir):

        orien=(self.orientation%360)/90

        if(dir=='u'):
            return self.allowable[0][int(orien)]
        if(dir=='j'):
            return self.allowable[1][int(orien)]
        if(dir=='h'):
            return self.allowable[2][int(orien)]
        if(dir=='k'):
            return self.allowable[3][int(orien)]



def run(sdk_conn):
    
    robot = sdk_conn.wait_for_robot()
    
    #robot.go_to_pose(Pose(200, 200, 0,angle_z=degrees(90)), relative_to_robot=False).wait_for_completed()
    #print (robot.pose)
    
    tasks = [0,0,90,0,270,0,0]
    field = Environment(5,5)
    bot = Agent((0,0),field)

    print('enter:')

    

    executed_moves=[]
    movesr=[]
    rflag=True
    poser=robot.pose

    l='m'
    
    while(True):
        
        if l=='q':
                vals=np.argmax(bot.Q,axis=1)
                for i in range(25):
                    print(action_map_inv[vals[i]]),
                    print('\t'),
                    if(i%5==4):
                        print('\n')
                l='m'
                continue


        if l=='c':
                plt.plot(movesr)
                plt.show()
                l='m'
                continue
        if  l=="r" :
                rflag=True        
        
        moves=bot.get_legal_moves()
        

        

        moves.extend(['p','l','o','c','q'])
        ind=bot.agentChoose()
        action_map_inv={0:'u',1:'j',2:'h',3:'k'}


        #print(ind)
        #print('chose move is ', action_map_inv[ind])
        if l=='m':
            task=input()
            if(task=='r'):
               rflag=True
               l='a'
        if l=='a':
            task_policy=bot.agentChoose()
        #print(task_policy)
            task = action_map_inv[task_policy]
            print (task)
        
        #task=np.random.choice(['u','h','j','k'],1)
        #task=task[0]
        
        
        action=task
        #print(action)
    #for i in tasks:    

        if task in ['u','h','j','k']:
            task=bot.get_task(task)

        if task not in moves:
            print('Illegal Move')
            #robot.play_anim_trigger(cozmo.anim.Triggers.DriveEndAngry).wait_for_completed()
            #print('Try',moves)
            continue

        executed_moves.append(task)
        

        bot.move_count+=1
        

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

        elif(task=='l'):
            print(bot.get_legal_dirs())

        elif(task=='o'):
            print(bot.orientation)

        elif(task=='c'):
            print(bot.current_state)

        elif(task=='q'):
            print(bot.Q)
        bot.print_map()
        #print(bot.Q,"Qmat")
        if(rflag):
            robot.turn_in_place(degrees(i)).wait_for_completed()
            robot.drive_straight(distance_mm(50),speed_mmps(200)).wait_for_completed()

        #if(bot.map[4][4]==1):
        #    robot.set_all_backpack_lights(cozmo.lights.blue_light)
        #bot.print_map()
        
        if(action in ['u','j','h','k']):
        
            action_map={'u':0,'j':1,'h':2,'k':3}
            selact=action_map[action]
        
            if(bot.current_state==(2,2) or bot.current_state==(4,0)):
                r=-10

            elif(bot.current_state==(4,4)):
                r=10
                
            elif(bot.current_state==(3,1)):
                r=3

            else:
                r=0
            
            bot.agentLearn(selact,r)
        
        
        
        if(bot.current_state[0]==4 and bot.current_state[1]==4):
            if(rflag):
            
                 robot.set_all_backpack_lights(cozmo.lights.blue_light)
                 robot.play_anim_trigger(cozmo.anim.Triggers.MajorWin).wait_for_completed()
                 robot.go_to_pose(poser, relative_to_robot=False).wait_for_completed()
            bot.print_map()
            
            #print(poser)
            #print(robot.pose)
            bot.current_state=(0,0)
            
            bot.orientation=0
            #print(executed_moves)
            executed_moves=[]
            movesr.append(bot.move_count)
            bot.move_count=0
            bot.map=np.zeros((5,5))
            print(movesr[-1],"latest moves")
            l=input('Amber:')
            
            if(l=="r"):
                rflag=True
                l='a'
            if(l=="diasble"):
                rflag=False 
                l='a'                
                            
                    

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