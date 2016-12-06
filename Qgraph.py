import sys

#import cozmo
#from cozmo.util import degrees, Pose, distance_mm, speed_mmps
import numpy as np
#import tkinter as tk
import matplotlib.pyplot as plt
#import seaborn as sns
#sns.set(color_codes=True)
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
    def __init__(self,start_state,environment,epsilon=0.9,rewardmax=10):
        
        self.current_state = start_state
        self.prev_state=start_state
        self.orientation = 0
        self.map = environment.states
        self.height = self.map.shape[0]
        self.width = self.map.shape[1]
        self.map[0][0]=1
        self.move_count=0
        self.environment=environment
        self.epsilon=epsilon
        self.rewardmax=rewardmax
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
        if(np.random.rand()<self.epsilon):
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



def run(no_of_runs,epsilon,rewardmax,demon=False,Q=[]):
    
    
    #robot.go_to_pose(Pose(200, 200, 0,angle_z=degrees(90)), relative_to_robot=False).wait_for_completed()
    #print (robot.pose)
    
    tasks = [0,0,90,0,270,0,0]
    field = Environment(5,5)
    bot = Agent((0,0),field,epsilon,rewardmax)
    k=0
    moves_like_jagger=[]
    if (demon):
       bot.Q=np.load("lfd5.npy")
       input=()
    if(len(Q)!=0):
       bot.Q=Q
       input=()
        
    print('enter:')

    

    executed_moves=[]
    movesr=[]
    l=1
    
    while(True):
        
        if l=='q':
                vals=np.argmax(bot.Q,axis=1)
                for i in range(25):
                    print(action_map_inv[vals[i]]),
                    print('\t'),
                    if(i%5==4):
                        print('\n')
                l=1
                continue
                
            
        moves=bot.get_legal_moves()
        

        

        moves.extend(['p','l','o','c','q'])
        ind=bot.agentChoose()
        action_map_inv={0:'u',1:'j',2:'h',3:'k'}
        #print(ind)
        #print('chose move is ', action_map_inv[ind])
        
        task_policy=bot.agentChoose()
        #print(task_policy)
        task = action_map_inv[task_policy]
        
        #task=np.random.choice(['u','h','j','k'],1)
        #task=task[0]
        
        #task=raw_input()
        
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


        #robot.turn_in_place(degrees(i)).wait_for_completed()
        #robot.drive_straight(distance_mm(50),speed_mmps(200)).wait_for_completed()

        #if(bot.map[2][2]==1):
        #    robot.set_all_backpack_lights(cozmo.lights.blue_light)
        #bot.print_map()
        
        if(action in ['u','j','h','k']):
        
            action_map={'u':0,'j':1,'h':2,'k':3}
            selact=action_map[action]
        
            if(bot.current_state==(2,2) or bot.current_state==(4,0)):
                r=-10

            elif(bot.current_state==(4,4)):
                r=bot.rewardmax

            else:
                r=0
            
            bot.agentLearn(selact,r)
        
        
        
        if(bot.current_state[0]==4 and bot.current_state[1]==4):
            #robot.set_all_backpack_lights(cozmo.lights.blue_light)
            #robot.play_anim_trigger(cozmo.anim.Triggers.MajorWin).wait_for_completed()
            #bot.print_map()
            #robot.go_to_pose(poser, relative_to_robot=False).wait_for_completed()
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
            if (len(movesr)>5):
               if(movesr[-1]==movesr[-2]==movesr[-3]==movesr[-4]==movesr[-5]):
                    print(len(movesr),"Movesr")
                    k=k+1
                    moves_like_jagger.append(len(movesr))
                    plt.plot(movesr)
                    plt.xlabel("No of episodes")
                    plt.ylabel("no of moves required to reach a goal")
                    
                    movesr=[]
            if(k>=no_of_runs):
               #print (moves_like_jagger,sum(moves_like_jagger)/len(moves_like_jagger))
               #plt.plot(moves_like_jagger)
               #plt.savefig('foof.png')
               #plt.show()
               #sns.lmplot(x="runs", y="No of episodes to converge", data=moves_like_jagger);
               plt.savefig("Decay curve")
               return(moves_like_jagger,sum(moves_like_jagger)/len(moves_like_jagger),bot.Q)
            #l=input()
            
                        
                    

        #print(bot.get_legal_moves())
        #print(bot.orientation)
        #print(bot.current_state)


if __name__ == '__main__':
     #cozmo.setup_basic_logging()
        
     try:
        epsilons=[0.7,0.8,0.9]
        no_of_runs=1
        
        jagger=[]
        strs=[]
        r=10
        ms=[]
        Q=[]
        for e in epsilons:
            j,m,Q=run(100,e,r,False,Q)
            jagger.append(j)
            ms.append(m)
        for i in range(len(epsilons)):
                   print ("for epsilon =",epsilons[i],"mean = ",ms[i] )       
        for l,j in enumerate(jagger):
            plt.plot(j,label=str(epsilons[l]))        
        plt.xlabel("no of runs")
        plt.ylabel("no of episodes to converge")
        plt.savefig('Comparision-lfd4.png')
                   
                

     except KeyboardInterrupt as e:
        sys.exit("Interrupted")