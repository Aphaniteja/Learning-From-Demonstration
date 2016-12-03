class GridAgent (Agent):
    def __init__(self, numactions, numstates, epsilon=0.05, alpha=0.5, \
                 gamma=.9, initialvalue=0.1, agentlambda=0.8):
        Agent.__init__(self)
        self.alpha = alpha
        self.initialvalue = initialvalue
        self.gamma = gamma
        self.epsilon = epsilon
        self.agentlambda = agentlambda
        self.recentsensations = []
        self.recentactions = []
        self.numactions = numactions
        self.numstates = numstates
        self.Q = [[self.initialvalue for i in range(self.numactions)] \
                  for j in range(self.numstates)]
        self.savedq = [[self.initialvalue for i in range(self.numactions)] \
                  for j in range(self.numstates)]
        self.changedstates = []
 
    def agentStartEpisode (self, sensation):
        self.recentsensations = []
        self.recentactions = []

    def agentchangestate(self, s):
        if not s in  self.changedstates:
            self.changedstates.append(s)
    
    def actionvalues (self, s):
        return [self.Q[s][a] for a in range(self.numactions)]

    def statevalue (self, s):
        if s == None:
            return 0
        elif s == 'terminal':
            return 0
        else:
            return max(self.actionvalues(s))

    def policy (self, state):
        return egreedy(self.epsilon, self.numactions, self.actionvalues(state))
    
    def agentChoose (self, sprime):    # epsilon greedy
        self.recentsensations = [sprime] + self.recentsensations
        if sprime != 'terminal':
            self.recentactions = [self.policy(sprime)] + self.recentactions
            return self.recentactions[0]

    def agentLearn (self, s, a, reward, sprime): #default is one step q
        self.Q[s][a] += self.alpha * (r + \
                                        (self.gamma * self.statevalue(sprime)) \
                                        - self.Q[s][a])

    def agentInit(self):
        self.recentsensations = []
        self.recentactions = []
        pass

    def agentfn(self, verbose, s, r=None):
        global lasts, lasta
        simcollect(self.sim, lasts, lasta, r, s)
        if r != None:
            self.agentLearn(lasts, lasta, r, s)
        else:
            self.agentStartEpisode(s)
        if s != 'terminal':
            a = self.agentChoose(s) 
            lasts, lasta = s, a
            if verbose:
                print("Agent chose action", a)
            return a
        else:
            return None