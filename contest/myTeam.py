# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

import collections

from captureAgents import CaptureAgent
import random, time, util
from util import nearestPoint
from game import Directions, Actions

import game

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'OffAgent', second = 'DefAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class OffAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    
    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)
    self.prevactions = []
    self.randomcounter = 5
    '''
    Your initialization code goes here, if you need any.
    '''


  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    actions = gameState.getLegalActions(self.index)
    #actions.remove('Stop')
    

    # ## find closest food
    # foods = self.getFood(gameState).asList()
    # dist_food, closest_food = min([(self.getMazeDistance(myPos, food), food) for food in foods])

    # if myState.numCarrying != 0:
    #   if dist_food <= 2: return self.decideMove(gameState, actions, closest_food)
    #   else: return self.decideMove(gameState, actions, self.start)


    
    # # evade ghost if required
    # if c_opp != None:
    #   if not p_opp:
    #     return self.decideMove(gameState, actions, c_opp, False)
    #   elif p_opp and not myState.isPacman: # try to get nearest pacman
    #     return self.decideMove(gameState, actions, c_opp)

    # return self.decideMove(gameState, actions, closest_food)
    score = self.getScore(gameState)
    action = max([(self.evaluate(gameState, act), act) for act in actions])[1]
    self.prevactions.insert(0,action)
    self.prevactions = self.prevactions if len(self.prevactions)<7 else self.prevactions[:6]

    if (len(self.prevactions)>5 and self.prevactions[0]==self.prevactions[2] and self.prevactions[0]==self.prevactions[4] and self.prevactions[1]==self.prevactions[3] and self.prevactions[1]==self.prevactions[5] and self.prevactions[0]!=self.prevactions[1] and score<=0):
      #print("random~~")
      actions.remove(self.prevactions[0])
      actions.remove('Stop')
      self.randomcounter = 4
      if len(actions)>=2:
        if self.prevactions[0] in actions:
          actions.remove(self.prevactions[0])
        self.lastRandom = random.choice(actions)    # bug!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      else:
        self.lastRandom = self.prevactions[0]
      return self.lastRandom
    if self.randomcounter != 5 and self.randomcounter!=0:
      if self.lastRandom in actions:
        self.randomcounter -= 1
        return self.lastRandom
      else:
        self.randomcounter = 5
    elif(self.randomcounter==0):
      self.randomcounter = 5
      
    if action in actions:
      return action
    return random.choice(action)

  def evaluate(self, gameState, action):
    features = self.getFeatures(gameState, action)
    weights = self.getWeights(gameState, action)
    return features * weights

  def decideMove(self, gameState, actions, target, approach=True):
    suitableMove = None
    distance = 0
    for act in actions:
      successor = gameState.generateSuccessor(self.index, act)
      myNextState = successor.getAgentState(self.index)
      myNextPos = myNextState.getPosition()
      thisDist = self.getMazeDistance(myNextPos, target)
      if approach: moreSuitable = thisDist < distance
      else: moreSuitable = thisDist > distance
      if suitableMove == None or moreSuitable:
        suitableMove = act
        distance = thisDist
    return suitableMove

  def getFeatures(self, gameState, action):
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()
    successor = gameState.generateSuccessor(self.index, action)
    myNextState = successor.getAgentState(self.index)
    myNextPos = myNextState.getPosition()

    foods = self.getFood(successor).asList()
    if len(foods)>=3:
      dist_food, closest_food = min([(self.getMazeDistance(myNextPos, food), food) for food in foods]) 
    else:
      dist_food = 0

    if (int(myNextPos[0]),int(myNextPos[1])) in self.getFood(gameState).asList():
      dist_food = 0 
    opponents = self.getOpponents(successor)
    oppStates = [successor.getAgentState(opponent) for opponent in opponents]
    oppPoses = [(state, state.getPosition()) for state in oppStates]
    visibleOpponents = [(self.getMazeDistance(myNextPos, pos), pos, state.isPacman) for state, pos in oppPoses if pos != None]
    d_opp, c_opp, p_opp = (999, None, None) if visibleOpponents == [] else min(visibleOpponents)

    walls = gameState.getWalls()
    midRange = []
    for i in range(1,walls.height):
      widthNum = walls.width/2-1 if self.red else walls.width/2
      if not gameState.hasWall(widthNum,i):
        midRange+=[(widthNum,i)]


    features = util.Counter()
    
    capsulesPos = gameState.getBlueCapsules() if gameState.isOnRedTeam(self.index) else gameState.getRedCapsules()
    if (len(capsulesPos) > 0) and (d_opp!=0) and (d_opp<8) :
      dis_cap = min([self.getMazeDistance(myNextPos, cap) for cap in capsulesPos])
      features['distToCap'] = dis_cap if dis_cap<d_opp else 0
    else:
      features['distToCap'] = 0

    features['distToGhost'] = 0 if  p_opp else d_opp



    if len(gameState.getLegalActions(self.index)) == 2 and (d_opp<=10 and d_opp!=0): deadend = 1
    else: deadend = 0

  
    
    features['distToFood'] = dist_food
    features['distToHome'] = min(self.getMazeDistance(myNextPos, mid) for mid in midRange)
    features['foodLeft'] = len(foods)
    features['gameScore'] = self.getScore(successor)
    features['deadend'] = deadend * (d_opp + 1)
    features['numCarrying'] = myNextState.numCarrying
    features['backhome'] = features['distToHome'] if (d_opp<=3) else 0

    otherTeamIndex = gameState.getBlueTeamIndices() if gameState.isOnRedTeam(self.index) else gameState.getRedTeamIndices()
    closestGIndex = otherTeamIndex[0]
    if visibleOpponents!=[]:
      closestGDist , closestGIndex = min([(self.getMazeDistance(gameState.getAgentState(j).getPosition(),myNextPos),j) for j in otherTeamIndex if gameState.getAgentState(j).getPosition()!=None])

    if self.isScared(gameState, closestGIndex):
      capTimeLeft = gameState.getAgentState(closestGIndex).scaredTimer
      features['distToCap'] = 0
      features['distToGhost'] = 0 
      if capsulesPos>5:
        features['backhome'] = 0
      if capTimeLeft<=3 and not p_opp:
        features['distToGhost'] = d_opp



    if (features['distToGhost']>=3):
      features['distToGhost'] = 0 

    #features['tooMuchFood'] = features['numCarrying']*features["distToHome"]*features["distToHome"]
    #features['distToFood'] = dist_food/(features['numCarrying']-10) if (features['numCarrying']>10) else dist_food

    if gameState.data.timeleft/4 <= features['distToHome']+2:
      features['backhome'] = features["distToHome"]*1.5
      features['distToFood'] = 0

    if (features['numCarrying']>1 and features['distToHome']<2) or (features['numCarrying']>3 and features['distToHome']<3) or (features['numCarrying']>5 and features['distToHome']<4) or (features['numCarrying']>8 and features['distToHome']<10)or (features['numCarrying']>12 and features['distToHome']<20):
      features['distToFood'] = 0

    return features


  def getWeights(self, gameState, action):
    weights = util.Counter()

    weights['distToGhost'] = 1050
    weights['distToFood'] = -60
    weights['distToCap'] = -1010
    weights['distToHome'] = -10
    weights['numCarrying'] = -10
    weights['foodLeft'] = -700
    weights['gameScore'] = 5000
    weights['deadend'] = -800
    weights['backhome'] = -1000
    #weights['tooMuchFood'] = -1

    return weights

  def isScared(self, gameState, index):
    """
    Says whether or not the given agent is scared
    """
    isScared = bool(gameState.data.agentStates[index].scaredTimer)
    return isScared




class DefAgent(CaptureAgent):

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)
    self.bottleNecks, self.numDots = self.findBottleneckWithMostPacdots(gameState)

    '''
    Your initialization code goes here, if you need any.
    '''


  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """
    actions = gameState.getLegalActions(self.index)

    # You can profile your evaluation time by uncommenting these lines
    #start = time.time()
    values = [self.evaluate(gameState, a) for a in actions]
    #print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

    maxValue = max(values)
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]



    foodLeft = len(self.getFood(gameState).asList())

    if foodLeft <= 2:
      bestDist = 9999
      for action in actions:
        successor = self.getSuccessor(gameState, action)
        pos2 = successor.getAgentPosition(self.index)
        dist = self.getMazeDistance(self.start,pos2)
        if dist < bestDist:
          bestAction = action
          bestDist = dist
      return bestAction

    return random.choice(bestActions)

  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

  def evaluate(self, gameState, action):
    """
    Computes a linear combination of features and feature weights
    """
    features = self.getFeatures(gameState, action)
    weights = self.getWeights(gameState, action)
    return features * weights

  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)

    myState = successor.getAgentState(self.index)
    myPos = myState.getPosition()

    # Computes whether we're on defense (1) or offense (0)
    features['onDefense'] = 1
    if myState.isPacman: features['onDefense'] = 0

    # Computes distance to invaders we can see
    enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)] 
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    features['numInvaders'] = len(invaders)

    

    if len(invaders) > 0: 
        dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
        disToCloestInvader = min(dists)
        features['targetDistance'] = disToCloestInvader
        #game reconition techniques;
        if ((len(invaders)==1) and (disToCloestInvader < 10) and (a.getPosition() != None for a in invaders)):
          invaderLoc = invaders[0].getPosition()
          foods = self.getFoodYouAreDefending(gameState).asList()
          invaderToFood, targetLoc = min([(self.getMazeDistance(invaderLoc, foodLoc), foodLoc) for foodLoc in foods])
          disToInvaderTarget = self.getMazeDistance(targetLoc, myPos)
          features['targetDistance'] = disToInvaderTarget
          if disToInvaderTarget >=2 and self.isScared(gameState, self.index):
            features['targetDistance'] = -disToInvaderTarget
    elif(self.getPreviousObservation()!=None):
        distsToB = [self.getMazeDistance(myPos, self.bottleNecks)]
        disToCloestB = min(distsToB)
        features['targetDistance'] = disToCloestB
    else:
        walls = gameState.getWalls()
        midPoint = (walls.width/2, walls.height/2)
        midPoints = [(midPoint[0],midPoint[1]+2),(midPoint[0],midPoint[1]+1),(midPoint[0],midPoint[1]-1),(midPoint[0]-1,midPoint[1]),midPoint]
        hasWall = False
        target = (walls.width/2, walls.height/2)
        for midpoint in midPoints:
            if not (gameState.hasWall(midpoint[0],midpoint[1])):
                notWall = True
                target = midpoint
        if notWall:
            disToMiddle = self.getMazeDistance(target, myPos)
        else:
             disToMiddle = self.getManhattanDistance(target, myPos)
        features['targetDistance'] = disToMiddle

    if action == Directions.STOP: features['stop'] = 1
    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1

    return features

  def getWeights(self, gameState, action):

    if self.isScared(gameState, self.index):
      return {'numInvaders': -1000, 'onDefense': 100, 'targetDistance': -0.5*(gameState.getAgentState(self.index).scaredTimer), 'stop': -10, 'reverse': -1}

    return {'numInvaders': -1000, 'onDefense': 100, 'targetDistance': -30, 'stop': -100, 'reverse': -2}

  def isScared(self, gameState, index):
        """
        Says whether or not the given agent is scared
        """
        isScared = bool(gameState.data.agentStates[index].scaredTimer)
        return isScared

  def manhattanDistance(self, xy1, xy2 ):
    "Returns the Manhattan distance between points xy1 and xy2"
    return abs( xy1[0] - xy2[0] ) + abs( xy1[1] - xy2[1] )

  def getFlowNetwork(self, gameState, startingPositions=None, endingPositions=None, defenseOnly=True):
    '''
    Returns the flow network.
    If starting positions are provided, also returns the source node
    If ending positions are provided, also returns the sink node
    Note: Always returns tuple
    '''
    source = (-1, -1)
    sink = (-2, -2)

    walls = gameState.getWalls()
    wallPositions = walls.asList()
    midPos = range(walls.width/4,walls.width/2) if self.red else range(walls.width/2,3*walls.width/4)
    possiblePositions = [(x, y) for x in midPos for y in range(walls.height) if (x, y) not in wallPositions and (not defenseOnly or self.positionIsHome((x, y), walls.width))]

    actions = [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]
    actionVectors = [Actions.directionToVector(action) for action in actions]
    # Change vectors from float to int
    actionVectors = [tuple(int(number) for number in vector) for vector in actionVectors]

    # Make source and sink

    network = FlowNetwork()

    # Add all vertices
    for position in possiblePositions:
        network.AddVertex(position)
    network.AddVertex(source)
    network.AddVertex(sink)

    # Add normal edges
    edges = EdgeDict()
    for position in possiblePositions:
        for vector in actionVectors:
            newPosition = (position[0] + vector[0], position[1] + vector[1])
            if newPosition in possiblePositions:
                edges[(position, newPosition)] = 1

    # Add edges attached to source
    for position in startingPositions or []:
        edges[(source, position)] = float('inf')

    for position in endingPositions or []:
        edges[(position, sink)] = float('inf')

    for edge in edges:
        network.AddEdge(edge[0], edge[1], edges[edge])

    retval = (network,)

    if startingPositions is not None:
        retval = retval + (source,)
    if endingPositions is not None:
        retval = tuple(retval) + (sink,)

    return retval

  def findBottleneckWithMostPacdots(self, gameState):

      startingPositions = self.getMiddlePositions(gameState)
      endingPositions = self.getFoodYouAreDefending(gameState).asList()
      network, source = self.getFlowNetwork(gameState, startingPositions=startingPositions)

      bottleneckCounter = collections.Counter()

      for dot in endingPositions:
          bottlenecks = network.FindBottlenecks(source, dot)
          if len(bottlenecks) == 1:
              bottleneckCounter[bottlenecks[0]] += 1
          network.reset()

      maxBottleneck = max(bottleneckCounter or [None], key=lambda vertex: bottleneckCounter[vertex])
      return maxBottleneck, bottleneckCounter[maxBottleneck]

  def getMiddlePositions(self, gameState):

      # Find the positions closest to the moiddle line so we can start there
      walls = gameState.getWalls()
      wallPositions = walls.asList()
      possiblePositions = [(x, y) for x in range(walls.width) for y in range(walls.height) if (x, y) not in wallPositions and self.positionIsHome((x, y), walls.width)]
      startX = walls.width / 2 - 1 if self.red else walls.width / 2
      startingPositions = [position for position in possiblePositions if position[0] == startX]
      return startingPositions

  def positionIsHome(self, position, gameWidth):
      isHome = not (self.red ^ (position[0] < gameWidth / 2))
      return isHome


#=================================================================================================
# ### Implementation of Ford-Fulkerson algorithm, taken from https://github.com/bigbighd604/Python/blob/master/graph/Ford-Fulkerson.py and heavily modified

class Edge(object):
    def __init__(self, u, v, w):
        self.source = u
        self.target = v
        self.capacity = w

    def __repr__(self):
        return "%s->%s:%s" % (self.source, self.target, self.capacity)

    def __eq__(self, other):
        return self.source == other.source and self.target == other.target


class FlowNetwork(object):
    def __init__(self):
        self.adj = {}
        self.flow = {}

    def AddVertex(self, vertex):
        self.adj[vertex] = []

    def GetEdges(self, v):
        return self.adj[v]

    def AddEdge(self, u, v, w=0):
        if u == v:
            raise ValueError("u == v")
        edge = Edge(u, v, w)
        redge = Edge(v, u, w)
        edge.redge = redge
        redge.redge = edge
        self.adj[u].append(edge)
        self.adj[v].append(redge)
        # Intialize all flows to zero
        self.flow[edge] = 0
        self.flow[redge] = 0

    def FindPath(self, source, target):

        currentVertex, currentPath, currentTotal = source, [], 0
        # Priority queue uses the maze distance between the entered point and its closest goal position to decide which comes first
        queue = util.PriorityQueueWithFunction(lambda entry: entry[2] + util.manhattanDistance(entry[0], target))

        visited = set()

        # Keeps track of visited positions
        while currentVertex != target:

            possibleVertices = [(edge.target, edge) for edge in self.GetEdges(currentVertex)]

            for vertex, edge in possibleVertices:
                residual = edge.capacity - self.flow[edge]
                if residual > 0 and not (edge, residual) in currentPath and (edge, residual) not in visited:
                    visited.add((edge, residual))
                    queue.push((vertex, currentPath + [(edge, residual)], currentTotal + 1))

            if queue.isEmpty():
                return None
            else:
                currentVertex, currentPath, currentTotal = queue.pop()

        return currentPath

    def FindBottlenecks(self, source, target):
        maxflow, leadingEdges = self.MaxFlow(source, target)
        paths = leadingEdges.values()

        bottlenecks = []
        for path in paths:
            for edge, residual in path:
                # Save the flows so we don't mess up the operation between path findings
                if self.FindPath(source, edge.target) is None:
                    bottlenecks.append(edge.source)
                    break
        #assert len(bottlenecks) == maxflow
        return bottlenecks

    def MaxFlow(self, source, target):
        # This keeps track of paths that go to our destination
        leadingEdges = {}
        path = self.FindPath(source, target)
        while path:
            leadingEdges[path[0]] = path
            flow = min(res for edge, res in path)
            for edge, res in path:
                self.flow[edge] += flow
                self.flow[edge.redge] -= flow

            path = self.FindPath(source, target)
        maxflow = sum([self.flow[edge] for edge in self.GetEdges(source)])
        return maxflow, leadingEdges

    def reset(self):
        for edge in self.flow:
            self.flow[edge] = 0


class EdgeDict(dict):
    '''
    Keeps a list of undirected edges. Doesn't matter what order you add them in.
    '''
    def __init__(self, *args, **kwargs):
        dict.__init__(self, *args, **kwargs)

    def __getitem__(self, key):
        return dict.__getitem__(self, tuple(sorted(key)))

    def __setitem__(self, key, val):
        return dict.__setitem__(self, tuple(sorted(key)), val)

    def __contains__(self, key):
        return dict.__contains__(self, tuple(sorted(key)))

    def getAdjacentPositions(self, key):
        edgesContainingKey = [edge for edge in self if key in edge]
        adjacentPositions = [[position for position in edge if position != key][0] for edge in edgesContainingKey]
        return adjacentPositions

