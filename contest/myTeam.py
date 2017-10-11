# -*- coding: cp936 -*-
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



from captureAgents import CaptureAgent
import random, time, util
from util import nearestPoint
from game import Directions
import game

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'OffTopAgent', second = 'OffBotAgent'):
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

class CustomAgent(CaptureAgent):
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
    self.walls = gameState.getWalls()
    self.mapHeight = self.walls.height
    self.mapWidth = self.walls.width
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''


  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    actions = gameState.getLegalActions(self.index)
    actions.remove('Stop')
    

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
    return max([(self.evaluate(gameState, act), act) for act in actions])[1]

  def evaluate(self, gameState, action):

    offFeatures = self.getOffFeatures(gameState, action)
    offWeights = self.getOffWeights(gameState, action)
    defFeatures = self.getDefFeatures(gameState, action)
    defWeights = self.getDefWeights(gameState, action)
    
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()
    successor = gameState.generateSuccessor(self.index, action)
    myNextState = successor.getAgentState(self.index)
    myNextPos = myNextState.getPosition()
    
    foods = self.getFood(successor).asList()
    if len(foods)>2:
        dist_food, closest_food = min([(self.getMazeDistance(myNextPos, food), food) for food in foods])
    else:
        dist_food = 0
    
    if (int(myNextPos[0]),int(myNextPos[1])) in self.getFood(gameState).asList():
        dist_food = 0
    opponents = self.getOpponents(successor)
    oppStates = [successor.getAgentState(opponent) for opponent in opponents]
    oppPoses = [(state, state.getPosition()) for state in oppStates]
    visibleOpponents = [(self.getMazeDistance(myPos, pos), pos, state.isPacman) for state, pos in oppPoses if pos != None]
    d_opp, c_opp, p_opp = (0, None, None) if visibleOpponents == [] else min(visibleOpponents)
    
    walls = gameState.getWalls()
    midRange = []
    for i in range(1,walls.height):
        if not gameState.hasWall(walls.width/2,i):
            midRange+=[(walls.width/2,i)]
    
    """如果没有看到"""
    if visibleOpponents != []:
        for opp in visibleOpponents:
            if opp[2]: return defFeatures * defWeights

    return offFeatures * offWeights
  
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

  def getOffFeatures(self, gameState, action):
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()
    successor = gameState.generateSuccessor(self.index, action)
    myNextState = successor.getAgentState(self.index)
    myNextPos = myNextState.getPosition()

    foods = self.getFood(successor).asList()
    if len(foods)>2:
      dist_food, closest_food = min([(self.getMazeDistance(myNextPos, food), food) for food in foods]) 
    else:
      dist_food = 0

    if (int(myNextPos[0]),int(myNextPos[1])) in self.getFood(gameState).asList():
      dist_food = 0 
    opponents = self.getOpponents(successor)
    oppStates = [successor.getAgentState(opponent) for opponent in opponents]
    oppPoses = [(state, state.getPosition()) for state in oppStates]
    visibleOpponents = [(self.getMazeDistance(myPos, pos), pos, state.isPacman, state.scaredTimer) for state, pos in oppPoses if pos != None]
    d_opp, c_opp, p_opp, s_opp = (9999, None, None, None) if visibleOpponents == [] else min(visibleOpponents)
    # print [opp[1] for opp in visibleOpponents if opp[3] == 0]
    ghosts = [] if visibleOpponents == [] else [opp[1] for opp in visibleOpponents if opp[3] == 0]
    # print ghosts, visibleOpponents
    dist_ghosts = [self.getMazeDistance(myPos, ghost) for ghost in ghosts]


    walls = gameState.getWalls()
    midRange = []
    for i in range(1,walls.height):
      if not gameState.hasWall(walls.width/2,i):
        midRange+=[(walls.width/2,i)]


    features = util.Counter()
    
    if dist_ghosts == []:
      features['distToGhost'] = 0
      features['distToCap'] = 0

    capsulesPos = gameState.getBlueCapsules() if gameState.isOnRedTeam(self.index) else gameState.getRedCapsules()
    if (len(capsulesPos) > 0) and dist_ghosts != []:
      dis_cap = min([self.getMazeDistance(myNextPos, cap) for cap in capsulesPos])
      features['distToCap'] = dis_cap if dis_cap<d_opp else 0
    else:
      features['distToCap'] = 0



    if len(gameState.getLegalActions(self.index)) == 2 and (d_opp<=10 and d_opp!=0): deadend = 1
    else: deadend = 0

  
    features['numCarrying'] = myNextState.numCarrying
    features['distToFood'] = dist_food
    features['distToHome'] = min(self.getMazeDistance(myNextPos, mid) for mid in midRange)
    features['foodLeft'] = len(foods)
    features['gameScore'] = self.getScore(successor)
    features['deadend'] = deadend * (d_opp + 1)
    features['backhome'] = features['distToHome'] if (d_opp<=3 and d_opp!=0 ) else 0
    
    return features


  def getOffWeights(self, gameState, action):
    weights = util.Counter()

    weights['distToGhost'] = 1050
    weights['distToFood'] = -60
    weights['distToCap'] = -1100
    weights['distToHome'] = -10
    weights['numCarrying'] = -1
    weights['foodLeft'] = -700
    weights['gameScore'] = 5000
    weights['deadend'] = -800
    weights['backhome'] = -1000
    
    return weights
  
  def getDefFeatures(self, gameState, action):
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
        if ((len(invaders)==1) and (disToCloestInvader > 5) and (a.getPosition() != None for a in invaders)):
            invaderLoc = invaders[0].getPosition()
            foods = self.getFoodYouAreDefending(gameState).asList()
            invaderDistsToFood = [(self.getMazeDistance(invaderLoc, foodLoc), foodLoc) for foodLoc in foods]
            invaderToFood, targetLoc = min(invaderDistsToFood) if invaderDistsToFood != [] else None, invaders[0].getPosition()
            disToInvaderTarget = self.getMazeDistance(targetLoc, myPos)
            features['targetDistance'] = disToInvaderTarget
    else:
        walls = gameState.getWalls()
        midPoint = (walls.width/2, walls.height/2+random.choice(range(-3,3)))
        if gameState.hasWall(midPoint[0],midPoint[1]):
            disToMiddle = self.manhattanDistance(myPos, midPoint)
        else:
            disToMiddle = self.getMazeDistance(midPoint, myPos)
            features['targetDistance'] = disToMiddle
                                        
    if action == Directions.STOP: features['stop'] = 1
    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1
    return features

  def getDefWeights(self, gameState, action):
    
    if self.isScared(gameState, self.index):
        return {'numInvaders': -1000, 'onDefense': 100, 'targetDistance': -0.2*gameState.getAgentState(self.index).scaredTimer, 'stop': -10, 'reverse': -1}
    
    return {'numInvaders': -1000, 'onDefense': 100, 'targetDistance': -20, 'stop': -100, 'reverse': -2}

  def isScared(self, gameState, index):
    """
    Says whether or not the given agent is scared
    """
    isScared = bool(gameState.data.agentStates[index].scaredTimer)
    return isScared
  def manhattanDistance(self, xy1, xy2 ):
      "Returns the Manhattan distance between points xy1 and xy2"
      return abs( xy1[0] - xy2[0] ) + abs( xy1[1] - xy2[1] )

class OffTopAgent(CustomAgent):

  def getOffFeatures(self, gameState, action):
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()
    successor = gameState.generateSuccessor(self.index, action)
    myNextState = successor.getAgentState(self.index)
    myNextPos = myNextState.getPosition()

    all_foods = self.getFood(successor).asList()
    foods = [food for food in all_foods if food[1] > self.mapHeight/2]
    foods = foods if foods != [] else all_foods
    if len(foods)>2:
      dist_food, closest_food = min([(self.getMazeDistance(myNextPos, food), food) for food in foods]) 
    else:
      dist_food = 0

    if (int(myNextPos[0]),int(myNextPos[1])) in self.getFood(gameState).asList():
      dist_food = 0 
    opponents = self.getOpponents(successor)
    oppStates = [successor.getAgentState(opponent) for opponent in opponents]
    oppPoses = [(state, state.getPosition()) for state in oppStates]
    visibleOpponents = [(self.getMazeDistance(myPos, pos), pos, state.isPacman, state.scaredTimer) for state, pos in oppPoses if pos != None]
    d_opp, c_opp, p_opp, s_opp = (9999, None, None, None) if visibleOpponents == [] else min(visibleOpponents)
    # print [opp[1] for opp in visibleOpponents if opp[3] == 0]
    ghosts = [] if visibleOpponents == [] else [opp[1] for opp in visibleOpponents if opp[3] == 0]
    # print ghosts, visibleOpponents
    dist_ghosts = [self.getMazeDistance(myPos, ghost) for ghost in ghosts]


    walls = gameState.getWalls()
    midRange = []
    for i in range(1,walls.height):
      if not gameState.hasWall(walls.width/2,i):
        midRange+=[(walls.width/2,i)]


    features = util.Counter()
    
    if dist_ghosts == []:
      features['distToGhost'] = 0
      features['distToCap'] = 0

    capsulesPos = gameState.getBlueCapsules() if gameState.isOnRedTeam(self.index) else gameState.getRedCapsules()
    if (len(capsulesPos) > 0) and dist_ghosts != []:
      dis_cap = min([self.getMazeDistance(myNextPos, cap) for cap in capsulesPos])
      features['distToCap'] = dis_cap if dis_cap<d_opp else 0
    else:
      features['distToCap'] = 0



    if len(gameState.getLegalActions(self.index)) == 2 and (d_opp<=10 and d_opp!=0): deadend = 1
    else: deadend = 0

  
    features['numCarrying'] = myNextState.numCarrying
    features['distToFood'] = dist_food
    features['distToHome'] = min(self.getMazeDistance(myNextPos, mid) for mid in midRange)
    features['foodLeft'] = len(foods)
    features['gameScore'] = self.getScore(successor)
    features['deadend'] = deadend * (d_opp + 1)
    features['backhome'] = features['distToHome'] if (d_opp<=3 and d_opp!=0 ) else 0
    
    return features


class OffBotAgent(CustomAgent):

  def getOffFeatures(self, gameState, action):
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()
    successor = gameState.generateSuccessor(self.index, action)
    myNextState = successor.getAgentState(self.index)
    myNextPos = myNextState.getPosition()

    all_foods = self.getFood(successor).asList()
    foods = [food for food in all_foods if food[1] < self.mapHeight/2]
    foods = foods if foods != [] else all_foods
    if len(foods)>2:
      dist_food, closest_food = min([(self.getMazeDistance(myNextPos, food), food) for food in foods]) 
    else:
      dist_food = 0

    if (int(myNextPos[0]),int(myNextPos[1])) in self.getFood(gameState).asList():
      dist_food = 0 
    opponents = self.getOpponents(successor)
    oppStates = [successor.getAgentState(opponent) for opponent in opponents]
    oppPoses = [(state, state.getPosition()) for state in oppStates]
    visibleOpponents = [(self.getMazeDistance(myPos, pos), pos, state.isPacman, state.scaredTimer) for state, pos in oppPoses if pos != None]
    d_opp, c_opp, p_opp, s_opp = (9999, None, None, None) if visibleOpponents == [] else min(visibleOpponents)
    # print [opp[1] for opp in visibleOpponents if opp[3] == 0]
    ghosts = [] if visibleOpponents == [] else [opp[1] for opp in visibleOpponents if opp[3] == 0]
    # print ghosts, visibleOpponents
    dist_ghosts = [self.getMazeDistance(myPos, ghost) for ghost in ghosts]


    walls = gameState.getWalls()
    midRange = []
    for i in range(1,walls.height):
      if not gameState.hasWall(walls.width/2,i):
        midRange+=[(walls.width/2,i)]


    features = util.Counter()
    
    if dist_ghosts == []:
      features['distToGhost'] = 0
      features['distToCap'] = 0

    capsulesPos = gameState.getBlueCapsules() if gameState.isOnRedTeam(self.index) else gameState.getRedCapsules()
    if (len(capsulesPos) > 0) and dist_ghosts != []:
      dis_cap = min([self.getMazeDistance(myNextPos, cap) for cap in capsulesPos])
      features['distToCap'] = dis_cap if dis_cap<d_opp else 0
    else:
      features['distToCap'] = 0



    if len(gameState.getLegalActions(self.index)) == 2 and (d_opp<=10 and d_opp!=0): deadend = 1
    else: deadend = 0

  
    features['numCarrying'] = myNextState.numCarrying
    features['distToFood'] = dist_food
    features['distToHome'] = min(self.getMazeDistance(myNextPos, mid) for mid in midRange)
    features['foodLeft'] = len(foods)
    features['gameScore'] = self.getScore(successor)
    features['deadend'] = deadend * (d_opp + 1)
    features['backhome'] = features['distToHome'] if (d_opp<=3 and d_opp!=0 ) else 0
    
    return features