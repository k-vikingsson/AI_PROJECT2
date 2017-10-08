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
    self.weights = {}
    self.weights['distToFood'] = -1
    self.weights['distToGhost'] = 10
    self.weights['distToHome'] = -1
    self.weights['numCarrying'] = -1
    self.weights['foodLeft'] = 1
    self.weights['gameScore'] = 1
    self.start = gameState.getAgentPosition(self.index)
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
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()

    # find closest food
    capss = self.getCapsules(gameState)
    # print capss
    # dist_caps, closest_caps = 0, None if capss == [] else min([(self.getMazeDistance(myPos, caps), caps) for caps in capss])
    foods = self.getFood(gameState).asList()
    # dist_food, closest_food = 0, None if foods == [] else min([(self.getMazeDistance(myPos, food), food) for food in foods])


    opponents = self.getOpponents(gameState)
    oppStates = [gameState.getAgentState(opponent) for opponent in opponents]
    oppPoses = [(state, state.getPosition()) for state in oppStates]
    visibleOpponents = [(self.getMazeDistance(myPos, pos), pos, state.isPacman, state.scaredTimer) for state, pos in oppPoses if pos != None]
    d_opp, c_opp, p_opp, s_opp = (9999, None, None, None) if visibleOpponents == [] else min(visibleOpponents)
    # print [opp[1] for opp in visibleOpponents if opp[3] == 0]
    ghosts = [] if visibleOpponents == [] else [opp[1] for opp in visibleOpponents if opp[3] == 0]
    # print ghosts, visibleOpponents
    dist_ghosts = [self.getMazeDistance(myPos, ghost) for ghost in ghosts]
    # print d_opp, c_opp, p_opp
    
    if capss != [] and ghosts != []: (dist_food, closest_food) = self.chooseTarget(myPos, capss, ghosts)
    elif foods != []: (dist_food, closest_food) = self.chooseTarget(myPos, foods, ghosts)
    else: return self.decideMove(gameState, actions, self.start)

    if closest_food == None:
      print "None", foods, ghosts, dist_food

    # print dist_food, closest_food

    if myState.numCarrying != 0 and [dist for dist in dist_ghosts if dist < 10] != []:
      # if dist_food < d_opp: return self.decideMove(gameState, actions, closest_food)
      return self.decideMove(gameState, actions, self.start)

    # evade ghost if required
    if dist_ghosts != [] and d_opp <= 5:
      return self.decideMove(gameState, actions, ghosts[0], False)
      # elif p_opp and not myState.isPacman: # try to get nearest pacman
      #   return self.decideMove(gameState, actions, c_opp)

    # if gameState.data.timeleft/4 <= self.getMazeDistance(myPos, self.start)/2:
    #   return self.decideMove(gameState, actions, self.start)
    return self.decideMove(gameState, actions, closest_food)


  def chooseTarget(self, myPos, targets, ghosts):
    if ghosts == []: 
      return (0, None) if targets == [] else min([(self.getMazeDistance(myPos, target), target) for target in targets])
    arrangedTargets = []
    for target in targets:
      weightedDistance = self.getMazeDistance(myPos, target)
      for ghost in ghosts:
        weightedDistance -= self.getMazeDistance(ghost, target) - self.getMazeDistance(myPos, ghost)
      arrangedTargets.append((weightedDistance, target))
    return (0, None) if arrangedTargets == [] else min(arrangedTargets)

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


    features = {}
    features['numCarrying'] = self.numCarrying

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

    '''
    Your initialization code goes here, if you need any.
    '''

  """def getFeatures(self, gameState, action):



  def fetWeights(self, gameState, action):
  """


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
        if ((len(invaders)==1) and (disToCloestInvader > 5) and (a.getPosition() != None for a in invaders)):
          invaderLoc = invaders[0].getPosition()
          foods = self.getFoodYouAreDefending(gameState).asList()
          if foods != []:
            invaderToFood, targetLoc = min([(self.getMazeDistance(invaderLoc, foodLoc), foodLoc) for foodLoc in foods])
            disToInvaderTarget = self.getMazeDistance(targetLoc, myPos)
            features['targetDistance'] = disToInvaderTarget
    else:
      walls = gameState.getWalls()
      disToMiddle = self.getMazeDistance(myPos, (walls.width/2,walls.height/2))
      features['targetDistance'] = disToMiddle

    if action == Directions.STOP: features['stop'] = 1
    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1

    return features

  def getWeights(self, gameState, action):

    if self.isScared(gameState, self.index):
      return {'numInvaders': -1000, 'onDefense': 100, 'targetDistance': -0.2*gameState.getAgentState(self.index).scaredTimer, 'stop': -10, 'reverse': -1}

    return {'numInvaders': -1000, 'onDefense': 100, 'targetDistance': -10, 'stop': -100, 'reverse': -2}

  def isScared(self, gameState, index):
        """
        Says whether or not the given agent is scared
        """
        isScared = bool(gameState.data.agentStates[index].scaredTimer)
        return isScared
