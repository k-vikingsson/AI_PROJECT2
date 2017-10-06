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
from game import Directions
import game
import time

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'OffensiveAgent', second = 'OffensiveAgent'):
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

class OffensiveAgent(CaptureAgent):
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
    self.firstMove = True
    self.targetFoods = self.getFood(gameState)
    # Q values
    self.QValue = util.Counter()
    CaptureAgent.registerInitialState(self, gameState)
    print self.targetFoods
    print ''


    ### LEARNING PARAMETERS ###
    self.epsilon = 0.2

    '''
    Your initialization code goes here, if you need any.
    '''
  ## reward function, tunable
  def reward(self, action, gameState):
    # agent current state
    mystate = gameState.getAgentState(self.index)
    # my next state
    nextGameState = gameState.generateSuccessor(self.index)
    myNextState = nextGameState.getAgentState(self.index)
    # food list
    foods = self.getFood(gameState).asList()
    nextFoods = self.getFood(nextGameState).asList()

    ## case: eat food
    if len(foods) - len(nextFoods) == 1:
      return 1/mystate.numCarrying
    ## case: lose foods
    if myNextState.configuration == mystate.start:
      return -10 * mystate.numCarrying
    ## case: return foods
    if mystate.isPacman and not myNextState.isPacman:
      return 10 * mystate.numCarrying

    return 0

  def learn(self, gameState):
    startTime = time.time()
    while time.time() - startTime <= 10:
      runEpisode(gameState)

  def runEpisode(gameState):
    


  def getQValue(self, gameState, action):
    return self.QValue[(gameState, action)]

  def computeValueFromQValues(self, gameState):
    maxValue = 0
    actions = gameState.getLegalActions(self.index)
    if len(actions) == 0: return 0
    val = -999
    for action in actions:
      val = max(val, self.getQValue(gameState, action))
    return val

  def computeActionFromQValues(self, gameState):
    bestVal = bestAction = None
    actions = gameState.getLegalActions(self.index)

    for action in actions:
      curVal = self.getQValue(gameState,action)
      if bestVal is None or bestVal < curVal:
        bestVal = curVal
        bestAction = action

    return bestAction

  def getAction(self, gameState):
    legalActions = gameState.getLegalActions(self.index)
    action = None
    "*** YOUR CODE HERE ***"
    explore = util.flipCoin(self.epsilon)
    if explore:
      return random.choice(legalActions)
    else:
      return self.computeActionFromQValues(gameState)
    return action

  def update(self, state, action, nextState, reward):
    self.QValue[(state,action)] = (1-self.alpha)*self.getQValue(state,action)+ self.alpha*(reward+self.discount*self.getValue(nextState))




  def chooseAction(self, gameState):
    actions = gameState.getLegalActions(self.index)

    '''
    You should change this in your own agent.
    '''

    return random.choice(actions)


# class DefensiveAgent(CaptureAgent):
#   """
#   A Dummy agent to serve as an example of the necessary agent structure.
#   You should look at baselineTeam.py for more details about how to
#   create an agent as this is the bare minimum.
#   """

#   def registerInitialState(self, gameState):
#     """
#     This method handles the initial setup of the
#     agent to populate useful fields (such as what team
#     we're on).

#     A distanceCalculator instance caches the maze distances
#     between each pair of positions, so your agents can use:
#     self.distancer.getDistance(p1, p2)

#     IMPORTANT: This method may run for at most 15 seconds.
#     """

#     '''
#     Make sure you do not delete the following line. If you would like to
#     use Manhattan distances instead of maze distances in order to save
#     on initialization time, please take a look at
#     CaptureAgent.registerInitialState in captureAgents.py.
#     '''
#     self.start = gameState.getPacmanPosition()
#     self.first_move = True
#     CaptureAgent.registerInitialState(self, gameState)

#     '''
#     Your initialization code goes here, if you need any.
#     '''


#   def chooseAction(self, gameState):
#     """
#     Picks among actions randomly.
#     """
#     actions = gameState.getLegalActions(self.index)

#     '''
#     You should change this in your own agent.
#     '''

#     return random.choice(actions)
