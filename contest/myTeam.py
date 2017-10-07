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

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'DefensiveAgent', second = 'DefensiveAgent'):
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

class DefensiveAgent(CaptureAgent):
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
    self.first_move = True
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''


  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    actions = gameState.getLegalActions(self.index)
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()

    ## find closest food
    foods = self.getFood(gameState).asList()
    closest_food = None
    dist_food = 0
    for food in foods:
      thisDist = self.getMazeDistance(myPos, food)
      if closest_food == None or thisDist < dist_food:
        closest_food = food
        dist_food = thisDist

    if myState.numCarrying != 0:
      if dist_food <= 2: pass
      else: return self.decideMove(gameState, actions, self.start)

    opponents = self.getOpponents(gameState)
    closer_opponent = None
    pacman_opponent = False
    dist_opponent = 0
    for opponent in opponents:
      opponentState = gameState.getAgentState(opponent)
      opponentPosition = opponentState.getPosition()
      opponentPacman = opponentState.isPacman
      if opponentPosition == None: continue
      thisDist = self.getMazeDistance(myPos, opponentPosition)
      if closer_opponent == None or thisDist < dist_opponent:
        closer_opponent = opponentPosition
        pacman_opponent = opponentPacman
        dist_opponent = thisDist
    # evade ghost if required
    if closer_opponent != None:
      if not pacman_opponent and myState.isPacman:
        return self.decideMove(gameState, actions, closer_opponent, False)
      elif pacman_opponent and not myState.isPacman: # try to get nearest pacman
        return self.decideMove(gameState, actions, closer_opponent)


    return self.decideMove(gameState, actions, closest_food)


    '''
    You should change this in your own agent.
    '''

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
