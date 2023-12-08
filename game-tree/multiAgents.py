# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)

        currentPos = currentGameState.getPacmanPosition()
        currentFood = currentGameState.getFood()
        currentFoodPositions = currentFood.asList()
        currentFoodDistances = [util.manhattanDistance(currentPos, food) for food in currentFoodPositions]
        currentGhostStates = currentGameState.getGhostStates()
        currentGhostPositions = [ghost.getPosition() for ghost in currentGhostStates]
        currentGhostDistances = [util.manhattanDistance(currentPos, ghost) for ghost in currentGhostPositions]
        currentScaredTimes = [ghostState.scaredTimer for ghostState in currentGameState.getGhostStates()]
        currentCapsulePosition = currentGameState.getCapsules()
        currentCapsuleDistances = [util.manhattanDistance(currentPos, capsule) for capsule in currentCapsulePosition]

        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newFoodPositions = newFood.asList()
        newFoodDistances = [util.manhattanDistance(newPos, food) for food in newFoodPositions]
        newGhostStates = successorGameState.getGhostStates()
        newGhostPositions = [ghost.getPosition() for ghost in newGhostStates]
        newGhostDistances = [util.manhattanDistance(newPos, ghost) for ghost in newGhostPositions]
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]
        newCapsulePosition = successorGameState.getCapsules()
        newCapsuleDistances = [util.manhattanDistance(newPos, capsule) for capsule in newCapsulePosition]


        if successorGameState.isWin():
            return float("inf")
        elif successorGameState.isLose():
            return float("-inf")

        score = 0

        if len(currentFoodPositions) > len(newFoodPositions):
            score += 40

        if (min(currentFoodDistances) > min(newFoodDistances)) and newFoodDistances:
            score += 25

        if len(currentCapsuleDistances) > len(newCapsuleDistances):
            score += 20

        if newCapsuleDistances and (min(currentCapsuleDistances) > min(newCapsuleDistances)):
            score += 10


        if newGhostDistances:
            if min(newGhostDistances) > min(currentGhostDistances):
                ghostScore = 15
            else:
                ghostScore = -5
            if sum(newScaredTimes) > 0:
                ghostScore *= -1

        score += ghostScore

        return successorGameState.getScore() - currentGameState.getScore() + score

def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        numAgents = gameState.getNumAgents()

        def minimax_check(gameState, depth, agentIndex):
            if (depth == self.depth and agentIndex == 0) or gameState.isWin() or gameState.isLose():
                return None, self.evaluationFunction(gameState)

            best_move = None
            if agentIndex == 0:
                final = float("-inf")
            else:
                final = float("inf")

            if agentIndex == numAgents -1:
                dep = depth + 1
            else:
                dep = depth

            for move in gameState.getLegalActions(agentIndex):
                gs = gameState.generateSuccessor(agentIndex, move)

                curr = minimax_check(gs, dep, (agentIndex + 1) % numAgents)[1]

                if (agentIndex == 0 and final < curr) or (agentIndex > 0 and final > curr):
                    final, best_move = curr, move

            return best_move, final

        return minimax_check(gameState, 0, 0)[0]

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        numAgents = gameState.getNumAgents()

        def alphaBeta(gameState, depth, agentIndex, alpha, beta):
            if gameState.isWin() or gameState.isLose() or (depth == self.depth and agentIndex == 0):
                return None, self.evaluationFunction(gameState)

            best_move = None
            if agentIndex == 0:
                final = float("-inf")
            else:
                final = float("inf")

            if agentIndex == numAgents - 1:
                dep = depth + 1
            else:
                dep = depth

            for move in gameState.getLegalActions(agentIndex):
                gs = gameState.generateSuccessor(agentIndex, move)

                curr = alphaBeta(gs, dep, (agentIndex + 1) % numAgents, alpha, beta)[1]

                if agentIndex == 0:
                    if final < curr:
                        best_move, final = move, curr

                    alpha = max(alpha, final)
                    if alpha >= beta:
                        return best_move, final
                else:
                    if final > curr:
                        best_move, final = move, curr

                    beta = min(beta, final)
                    if beta <= alpha:
                        return best_move, final

            return best_move, final

        return alphaBeta(gameState, 0, 0, float("-inf"), float("inf"))[0]


class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        numAgents = gameState.getNumAgents()

        def expectimax(gameState, depth, agentIndex):
            if gameState.isWin() or gameState.isLose() or (depth == self.depth and agentIndex == 0):
                return None, self.evaluationFunction(gameState)

            best_move = None
            if agentIndex == 0:
                final = float("-inf")
            else:
                final = 0

            if agentIndex == numAgents - 1:
                dep = depth + 1
            else:
                dep = depth

            actions = gameState.getLegalActions(agentIndex)

            for move in actions:
                gs = gameState.generateSuccessor(agentIndex, move)

                curr = expectimax(gs, dep, (agentIndex + 1) % numAgents)[1]
                if agentIndex == 0:
                    if final < curr:
                        best_move, final = move, curr
                else:
                    final += curr * (1 / len(actions))
            return best_move, final

        return expectimax(gameState, 0, 0)[0]

def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    pos = currentGameState.getPacmanPosition()
    score = currentGameState.getScore()
    ghostPositions = [ghost.getPosition() for ghost in currentGameState.getGhostStates()]
    ghostDistances = [util.manhattanDistance(pos, ghost) for ghost in ghostPositions]
    foodPositions = currentGameState.getFood().asList()
    foodDistances = [1 / util.manhattanDistance(pos, food) for food in foodPositions]
    capsulePositions = currentGameState.getCapsules()
    capsuleDistances = [1 / util.manhattanDistance(pos, capsule) for capsule in capsulePositions]
    scaredTimes = [ghost.scaredTimer for ghost in currentGameState.getGhostStates()]

    new_score = 0
    if foodDistances:
        new_score += 10 * float(sum(foodDistances)) / len(foodDistances)
    else:
        new_score = float("inf")

    if capsuleDistances:
        if sum(scaredTimes) == 0:
            weight = 0
        else:
            weight = -1
        new_score += weight * 10 * float(sum(capsuleDistances)) / len(capsuleDistances) * weight

    if ghostDistances:
        if sum(scaredTimes) > 0:
            weight = 0.8
        else:
            weight = -1.2
        new_score += 0.35 * weight * min(ghostDistances) / 2

    return score + new_score


# Abbreviation
better = betterEvaluationFunction
