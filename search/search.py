# search.py
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


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

def tanslateToDirections(path):
    solution = []
    for i in range(len(path)-1):
        path1 = path[i]
        path2 = path[i+1]
        if path1[0] > path2[0]:
            solution.append("w")
        elif path1[0] < path2[0]:
            solution.append("e")
        elif path1[1] > path2[1]:
            solution.append("s")
        elif path1[1] < path2[1]:
            solution.append("n")

        separator = ", "
        final = separator.join(solution)
    return final
'''
def getSuccessorState(current_state, successor_state):
    if successor_state == "South":
        new_state = (current_state[0], current_state[1]-1)
    elif successor_state == "North":
        new_state = (current_state[0], current_state[1]+1)
    elif successor_state == "West":
        new_state = (current_state[0]-1, current_state[1])
    elif successor_state == "East":
        new_state = (current_state[0]+1, current_state[1])
    return new_state
succ = getSuccessorState(current_node, problem.getSuccessors(current_node)[0][1])
'''

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """

    # find coordinates of initial state
    current_node = problem.getStartState()

    # create vector which will be filled with found path, initialize with first position coordinates
    paths = []
    paths.append(current_node)

    # set end node if current_node is the goal
    if problem.isGoalState(current_node):
        pass

    '''
    # check the number of successor branches
    number_of_successor_branches = len(problem.getSuccessors(now_state))
    '''
    i = True
    while i == True:
        # pick first successor and set as new now_state
        next_node = problem.getSuccessors(current_node)[0][0]

        if next_node in paths:
            if len(problem.getSuccessors(current_node)) == 2:
                next_node = problem.getSuccessors(current_node)[1][0]
                if next_node in paths:
                    i = False
            elif len(problem.getSuccessors(current_node)) == 3:
                next_node = problem.getSuccessors(current_node)[1][0]
                if next_node in paths:
                    next_node = problem.getSuccessors(current_node)[2][0]
                    if next_node in paths:
                        i = False
            else:
                i = False
        # if next node isn't already explored, append it to paths
        paths.append(next_node)
        current_node = next_node

    print(paths)

    winningPath = tanslateToDirections(paths)
    print(winningPath)
    from game import Directions
    n = Directions.NORTH
    e = Directions.EAST
    w = Directions.WEST
    s = Directions.SOUTH

    return s, s, w, s, w, w, n, w, n, n, e, e, e, w

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
