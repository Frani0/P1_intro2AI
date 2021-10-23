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

visited_nodes = []

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
    from game import Directions
    solution = []
    for i in range(len(path)-1):
        path1 = path[i]
        path2 = path[i+1]
        if path1[0] > path2[0]:
            solution.append(Directions.WEST)
        elif path1[0] < path2[0]:
            solution.append(Directions.EAST)
        elif path1[1] > path2[1]:
            solution.append(Directions.SOUTH)
        elif path1[1] < path2[1]:
            solution.append(Directions.NORTH)
    return solution


def discoverPath(problem, current_node, path):
    i = True
    while i:
        if problem.isGoalState(current_node):
            is_goal = True
            break
        # pick first successor and set as the next node
        next_node = problem.getSuccessors(current_node)[0][0]
        #todo handle it when there's only 1 successor!!

        # find number of successors
        number_of_successors = len(problem.getSuccessors(current_node))
        # check all the successors if they have been discovered
        if next_node in visited_nodes:
            if number_of_successors == 0:
                next_node = problem.getSuccessors(current_node)[1][0]
                i = False
                is_goal = False
            else:
                for x in range(number_of_successors-1):
                    next_node = problem.getSuccessors(current_node)[x+1][0]
                    if next_node in visited_nodes:
                        i = False
                        is_goal = False

        # if next node isn't already explored, append it to paths
        path.append(next_node)
        if next_node not in visited_nodes:
            visited_nodes.append(next_node)
        current_node = next_node
        if problem.isGoalState(current_node):
            is_goal = True
            break

    return is_goal, path


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
    global visited_nodes

    # find coordinates of initial state
    current_node = problem.getStartState()

    # create vector which will be filled with found path, initialize with first position coordinates
    path = []
    path.append(current_node)
    visited_nodes.append(current_node)

    # call function to find a first path
    is_goal, path = discoverPath(problem, current_node, path)

    j = True
    while j:
        # if we found the goal, return the path
        if is_goal:
            winning_path = tanslateToDirections(path)
            j = False

        # if we didn't find the goal, repeat with new direction after last good node
        else:
            good_node = False
            for i in range(len(path)):
                number_of_successors = len(problem.getSuccessors(path[-1-i]))
                if number_of_successors > 2:
                    last_good_node = path[-1-i]
                    for j in range(number_of_successors):
                        if problem.getSuccessors(last_good_node)[j][0] not in visited_nodes:
                            good_node = True
                    if good_node:
                        break
            index = path.index(last_good_node)
            path = path[:index+1]
            for i in range(number_of_successors):
                if problem.getSuccessors(last_good_node)[i][0] not in visited_nodes:
                    new_next_node = problem.getSuccessors(last_good_node)[i][0]
                    path.append(new_next_node)
                    visited_nodes.append(new_next_node)
                    break

            is_goal, path = discoverPath(problem, new_next_node, path)


    return winning_path

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
