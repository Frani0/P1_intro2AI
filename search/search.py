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
import heapq

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


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """Search the deepest nodes in the search tree first."""

    # find coordinates of root node,
    starting_node = problem.getStartState()
    # create empty list with visited nodes
    visited_nodes = []
    # initialize the stack with the starting node as first node & empty list for path
    # the path will be linked to the corresponding node so if we take a wrong turn and have to go back
    # the path will be "overwritten"
    stack = util.Stack()
    stack.push((starting_node, []))
    # as long as the stack is not empty
    while not stack.isEmpty():
        # pop the last entry with its corresponding path
        (popped_node, path) = stack.pop()
        # if the popped node is the goal, break
        if problem.isGoalState(popped_node):
            break
        # get successors
        successors = problem.getSuccessors(popped_node)
        # add the successor to visited nodes
        visited_nodes.append(popped_node)
        # for all the successor nodes
        for i in successors:
            successor_coord = i[0]
            successor_direction = i[1]
            # the successor has not been visited before
            if successor_coord not in visited_nodes:
                # update the path
                new_path = path + [successor_direction]
                # add the node to the stack and add direction to path
                stack.push((successor_coord, new_path))
    return path

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    # find coordinates of root node,
    starting_node = problem.getStartState()
    # create list with visited nodes, fill with starting node
    visited_nodes = [starting_node]
    # initialize the queue with the starting node as first node & empty list for path
    queue = util.Queue()
    queue.push((starting_node, []))
    # as long as the queue is not empty
    while not queue.isEmpty():
        # pop the last entry
        (popped_node, path) = queue.pop()
        # if the successor node is the goal, break
        if problem.isGoalState(popped_node):
            break
        # get successors
        successors = problem.getSuccessors(popped_node)
        # for all the successor nodes
        for i in successors:
            successor_coord = i[0]
            successor_direction = i[1]
            # the successor has not been visited before
            if successor_coord not in visited_nodes:
                # update the path
                new_path = path + [successor_direction]
                # add the successor to visited nodes
                visited_nodes.append(successor_coord)
                print("visited nodes", visited_nodes)
                # add the node to the stack and add direction to path
                queue.push((successor_coord, new_path))
    return path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    # find coordinates of root node,
    starting_node = problem.getStartState()
    # create empty list with visited nodes
    visited_nodes = []
    # initialize the priority queue with the starting node as first node  & empty list for path
    # Cost and priority are 0 since it's the starting node
    queue = util.PriorityQueue()
    queue.push((starting_node, []), 0)
    # as long as the priority queue is not empty
    while not queue.isEmpty():
        # pop the last entry
        (popped_node, path) = queue.pop()
        # if the popped node has not been visited yet
        if popped_node not in visited_nodes:
            # add the successor to visited nodes
            visited_nodes.append(popped_node)
            # if the successor node is the goal, return the path
            if problem.isGoalState(popped_node):
                return path
            # get successors
            successors = problem.getSuccessors(popped_node)
            # for all the successor nodes
            for i in successors:
                successor_coord = i[0]
                successor_direction = i[1]
                # the successor has not been visited before
                if successor_coord not in visited_nodes:
                    # update the path
                    new_path = path + [successor_direction]
                    # calculate cost of path
                    cost = problem.getCostOfActions(new_path)
                    # add the node,its  path and cost of path into queue
                    queue.push((successor_coord, new_path), cost)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    # find coordinates of root node,
    starting_node = problem.getStartState()
    # create empty list with visited nodes
    visited_nodes = []
    # initialize the priority queue with the starting node as first node  & empty list for path
    # Cost and priority are 0 since it's the starting node
    queue = util.PriorityQueue()
    queue.push((starting_node, []), 0)
    # as long as the priority queue is not empty
    while not queue.isEmpty():
        # pop the last entry
        (popped_node, path) = queue.pop()
        # if the popped node has not been visited yet
        if popped_node not in visited_nodes:
            # add the successor to visited nodes
            visited_nodes.append(popped_node)
            # if the successor node is the goal, return the path
            if problem.isGoalState(popped_node):
                return path
            # get successors
            successors = problem.getSuccessors(popped_node)
            # for all the successor nodes
            for i in successors:
                successor_coord = i[0]
                successor_direction = i[1]
                # the successor has not been visited before
                if successor_coord not in visited_nodes:
                    # update the path
                    new_path = path + [successor_direction]
                    # calculate cost backward path (same as ucs)
                    backward_cost = problem.getCostOfActions(new_path)
                    # calculate forward cost (according to heuristic function)
                    forward_cost = heuristic(successor_coord, problem)
                    # add both cost for total cost
                    total_cost = backward_cost + forward_cost
                    # add the node, its path and cost of path into queue
                    queue.push((successor_coord, new_path), total_cost)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
