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

visited_nodes = []
winning_path = []

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


def discoverPathDFS(problem, current_node, path):
    i = True
    while i:
        if problem.isGoalState(current_node):
            is_goal = True
            break
        # pick first successor and set as the next node
        next_node = problem.getSuccessors(current_node)[0][0]

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

def discoverPathBFS(problem, current_node, path):
    is_goal = False
    number_successors = len(problem.getSuccessors(current_node))
    for i in range(number_successors):
        next_node = problem.getSuccessors(current_node)[i][0]
        if next_node in visited_nodes:
            pass
        else:
            path.push(next_node)
            if problem.isGoalState(next_node):
                is_goal = True
                break
            visited_nodes.append(next_node)
    path.pop()

    return is_goal, path


def findWinningPath(problem, current_node, visited_nodes, first_node):

    # create list for winning path & fill it with last node
    winning_path = [current_node]

    k = True
    while k:
        successors = problem.getSuccessors(current_node)
        # if there is only one successor, this one must be the parent node.
        if len(successors) == 1:
            current_node = successors[0][0]
        else:
            # find positions of successor nodes in visited nodes
            nodes = []
            index = []
            for i in range(len(successors)):
                if successors[i][0] in visited_nodes:
                    nodes.append(successors[i][0])
                    index.append(visited_nodes.index(successors[i][0]))
            # if only exactly one node has been visited, set this one as current node
            if len(nodes) == 1:
                current_node = nodes[0]
            # if more than one node has been visited, find one that comes earliest in visited nodes.
            elif len(nodes) > 1:
                index_max = min(index)
                current_node = visited_nodes[index_max]
            # if none of the nodes have been visited,
            else:
                print('??')
                break

        winning_path.append(current_node)
        #if the next node is the first node, we are done finding the path
        if current_node == first_node:
            break
            k = False

    return winning_path


def DFSRecursive(problem, nodes, current_node):
    if problem.isGoalState(current_node):
        print(nodes.list)
        return nodes
    else:
        successors = problem.getSuccessors(current_node)
        number_successors = len(successors)
        for i in range(number_successors):
            successor_coord, successor_direction, _ = successors[i]

            for x in nodes.list:
                if successor_coord == x[0]:
                    a = True
                    break
                else:
                    a = False
            if not a:
                nodes.push((successor_coord, successor_direction, current_node))
                DFSRecursive(problem, nodes, nodes.list[-1][0])
        return nodes


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

    # initialize a queue with the starting node
    starting_node = problem.getStartState()
    nodes = util.Stack()
    nodes.push((starting_node, None, None))
    # call recursive dfs function
    nodes = DFSRecursive(problem, nodes, starting_node)
    print(nodes.list)
    # initialize path with goal direction
    current_node = nodes.pop()
    path = [current_node[1]]
    # find parent of respective node and add its direction to list
    b = True
    while b:
        for x in nodes.list:
            # if the parent of current node is found
            if x[0] == current_node[2]:
                # append its direction to list
                path.append(current_node[1])
                # update current node to found parent
                current_node = x
                # if current node is the starting node, finish
                if current_node[0] == starting_node:
                    b = False
                    break

    # reverse path to go from start to goal
    path.reverse()

    return path

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    # find coordinates of root node,
    starting_node = problem.getStartState()
    # initialize the queue with the starting node as first node
    queue = util.Queue()
    queue.push((starting_node, None, None))
    # mark root node as explored
    explored_nodes = [starting_node]
    while not queue.isEmpty():
        action, popped_node = queue.pop()
        print(action)
        successors = problem.getSuccessors(popped_node[0])
        number_successors = len(successors)
        if problem.isGoalState(popped_node[0]):
            print("final", popped_node)
        for i in range(number_successors):
            next_node = successors[i]
            print("next node", next_node)
            successor_coord = next_node[0]
            if successor_coord not in explored_nodes:
                explored_nodes.append(successor_coord)
                nodes.push(successor_coord)
        print(nodes.list)

    return None

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    priorities = []
    # create priority queue and get starting node
    priority_queue = util.PriorityQueue()
    starting_node = problem.getStartState()
    # Insert the root node into the priority queue (with priority 0) add root to visited nodes
    # priority_queue.push(starting_node, 0)
    visited_nodes.append(starting_node)

    # add children to priority queue
    starting_info = problem.getSuccessors(starting_node)
    for i in range(len(starting_info)):
        item = starting_info[i][0]
        priority = starting_info[i][2]
        priority_queue.push(item, priority)

    # Repeat while the queue is not empty:
    while not priority_queue.isEmpty():
        # Remove the element with the highest priority
        highest_priority = priority_queue.heap[0][0]
        removed_item = priority_queue.pop()
        visited_nodes.append(removed_item)
        # If the removed node is the destination, print total cost and stop the algorithm
        if problem.isGoalState(removed_item):
            break
        # Else, enqueue all the children of the current node to the priority queue,
        # with their cumulative cost from the root as priority
        else:
            info = problem.getSuccessors(removed_item)
            for i in range(len(info)):
                item = info[i][0]
                priority = info[i][2] + highest_priority
                if item not in visited_nodes:
                    priority_queue.update(item, priority)

    # from list of visited nodes and goal coord, find the actual winning path
    path = findWinningPath(problem, removed_item, visited_nodes, starting_node)
    # since path is from goal to beginning, reverse
    path.reverse()
    # translate path to winning directions
    winning_path = tanslateToDirections(path)
    util.raiseNotDefined()
    return winning_path


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    # use uniform cost search but add heuristic function.
    uniformCostSearch(problem)

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
