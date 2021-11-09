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

def findWinningPathBFS(problem, current_node, visited_nodes, first_node):

    # create list for winning path & fill it with last node
    winning_path = []
    winning_path.append(current_node)

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
            else:
                index_max = min(index)
                current_node = visited_nodes[index_max]

        winning_path.append(current_node)
        #if the next node is the first node, we are done finding the path
        if current_node == first_node:
            break
            k = False

    return winning_path


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
    global visited_nodes

    # find coordinates of initial state
    current_node = problem.getStartState()

    # create vector which will be filled with found path, initialize with first position coordinates
    path = []
    path.append(current_node)
    visited_nodes.append(current_node)

    # call function to find a first path
    is_goal, path = discoverPathDFS(problem, current_node, path)

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

            is_goal, path = discoverPathDFS(problem, new_next_node, path)


    return winning_path

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    global visited_nodes

    # find coordinates of initial state, find number of successors
    first_node = problem.getStartState()

    # if the first node is the goal node, skip all code and return none
    if problem.isGoalState(first_node):
        return None
    else:
        # create vector which will be filled with found path, initialize with first position coordinates
        path = util.Queue()
        path.push(first_node)

        # create a list with all visited nodes
        visited_nodes.append(first_node)

        # add successors to path, delete current_node from path
        i = True
        while i:
            for j in range(len(path.list)):
                current_node = path.list[-1]
                is_goal, path = discoverPathBFS(problem, current_node, path)
                if is_goal:
                    i = False
                    break
                if path.isEmpty():
                    i = False
                    break

        # this is the coordinate of the goal
        goal_coord = path.list[0]
        # from list of visited nodes and goal coord, find the actual winning path
        path = findWinningPathBFS(problem, goal_coord, visited_nodes, first_node)
        # since path is from goal to beginning, reverse
        path.reverse()
        # translate path to winning directions
        winning_path = tanslateToDirections(path)

        return winning_path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    # Insert the root node into the priority queue (with priority 0)
    priority_queue = util.PriorityQueue()
    starting_node = problem.getStartState()
    priority_queue.push(starting_node, 0)
    # add children to priority queue
    for i in range(len(problem.getSuccessors(starting_node))):
        item = problem.getSuccessors(starting_node)[i][0]
        priority = problem.getSuccessors(starting_node)[i][2]
        priority_queue.push(item, priority)

    # Repeat while the queue is not empty:
    while not priority_queue.isEmpty():
        # Remove the element with the highest priority
        highest_priority = priority_queue.heap[0][0]
        removed_item = priority_queue.pop()
        # If the removed node is the destination, print total cost and stop the algorithm
        if problem.isGoalState(removed_item):
            print("total cost =", highest_priority)
            break
        # Else, enqueue all the children of the current node to the priority queue,
        # with their cumulative cost from the root as priority
        else:
            for i in range(len(problem.getSuccessors(removed_item))):
                item = problem.getSuccessors(removed_item)[i][0]
                priority = problem.getSuccessors(removed_item)[i][2] + highest_priority
                priority_queue.update(item, priority)

        print(priority_queue.heap)



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
