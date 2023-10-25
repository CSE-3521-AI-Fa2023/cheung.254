# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
from util import heappush, heappop
class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
      """
      Returns the start state for the search problem
      """
      util.raiseNotDefined()

    def isGoalState(self, state):
      """
      state: Search state

      Returns True if and only if the state is a valid goal state
      """
      util.raiseNotDefined()

    def getSuccessors(self, state):
      """
      state: Search state

      For a given state, this should return a list of triples,
      (successor, action, stepCost), where 'successor' is a
      successor to the current state, 'action' is the action
      required to get there, and 'stepCost' is the incremental
      cost of expanding to that successor
      """
      util.raiseNotDefined()

    def getCostOfActions(self, actions):
      """
      actions: A list of actions to take

      This method returns the total cost of a particular sequence of actions.  The sequence must
      be composed of legal moves
      """
      util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    Your search algorithm needs to return a list of actions that reaches
    the goal. Make sure that you implement the graph search version of DFS,
    which avoids expanding any already visited states. 
    Otherwise your implementation may run infinitely!
    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    """
    YOUR CODE HERE
    """

    closed = set() 
    fringe = util.Stack() # Holds node to be explored, and actions to get there
    fringe.push((problem.getStartState(), [])) # Push first state to fringe
    # debugCount = 0
    while not fringe.isEmpty(): # empty fringe means failure, since no solution has been found
       state = fringe.pop() 
       node = state[0]
       actions = state[1]

        # add un-traversed nodes to closed set and check goal
       if node not in closed:
          closed.add(node)

          # if node is goal, return list of actions taken to find it
          if problem.isGoalState(node):
             return actions
          
          # If node is not goal, traverse each successor, keeping track of the actions taken to reach each one by appending to its actions list.
          for neighbor in problem.getSuccessors(node):
             newActions = actions.copy()
             newActions.append(neighbor[1])
             fringe.push((neighbor[0],newActions))
    print("SOLUTION NOT FOUND")



    util.raiseNotDefined()
    

def breadthFirstSearch(problem):
    """
    YOUR CODE HERE
    """

    closed = set()
    fringe = list()
    fringe.insert(0,(problem.getStartState(), []))

    while not len(fringe)== 0:
       # traverse nodes in bfs
       state = fringe.pop()  # pop first entry (FIFO Queue)
       node = state[0]
       actions = state[1]
       
       if node not in closed:
        closed.add(node)

        if problem.isGoalState(node):
           return actions
        
        for neighbor in problem.getSuccessors(node):
           newActions = actions.copy()
           newActions.append(neighbor[1])
           fringe.insert(0, (neighbor[0], newActions))
    print("SOLUTION NOT FOUND")

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """
    YOUR CODE HERE
    """
    closed = set()
    fringe = util.PriorityQueue() # priority queue
    fringe.push((problem.getStartState(), []), 0)

    while not fringe.isEmpty(): # pop items in priority
       item= fringe.pop()
       node = item[0]
       actions = item[1]

       if node not in closed:
        closed.add(node)

        if problem.isGoalState(node):
           print(actions)
           return actions 
        
        for neighbor in problem.getSuccessors(node):
           newActions = actions.copy()
           newActions.append(neighbor[1])
           fringe.push((neighbor[0], newActions), problem.getCostOfActions(newActions))
    print("SOLUTION NOT FOUND")

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """
    YOUR CODE HERE
    """
    closed = set()
    fringe = util.PriorityQueue() # priority queue
    h_start = heuristic(problem.getStartState(), problem)
    fringe.push((problem.getStartState(), []), h_start)


    while not fringe.isEmpty(): # pop items in priority
       item= fringe.pop()
       node = item[0]
       actions = item[1]

       if node not in closed:
        closed.add(node)

        if problem.isGoalState(node):
           print(actions)
           return actions 
        
        for neighbor in problem.getSuccessors(node):
           newActions = actions.copy()
           newActions.append(neighbor[1])
           heur = heuristic(neighbor[0], problem)
           fringe.push((neighbor[0], newActions), (problem.getCostOfActions(newActions)) + heur)
    print("SOLUTION NOT FOUND")

    # orders by f(n) = g(n) + h(n)

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
