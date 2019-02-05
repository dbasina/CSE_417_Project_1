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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    # import directions. If not, program gets crazy.
    from game import Directions

    # Initialize variables that are used for recursion
    start = problem.getStartState();
    visitedstates=[];
    actionStack = util.Stack();
    foundGoalState= [False];

    #This method's name is self explanatory
    # the recursion assumes that the node that its currently at is the start state.
    # we are creating sub problems from the main problem until we reach the case where
    # start = goal . Then, we retrace the path from goal to start and collect the directions
    # simultaneously while retracing back.
    # Variables breakdown:
    # start = starting node
    # visitedstates = a list to keep track of visited states
    # actionStack= stack that holds the actions
    # foundGoalState = a list that contains false until we reach goal state and then append a true to it

    def recursivePathGenerator(start,visitedstates,actionStack,foundGoalState,problem):

        import util as util

        #Print Check
        print "State: ",start
        print "successors:",problem.getSuccessors(start)
        print "Visited:",visitedstates
        print "found:", foundGoalState
        print "\n"

        # add current node to visited states
        visitedstates.append(start);

        # Check for goal state
        if (problem.isGoalState(start)):
            print start
            foundGoalState.append(True);
            print foundGoalState
            return

        # Recursive step. Get successors and visit depth first.
        else:

            nodeSuccessors = problem.getSuccessors(start);
            numberOfSuccessors = len(nodeSuccessors);
            popCounter=0;

            # Check if all the successors have been visited previously
            for i in nodeSuccessors:
                if (i[0] in visitedstates):
                    popCounter = popCounter+1;

            # if some nodes still need to be visited, search for goal node recursively.
            if(popCounter != numberOfSuccessors):
                for i in nodeSuccessors:

                    if(i[0] not in visitedstates):

                        recursivePathGenerator(i[0],visitedstates,actionStack,foundGoalState,problem)
                        if (foundGoalState[len(foundGoalState)-1]):
                            print i[1]
                            actionStack.push(i[1]);
                            print "Goal Found, Retracing..."
                            return;
            # If popcounter == number of successors, we return since we didn't find goal node in our path.
            else:
                return;


    # Call our recursive function
    recursivePathGenerator(start,visitedstates,actionStack,foundGoalState,problem);

    #initialize return value
    retval =[]

    #While stack is not empty, extract and append elements from stack to return value.
    while (not actionStack.isEmpty()):
        retval.append(actionStack.pop());
    # Print Check
    print retval

    #Return
    return retval;



    # define recursivePathGenerator that takes the coordinates tuple
    # generate a path list in the recursion
    # base case: check if coordinate is isGoalState
    # else create for i that iterates over s.getSuccessors
    # add the action of the i into our return list
    # call the recursivePathGenerator from for loops on i's coordinates.
    # return list from recursion
    # return list from depthFirstSearch
    util.raiseNotDefined()

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
