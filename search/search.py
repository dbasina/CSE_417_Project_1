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

    #print "Start:", problem.getStartState()
    #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    #print "Start's successors:", problem.getSuccessors(problem.getStartState())
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

        ##print Check
        ##print "State: ",start
        ##print "successors:",problem.getSuccessors(start)
        ##print "Visited:",visitedstates
        ##print "found:", foundGoalState
        ##print "\n"

        # add current node to visited states
        visitedstates.append(start);

        # Check for goal state
        if (problem.isGoalState(start)):
            ##print start
            foundGoalState.append(True);
            ##print foundGoalState
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
                            ##print i[1]
                            actionStack.push(i[1]);
                            ##print "Goal Found, Retracing..."
                            return;
            # If popcounter == number of successors, we return since we didn't find goal node in our path.
            else:
                return;


    # Call the recursive function
    recursivePathGenerator(start,visitedstates,actionStack,foundGoalState,problem);

    #initialize return value
    retval =[]

    #While stack is not empty, extract and append elements from stack to return value.
    while (not actionStack.isEmpty()):
        retval.append(actionStack.pop());
    # #print Check
    #print retval

    #Return
    return retval;

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    """ Algorithm:
        We use the method iterativeGoalGenerator to get to the goal node
        using breadthFirstSearch. In the process we are also creating a list
        of tuples of (parent,child). parent:(x,y) child: ((x,y),direction,cost)

        We assign each successor node it's parent and store it in the parentChild[]
        list. We need this list to backtrack and generate the path to goal nodeself.

        assignParent(parent,child): creates the tuple and adds to the list.

        returnParent(child): traverses the list and returns (parent,direction)

        actionStackGenerator: generates the list of actions and stores in actionStack

        iterativeGoalGenerator: finds the goal iteratively using breadthFirstSearch 
        """
    def assignParent(parent,child):
        relationship =(parent,child);
        parentChild.append(relationship);

    def returnParent(child):
        for i in parentChild:
            parentLocation = i[0]
            childNode =i[1];
            childLocation = childNode[0];
            if (childLocation == child):
                return (parentLocation,childNode[1]);

    def actionStackGenerator(actionStack,start,goal):
        retval = returnParent(goal);
        while (retval[0] != start):
            actionStack.push(retval[1]);
            retval = returnParent(retval[0]);
        actionStack.push(retval[1]);

    def iterativeGoalGenerator(problem,visitedStates,nodeQueue):
        # iterates till we find goal.
        # Each node has its parent's location sparing the start state that helps us backtrack.
        retval =(0,0);
        while(not nodeQueue.isEmpty()):
            #Pop node from queue and add to visited list.
            node = nodeQueue.pop();

            if (node not in visitedStates):
                #Add node to visitedstates
                visitedStates.append(node);

                #check if node is goal node.
                if (problem.isGoalState(node)):
                    retval = node;
                    #print "Goal state found breaking"
                    #print node;
                    return node;
                #Add successor nodes to queue.
                else:
                    #print "node not found yet"
                    # Get the successors of the node.
                    successors = problem.getSuccessors(node);
                    for i in successors:
                        if (i[0] not in visitedStates):
                            assignParent(node,i);
                            nodeQueue.push(i[0]);



    # Start Program
    start = problem.getStartState();
    visitedStates=[];
    nodeQueue = util.Queue();
    actionStack = util.Stack();
    foundGoalState= [False];
    nodeQueue.push(start);
    parentChild=[]
    retval = [];
    goal = iterativeGoalGenerator(problem,visitedStates,nodeQueue)
    actionStackGenerator(actionStack,start,goal);

    while(not actionStack.isEmpty()):
        l = actionStack.pop();
        retval.append(l);

    return retval;


    util.raiseNotDefined();



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
