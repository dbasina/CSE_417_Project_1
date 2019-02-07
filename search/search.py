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

def aStarSearch(problem, heuristic= nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    """Algorithm:
        We use a similiar structure to what we used in breadthFirstSearch.
        We maintain a parent child list. But unlike in breadthFirstSearch where we
        only used the node locations, we need to store the node tuple that is returned
        from calling getSuccessors method. This is the key to running A* search since it
        provides every node its uniqueness. Which helps us in backtracking with the shortest path.

        The assumptions of A* search finding the shortest path plays a crucial role in the workings of the algorithm.
        Using this peice of information, we can safely backtrack to the node that called it all the way back to the start.
        It is in this backtracking process, saving the entire node information as opposed to saving only the location comes into play.

        assignParent: This method assigns the parent of a child node right after the getSuccessors method is called. A tuple of this data is created
                      and stored in the parentChild list that we later use for backtracking the path and compuing costs to nodes.

        returnParent: This method returns the parent of a child node.

        returnCost  : This method returns the 1 step cost to move to the child node from its parent node. Since two nodes can have the same children,
                      we need this method to first identify its parent before computing the cost to the child.

        computeCostToNode: This method computes the minimum cost for reaching a node in the tree by iteratively calling returnCost on parentNode of the goal node,
                            until we reach start.

        actionStackGenerator: This method is similar to computeCostToNode in that it iteratively calls the parentNode to the child node while simultaneously building the
                              actionStack that we will use to generate the return list.

        aStarGoalGenerator:  This method uses a* search to find the goal node, while simultaneously building the following datastructures
                             - nodeQueue: Used to set up nodes that will get popped and either ignored or expanded based on whether they have already been visited.
                             - visitedStates: Keeps track of only the node locations that have been visited.
                             - heuristicList: This list keeps track of new nodes that have been expanded along with the value of their cost functions.
                                              we sort this list and pop the first element out and add it to the node queue which then gets popped for the next iteration.
                             Note: In the graph search version of the algorithm, we do not expand nodes that have already been visited even though they posses the least value of
                                   the cost function. Therefore, we need to make sure that visitedStates keeps track of only the locations. This is because the heuristicList will still pick up
                                   nodes that have been visited. We handle these nodes in the outermost else and prevent reprocessing of the nodes.
        """
    def assignParent(parent,child):
        relationship =(parent,child);
        parentChild.append(relationship);

    def returnParent(child):
        for i in parentChild:
            parentNode = i[0]
            childNode =i[1];
            if (childNode == child):
                return (parentNode,childNode[1]);

    def returnCost(child):

        for i in parentChild:
            parentNode = i[0]
            childNode =i[1];
            if (childNode == child):
                return (parentNode,childNode[2]);

    def actionStackGenerator(actionStack,start,goal):
        retval = returnParent(goal);
        while (retval[0] != start):
            actionStack.push(retval[1]);
            retval = returnParent(retval[0]);
        actionStack.push(retval[1]);

    def computeCostToNode(start,goal):
        sum=0;
        retval = returnCost(goal);
        while (retval[0] != start):
            sum = sum+retval[1];
            retval = returnCost(retval[0]);
        sum=sum+retval[1];
        return sum;


    def aStarGoalGenerator(nodeQueue,visitedStates,problem,heuristic,start):
        # iterates till we find goal.
        # Each node has its parent's location sparing the start state that helps us backtrack.
        retval =(0,0);
        heuristicList = [];
        while (not nodeQueue.isEmpty()):
            #Pop node from queue and add to visited list.
            node = nodeQueue.pop();

            if (node[0] not in visitedStates):
                #Add node to visitedstates
                visitedStates.append(node[0]);

                #check if node is goal node.
                if (problem.isGoalState(node[0])):
                    retval = node;
                    return node;
                #Add successor nodes to queue.
                else:
                    #print "node not found yet"
                    # Get the successors of the node.

                    successors = problem.getSuccessors(node[0]);
                    for i in successors:
                        if (i[0] not in visitedStates):
                            assignParent(node,i);
                            #print parentChild
                            heuristicValue = heuristic(i[0],problem);
                            costValue = computeCostToNode(start,i);
                            fn = costValue+heuristicValue;
                            valueTuple = (i,fn);
                            #if (heuristicTuple not in heuristicList):
                            heuristicList.append(valueTuple);


                    heuristicList.sort(key=lambda tup: tup[1]);
                    l = heuristicList.pop(0);
                    nodeQueue.push(l[0]);
            else:
                i = heuristicList.pop(0);
                nodeQueue.push(i[0]);

    # Start Programs
    startLocation = problem.getStartState();
    start=(startLocation,0,0);
    visitedStates=[];
    nodeQueue = util.Queue();
    actionStack = util.Stack();
    foundGoalState= [False];
    nodeQueue.push(start);
    parentChild=[];
    retval = [];
    goal = aStarGoalGenerator(nodeQueue,visitedStates,problem,heuristic,start)
    while (not actionStack.isEmpty()):
        l = actionStack.pop();
        print l;

    actionStackGenerator(actionStack,start,goal);
    while (not actionStack.isEmpty()):
        l = actionStack.pop();
        retval.append(l);
    return retval;



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
