import heapq
import time


# While doing the search algorithms, it is important to keep track of the path / previous states
# that brought us here. this is key to keep track of the solution path and the cumulative cost.

class Node:

    # keep track of 4 crucial piece of information that is needed for the final output.
    def __init__(self, state, parent, action, path_cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def solution(self):
        node, path_back = self, []
        
        # loop while the state we are in is not None or if we are not at the beginning.
        # and keep going back to the parent.
        while node and node.action is not None:
            path_back.append(node.action)
            node = node.parent

        # in the end, reverse the list so that it looks in the right order from the beginning
        return list(reversed(path_back))

    # we will use this to convert a state into a hashable and comparable form so that it can be stored in a set or dictionary for the algorithms.
    def state_key(self):
        return tuple(self.state)


# here we will define the ucs algorithm, which focuses on the cheapest expansion first on any current state.
def ucs(problem):

    start_time = time.time()

    # frontier will serve as our priority queue (min heap) which will keep the minimum cost expansion at the top for easy access.
    frontier = []  

    # we define the start Node with the initial state problem and default values.
    start = Node(problem.initial_state(), parent=None, action=None, path_cost=0)

    # we push the starting node into the frontier in the format: [path cost, tie breaker (for when two nodes have the same cost), starting node]
    heapq.heappush(frontier, (start.path_cost, 0, start))

    # keeping a set of visited states (prevents revisiting them as sets do not allow duplicates)
    explored = set()

    # this variable will count how many nodes were taken out of the frontier for usage.
    nodes_expanded = 0

    # tie is required in our design as our priority queue needs to compare items
    # if two nodes have the same path cost, it only compares by Node objects (raises an error), so we add
    # this tie as a unique identifier assigned to each incrementally with the purpose of distinguishing
    # these two nodes.
    tie = 1

    while frontier:

        # take the lowest cost node from the heap and store only the node
        # as we are currently only focusing on the node.
        _, _, node = heapq.heappop(frontier)

        # check if this node state has been explored before. if so, skip it.
        # otherwise, store it so we do not have to repeat work
        if node.state_key() in explored:
            continue

        # add the state in tuple form so it can be used inside a set.
        explored.add(node.state_key())

        # if goal is reached, save the information to a dictionary.
        if problem.goal_test(node.state):
            elapsed = time.time() - start_time
            return {
                'solution_state': node.state,
                'solution_actions': node.solution(),
                'path_cost': node.path_cost,
                'nodes_expanded': nodes_expanded,
                "time" : elapsed
            }

        # if not, we continue expanding the nodes, and counting
        nodes_expanded += 1

        # get all possible actions we can from the current node/state (finding the first unassigned task)
        for action in problem.actions(node.state):

            # like we explained the other file, result() returns a new state after applying the action.
            child_state = problem.result(node.state, action)

            # calculate the new cost by adding the current cost and adding 1 using the step_cost()
            child_cost = problem.path_cost(node.path_cost, node.state, action, child_state)

            # the new state is wrapped into a Node linking it to its parent so that it can be printed later
            child = Node(child_state, parent=node, action=action, path_cost=child_cost)

            # if this has not been explored before, we push it to the frontier with its cost and the current tie
            # afterwards, we increment the tie for the next node.
            if child.state_key() not in explored:
                heapq.heappush(frontier, (child.path_cost, tie, child))
                tie += 1

    # if UCS explored everything reachable but found no solution, we return a dictionary with 'None' values
    elapsed = time.time() - start_time
    return {'solution_state': None, 'solution_actions': None, 'path_cost': None, 'nodes_expanded': nodes_expanded}


# now here we will be implementing the A* algorithm with the help of the heuristic function
def a_star(problem, heuristic):

    start_time = time.time()
    
    # once again we will be using the priority queue (heap) as a frontier
    frontier = []  

    # pretty much the similar steps as above with a few exceptions.
    start = Node(problem.initial_state(), parent=None, action=None, path_cost=0)

    # this time around we are gonna need the f value expressed as:
    # the sum of g (cost so far) and h (heuristic value / predicted cost to reach the goal)
    # f in itself describes the total guess of the best choice on which node should be explored next.
    start_f = start.path_cost + heuristic(start.state)

    # the cost which will be considered in the frontier this time will be the f value.
    heapq.heappush(frontier, (start_f, 0, start))

    # in A*, sometimes the algorith will discover the same state again but with a cheaper cost
    explored = dict()  # will keep state_key and best path cost / best g(n) so far as its value

    nodes_expanded = 0

    tie = 1

    while frontier:

        # this time we need the state and the f(n)
        f, _, node = heapq.heappop(frontier)

        # if we have already visited this state before with a better g(n) value for this state, skip
        # there is no need to explore or expand this current one.
        if node.state_key() in explored and explored[node.state_key()] <= node.path_cost:
            continue

        # if not already visited, add the current path cost to the key of this node state.
        explored[node.state_key()] = node.path_cost

        if problem.goal_test(node.state):
            elapsed = time.time() - start_time
            return {
                'solution_state': node.state,
                'solution_actions': node.solution(),
                'path_cost': node.path_cost,
                'nodes_expanded': nodes_expanded,
                'time': elapsed
            }

        nodes_expanded += 1

        for action in problem.actions(node.state):
            
            # child operations follow the same routine as above.
            child_state = problem.result(node.state, action)
            child_cost = problem.path_cost(node.path_cost, node.state, action, child_state)
            child = Node(child_state, parent=node, action=action, path_cost=child_cost)

            # now we have to do the operations for the child and where to expand from there.
            g = child.path_cost

            # the f value of the child is the prioritized value (cot of the predicted full path)
            # and will be used to point out which node to expand next
            f_child = g + heuristic(child.state)

            # same process here as above, ignore states that we have visited before 
            # and that have a better g(n) cost.
            if child.state_key() in explored and explored[child.state_key()] <= g:
                continue

            # in the end push this child state to the heap with its f(n) value
            # and update the tie value for next states.
            heapq.heappush(frontier, (f_child, tie, child))
            tie += 1
    
    # return a dictionary with values as "None" if no solution was found.
    elapsed = time.time() - start_time
    return {'solution_state': None, 'solution_actions': None, 'path_cost': None, 'nodes_expanded': nodes_expanded}
