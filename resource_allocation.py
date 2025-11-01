import time

class ResourceAllocationProblem:

    # build a constructor that will define a state / environment of the algorithm
    def __init__(self, servers, tasks):
        # the servers and tasks lists will be taken as copies so that the original list are not modified.
        self.servers = servers[:]           
        self.tasks = tasks[:]
        self.nr_of_servers = len(servers)
        self.nr_of_tasks = len(tasks)

    # create the initial state with all of the tasks being unassigned (value of -1)
    def initial_state(self):
        return [-1] * self.nr_of_tasks

    # defines the current loads of the servers by the currently assigned tasks.
    def server_loads(self, state):
        loads = [0] * self.nr_of_servers
        for task, server in enumerate(state):
            if server != -1:
                loads[server] += self.tasks[task]
        return loads

    # returns all possible actions from the current state
    def actions(self, state):

        # calculate how much load each server currently has
        loads = self.server_loads(state)

        # initialize a list to store all possible actions
        possible = []

        # in this loop we will check each task and if their assignment is -1
        # that means they have not been assigned to a server.
        for task, assignment in enumerate(state):
            if assignment == -1:
                # we get the requirement of that task
                requirement = self.tasks[task]

                # and for every server we check, if their current capacity
                # considering the load as well is not exceeded, then we assign it to it.
                # we then save the action as a tuple (task, assigned server)
                for server in range(self.nr_of_servers):
                    if loads[server] + requirement <= self.servers[server]:
                        possible.append((task, server))

        return possible

    # function that will be used to create new states using the actions
    # from the possible list that we created. (updating the task index with a server index)
    def result(self, state, action):
        task, server = action
        new_state = state[:]
        new_state[task] = server
        return new_state

    # the goal will have been achieved if every task has been assigned to a server.
    # so we check if every index in the state is different from -1.
    def goal_test(self, state):
        return all(x != -1 for x in state) # all returns True if every iteration within it is true.
                                           # in our case it would mean the goal state reached.

    # each assignment will simply have a cost of 1.
    # the algorithm will logically have to find a combination which takes the least assignments (efficient) 
    def step_cost(self, state_from, action, state_to):
        return 1

    # we will use this function to sum all the steps along a certain path that is taken
    # and see the costs of all opportunities.
    # this is necessary for the algorithms so they can know which node to expand next.
    def path_cost(self, path_cost_so_far, state_from, action, state_to):
        return path_cost_so_far + self.step_cost(state_from, action, state_to)

    # the logic chosen for the heuristic of the A* algorithm is that it will collect
    # all tasks that have not been assigned yet. value will be 0 if there are not any.
    # then after that, the sum of the requirements of the remaining tasks will be divided by the value of
    # of the server with the maximum capacity. 
    
    # this gives a rough estimation of the minimum number of assignments needed to finish the remaining tasks
    # or how far it is from the goal and it helps A* distinguish which state looks closer to the goal.
    # for example if we divide the total requirement by the capacity of the biggest server,
    # we tell ourselves, if i put all of them there, id need at least x amount of steps left.

    def heuristic(self, state):
        remaining_tasks = [self.tasks[i] for i, s in enumerate(state) if s == -1]
        if not remaining_tasks:
            return 0
        max_cap = max(self.servers)
        return sum(remaining_tasks) / max_cap


    # simple printing function for the solution in the main file.
    def print_solution(self, state):
        loads = self.server_loads(state)

        for task, server in enumerate(state):
            print(f"Task {task}: Server {server} (requirement: {self.tasks[task]})")

        print("\nServer loads:\n")
        for server, (capacity, load) in enumerate(zip(self.servers, loads)):
            print(f'Server {server}: Load {load} / {capacity}')

        print("-" * 40)