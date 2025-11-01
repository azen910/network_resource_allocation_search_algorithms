# main.py where we put the algorithm to the test
from resource_allocation import ResourceAllocationProblem
from search import ucs, a_star


def run_example():
    servers = [10, 25, 20, 10, 20, 10]     # server capacities
    tasks = [15, 5, 6, 4, 25, 15, 3, 2, 5]      # task requirements

    problem = ResourceAllocationProblem(servers, tasks)

    print("=== Problem Summary ===")
    print(f"Servers (capacities): {servers}")
    print(f"Tasks (requirements): {tasks}\n")



    # UCS ALGORITHM
    print("Running Uniform Cost Search (UCS)...")
  
    result_ucs = ucs(problem)
    
    if result_ucs['solution_state']:
        print("UCS found solution:\n")
        print(f"UCS Nodes expanded: {result_ucs['nodes_expanded']}")
        print(f"UCS Path cost: {result_ucs['path_cost']}")
        print(f"UCS time taken: {result_ucs['time']}")
        print(f"UCS Solution actions: {result_ucs['solution_actions']}\n")
        print(problem.print_solution(result_ucs['solution_state']))
    else:
        print("UCS did not find a solution.")
    print('\n' + '='*40 + '\n')



    # A* ALGORITHM
    print("Running A* Search...")
  
    result_astar = a_star(problem, problem.heuristic)

    if result_astar['solution_state']:
        print("A* found solution:\n")
        print(f"A* Nodes expanded: {result_astar['nodes_expanded']}")
        print(f"A* Path cost: {result_astar['path_cost']}")
        print(f"A* time taken: {result_astar['time']}")
        print(f"A* Solution actions: {result_astar['solution_actions']}\n")
        print(problem.print_solution(result_astar['solution_state']))
    else:
        print("A* did not find a solution.")


run_example()


