from copy import deepcopy
import queue
import sys
from datetime import datetime

class Node:
    def __init__(self, current_state, parent, action, depth, cost, f, action_move=None):
        self.current_state = current_state
        self.parent = parent
        self.action = action
        self.depth = depth
        self.cost = cost
        self.action_move = action_move
        self.f = f

    def __lt__(self, other):
        return self.cost < other.cost

# Calulating the heuristic
def h(current_state, goal_state):
    distance = 0
    for i in range(3):
        for j in range(3):
            if current_state[i][j] != goal_state[i][j]:
                for m in range(3):
                    for n in range(3):
                        if current_state[i][j] == goal_state[m][n]:
                            distance += (abs(i-m) + abs(j-n))*current_state[i][j]
    return distance

# Finding the blank tile in the state
def blank_tile(current_state):
    for i in range(3):
        for j in range(3):
            if current_state[i][j] == 0:
                return i, j

# Checking the actions that can be performed
# from the current state
def actions_to_perform(current_stat):
    # Getting blank tile
    i, j = blank_tile(current_stat)
    actions = []

    if i == 0:
        actions.append("down")
    elif i == 1:
        actions.append("up")
        actions.append("down")
    elif i == 2:
        actions.append("up")

    if j == 0:
        actions.append("right")
    elif j == 1:
        actions.append("left")
        actions.append("right")
    elif j == 2:
        actions.append("left")

    return actions

def move_tile(currentState, action):
    copy_state = deepcopy(currentState)
    i, j = blank_tile(copy_state)
    cost = 0
    if action == "up":
        copy_state[i][j] = copy_state[i-1][j]
        copy_state[i-1][j] = 0
        cost = copy_state[i][j]
    elif action == "down":
        copy_state[i][j] = copy_state[i+1][j]
        copy_state[i+1][j] = 0
        cost = copy_state[i][j]
    elif action == "left":
        copy_state[i][j] = copy_state[i][j-1]
        copy_state[i][j-1] = 0
        cost = copy_state[i][j]
    elif action == "right":
        copy_state[i][j] = copy_state[i][j+1]
        copy_state[i][j+1] = 0
        cost = copy_state[i][j]

    action_to_do = "Move " + str(copy_state[i][j]) + " " + trackpath(action)

    return copy_state, cost, action_to_do

def expand_node(current_node, goal, search_type):
    # get actions
    actions = actions_to_perform(current_node.current_state)
    children = []

    for action in actions:
        child_state, cost, action_to_do = move_tile(
            current_node.current_state, action)
        totalCost = current_node.cost + cost
        if search_type == "greedy":
            heuristic = h(child_state, goal)
        else:
            heuristic = totalCost + h(child_state, goal)
        # heuristic = 0
        child_node = Node(child_state, current_node, action,
                          current_node.depth+1, totalCost, heuristic, action_to_do)
        children.append(child_node)

    return children

# Breadth First Search
def bfs(start, goal, flag, file1):
    if flag == True:
        file1.write("BFS Search\n")
    initial_state = Node(start, None, None, 0, 0, 0, "Start")
    queue = []
    visited = set()
    queue.append(initial_state)

    popped = 0
    queue_size = 0
    nodes_generated = 1
    nodes_expanded = 0
    while queue:
        queue_size = max(queue_size, len(queue))
        current_node = queue.pop(0)
        popped += 1

        if current_node.current_state == goal:
            if flag == True:
                writefile3(file1, current_node, popped,
                           nodes_expanded, nodes_generated, queue_size)
            return current_node, popped, queue_size, nodes_generated, nodes_expanded
        if tuple(map(tuple, current_node.current_state)) not in visited:
            visited.add(tuple(map(tuple, current_node.current_state)))
            nodes_expanded += 1
            children = expand_node(current_node, goal, "bfs")
            nodes_generated += len(children)

            if flag == True:
                writefile2(file1, current_node, children, visited)

            for j in children:
                if tuple(map(tuple, j.current_state)) not in visited:
                    queue.append(j)

        if flag == True:
            writefile_bfs(file1, queue)

    return None

# Uniform Cost Search
def ucs(start, goal, flag, file1):
    if flag == True:
        file1.write("UCS Search\n")

    initial_state = Node(start, None, None, 0, 0, 0, "Start")
    # Priority Queue
    queue_priority = queue.PriorityQueue()
    visited = set()
    queue_priority.put((initial_state.cost, initial_state))

    popped = 0
    queue_size = 0
    nodes_generated = 1
    nodes_expanded = 0
    while not queue_priority.empty():
        queue_size = max(queue_size, queue_priority.qsize())
        current_node = queue_priority.get()[1]
        popped += 1

        if current_node.current_state == goal:
            if flag == True:
                writefile3(file1, current_node, popped,
                           nodes_expanded, nodes_generated, queue_size)
            return current_node, popped, queue_size, nodes_generated, nodes_expanded
        if tuple(map(tuple, current_node.current_state)) not in visited:
            visited.add(tuple(map(tuple, current_node.current_state)))
            nodes_expanded += 1
            children = expand_node(current_node, goal, "ucs")
            nodes_generated += len(children)

            if flag == True:
                writefile2(file1, current_node, children, visited)

            for j in children:
                if tuple(map(tuple, j.current_state)) not in visited:
                    queue_priority.put((j.cost, j))

        if flag == True:
            writefile(file1, queue_priority)
            
    return None


# Depth First Search
def dfs(start, goal, flag, file1):
    if flag == True:
        file1.write("DFS Search\n")

    initial_state = Node(start, None, None, 0, 0, 0, "Start")
    stack = []
    visited = set()
    stack.append(initial_state)

    popped = 0
    stack_size = 0
    nodes_generated = 1
    nodes_expanded = 0
    while stack:
        stack_size = max(stack_size, len(stack))
        current_node = stack.pop()
        popped += 1
        if current_node.current_state == goal:
            if flag == True:
                writefile3(file1, current_node, popped,
                           nodes_expanded, nodes_generated, stack_size)
            return current_node, popped, stack_size, nodes_generated, nodes_expanded
        if tuple(map(tuple, current_node.current_state)) not in visited:
            visited.add(tuple(map(tuple, current_node.current_state)))
            nodes_expanded += 1
            children = expand_node(current_node, goal, "dfs")
            nodes_generated += len(children)

            if flag == True:
                writefile2(file1, current_node, children, visited)

            for j in children:
                if tuple(map(tuple, j.current_state)) not in visited:
                    stack.append(j)
        if flag == True:
            writefile_bfs(file1, stack)

    return None

# Depth limited search
def dls(start, goal, limit, flag, file1):

    if flag == True:
        file1.write("DLS Search\n")
        
    initial_state = Node(start, None, None, 0, 0, 0, "Start")
    popped = 0
    nodes_generated = 1
    nodes_expanded = 0
    stack = [(initial_state, 0)]
    stack_size = 0
    visited = set()

    while stack:
        stack_size = max(stack_size, len(stack))
        current_node, depth = stack.pop()
        popped += 1
        if current_node.current_state == goal:
            if flag == True:
                writefile3(file1, current_node, popped,
                           nodes_expanded, nodes_generated, stack_size)
            return current_node, popped, stack_size, nodes_generated, nodes_expanded
        if depth < limit:
            nodes_expanded += 1
            children = expand_node(current_node, goal, "dls")
            nodes_generated += len(children)
            if flag == True:
                writefile2(file1, current_node, children, visited)
            for child in children:
                if child.depth <= limit:
                    stack.append((child, depth + 1))
        if flag == True:
            for i in stack:
                file1.write("< state = %s, action = %s, g(n) = %d, d = %d, f(n) = %d, parent = %s >\n" % (
                    i[0].current_state, i[0].action_move, i[0].cost, i[0].depth, i[0].f, "None" if i[0].parent == None else i[0].parent.current_state))

    return None, popped, stack_size, nodes_generated, nodes_expanded

# Iterative deepening search
def ids(start, goal, flag, file1):
    d = 0
    while True:
        if flag == True:
            file1.write("Depth = %d\n" %(d))
        result, popped, stack_size, nodes_generated, nodes_expanded = dls(
            start, goal, d, flag, file1)
        if result is not None:
            return result, popped, stack_size, nodes_generated, nodes_expanded
        if flag == True:
            file1.write("Depth limit reached.\n\n")
        d += 1


# Greedy Search
def greedy(start, goal, flag, file1):
    if flag == True:
        file1.write("Greedy Search\n")
    initial_state = Node(start, None, None, 0, 0, 0, "Start")

    queue_priority = queue.PriorityQueue()
    visited = set()
    queue_priority.put((initial_state.cost, initial_state))

    popped = 0
    queue_size = 0
    nodes_generated = 1
    nodes_expanded = 0
    while not queue_priority.empty():
        queue_size = max(queue_size, queue_priority.qsize())
        current_node = queue_priority.get()[1]
        popped += 1

        if current_node.current_state == goal:
            if flag == True:
                writefile3(file1, current_node, popped,
                           nodes_expanded, nodes_generated, queue_size)
            return current_node, popped, queue_size, nodes_generated, nodes_expanded
        if tuple(map(tuple, current_node.current_state)) not in visited:
            visited.add(tuple(map(tuple, current_node.current_state)))
            nodes_expanded += 1
            children = expand_node(current_node, goal, "greedy")
            nodes_generated += len(children)
            if flag == True:
                writefile2(file1, current_node, children, visited)

            for j in children:
                queue_priority.put((j.f, j))
        if flag == True:
            writefile(file1, queue_priority)

    return None


# A* Search
def a_star(start, goal, flag, file1):
    if flag == True:
        file1.write("A* Search\n")

    initial_state = Node(start, None, None, 0, 0, h(start, goal), "Start")
    # Priority Queue
    queue_priority = queue.PriorityQueue()
    visited = set()
    queue_priority.put((initial_state.f, initial_state))

    popped = 0
    queue_size = 0
    nodes_generated = 1
    nodes_expanded = 0
    while not queue_priority.empty():
        queue_size = max(queue_size, queue_priority.qsize())
        current_node = queue_priority.get()[1]
        popped += 1

        if current_node.current_state == goal:
            if flag == True:
                writefile3(file1, current_node, popped,
                           nodes_expanded, nodes_generated, queue_size)
            return current_node, popped, queue_size, nodes_generated, nodes_expanded

        if tuple(map(tuple, current_node.current_state)) not in visited:
            visited.add(tuple(map(tuple, current_node.current_state)))
            nodes_expanded += 1
            children = expand_node(current_node, goal, "a_star")
            nodes_generated += len(children)
            if flag == True:
                writefile2(file1, current_node, children, visited)

            for j in children:
                queue_priority.put((j.f, j))

        if flag == True:
            writefile(file1, queue_priority)

    return None


def writefile_bfs(file1, queue):
    for i in queue:
        file1.write("< state = %s, action = %s, g(n) = %d, d = %d, f(n) = %d, parent = %s >\n" % (
            "None" if i.current_state == None else i.current_state, i.action_move, i.cost, i.depth, i.f, "None" if i.parent == None else i.parent.current_state))


def writefile(file1, queue_priority):
    for i in queue_priority.queue:
        file1.write("< state = %s, action = %s, g(n) = %d, d = %d, f(n) = %d, parent = %s >\n" % (
            i[1].current_state, i[1].action_move, i[1].cost, i[1].depth, i[1].f, "None" if i[1].parent == None else i[1].parent.current_state))


def writefile2(file1, current_node, children, visited):
    file1.write("Generating successors to < state = %s, action = %s, g(n) = %d, d = %d, f(n) = %d, parent = %s >:\n" % (current_node.current_state,
                current_node.action_move, current_node.cost, current_node.depth, current_node.f, "None" if current_node.parent == None else current_node.parent.current_state))
    file1.write("\t%d Successors generated \n" % len(children))
    file1.write("\t Closed: %s\n" % list(visited))
    file1.write("\t Fringe: \n")


def writefile3(file, node, popped, nodes_expanded, nodes_generated, max_fringe_size):
    file.write("\n\tNodes Popped: %d\n \tNodes Expanded: %d\n \tNodes Generated: %d\n \tMax Fringe Size: %d\n \tSolution found at depth %d with cost %d\n" % (
        popped, nodes_expanded, nodes_generated, max_fringe_size, node.depth, node.cost))


def file_read(filename):
    arr = []
    file = open(filename, "r")
    for f in file:
        list = []
        if (f == "END OF FILE"):
            break
        line = f.split()
        for i in line:
            list.append(int(i))
        arr.append(list)
    file.close()

    return arr


def trackPath(node):
    path = []
    print("Steps: ")
    while node.parent != None:
        path.append(node.action_move)
        node = node.parent

    path.reverse()
    for i in path:
        print("\t", i)


def trackpath(action):
    if action == "up":
        return "Down"
    elif action == "down":
        return "Up"
    elif action == "left":
        return "Right"
    elif action == "right":
        return "Left"


def printAns(finalState, popped, nodes_expanded, nodes_generated, max_fringe_size):
    print("Nodes Popped:", popped)
    print("Nodes Expanded:", nodes_expanded)
    print("Nodes Generated:", nodes_generated)
    print("Max Fringe Size:", max_fringe_size)
    print("Solution found at depth %d with cost %d." %
          (finalState.depth, finalState.cost))
    # trackPath(finalState)


def main():
    # Reading start and goal states from the files
    if len(sys.argv) < 3:
        print("Missing arguments. Please enter the start and goal state files")
        return 0
    dump_flag = False
    start_name = sys.argv[1]
    goal_name = sys.argv[2]
    file_write = None
    
    if len(sys.argv) < 4:
        metho = "a_star"
    else:
        metho = (sys.argv[3])

    method = globals()[metho]
        
    if len(sys.argv) == 5:
        if sys.argv[4] == "true" or sys.argv[4] == "True":
            dump_flag = True
    if dump_flag == True:
        filename= "trace-" + datetime.now().strftime("%Y%m%d-%H%M%S") + ".txt"
        file_write = open(filename, "w")
        file_write.write("Command Line Arguments: ['%s', '%s', '%s', '%s']\n" %(start_name, goal_name, metho, dump_flag))
        file_write.write("Method Selected: %s\n" %(metho))
        file_write.write("Running %s\n" %(metho))

    start = file_read(start_name)
    goal = file_read(goal_name)

    if metho == "dls":
        depth = int(input("Enter depth: "))
        ans, popped, max_fringe_size, nodes_generated, nodes_expanded = method(
            start, goal, depth, dump_flag, file_write)
    else:
        ans, popped, max_fringe_size, nodes_generated, nodes_expanded = method(
            start, goal, dump_flag, file_write)

    if ans is None:
        print("No solution found")
    else:
        printAns(ans, popped, nodes_expanded, nodes_generated, max_fringe_size)
        
    if dump_flag == True:
        file_write.close()

if __name__ == '__main__':
    main()