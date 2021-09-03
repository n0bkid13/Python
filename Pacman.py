// One of the more intersting coding problems here

# START


import bisect,sys

class Problem:
    def __init__(self, initial, goal=None):
        self.initial = initial
        self.goal = goal

    def successor(self, state):
        raise NotImplementedError

    def actions(self, state):
        raise NotImplementedError

    def result(self, state, action):
        raise NotImplementedError

    def goal_test(self, state):
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        return c + 1

    def value(self):
        raise NotImplementedError


class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0  # search depth
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):

        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        next_state = problem.result(self.state, action)
        return Node(next_state, self, action,
                    problem.path_cost(self.path_cost, self.state,
                                      action, next_state))

    def solution(self):
        return [node.action for node in self.path()[1:]]

    def solve(self):
        return [node.state for node in self.path()[0:]]

    def path(self):
        x, result = self, []
        while x:
            result.append(x)
            x = x.parent
        result.reverse()
        return result

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)



class Queue:
    def __init__(self):
        raise NotImplementedError

    def append(self, item):
        raise NotImplementedError

    def extend(self, items):
        raise NotImplementedError

    def pop(self):
        raise NotImplementedError

    def __len__(self):
        raise NotImplementedError

    def __contains__(self, item):
        raise NotImplementedError


class Stack(Queue):

    def __init__(self):
        self.data = []

    def append(self, item):
        self.data.append(item)

    def extend(self, items):
        self.data.extend(items)

    def pop(self):
        return self.data.pop()

    def __len__(self):
        return len(self.data)

    def __contains__(self, item):
        return item in self.data


class FIFOQueue(Queue):

    def __init__(self):
        self.data = []

    def append(self, item):
        self.data.append(item)

    def extend(self, items):
        self.data.extend(items)

    def pop(self):
        return self.data.pop(0)

    def __len__(self):
        return len(self.data)

    def __contains__(self, item):
        return item in self.data


class PriorityQueue(Queue):

    def __init__(self, order=min, f=lambda x: x):
        assert order in [min, max]
        self.data = []
        self.order = order
        self.f = f

    def append(self, item):
        bisect.insort_right(self.data, (self.f(item), item))

    def extend(self, items):
        for item in items:
            bisect.insort_right(self.data, (self.f(item), item))

    def pop(self):
        if self.order == min:
            return self.data.pop(0)[1]
        return self.data.pop()[1]

    def __len__(self):
        return len(self.data)

    def __contains__(self, item):
        return any(item == pair[1] for pair in self.data)

    def __getitem__(self, key):
        for _, item in self.data:
            if item == key:
                return item

    def __delitem__(self, key):
        for i, (value, item) in enumerate(self.data):
            if item == key:
                self.data.pop(i)

def tree_search(problem, fringe):
    fringe.append(Node(problem.initial))
    while fringe:
        node = fringe.pop()
        print(node.state)
        if problem.goal_test(node.state):
            return node
        fringe.extend(node.expand(problem))
    return None


def breadth_first_tree_search(problem):
    return tree_search(problem, FIFOQueue())


def depth_first_tree_search(problem):

    return tree_search(problem, Stack())

def graph_search(problem, fringe):
    closed = set()
    fringe.append(Node(problem.initial))
    while fringe:
        node = fringe.pop()
        if problem.goal_test(node.state):
            return node
        if node.state not in closed:
            closed.add(node.state)
            fringe.extend(node.expand(problem))
    return None


def breadth_first_graph_search(problem):
    return graph_search(problem, FIFOQueue())


def depth_first_graph_search(problem):
    return graph_search(problem, Stack())


def depth_limited_search(problem, limit=50):
    def recursive_dls(node, problem, limit):
        """Помошна функција за depth limited"""
        cutoff_occurred = False
        if problem.goal_test(node.state):
            return node
        elif node.depth == limit:
            return 'cutoff'
        else:
            for successor in node.expand(problem):
                result = recursive_dls(successor, problem, limit)
                if result == 'cutoff':
                    cutoff_occurred = True
                elif result is not None:
                    return result
        if cutoff_occurred:
            return 'cutoff'
        return None

    return recursive_dls(Node(problem.initial), problem, limit)


def iterative_deepening_search(problem):
    for depth in range(sys.maxsize):
        result = depth_limited_search(problem, depth)
        if result is not 'cutoff':
            return result


def uniform_cost_search(problem):
    return graph_search(problem, PriorityQueue(min, lambda a: a.path_cost))


#END



def levo(side_pos):
    if side_pos == 'istok':
        side_pos = 'sever1'
    if side_pos == 'zapad':
        side_pos = 'jug1'
    if side_pos == 'sever':
        side_pos = 'zapad1'
    if side_pos == 'jug':
        side_pos = 'istok1'

    if side_pos == 'sever1':
        side_pos = 'sever'
    if side_pos == 'jug1':
        side_pos = 'jug'
    if side_pos == 'zapad1':
        side_pos = 'zapad'
    if side_pos == 'istok1':
        side_pos = 'istok'

    return side_pos


def desno(side_pos):
    if side_pos == 'istok':
        side_pos = 'jug1'
    if side_pos == 'zapad':
        side_pos = 'sever1'
    if side_pos == 'sever':
        side_pos = 'istok1'
    if side_pos == 'jug':
        side_pos = 'zapad1'

    if side_pos == 'sever1':
        side_pos = 'sever'
    if side_pos == 'jug1':
        side_pos = 'jug'
    if side_pos == 'zapad1':
        side_pos = 'zapad'
    if side_pos == 'istok1':
        side_pos = 'istok'

    return side_pos


def nazad(side_pos):
    if side_pos == 'istok':
        side_pos = 'zapad1'
    if side_pos == 'zapad':
        side_pos = 'istok1'
    if side_pos == 'sever':
        side_pos = 'jug1'
    if side_pos == 'jug':
        side_pos = 'sever1'

    if side_pos == 'sever1':
        side_pos = 'sever'
    if side_pos == 'jug1':
        side_pos = 'jug'
    if side_pos == 'zapad1':
        side_pos = 'zapad'
    if side_pos == 'istok1':
        side_pos = 'istok'

    return side_pos


def ProdolzhiPravo(x_pos, y_pos, side_pos, obstacles):
    if side_pos == 'istok':
        if (x_pos + 1, y_pos) not in obstacles and x_pos + 1 < 10:
            x_pos += 1
    if side_pos == 'sever':
        if (x_pos, y_pos + 1) not in obstacles and y_pos + 1 < 10:
            y_pos += 1
    if side_pos =='jug':
        if (x_pos, y_pos - 1) not in obstacles and y_pos - 1 >= 0:
            y_pos -= 1
    if side_pos == 'zapad':
        if (x_pos - 1, y_pos) not in obstacles and x_pos - 1 >= 0:
            x_pos -= 1
    return x_pos, y_pos, side_pos


def ProdolzhiNazad(x_pos, y_pos, side_pos, obstacles):
    side_pos = nazad(side_pos)
    if side_pos == 'istok':
        if (x_pos + 1, y_pos) not in obstacles and x_pos + 1 < 10:
            x_pos += 1
    if side_pos == 'sever':
        if (x_pos, y_pos + 1) not in obstacles and y_pos + 1 < 10:
            y_pos += 1
    if side_pos =='jug':
        if (x_pos, y_pos - 1) not in obstacles and y_pos - 1 >= 0:
            y_pos -= 1
    if side_pos == 'zapad':
        if (x_pos - 1, y_pos) not in obstacles and x_pos - 1 >= 0:
            x_pos -= 1
    return x_pos, y_pos, side_pos


def ProdolzhiLevo(x_pos, y_pos, side_pos, obstacles):
    side_pos = levo(side_pos)
    if side_pos == 'istok':
        if (x_pos + 1, y_pos) not in obstacles and x_pos + 1 < 10:
            x_pos += 1
    if side_pos == 'sever':
        if (x_pos, y_pos + 1) not in obstacles and y_pos + 1 < 10:
            y_pos += 1
    if side_pos =='jug':
        if (x_pos, y_pos - 1) not in obstacles and y_pos - 1 >= 0:
            y_pos -= 1
    if side_pos == 'zapad':
        if (x_pos - 1, y_pos) not in obstacles and x_pos - 1 >= 0:
            x_pos -= 1
    return x_pos, y_pos, side_pos


def ProdolzhiDesno(x_pos, y_pos, side_pos, obstacles):
    side_pos = desno(side_pos)
    if side_pos == 'istok':
        if (x_pos + 1, y_pos) not in obstacles and x_pos + 1 < 10:
            x_pos += 1
    if side_pos == 'sever':
        if (x_pos, y_pos + 1) not in obstacles and y_pos + 1 < 10:
            y_pos += 1
    if side_pos =='jug':
        if (x_pos, y_pos - 1) not in obstacles and y_pos - 1 >= 0:
            y_pos -= 1
    if side_pos == 'zapad':
        if (x_pos - 1, y_pos) not in obstacles and x_pos - 1 >= 0:
            x_pos -= 1
    return x_pos, y_pos, side_pos


class Pacman(Problem):
    def __init__(self, obstacles, initial, goal=None):
        super().__init__(initial, goal)
        self.obstacles = obstacles

    def successor(self, state):
        successors = dict()
        x_pac = state[0]
        y_pac = state[1]
        side = state[2]
        snack_list = state[3]
        snack_list = list(snack_list)

        new_pos = ProdolzhiPravo(x_pac, y_pac, side, self.obstacles)
        if [new_pos] != [x_pac, y_pac, side]:
            successors['ProdolzhiPravo'] = (new_pos[0], new_pos[1], new_pos[2],
                                            tuple([s for s in snack_list if s[0] != new_pos[0] or s[1] != new_pos[1]]))
            
        new_pos = ProdolzhiNazad(x_pac, y_pac, side, self.obstacles)
        if [new_pos] != [x_pac, y_pac, side]:
            successors['ProdolzhiNazad'] = (new_pos[0], new_pos[1], new_pos[2],
                                            tuple([s for s in snack_list if s[0] != new_pos[0] or s[1] != new_pos[1]]))
		
        new_pos = ProdolzhiLevo(x_pac, y_pac, side, self.obstacles)
        if [new_pos] != [x_pac, y_pac, side]:
            successors['SvrtiLevo'] = (new_pos[0], new_pos[1], new_pos[2],
                                       tuple([s for s in snack_list if s[0] != new_pos[0] or s[1] != new_pos[1]]))
        
        
        new_pos = ProdolzhiDesno(x_pac, y_pac, side, self.obstacles)
        if [new_pos] != [x_pac, y_pac, side]:
            successors['SvrtiDesno'] = (new_pos[0], new_pos[1], new_pos[2],
                                        tuple([s for s in snack_list if s[0] != new_pos[0] or s[1] != new_pos[1]]))




        return successors

    def actions(self, state):
        return self.successor(state).keys()

    def result(self, state, action):
        return self.successor(state)[action]

    def goal_test(self, state):
        snacks = state[3]
        return len(snacks) == 0


if __name__ == '__main__':
    pac_x = int(input())
    pac_y = int(input())
    pos_side = input()
    num_snacks = int(input())
    snack_pos = []
    for i in range(0, num_snacks):
        k = input()
        snack_pos.append((int(k[0]), int(k[2])))
    obstacle_list = [(1, 2), (1, 3), (1, 4), (0, 6), (0, 8),
                     (0, 9), (1, 9), (2, 9), (3, 6), (3, 9),
                     (4, 1), (4, 5), (4, 6), (4, 7), (5, 1),
                     (5, 6), (6, 0), (6, 1), (6, 2), (6, 9),
                     (8, 1), (8, 4), (8, 7), (8, 8), (9, 4),
                     (9, 7), (9, 8)]
    pacman = Pacman(obstacle_list, (pac_x, pac_y, pos_side, tuple(snack_pos)))

    result = breadth_first_graph_search(pacman)

    print(result.solution())
