import heapq
from collections import deque


class Node:
    def __init__(self, state):
        self.state = state
        self.prev_move = None
        self.zero_pos = None
        self.possible_moves = set()
        self.possible_states = []
        self.g = None
        self.pred = None

    def __lt__(self, other):
        return id(self) < id(other)


class Puzzle:
    def __init__(self, grid_size, start_state, goal_state):
        self.grid_size = grid_size
        self.start_state = Node(start_state)
        self.goal_state = Node(goal_state)
        # self.available_moves=set()
        # self.available_states=[]

    def get_zero_pos(self, q):

        for i, v in enumerate(q.state):
            if v == 0:
                return i

    def get_moves(self, idx):  # up or down or left or right
        n = self.grid_size
        l = set()
        if idx+n < n*n:
            l.add(n+idx)
        if idx % n != 0:
            l.add(-1+idx)
        if (idx+1) % n != 0:
            l.add(1+idx)
        if idx-n >= 0:
            l.add(-1*n + idx)
        return l

    def do_move(self, my_moves, q_node):  # apply the move ans store the state
        # self.available_moves
        # self.available_states
        states = []
        for idx in my_moves:
            arr = q_node.state[:]
            arr[q_node.zero_pos], arr[idx] = arr[idx], arr[q_node.zero_pos]
            n = Node(arr)
            n.prev_move = q_node.zero_pos
            states.append(n)
        return states

    def h(self, choice, s):
        """
        Heurisitc Function
        choice 1:Hamming Distance 
        choice 2:Manhattan Distance 
        """
        if choice == 1:
            return self.hamming(s)
        else:
            return self.manhattan(s)

    def hamming(self, s):
        d = 0
        g = self.goal_state

        for i, j in zip(s.state, g.state):
            if i != j and i != 0:
                d += 1
        return d

    def manhattan(self, s):
        res, cx, cy = 0, 0, 0
        g = self.goal_state
        for i, j in zip(s.state, g.state):
            if i != j and i != 0:
                x = int(abs(i-1)/self.grid_size)
                y = int(abs(i-(1+(x*self.grid_size))))
                res += abs(cx-x) + abs(cy-y)
            cy += 1
            if cy == self.grid_size:
                cy = 0
                cx += 1
        return res

    def print_grid(self, path):
        state_counter = 0
        for node in path:
            count = 0
            print('state: {0}'.format(state_counter), end='\n')
            for i in range(self.grid_size):
                for j in range(self.grid_size):
                    print(node.state[count], end=" ")
                    count += 1
                print(end='\n')
            print('-'*10, end='\n')
            state_counter += 1

    def A_star(self, choice):  # we will store states,and state represents the grid
        """
        choice 1:Hamming Distance 
        choice 2:Manhattan Distance 
        """
        print("Running")
        if choice == 1:
            print("Using Hamming Distance")
        else:
            print("Using Manhattan Distance")

        q_open = []
        heapq.heapify(q_open)

        closed = set()
        visited = []
        visited.append(self.start_state.state)
        # pred=dict()
        # g=dict()
        self.start_state.pred = -1
        self.start_state.g = 0

        heapq.heappush(q_open, (0, self.start_state))

        while q_open:

            _, q = heapq.heappop(q_open)  # extract state with least f
            closed.add(q)
            #take q's childrens and put their pred

            if q.state == self.goal_state.state:

                temp = q
                path = []
                path.append(temp)
                while temp.pred != -1:
                    path.append(temp.pred)
                    temp = temp.pred

                # self.returned_cost=g[end] # we need to return the number of steps needed,and this will present the number of states needed
                end = time.time()
                print("taken_time using {} distance =".format(
                    "Manhattan" if choice == 2 else "Hamming"), end-start)
                return self.print_grid(path[::-1])

            q.zero_pos = self.get_zero_pos(q)
            # self.zero_pre_pos=self.zero_position
            q.possible_moves = self.get_moves(q.zero_pos)
            q.possible_states = self.do_move(
                q.possible_moves-{q.prev_move}, q)  # get new states/nodes

            for child in q.possible_states:  # childrens will reprenst the possible states with my possible movements
                if child.state not in visited:
                    visited.append(child.state)
                    if child not in q_open and child.g is None:
                        child.pred = q
                        child.g = q.g+1  # 1 unit

                        child.f = child.g+self.h(choice, child)
                        heapq.heappush(q_open, (child.f, child))

                    else:

                        if q.g+1 < child.g:
                            child.g = q.g+1
                            child.pred = q


    def bfs(self):

        #g
        #pred
        visited = []
        que = deque()

        self.start_state.g = 0
        self.start_state.pred = -1

        que.appendleft(self.start_state)

        visited.append(self.start_state.state)
        while que:
            q = que.pop()

            q.zero_pos = self.get_zero_pos(q)
            # self.zero_pre_pos=self.zero_position
            q.possible_moves = self.get_moves(q.zero_pos)
            q.possible_states = self.do_move(
                q.possible_moves-{q.prev_move}, q)  # get new states/nodes

            for child in q.possible_states:
                if child.state not in visited:
                    visited.append(child.state)
                    child.g = q.g+1
                    child.pred = q

                    que.appendleft(child)

                    if child.state == self.goal_state.state:

                        temp = child
                        path = []
                        path.append(temp)
                        while temp.pred != -1:
                            path.append(temp.pred)
                            temp = temp.pred

                        end = time.time()
                        print("taken_time using BFS = ", end-start)
                        return self.print_grid(path[::-1])


if __name__ == "__main__":

    g = 123804765
    s = 283164705
    s = [int(i) for i in str(s)]
    g = [int(i) for i in str(g)]

    p = Puzzle(grid_size=3, start_state=s, goal_state=g)
    import time
    start = time.time()
    choice = 2

    """    
    choice 1:Hamming Distance
    choice 2:Manhattan Distance
    """
    p.A_star(choice)
    p.bfs()