import numpy as np


class Cell:
    """
    Class cell represents a cell in the world which have the property
    position : The position of the represented by  tupleof x and y
    coordinates initially set to (0,0)
    parent : This contains the parent cell object which we visited
    before arrinving this cell
    g,h,f : The parameters for constructing the heuristic function
    which can be any function. for simplicity used line
    distance
    """

    def __init__(self):
        self.position = (0, 0)
        self.parent = None

        self.g = 0
        self.h = 0
        self.f = 0

    """
    overrides equals method because otherwise cell assign will give
    wrong results
    """

    def __eq__(self, cell):
        return self.position == cell.position

    def showcell(self):
        print(self.position)


class Gridworld:
    """
    Gridworld class represents the  external world here a grid M*M
    matrix
    world_size: create a numpy array with the given world_size default is 5
    """

    def __init__(self, world_size=(68, 11)):
        self.w = np.zeros(world_size)
        self.world_x_limit = world_size[0]
        self.world_y_limit = world_size[1]

    def show(self):
        print(self.w)

    def get_neigbours(self, cell):
        """
        Return the neighbours of cell
        """
        neughbour_cord = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]
        current_x = cell.position[0]
        current_y = cell.position[1]
        neighbours = []
        for n in neughbour_cord:
            x = current_x + n[0]
            y = current_y + n[1]
            if 0 <= x < self.world_x_limit and 0 <= y < self.world_y_limit and self.w[x][y] != 2:
                c = Cell()
                c.position = (x, y)
                c.parent = cell
                neighbours.append(c)
        return neighbours


def astar(world, start, goal):
    """
    Implementation of a start algorithm
    world : Object of the world object
    start : Object of the cell as  start position
    stop  : Object of the cell as goal position
    >>> p = Gridworld()
    >>> start = Cell()
    >>> start.position = (0,0)
    >>> goal = Cell()
    >>> goal.position = (4,4)
    >>> astar(p, start, goal)
    [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]
    """
    _open = []
    _closed = []
    _open.append(start)

    while _open:
        min_f = np.argmin([n.f for n in _open])
        current = _open[min_f]
        _closed.append(_open.pop(min_f))
        if current == goal:
            break
        for n in world.get_neigbours(current):
            for c in _closed:
                if c == n:
                    continue
            n.g = current.g + 1
            x1, y1 = n.position
            x2, y2 = goal.position
            n.h = (y2 - y1) ** 2 + (x2 - x1) ** 2
            n.f = n.h + n.g

            for c in _open:
                if c == n and c.f < n.f:
                    continue
            _open.append(n)
    path = []
    while current.parent is not None:
        # print(list(current.position))
        path.append(list(current.position))
        current = current.parent
    path.append(list(current.position))
    return path[::-1]

def obstacle_padding(world, obstacle):
    if obstacle == None:
        return world
    neughbour_cord1 = [
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),

        (-2,-2),
        (-2,-1),
        (-2,0),
        (-2,1),
        (-2,2),
        (-1,2),
        (0,2),
        (1,2),
        (2,2),
        (2,1),
        (2,0),
        (2,-1),
        (2,-2),
        (1,-2),
        (0,-2),
        (-1,-2),
    ]
    for curr in neughbour_cord1:
        if 0 <= obstacle[0] + curr[0] < world.world_x_limit and 0 <= obstacle[1] + curr[1] < world.world_y_limit:
            world.w[obstacle[0]+curr[0]][obstacle[1]+curr[1]] = 2
    return world

def generate_map(s = (0,0), g = (34,5), obstacle = None):
    world = Gridworld()
    #   stat position and Goal
    start = Cell()
    start.position = s
    goal = Cell()
    goal.position = g
    ###deteccc person, padding with 2
    # obstacle = (34,10)
    obstacle_padding(world, obstacle)
    s = astar(world, start, goal)
    #   Just for visual reasons
    for i in s:
        world.w[i] = 1

    print(world.w)
    path = s
    print(path)
    return path
path = generate_map()
path_points_x = [float(point[0]) for point in path]

def extra(path):
    curr_angle = 90
    for curr in path:
        curr.append(curr_angle)
    print(path)
