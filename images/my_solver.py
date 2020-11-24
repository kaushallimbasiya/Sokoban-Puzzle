'''

    2019 CAB320 Sokoban assignment

The functions and classes defined in this module will be called by a marker script. 
You should complete the functions and classes according to their specified interfaces.

You are not allowed to change the defined interfaces.
That is, changing the formal parameters of a function will break the 
interface and triggers to a fail for the test of your code.

# by default does not allow push of boxes on taboo cells
SokobanPuzzle.allow_taboo_push = False

# use elementary actions if self.macro == False
SokobanPuzzle.macro = False

'''

# you have to use the 'search.py' file provided
# as your code will be tested with this specific file
import search
import time
from search import breadth_first_graph_search, astar_graph_search, greedy_best_first_graph_search
import sokoban

import itertools

import sokoban

import math


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


#-----------------------------------------------------------------------------
def my_team():
    '''
    Return the list of the team members of this assignment submission as a list
    of triplet of the form (student_number, first_name, last_name)

    '''
    return [(9187928, 'James', 'Snewin'), (10083154, 'Lucas', 'Browne')]

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# GLOBAL VARIABLES AND FUNCTIONS
UP    = ( 0, -1)
DOWN  = ( 0,  1)
LEFT  = (-1,  0)
RIGHT = ( 1,  0)

MOVEMENTS = {"Up": UP,
             "Down": DOWN,
             "Left": LEFT,
             "Right": RIGHT}

def tuple_add(a,b):
    return (a[0] + b[0], a[1] + b[1])

def tuple_subtract(a,b):
    return (a[0] - b[0], a[1] - b[1])
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def taboo_cells(warehouse):
    '''
    Identify the taboo cells of a warehouse. A cell inside a warehouse is
    called 'taboo' if whenever a box get pushed on such a cell then the puzzle
    becomes unsolvable.
    When determining the taboo cells, you must ignore all the existing boxes,
    simply consider the walls and the target cells.
    Use only the following two rules to determine the taboo cells;
     Rule 1: if a cell is a corner inside the warehouse and not a target,
             then it is a taboo cell.
     Rule 2: all the cells between two corners inside the warehouse along a
             wall are taboo if none of these cells is a target.

    @param warehouse: a Warehouse object

    @return
       A string representing the puzzle with only the wall cells marked with
       an '#' and the taboo cells marked with an 'X'.
       The returned string should NOT have marks for the worker, the targets,
       and the boxes.
    '''

    '''
    Lecture code for the queens problem
    For all i,j,k (Xij, Xik) in {(0, 0),(0, 1),(1, 0)}
    For all i,j,k (Xij, Xij) in {(0, 0),(0, 1),(1, 0)}
    For all i,j,k (Xij, X(i+k,j+k)) in {(0, 0),(0, 1),(1, 0)}
    For all i,j,k (Xij, X(i+k,j-k)) in {(0, 0),(0, 1),(1, 0)}

    means they can't be in the same row and then they can't be in the same diagonals
    '''

    # Set the list up to print so i can see wtf i am doing.
    visualise = str(warehouse)

    level_row = [list(x) for x in visualise.split('\n')]
    level_col = [list(x) for x in zip(*level_row)]

    # In Python 3, map returns an iterator, so to get a list from the last solution
    # you need list(map(list, zip(*lis))) or [*map(list, zip(*lis))]

    # empty list for taboo location [x, y]
    global taboo
    taboo = []
    taboo_corner = []

    # dimensions of the warehouse loaded
    wall_x_max = len(level_col)
    wall_y_max = len(level_row)

    # First and last locations of the walls to define the playing area
    final_x_loc = []
    first_x_loc = []
    final_y_loc = []
    first_y_loc = []

    # since the printed version was already a list of lists it was easier to just find the last # which would be the end
    for i in range(wall_y_max):
        final_x = next(wall_x_max - i for i, j in enumerate(reversed(level_row[i]), 1) if j != ' ')
        first_x = next(row for row, x in enumerate(level_row[i]) if x != ' ')
        first_x_loc.append(first_x)
        final_x_loc.append(final_x)

    # since the printed version was already a list of lists it was easier to just find the last # which would be the end
    for i in range(wall_x_max):
        final_y = next(wall_y_max - i for i, j in enumerate(reversed(level_col[i]), 1) if j != ' ')
        first_y = next(col for col, y in enumerate(level_col[i]) if y != ' ')
        first_y_loc.append(first_y)
        final_y_loc.append(final_y)


    for i in range(wall_x_max - 1):
        for j in range(wall_y_max - 1):
            if j > first_y_loc[i] and j < final_y_loc[i] and i < final_x_loc[j] and i > first_x_loc[j] and (
                    i, j) not in warehouse.targets:
                if (i + 1, j) in warehouse.walls or (i - 1, j) in warehouse.walls:
                    if (i, j + 1) in warehouse.walls or (i, j - 1) in warehouse.walls:
                        if ([i, j]) not in taboo and (i, j) not in warehouse.walls:
                            taboo_corner.append([i, j])


    # Remove all items except taboo spaces and walls
    for i in range(wall_y_max - 1):
        level_row[i] = [item.replace('@', ' ')
                            .replace('*', ' ')
                            .replace('$', ' ')
                            .replace('.', ' ')
                        for item in level_row[i]]

    for i in range(wall_x_max - 1):
        level_col[i] = [item.replace('@', ' ')
                            .replace('*', ' ')
                            .replace('$', ' ')
                            .replace('.', ' ')
                        for item in level_col[i]]

    # Select the items in testy that correspond with taboo and replace them with an 'X'
    for ([i, j]) in taboo_corner:
        level_row[j][i] = 'X'  # i and j are back to front because they are lists of lists not an actual matrix

    # print('\n'.join(map(' '.join, level_row)))

    """
    1. Iterate through the length and width of the game space
    2. Find a location previously marked as a corner
    3. Start a new index and loop from the corner location
    4. Continue searching the row or column until you reach a wall or target. If so break the loop
    5. If the new index "x" has moved more than 1 space and reached another corner break that loop, x index should 
        remember the location of the next corner
    6. Finally i loop over the map array from index i to index x which should be from one corner to another with 
        no walls or targets along that row.
    7. Repeat for columns.
    """

    for i in range(wall_x_max - 1):
        for j in range(wall_y_max - 1):
            taboo_line = False  # I set this so the same line can have multiple lines against walls for the big maps
            if ([i, j]) in taboo_corner:
                for y in range(j + 1, wall_y_max - 1):
                    if (i, y) in warehouse.walls or (i, y) in warehouse.targets:
                        break
                    if (i + 1, y) not in warehouse.walls and (i - 1, y) not in warehouse.walls:
                        break
                    if ([i, y]) in taboo_corner:
                        for col in range(j + 1, y):
                            if ([i, col]) not in taboo_corner:
                                taboo.append([i, col])



    for j in range(wall_y_max - 1):
        for i in range(wall_x_max - 1):
            taboo_line = False
            if ([i, j]) in taboo_corner:
                for x in range(i + 1, wall_x_max):
                    if (x, j) in warehouse.walls or (x, j) in warehouse.targets:
                        break
                    if (x, j + 1) not in warehouse.walls and (x, j - 1) not in warehouse.walls:
                        break
                    if ([x, j]) in taboo_corner:
                        for row in range(i + 1, x):
                            if [row, j] not in taboo_corner:
                                taboo.append([row, j])



    for i in range(wall_x_max - 1):
        for j in range(wall_y_max - 1):
            if ([i, j]) in taboo:
                level_row[j][i] = 'X'

    # Turn the list of strings into 2 dimensions
    #print('\n'.join(map(' '.join, level_row)))
    taboo = taboo_corner + taboo
    taboo.sort()
    return ('\n'.join(map(''.join, level_row)))




# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def manhattan_distance(a, b):
    """
    Manhattan distance heuristic
    :param a: tuple 1
    :param b: tuple 2
    :return: The manhattan distance between two tuple
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class path_finder(search.Problem):
    """
    An instance of the path finder class represents a smaller problem
    to the SokobanPuzzle class. Path finder is used to see if a worker
    may reach a destination cell
    """
    def __init__(self, warehouse, goal, initial):
        self.puzzle = warehouse
        self.goal = goal
        self.initial = initial
        self.walls = self.puzzle.walls
        self.boxes = self.puzzle.boxes

    def actions(self, state):
        actions = list()
        for direction, movement in MOVEMENTS.items():
            new_state = tuple_add(state, movement)
            if new_state not in self.walls and new_state not in self.boxes:
                actions.append(direction)

        return actions

    def result(self, state, action):
        state = tuple_add(state, MOVEMENTS[action])
        return state

    def path_cost(self, c, state1, action, state2):
        return c + 1

    def goal_test(self, state):
        return state == self.goal

    def h(self, n):
        state = n.state
        return manhattan_distance(state, self.goal)

def can_go_there(warehouse, dst):
    '''
    Determine whether the worker can walk to the cell dst=(row,column)
    without pushing any box.

    @param warehouse: a valid Warehouse object

    @return
      True if the worker can walk to cell dst=(row,column) without pushing any box
      False otherwise

    RETURNS (X, Y)
    '''
    def h(n):
        state = n.state
        return manhattan_distance(state, dst)
    puzzle = path_finder(warehouse, dst, warehouse.worker)
    path = astar_graph_search(puzzle, h)

    return path is not None

def diagonal_neighbours(coordinate):
    """
    This function takes in a coordinate and calculates all the
    surrounding 2x2 coordinates
    :rtype: returns a nested tuple of coordinates
    """
    commands = ((-1, -1), (0, -1), (1, -1), (0, 0), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))
    neighbours = list()
    for command in commands:
        neighbours.append(tuple_add(coordinate, command))

    return neighbours

def neighbour_boxes(coordinate):
    """
    This function returns the 2x2 neighbours
    of a coordinate
    :rtype: List of neighboours
    """
    cells = diagonal_neighbours(coordinate)
    for n in [0, 1, 3, 4]:
        yield [ cells[n], cells[n+1], cells[n+3], cells[n+4] ]

def number_of_walls_or_boxes(wh, list_of_coordinates):
    """
    This function calculates the number of neighbouring
    walls and boxes
    :rtype: tuple of box and wall count
    """
    total_walls = 0
    total_boxes = 0
    for cell in list_of_coordinates:
        if cell in wh.walls:
            total_walls += 1
        elif cell in wh.boxes:
            total_boxes += 1

    return (total_walls, total_boxes)

def deadlock_check(warehouse, box):
    """
    This function calculates whether a box is within a deadlock
    position based of its 2x2 coordinate neighbours
    """
    for squares in neighbour_boxes(box):
        num_walls, num_boxes = number_of_walls_or_boxes(warehouse, squares)
        # Deadlock condition 1
        if num_boxes == 2 and num_walls == 2:
            return "DEADLOCK"
        # Deadlock condition 2
        elif num_boxes == 3 and num_walls == 1:
            return "DEADLOCK"
        # Deadlock condition 3
        elif num_boxes == 4:
            return "DEADLOCK"

    # Two boxes against a wall
    if (box + UP in warehouse.walls or box + DOWN in warehouse.walls) and \
            (box + LEFT in warehouse.boxes or box + RIGHT in warehouse.boxes):
        return "DEADLOCK"
    if (box + LEFT in warehouse.walls or box + RIGHT in warehouse.walls) and \
            (box + UP in warehouse.boxes or box + DOWN in warehouse.boxes):
        return "DEADLOCK"

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def update_warehouse(warehouse, state):
    """
    This function updates the warehouse to the current state
    :param warehouse: warehouse object
    :param state: state of worker and boxes
    """
    warehouse.boxes = state[1]
    warehouse.worker = state[0]
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
class SokobanPuzzle(search.Problem):
    def __init__(self, warehouse, allow_taboo_push, macro):
        self.puzzle = warehouse
        self.allow_taboo_push = allow_taboo_push
        self.macro = macro
        self.goal = self.puzzle.targets
        self.initial = (warehouse.worker, tuple(self.puzzle.boxes))
        self.boxes = self.puzzle.boxes
        self.walls = self.puzzle.walls
        self.taboo = taboo_cells(self.puzzle)
        self.taboo_coordinates = tuple(tuple(x) for x in taboo)

    def actions_elem(self, state):
        """
        Return the list of actions that can be executed in the given state.
        Actions can be defined as ["Up", "Down", "Left", "Right"]
        As specified in the header comment of this class, the attributes
        'self.allow_taboo_push' and 'self.macro' should be tested to determine
        what type of list of actions is to be returned.
        """
        worker, box_positions = state
        actions = list()

        # Check up, down, left, right
        for direction, movement in MOVEMENTS.items():
            # Move the worker
            worker_updated = tuple_add(worker, movement)
            # Move the box
            box_updated = tuple_add(worker_updated, movement)
            # Check if worker is moving into a wall
            if worker_updated not in self.walls:
                # Check if the worker is pushing a box
                if worker_updated in box_positions:
                    # Check if pushing box doesn't violate illegal conditions
                    if box_updated not in self.walls\
                                and box_updated not in self.taboo_coordinates\
                                and box_updated not in box_positions:
                        actions.append(direction)
                else:
                    actions.append(direction)
        return actions

    def actions_macro(self, state):
        """
        This function returns all available action at a current state
        :rtype: list of actions
        """
        worker, box_positions = state
        actions = []
        update_warehouse(self.puzzle, state)

        for box in box_positions:
            for direction, movement in MOVEMENTS.items():

                # Position of worker who is pushing box
                adjacent_position = tuple_subtract(box, movement)
                # Position of where the box is pushed
                updated_position = tuple_add(box, movement)
                # Check if updated box positon will cause a deadlock

                if updated_position not in self.taboo_coordinates\
                        and updated_position not in self.walls\
                        and updated_position not in box_positions:
                    if deadlock_check(self.puzzle, updated_position) is not "DEADLOCK":
                    # Check if the worker can reach pushing position
                        if can_go_there(self.puzzle, adjacent_position):
                            actions.append((box, direction))
        return actions

    def actions(self, state):
        if self.allow_taboo_push:
            return self.actions_elem(state)
        else:
            return self.actions_macro(state)

    def result_elem(self, state, action):

        worker, box_positions = state
        worker_updated = tuple_add(worker, MOVEMENTS[action])
        box_positions = list(box_positions)
        # Check if the worker is pushing a box

        if worker_updated in box_positions:
            # Update the positon of the box
            box_positions[box_positions.index(worker_updated)] = tuple_add(worker_updated, MOVEMENTS[action])
        result_state = (worker_updated, tuple(box_positions))

        return result_state

    def result_macro(self, state, actions):
        worker = state[0]
        boxes = state[1]
        boxes = list(boxes)
        # Workers location after box is pushed
        worker = actions[0]
        # Move the box
        box_updated = tuple_add(actions[0], MOVEMENTS[actions[1]])
        boxes[boxes.index(actions[0])] = box_updated
        boxes = tuple(boxes)

        return (worker, boxes)

    def result(self, state, action):
        if self.allow_taboo_push:
            return self.result_elem(state, action)
        else:
            return self.result_macro(state, action)

    def goal_test(self, state):
        return set(state[1]) == set(self.goal)

    def path_cost(self, c, state1, action, state2):
        return c + 1

    def h(self, n):
        """
        :rtype: returns the manhattan distance between worker and goals
        if elementary is involved.
        If macro is is set to true use the h function.
        """
        return h(self, n)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def h(problem, node):
    '''
    Heuristic for goal state, found through research.
    '''

    assert len(node.state[1]) == len(problem.goal)

    box_positions = list(node.state[1])
    target_positions = list(problem.goal)
    worker = node.state[0]

    ## Resulting heuristic
    total = 0

    while box_positions:
        minDistance = float('inf')
        minBox = box_positions[0]
        minTarget = box_positions[0]

        ## Find the minimum distance from 1 box to a target
        for box in box_positions:
            for target in target_positions:
                distance = manhattan_distance(box, target)
                if distance < minDistance:
                    minDistance = distance
                    minBox = box
                    minTarget = target

        box_positions.remove(minBox)
        target_positions.remove(minTarget)
        total += minDistance

    ## Repeat process for worker
    minWorkerDistance = float('inf')

    for box in box_positions:
        distance = manhattan_distance(box, worker)
        if distance < minWorkerDistance:
            minWorkerDistance = distance
    total += minWorkerDistance

    return total

def check_action_seq(warehouse, action_seq):
    '''

    Determine if the sequence of actions listed in 'action_seq' is legal or not.

    Important notes:
      - a legal sequence of actions does not necessarily solve the puzzle.
      - an action is legal even if it pushes a box onto a taboo cell.

    @param warehouse: a valid Warehouse object

    @param action_seq: a sequence of legal actions.
           For example, ['Left', 'Down', Down','Right', 'Up', 'Down']

    @return
        The string 'Failure', if one of the action was not successful.
           For example, if the agent tries to push two boxes at the same time,
                        or push one box into a wall.
        Otherwise, if all actions were successful, return
               A string representing the state of the puzzle after applying
               the sequence of actions.  This must be the same string as the
               string returned by the method  Warehouse.__str__()
    '''

    ##         "INSERT YOUR CODE HERE"
    (x, y) = warehouse.worker

    boxes = list(warehouse.boxes)
    print(action_seq)
    for action in action_seq:
        if action == "Up":
            dy = y - 1
            if (x, dy) in warehouse.walls:
                return "Failure"
            if (x, dy) in warehouse.boxes and ((x, dy - 1) in warehouse.walls or (x, dy - 1) in warehouse.boxes):
                return "Failure"
            elif (x, dy) in warehouse.boxes:
                boxes[boxes.index(x, dy)] = (x, dy - 1)
            else:
                y = dy
                warehouse.worker = (x, dy)
        elif action == "Down":
            dy = y + 1
            if (x, dy) in warehouse.walls:
                return "Failure"
            if (x, dy) in warehouse.boxes and ((x, dy + 1) in warehouse.walls or (x, dy + 1) in warehouse.boxes):
                return "Failure"
            elif (x, dy) in warehouse.boxes:
                boxes[boxes.index(x, dy)] = (x, dy - 1)
            else:
                y = dy
                warehouse.worker = (x, dy)
        elif action == "Left":
            dx = x - 1
            if (dx, y) in warehouse.walls:
                return "Failure"
            if (dx, y) in warehouse.boxes and ((dx - 1, y) in warehouse.walls or (dx - 1, y) in warehouse.boxes):
                return "Failure"
            elif (dx, y) in warehouse.boxes:
                boxes[boxes.index(dx, y)] = (dx - 1, y)
            else:
                x = dx
                warehouse.worker = (dx, y)
        elif action == "Right":
            dx = x + 1
            if (dx, y) in warehouse.walls:
                return "Failure"
            if (dx, y) in warehouse.boxes and ((dx + 1, y) in warehouse.walls or (dx + 1, y) in warehouse.boxes):
                return "Failure"
            elif (dx, y) in warehouse.boxes:
                boxes[boxes.index(dx, y)] = (dx + 1, y)
            else:
                x = dx
                warehouse.worker = (dx, y)

        updated_warehouse = warehouse
        updated_warehouse.worker = ([x, y])
        updated_warehouse.boxes = boxes
    #print(updated_warehouse)
    return str(updated_warehouse)

def solve_sokoban_elem(warehouse):
    '''
    This function should solve using elementary actions
    the puzzle defined in a file.

    @param warehouse: a valid Warehouse object

    @return
        If puzzle cannot be solved return the string 'Impossible'
        If a solution was found, return a list of elementary actions that solves
            the given puzzle coded with 'Left', 'Right', 'Up', 'Down'
            For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
            If the puzzle is already in a goal state, simply return []
    '''

    puzzle = SokobanPuzzle(warehouse, True, False)
    t0 = time.time()
    # sol = greedy_best_first_graph_search(puzzle, puzzle.h)
    sol = astar_graph_search(puzzle, puzzle.h)
    # sol = search.depth_first_graph_search(puzzle)
    # sol = breadth_first_graph_search(puzzle)

    t1 = time.time()
    print(' elem took {:.6f} seconds'.format(t1 - t0))
    if sol is None:
        return ['Impossible']
    else:
        print("LENGTH", len(sol.solution()))
        print("SOLUTION", sol.solution())
        return sol.solution()

def solve_sokoban_macro(warehouse):
    '''
    Solve using macro actions the puzzle defined in the warehouse passed as
    a parameter. A sequence of macro actions should be
    represented by a list M of the form
            [ ((r1,c1), a1), ((r2,c2), a2), ..., ((rn,cn), an) ]
    For example M = [ ((3,4),'Left') , ((5,2),'Up'), ((12,4),'Down') ]
    means that the worker first goes the box at row 3 and column 4 and pushes it left,
    then goes to the box at row 5 and column 2 and pushes it up, and finally
    goes the box at row 12 and column 4 and pushes it down.

    @param warehouse: a valid Warehouse object

    @return
        If puzzle cannot be solved return the string 'Impossible'
        Otherwise return M a sequence of macro actions that solves the puzzle.
        If the puzzle is already in a goal state, simply return []
    '''
    puzzle = SokobanPuzzle(warehouse, False, True)
    t0 = time.time()
    #sol = greedy_best_first_graph_search(puzzle, puzzle.h)
    sol = astar_graph_search(puzzle, puzzle.h)
    #sol = search.depth_first_graph_search(puzzle)
    #sol = breadth_first_graph_search(puzzle)

    t1 = time.time()
    print(' macro took {:.6f} seconds'.format(t1 - t0))
    if sol is None:
        return ['Impossible']
    else:
        print("LENGTH", len(sol.solution()))
        solution = list()
        for coord, box in sol.solution():
            solution.append(((coord[1], coord[0]), box))
        print("SOLUTION", solution)
        return solution

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def main():
    wh = sokoban.Warehouse()
    wh.load_warehouse("./warehouses/warehouse_57.txt")
    solve_sokoban_elem(wh)
    solve_sokoban_macro(wh)


if __name__ == '__main__':
    main()

