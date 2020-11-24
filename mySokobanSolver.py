'''

    2020 CAB320 Sokoban assignment


The functions and classes defined in this module will be called by a marker script. 
You should complete the functions and classes according to their specified interfaces.
No partial marks will be awarded for functions that do not meet the specifications
of the interfaces.


You are NOT allowed to change the defined interfaces.
That is, changing the formal parameters of a function will break the 
interface and results in a fail for the test of your code.
This is not negotiable! 


'''

# You have to make sure that your code works with 
# the files provided (search.py and sokoban.py) as your code will be tested 
# with these files
import search 
import sokoban
import time


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Helpers

def walls_On_Every_Side_Of_Worker( walls, coordinate ):
    ''' 
    This function returns coordinates of walls on every side of the worker
    
    Walls parameter: Object walls of the warehouse
    Coordinate parameter: Position of the worker
    '''
    
    ( x, y ) = coordinate

    walls_On_Every_Side_Of_Worker = []

    if ( x + 1, y ) in walls:
        walls_On_Every_Side_Of_Worker.append( ( x + 1, y ) )
    if ( x - 1, y ) in walls:
        walls_On_Every_Side_Of_Worker.append( ( x - 1, y ) )
    if ( x, y + 1 ) in walls:
        walls_On_Every_Side_Of_Worker.append( ( x, y + 1) )
    if ( x, y - 1 ) in walls:
        walls_On_Every_Side_Of_Worker.append( ( x, y - 1 ) )
    return walls_On_Every_Side_Of_Worker 

def calc_movement(current, direction):
    """
    Calculates new co-ordinate of current position in regards to direction
    """
    return (current[0] + direction[0], current[1] + direction[1])

def manhattan_distance(state1,state2):
    return abs(state1[0]-state2[0]) + abs(state1[1]-state2[1])

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def my_team():
    '''
    Return the list of the team members of this assignment submission as a list
    of triplet of the form (student_number, first_name, last_name)
    '''
    # return [ (1234567, 'Ada', 'Lovelace'), (1234568, 'Grace', 'Hopper'), (1234569, 'Eva', 'Tardos') ]
    return [ ( 10032029, 'Kaushal Kishorbhai', 'Limbasiya' ), (9954953, 'Lucas', 'Wickham'), (8890463, 'Michael', 'Gourlay') ]
    
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def taboo_cells(warehouse):
    '''  
    Identify the taboo cells of a warehouse. A cell inside a warehouse is 
    called 'taboo'  if whenever a box get pushed on such a cell then the puzzle 
    becomes unsolvable. Cells outside the warehouse should not be tagged as taboo.
    When determining the taboo cells, you must ignore all the existing boxes, 
    only consider the walls and the target  cells.  
    Use only the following two rules to determine the taboo cells;
     Rule 1: if a cell is a corner and not a target, then it is a taboo cell.
     Rule 2: all the cells between two corners along a wall are taboo if none of 
             these cells is a target.
    
    @param warehouse: 
        a Warehouse object with a worker inside the warehouse

    @return
       A string representing the puzzle with only the wall cells marked with 
       a '#' and the taboo cells marked with a 'X'.  
       The returned string should NOT have marks for the worker, the targets,
       and the boxes.  
    '''
    # Rule 1
    taboo_corners = []

    # limit each column
    col_start = [min((wy for (wx,wy) in warehouse.walls if wx == x), default=0) \
        for x in range(warehouse.ncols)]
    col_end = [max((wy for (wx,wy) in warehouse.walls if wx == x), default=warehouse.nrows) \
        for x in range(warehouse.ncols)]

    for y in range(warehouse.nrows):
        # limit the row
        row_start = min((wx for (wx,wy) in warehouse.walls if wy == y), default=0)
        row_end = max((wx for (wx,wy) in warehouse.walls if wy == y), default=warehouse.ncols)
        for x in range(warehouse.ncols):
            # check if current location is in wall space
            in_walls = x in range(row_start, row_end+1) and \
                y in range(col_start[x], col_end[x]+1)

            if (x,y) not in warehouse.walls and \
                (x,y) not in warehouse.targets and in_walls:
                # relative to current location
                wall_above = (x,y-1) in warehouse.walls
                wall_below = (x,y+1) in warehouse.walls
                wall_left = (x-1,y) in warehouse.walls
                wall_right = (x+1,y) in warehouse.walls

                # check if a corner
                if ((wall_above or wall_below) and \
                    (wall_left or wall_right)):
                    taboo_corners.append((x,y))
                    continue

    # Rule 2
    taboo_between = []

    # generator of lines between corners on the same x or y
    for ((x0, y0), (x1, y1)) in (((x0, y0), (x1, y1)) for (x0, y0) in taboo_corners \
        for (x1, y1) in taboo_corners if (x0, y0) != (x1, y1) and (x0 == x1 or y0 == y1)):
            between = []
            valid = True
            # on same x
            if x0 == x1:
                wall_left = True
                wall_right = True
                # check for continuous walls
                for y in range(min(y0,y1)+1, max(y0,y1)):
                    wall_left = wall_left and (x0-1,y) in warehouse.walls
                    wall_right = wall_right and (x0+1,y) in warehouse.walls
                    if (x0, y) not in warehouse.walls and \
                        (x0, y) not in warehouse.targets and \
                            (wall_left or wall_right):
                        between.append((x0, y))
                    else:
                        valid = False
                        continue
            # on same y
            elif y0 == y1:
                wall_above = True
                wall_below = True
                for x in range(min(x0,x1)+1, max(x0,x1)):
                    # check for continuous walls
                    wall_above = wall_above and (x,y-1) in warehouse.walls
                    wall_below = wall_below and (x,y+1) in warehouse.walls
                    if (x, y0) not in warehouse.walls and \
                        (x, y0) not in warehouse.targets and \
                            (wall_above or wall_below):
                        between.append((x, y0))
                    else:
                        valid = False
                        continue
            # if all the locations are valid
            if valid:
                taboo_between.extend(between)

    # merge the corners and cells between them to a string array
    taboo_str = []
    for y in range(warehouse.nrows):
        for x in range(warehouse.ncols):
            if (x,y) in warehouse.walls:
                taboo_str.append("#")
            elif (x,y) in taboo_corners or (x,y) in taboo_between:
                taboo_str.append("X")
            else:
                taboo_str.append(" ")
        if y < warehouse.nrows-1:
            taboo_str.append("\n")
        
    # join the string array to a string
    return "".join(taboo_str)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class SokobanPuzzle(search.Problem):
    '''
    An instance of the class 'SokobanPuzzle' represents a Sokoban puzzle.
    An instance contains information about the walls, the targets, the boxes
    and the worker.

    Your implementation should be fully compatible with the search functions of 
    the provided module 'search.py'. 
    
    Each SokobanPuzzle instance should have at least the following attributes
    - self.allow_taboo_push
    - self.macro
    
    When self.allow_taboo_push is set to True, the 'actions' function should 
    return all possible legal moves including those that move a box on a taboo 
    cell. If self.allow_taboo_push is set to False, those moves should not be
    included in the returned list of actions.
    
    If self.macro is set True, the 'actions' function should return 
    macro actions. If self.macro is set False, the 'actions' function should 
    return elementary actions.        
    '''
    
    def __init__(self, warehouse):
        self.warehouse = warehouse
        self.initial = (warehouse.worker, tuple(warehouse.boxes))
        self.goal = warehouse.targets
        self.walls = warehouse.walls
        # by default does not allow push of boxes on taboo cells
        self.allow_taboo_push = False
        # use elementary actions if self.macro == False
        self.macro = False
        self.taboo = sokoban.find_2D_iterator(taboo_cells(self.warehouse).split('\n'), 'X')
      

        # define the actions
    def actions_macro(self, state):
        '''
        return action as a direction in the (x, y) coordinate for macro

        @param state: coordinate of the worker and boxes

        '''
        state_Of_Worker = state[0]
        state_Of_Boxes = list(state[1])
        Directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        directions_In_Words = ['Left', 'Right', 'Up', 'Down']
        actions = []
        for boxState in state_Of_Boxes:
            taboo_cells_list = sokoban.find_2D_iterator(taboo_cells(self.warehouse).split('\n'), 'X')
            for dId, direction in enumerate(Directions):
                # moves boxes in the before state
                before_box_x = boxState[0] + direction[0]
                before_box_y = boxState[1] + direction[1]
                before_box_state = (before_box_x, before_box_y)
                # moves boxes in the after state
                after_box_x = boxState[0] - direction[0]
                after_box_y = boxState[1] - direction[1]
                after_box_state = (after_box_x, after_box_y)
                if after_box_state in self.warehouse.walls or before_box_state in self.warehouse.walls:
                    continue
                if after_box_state in state_Of_Boxes or before_box_state in state_Of_Boxes:
                    continue
                if not self.allow_taboo_push:
                    if after_box_state in taboo_cells_list:
                        continue
                temp_warehouse = self.warehouse.copy(state_Of_Worker, state_Of_Boxes)
                if can_go_there(temp_warehouse, (before_box_state[1], before_box_state[0]))  :
                    actions.append(((boxState[1], boxState[0]), directions_In_Words[dId]))
                    search
        return actions
    
    
    def actions_elementary(self,state):
        
        # MOVEMENT HELPERS
        UP = (0,-1,"Up")
        DOWN = (0 , 1 ,"Down")
        LEFT = (-1, 0 ,"Left")
        RIGHT = (1 ,0 ,"Right")
       
        worker_state = state[0]
        boxes_state = state[1]
        actions = []
        
        
        # WORKER MOVEMENT
        for direction in [UP,DOWN,LEFT,RIGHT]:
            new_state = calc_movement(worker_state, direction)
            
            if new_state not in walls_On_Every_Side_Of_Worker(self.walls, 
                                                                  worker_state):
                # BOX MOVEMENT
                if new_state in boxes_state:
                   
                    new_box = calc_movement(new_state, direction)
                    if new_box not in walls_On_Every_Side_Of_Worker(self.walls,worker_state):
                        if new_box not in boxes_state :
                
                                    actions.append(direction[2])
                        else:
                            break
                        
                else:
                    actions.append(direction[2])
        return actions

    def actions(self, state):
        """
        Return the list of actions that can be executed in the given state.
        
        As specified in the header comment of this class, the attributes
        'self.allow_taboo_push' and 'self.macro' should be tested to determine
        what type of list of actions is to be returned.
        """     
        if self.macro == False:
            return self.actions_elementary(state)
        else:
            return self.actions_macro(state)
        
    
    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
    
        
        if self.macro == False:
            
            worker_state = state[0]
            boxes_state = list(state[1])
             
            # WORKERS MOVEMENT
            if action == 'Left':
                new_state = worker_state[0] - 1 , worker_state[1]
            if action == 'Right':
                new_state = worker_state[0] + 1 , worker_state[1]     
            if action == 'Up':
                new_state = worker_state[0] , worker_state[1] - 1    
            if action == 'Down':
                new_state = worker_state [0] , worker_state[1] + 1
             
        
            # MOVE BOX ONLY WHEN WORKER IS ON BOX POSITION
            if new_state == boxes_state or new_state in boxes_state:
                if action == 'Left':
                    new_box = new_state[0] - 1 , new_state[1]
                if action == 'Right':
                    new_box = new_state[0] + 1 , new_state[1]     
                if action == 'Up':
                    new_box = new_state[0] , new_state[1] - 1    
                if action == 'Down':
                    new_box = new_state [0] , new_state[1] + 1
                
                boxes_state.remove(new_state)
                boxes_state.append(new_box)
                
        # ElEMENTARY ACTIONS
        else:
            worker_state = action[0]
            action = action[1]
            boxes_state = list(state[1])
            
            new_state = worker_state
            new_state = new_state[1], new_state[0] 
        
            # MOVE BOX ONLY WHEN WORKER IS ON BOX POSITION
            if new_state == boxes_state or new_state in boxes_state:
                if action == 'Left':
                    new_box = new_state[0] - 1 , new_state[1]
                if action == 'Right':
                    new_box = new_state[0] + 1 , new_state[1]     
                if action == 'Up':
                    new_box = new_state[0] , new_state[1] - 1    
                if action == 'Down':
                    new_box = new_state [0] , new_state[1] + 1
                
                boxes_state.remove(new_state)
                boxes_state.append(new_box)
        
        results = tuple(new_state), tuple(boxes_state)
        return results
    
        
    def path_cost(self, c, state1, action, state2):
        """
        Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path.
        """
        return c + 1 # Elementary cost is one
    
       
    
    def goal_test(self, state):
        """
        Return True if the state is a goal. The default method compares the
        state to self.goal, as specified in the constructor. Override this
        method if checking against a single self.goal is not enough.
        """  
        # check all boxes are on goals
        boxes = state[1]
        for box in boxes:
            if box not in self.goal:
                return False
        return True
        

    def h(self,node):  
        """
        Heuristic
        Distance 1-Calculates Manhatten Distance  between boxes and the worker .
        Distance sum-Between nearest box and target
        dist- both distances added together
        """
        boxes = list(node.state[1])
        worker = node.state[0]
        targets = self.goal
        dist_sum = 0
        
        for box in boxes:
            min_dist = float('inf')
            for target in targets:
                # distance between box and target
                new_dist = manhattan_distance(box,target)
                if new_dist < min_dist:
                    min_dist = new_dist
            dist_sum += new_dist

        return dist_sum
    
    def print_result(self, goal_node):
        """
        Shows solution represented by a specific goal node.
        For example, goal node could be obtained by calling 
            goal_node = breadth_first_tree_search(problem)
        """
        # path is list of nodes from initial state (root of the tree)
        # to the goal_node

        result=[]
        # Check if agent achieved the goal
        if goal_node is not None:
            path = goal_node.path()
            
            for count, node in enumerate(path):
                if node.action is not None:
                    result.append(node.action)
            return(result)
            
        #RETURN 'Impossible' if no path to goal node  
        else:
            return 'Impossible'
   
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def check_elem_action_seq(warehouse, action_seq):
    '''
    
    Determine if the sequence of actions listed in 'action_seq' is legal or not.
    
    Important notes:
      - a legal sequence of actions does not necessarily solve the puzzle.
      - an action is legal even if it pushes a box onto a taboo cell.
        
    @param warehouse: a valid Warehouse object

    @param action_seq: a sequence of legal actions.
           For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
           
    @return
        The string 'Impossible', if one of the action was not successul.
           For example, if the agent tries to push two boxes at the same time,
                        or push one box into a wall.
        Otherwise, if all actions were successful, return                 
               A string representing the state of the puzzle after applying
               the sequence of actions.  This must be the same string as the
               string returned by the method  Warehouse.__str__()
    '''

    def check_actions(warehouse,action):
        '''
        VALIDATES A SEQUENCE OF ACTIONS AND CHECKS IF LEGAL
        INPUT ACTION SEQUENCE
        OUTPUT: IF LEGAL, A WAREHOUSE REPRESENTATION AFTER ACTIONS
        IF ILLEGAL, STRING 'Impossible'   
        '''
        # MOVEMENT HELPERS
        UP = (0,-1,"Up")
        DOWN = (0 , 1 ,"Down")
        LEFT = (-1, 0 ,"Left")
        RIGHT = (1 ,0 ,"Right")
    
        worker_state = warehouse.worker
        boxes_state = warehouse.boxes
        #ASSIGN VARIABLE DIRECTION TO CURRENT ACTION SEQUENCE
        direction = action
        
        
        # WORKER MOVEMENT
        for direct in [UP,DOWN,LEFT,RIGHT]:
            if direction == direct[2]:
                direction = direct
                new_state = calc_movement(worker_state, direction)
            
        if new_state not in walls_On_Every_Side_Of_Worker(warehouse.walls, worker_state): 
                # CHECK IF WORKER PUSH BOX
                if new_state in boxes_state:
                    new_box = calc_movement(new_state, direction)
                    # CHECK LEGAL ACTIONS FOR BOXES
                    if new_box not in walls_On_Every_Side_Of_Worker(warehouse.walls, 
                                                                worker_state):
                        #CHECK IF BOX IS TRYING TO PUSH OTHER BOX
                        if new_box not in warehouse.boxes:
                            
                            warehouse.boxes.remove(new_state)
                            warehouse.boxes.append(new_box)
                        
                warehouse.worker = new_state
                return True
        else:
                return False
    
    for action in action_seq:
        if check_actions(warehouse,action) == False:
            return "Impossible"
        else:
            continue 
    return warehouse.__str__()

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def solve_sokoban_elem(warehouse):
    '''    
    This function should solve using A* algorithm and elementary actions
    the puzzle defined in the parameter 'warehouse'.
    
    In this scenario, the cost of all (elementary) actions is one unit.
    
    @param warehouse: a valid Warehouse object

    @return
        If puzzle cannot be solved return the string 'Impossible'
        If a solution was found, return a list of elementary actions that solves
            the given puzzle coded with 'Left', 'Right', 'Up', 'Down'
            For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
            If the puzzle is already in a goal state, simply return []
    '''
    
    problem = SokobanPuzzle(warehouse)
    node = search.astar_graph_search(problem)
    results = problem.print_result(node)

    return results
    
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class CanGoThereProblem(search.Problem):
    """
    Problem used by can_go_there
    """
    def __init__(self, initial, warehouse, goal):
        self.initial = initial
        self.warehouse = warehouse
        self.goal = goal
    
    def actions(self, state):
        offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for offset in offsets:
            new_state = ((state[0] + offset[0], state[1] + offset[1]))
            if new_state not in self.warehouse.boxes \
                    and new_state not in self.warehouse.walls:
                yield offset     

    def h(self, node):
        """
        Heuristic
        """
        return manhattan_distance(node.state, self.goal)
                
    def result(self, state, action):
        return ((state[0] + action[0], state[1] + action[1]))

def can_go_there(warehouse, dst):
    '''    
    Determine whether the worker can walk to the cell dst=(row,column) 
    without pushing any box.
    
    @param warehouse: a valid Warehouse object

    @return
      True if the worker can walk to cell dst=(row,column) without pushing any box
      False otherwise
    '''
    
    # warehouse is not always fully initialised
    warehouse.from_string(str(warehouse))

    # swap coordinates
    dst = (dst[1], dst[0])
    
    if dst in warehouse.walls or dst in warehouse.boxes:
        return False

    if dst == warehouse.worker:
        return True
    
    problem = CanGoThereProblem(warehouse.worker, warehouse, dst)
    node = search.astar_graph_search(problem)
    return node is not None

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def solve_sokoban_macro(warehouse):
    '''    
    Solve using using A* algorithm and macro actions the puzzle defined in 
    the parameter 'warehouse'. 
    
    A sequence of macro actions should be 
    represented by a list M of the form
            [ ((r1,c1), a1), ((r2,c2), a2), ..., ((rn,cn), an) ]
    For example M = [ ((3,4),'Left') , ((5,2),'Up'), ((12,4),'Down') ] 
    means that the worker first goes the box at row 3 and column 4 and pushes it left,
    then goes to the box at row 5 and column 2 and pushes it up, and finally
    goes the box at row 12 and column 4 and pushes it down.
    
    In this scenario, the cost of all (macro) actions is one unit. 

    @param warehouse: a valid Warehouse object

    @return
        If the puzzle cannot be solved return the string 'Impossible'
        Otherwise return M a sequence of macro actions that solves the puzzle.
        If the puzzle is already in a goal state, simply return []
    '''
    
    problem = SokobanPuzzle(warehouse)
    problem.macro = True

    # heuristic - maybe unneeded
    def h(node):
        boxes = list(node.state[1])
        goals = warehouse.targets
        total = 0
        for box in boxes:
            shortest = 10000000
            for goal in goals:
                distance = manhattan_distance(box,goal)
                if shortest == None or distance < shortest:
                    shortest = distance
            total = total + shortest
        return total

    start = time.time()
    node = search.astar_graph_search(problem)
    end = time.time()
    print (end - start)
    results = problem.print_result(node)
    return results

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class WeightedSokobanProblem(search.Problem):
    """
    Weighted Boxes version of the problem
    """
    def __init__(self, warehouse, push_costs = None):
        self.warehouse = warehouse
        self.initial = (warehouse.worker, tuple(warehouse.boxes))
        self.goal = warehouse.targets
        self.walls = warehouse.walls
        self.push_costs = push_costs
        self.taboo = sokoban.find_2D_iterator(taboo_cells(self.warehouse).split('\n'), 'X')
        
    
    def actions(self,state):
        
        # MOVEMENT HELPERS
        UP = (0,-1,"Up")
        DOWN = (0 , 1 ,"Down")
        LEFT = (-1, 0 ,"Left")
        RIGHT = (1 ,0 ,"Right")
       
        worker_state = state[0]
        boxes_state = state[1]
        actions = []
        
        
        # WORKER MOVEMENT
        for direction in [UP,DOWN,LEFT,RIGHT]:
            new_state = calc_movement(worker_state, direction)
            
            if new_state not in walls_On_Every_Side_Of_Worker(self.walls, 
                                                                  worker_state):
                # BOX MOVEMENT
                if new_state in boxes_state:
                   
                    new_box = calc_movement(new_state, direction)
                    if new_box not in walls_On_Every_Side_Of_Worker(self.walls,worker_state):
                        if new_box not in boxes_state :
                
                                    actions.append(direction[2])
                        else:
                            break
                        
                else:
                    actions.append(direction[2])
        return actions


        
    
    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        
      
        worker_state = state[0]
        boxes_state = list(state[1])
        
       
        if action == 'Left':
            new_state = worker_state[0] - 1 , worker_state[1]
        if action == 'Right':
            new_state = worker_state[0] + 1 , worker_state[1]     
        if action == 'Up':
            new_state = worker_state[0] , worker_state[1] - 1    
        if action == 'Down':
            new_state = worker_state [0] , worker_state[1] + 1
    
    
        # MOVE BOX ONLY WHEN WORKER IS ON BOX POSITION
        if new_state == boxes_state or new_state in boxes_state:
            if action == 'Left':
                new_box = new_state[0] - 1 , new_state[1]
            if action == 'Right':
                new_box = new_state[0] + 1 , new_state[1]     
            if action == 'Up':
                new_box = new_state[0] , new_state[1] - 1    
            if action == 'Down':
                new_box = new_state [0] , new_state[1] + 1
            
            boxes_state.remove(new_state)
            boxes_state.append(new_box)
     
            
        results = tuple(new_state), tuple(boxes_state)
        return results
    
    
        
    def path_cost(self, c, state1, action, state2):
        ''' 
        Assigns a push cost to appropriate box when moved
        Finds index of box and assigns it ith push cost
        state[1] = box in state before being pushed
        '''
       
        if state1[1] != state1[1]:
            box = state1[1].index(state2[0])
            path_cost = self.push_costs[box]
            return c + path_cost
           
        return c + 1  # Elementary cost is one
    
       
    
    def goal_test(self, state):
        """
        Return True if the state is a goal. The default method compares the
        state to self.goal, as specified in the constructor. Override this
        method if checking against a single self.goal is not enough.
        """
        # check all boxes are on goals
        boxes = state[1]
        for box in boxes:
            if box not in self.goal:
                return False
        return True
                
    
    
    def h(self,node):  
        """
        Heuristic
        Distance 1-Calculates Manhatten Distance  between boxes and the worker .
        Distance 3-Between nearest box and worker 
        distance_sum- both distances added together
        """
        
        
        # COMPLETED BUT NEEDS OPTIMIZING
        dist = 0
        worker = list(node.state[0])
        boxes = list(node.state[1])
        distance_sum = 0
        min_dist = float('inf')
        targets = self.goal
        
        
        for box in boxes:
            for target in targets:
            #DISANCE BETWEEN BOX AND GOAL
                distance1 = manhattan_distance(target,box)
                distance3 = manhattan_distance(worker,box)
                distance1 += distance3
                if distance1 < min_dist:
                    min_dist = distance1
                    distance_sum+=distance1
        
        dist = min_dist
        return dist
    
   
    def print_result(self, goal_node):
        """
        Shows solution represented by a specific goal node.
        For example, goal node could be obtained by calling 
            goal_node = breadth_first_tree_search(problem)
        """
        # path is list of nodes from initial state (root of the tree)
        # to the goal_node
        
        # Check if agent achieved the goal
        result=[]
        if goal_node != None:
            path = goal_node.path()
            
            for count, node in enumerate(path):
                if node.action is not None:
                    result.append(node.action)

            return result
        else:
            # RETURN 'Impossible' if no path to goal node
            return 'Impossible'

def solve_weighted_sokoban_elem(warehouse, push_costs):
    '''
    In this scenario, we assign a pushing cost to each box, whereas for the
    functions 'solve_sokoban_elem' and 'solve_sokoban_macro', we were 
    simply counting the number of actions (either elementary or macro) executed.
    
    When the worker is moving without pushing a box, we incur a
    cost of one unit per step. Pushing the ith box to an adjacent cell 
    now costs 'push_costs[i]'.
    
    The ith box is initially at position 'warehouse.boxes[i]'.
        
    This function should solve using A* algorithm and elementary actions
    the puzzle 'warehouse' while minimizing the total cost described above.
    
    @param 
     warehouse: a valid Warehouse object
     push_costs: list of the weights of the boxes (pushing cost)

    @return
        If puzzle cannot be solved return 'Impossible'
        If a solution exists, return a list of elementary actions that solves
            the given puzzle coded with 'Left', 'Right', 'Up', 'Down'
            For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
            If the puzzle is already in a goal state, simply return []
    '''
    
    problem = WeightedSokobanProblem(warehouse, push_costs)
    
    node = search.astar_graph_search(problem)

    return problem.print_result(node)
                 
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
