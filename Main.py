# Program to solve maze using hamster robot 
from roboid import *
import time
import numpy as np 
import heapq # Required for A*

# --- Robot Configuration ---
hamster = HamsterS()
WALL_THRESHOLD = 25
STEP_DISTANCE = 9.0  #value to move from one cell to another 
STEP_SPEED = 55 # Speed of movement 
seconds = 0.2 # sleep time

# --- Alignment Settings ---
ALIGN_SPEED = 50 
ALIGN_DURATION = 0.6
ALIGN_PULL_AWAY = 2.2
SMALL_STEPS_FORWARD = 2.5
SMALL_STEPS_BACKWARD = 2.9
SMALL_STEPS_FORWARD_check = 1.5
SMALL_STEPS_BACKWARD_check = 1.5

# --- Map Definitions ---
UNKNOWN = "."
VISITED_CHAR = "*" 
END_GOAL = "E"
START_CHAR = "S"
ROBOT_HEADINGS = ["^", ">", "v", "<"]

# --- MAZE ---
MAZE_ROWS = 5
MAZE_COLS = 7
print_maze = [[UNKNOWN for _ in range(MAZE_COLS)] for _ in range(MAZE_ROWS)]
start_pos = (4,6)
end_pos = (2, 3)
print_maze[end_pos[0]][end_pos[1]] = END_GOAL 
current_pos = start_pos
current_heading = 0 # 0=N, 1=E, 2=S, 3=W

# walls stored as a set of blocked connections
known_walls = set() 
visited = set()

# --- A* Manhattan Distance ---
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# --- A* SEARCH ---
def get_a_star_path(start, goal, blocked_connections):
    """
    Standard A* Search.
    Uses the 'known_walls' set to find paths.
    """
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1] 

        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dr, current[1] + dc)
            nr, nc = neighbor
            
            if not (0 <= nr < MAZE_ROWS and 0 <= nc < MAZE_COLS):
                continue
            
            if (current, neighbor) in blocked_connections:
                continue
                
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
                
    return None # No path found

# --- Back and forth motion for alignment ---
def align_against_wall():
    hamster.move_backward(SMALL_STEPS_BACKWARD, STEP_SPEED) 
    hamster.move_forward(SMALL_STEPS_FORWARD, STEP_SPEED) 
    time.sleep(seconds) 

# --- check for walls on all sides
def check_paths_and_align():
    """Scans all 4 directions."""
    current_direction = "N"
    paths = {"front":"clear", "back":"clear", "right": "clear", "left": "clear"}

    # Get left and right proximity 
    cl1 = hamster.left_proximity()
    cr1 = hamster.right_proximity()
    hamster.move_backward(SMALL_STEPS_BACKWARD_check, STEP_SPEED) 
    cl2 = hamster.left_proximity()
    cr2 = hamster.right_proximity()
    print("left proximity: ", (np.floor((cl1+cl2)/2)))
    print("right proximity: ", (np.floor((cr1+cr2)/2)))
    hamster.move_forward(SMALL_STEPS_FORWARD_check, STEP_SPEED) 
    if (np.floor((cl1+cl2)/2)) > WALL_THRESHOLD:
        paths['left'] = 'wall'
       
    if (np.floor((cr1+cr2)/2)) > WALL_THRESHOLD:
        paths['right'] = "wall"
       
    # Get front and back proximity 
    hamster.turn_left(90,50)
    current_direction = "W"
    hamster.move_backward(SMALL_STEPS_BACKWARD_check, STEP_SPEED) 
    cl1 = hamster.left_proximity()
    cr1 = hamster.right_proximity()
    hamster.move_forward(SMALL_STEPS_FORWARD_check, STEP_SPEED) 
    cl2 = hamster.left_proximity()
    cr2 = hamster.right_proximity()
    print("back proximity: ", min(cl1, cl2))
    print("front proximity: ", min(cr1, cr2))
    if min(cl1, cl2)> WALL_THRESHOLD:
        paths['back'] = 'wall'
    if min(cr1, cr2)> WALL_THRESHOLD:
        paths["front"] = "wall"
        
    # reset back to the same position
    hamster.turn_right(90,50)
    current_direction = "N"


    # check where walls are and align by those walls 
    if paths['left'] == "wall":
        if current_direction == "N":
            hamster.turn_right(90,STEP_SPEED)
        if current_direction == "S":
            hamster.turn_left(90, STEP_SPEED)
        if current_direction == "W":
            hamster.turn_right(180, STEP_SPEED)
        current_direction = "E" 
        align_against_wall()
        time.sleep(seconds)

    if paths['right'] == "wall" :
        if current_direction == "N":
            hamster.turn_left(90,STEP_SPEED)
        if current_direction == "S":
            hamster.turn_right(90, STEP_SPEED)
        if current_direction == "E":
            hamster.turn_left(180, STEP_SPEED)
        current_direction = "W"
        align_against_wall()
        time.sleep(seconds)

    if paths["back"] == "wall":
        if current_direction == "W":
            hamster.turn_right(90,STEP_SPEED)
        if current_direction == "S":
            hamster.turn_right(180, STEP_SPEED)
        if current_direction == "E":
            hamster.turn_left(90, STEP_SPEED)
        current_direction = "N"
        align_against_wall()
        time.sleep(seconds)

    if paths["front"] == "wall":
        if current_direction == "W":
            hamster.turn_left(90,STEP_SPEED)
        if current_direction == "E":
            hamster.turn_right(90, STEP_SPEED)
        if current_direction == "N":
            hamster.turn_left(180, STEP_SPEED)
        current_direction = "S"
        align_against_wall()
        time.sleep(seconds)

    # reset back to the same direction 
    if current_direction == "W":
        hamster.turn_right(90,STEP_SPEED)
    if current_direction == "S":
        hamster.turn_right(180, STEP_SPEED)
    if current_direction == "E":
        hamster.turn_left(90, STEP_SPEED)
        current_direction = "N"

    return paths

# --- Forward movement ---
def move_one_step():
    """Moves the robot forward one full grid square."""
    hamster.move_forward(STEP_DISTANCE, STEP_SPEED)
    time.sleep(seconds)
    
    hamster.stop()
    time.sleep(seconds) 
    print_maze_map() # Call the new print function

# --- printing maze on every step 
def print_maze_map():
    """Prints the robot's current understanding of the maze."""
    print("--- Current Maze Memory ---")
    for r, row in enumerate(print_maze):
        row_str = []
        for c, cell in enumerate(row):
            if (r, c) == current_pos:
                row_str.append(ROBOT_HEADINGS[current_heading])
            elif (r, c) == end_pos:
                row_str.append(END_GOAL)
            elif (r, c) == start_pos:
                row_str.append(START_CHAR)
            elif print_maze[r][c] == VISITED_CHAR:
                row_str.append(VISITED_CHAR)

            else:
                row_str.append(UNKNOWN)
        print(" ".join(row_str))
    
    print(f"Known Walls: {known_walls}")
    print("---------------------------")

# --- checking which direction to move on the basis of next node and turn accordingly ---
def perform_move(direction):
    """Turns the robot, aligns, and moves one step."""
    print(f"Action: Moving {direction}")
    if direction == 'left':
        hamster.turn_left(90, STEP_SPEED)
        time.sleep(seconds)
    elif direction == 'right':
        hamster.turn_right(90, STEP_SPEED)
        time.sleep(seconds)
    elif direction == 'back':
        hamster.turn_right(180, STEP_SPEED)
        time.sleep(seconds)
    
    # After turning, align against the new "back" wall
    if direction != 'front':
        align_against_wall()
   
    move_one_step()

# --- This is for final run to make it move faster without aligning ---
def final_move(direction): 
    # turn direction on the basis of new node 
    print(f"Action: Moving {direction}")
    if direction == 'left':
        hamster.turn_left(90, 75)
    elif direction == 'right':
        hamster.turn_right(90, 75)
    elif direction == 'back':
        hamster.turn_right(180, 75)

    print("Moving 9cm...")
    hamster.move_forward(STEP_DISTANCE, 70)    
    hamster.stop()
    # --- END OF FIX ---

# --- Updates the 'known_walls' set with new information ---
def update_map_from_scan(pos, heading, sensor_data):
    coords = get_neighbor_coords(pos, heading)
    for direction, (nr, nc) in coords.items():
        if (0 <= nr < MAZE_ROWS and 0 <= nc < MAZE_COLS):
            if sensor_data.get(direction) == 'wall':
                # Add the wall in both directions
                known_walls.add( (pos, (nr, nc)) )
                known_walls.add( ((nr, nc), pos) )

# --- get the coordinates of neighbours ---
def get_neighbor_coords(pos, heading):
    (r, c) = pos
    moves = {
        0: {'front': (r-1, c), 'back': (r+1, c), 'left': (r, c-1), 'right': (r, c+1)},
        1: {'front': (r, c+1), 'back': (r, c-1), 'left': (r-1, c), 'right': (r+1, c)},
        2: {'front': (r+1, c), 'back': (r-1, c), 'left': (r, c+1), 'right': (r, c-1)},
        3: {'front': (r, c-1), 'back': (r, c+1), 'left': (r+1, c), 'right': (r-1, c)}
    }
    return moves[heading]

# --- get the direction of new node with respect to current direction 
def calculate_move_direction(current_pos, current_heading, next_pos):
    """Calculates relative turn needed to face the next node."""
    (cr, cc) = current_pos
    (nr, nc) = next_pos
    if (current_heading == 0 and nr < cr) or (current_heading == 1 and nc > cc) or \
       (current_heading == 2 and nr > cr) or (current_heading == 3 and nc < cc):
        return 'front', current_heading
    if (current_heading == 0 and nc < cc) or (current_heading == 1 and nr < cr) or \
       (current_heading == 2 and nc > cc) or (current_heading == 3 and nr > cr):
        return 'left', (current_heading - 1) % 4
    if (current_heading == 0 and nc > cc) or (current_heading == 1 and nr > cr) or \
       (current_heading == 2 and nc < cc) or (current_heading == 3 and nr < cr):
        return 'right', (current_heading + 1) % 4
    return 'back', (current_heading + 2) % 4

# --- Main Logic ---
path_to_goal = []

try:
    # === PART 1: EXPLORE TO GOAL ===
    print("Starting Optimistic A* Exploration...")
    print_maze[current_pos[0]][current_pos[1]] = VISITED_CHAR
    visited.add(current_pos) # Add the start position to visited

    while current_pos != end_pos:
        
        # 1. Update Map (Facts)
        sensor_data = check_paths_and_align()
        update_map_from_scan(current_pos, current_heading, sensor_data)

        # 2. Replan Path (The Brain)
        path_to_goal = get_a_star_path(current_pos, end_pos, known_walls)
        
        if not path_to_goal or len(path_to_goal) < 2:
            print("!! ERROR: No path to goal found! !!")
            break
            
        next_node = path_to_goal[1]
        
        # 3. Move to the next step
        print(f"Plan: {path_to_goal}")
        print(f"Moving to: {next_node}")
        
        direction, new_heading = calculate_move_direction(current_pos, current_heading, next_node)
        perform_move(direction)
        
        current_pos = next_node
        current_heading = new_heading
        visited.add(current_pos) # Add every new step to visited
        
        if current_pos == end_pos:
            print_maze[current_pos[0]][current_pos[1]] = END_GOAL
            print(f"--- 🏁 GOAL REACHED at {end_pos}! ---")
            break # Exit loop to start the return trip
        else:
            print_maze[current_pos[0]][current_pos[1]] = VISITED_CHAR
    
    # === PART 2: RETURN TO START ===
    if current_pos == end_pos:
        print("\n--- STARTING RETURN TO START ---")
        
        # 1. We are at the GOAL. We will now find the
        #    shortest path from the GOAL back to the START
        #    using the 'known_walls' map we just built.
        print(f"Calculating return path from {end_pos} to {start_pos}...")
        return_path = get_a_star_path(current_pos, start_pos, known_walls)
        
        if not return_path or len(return_path) < 2:
            print("!! ERROR: No path back to start found! !!")
        else:
            # 2. Follow the new return_path
            for next_node in return_path[1:]:      
                print(f"Return Path: {return_path}")
                print(f"Returning to: {next_node}")  
                direction, new_heading = calculate_move_direction(current_pos, current_heading, next_node)
                perform_move(direction)     
                current_pos = next_node
                current_heading = new_heading
                
                if current_pos == start_pos:
                    print(f"--- 🏠 RETURNED TO START at {start_pos}! ---")
                    break
        
    # === PART 3: FINAL RUN ===
    if current_pos == start_pos:
        time.sleep(1.5)
        print("\n--- STARTING FINAL RUN")
        
        final_path = return_path[::-1]
        hamster.turn_left(180, STEP_SPEED)
        current_heading = 0
        align_against_wall()
        if not final_path or len(final_path) < 2:
            print("!! Error. no path back to start found !!")
        else:
            for next_node in final_path[1:]:
                print(f"Start path: {final_path}")
                print(f"going to : {next_node}")
                direction, new_heading = calculate_move_direction(current_pos,current_heading, next_node)
                final_move(direction)
                current_pos = next_node
                current_heading = new_heading
                if current_pos == end_pos:
                    print(f"---👑 reached!!!! Hurrah")
                    break
        print_maze_map()


except KeyboardInterrupt:
    hamster.stop()
    print("Program stopped.")

except KeyboardInterrupt:
    hamster.stop()
    print("Program stopped.")