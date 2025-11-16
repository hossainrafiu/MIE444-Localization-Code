from heapq import heappush, heappop
from typing import List, Tuple, Optional

class PathfindingRobot:
    """
    A simple-to-use pathfinding class for robot navigation.
    
    Block Types:
    0 - No Walls
    1 - One Wall
    2 - Two Adjacent Walls
    3 - 3 Walls
    4 - 4 Walls (Enclosed and Unreachable)
    5 - Two Opposite Walls
    """
    
    # Orientation mapping: 0=N, 1=E, 2=S, 3=W
    FORWARD_VECS = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    LEFT_VECS = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # lateral left per orientation
    RIGHT_VECS = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # lateral right per orientation
    
    default_game_map = [
        [2, 1, 5, 2, 4, 3, 4, 3],
        [1, 2, 4, 2, 5, 0, 5, 1],
        [5, 4, 3, 4, 4, 5, 4, 5],
        [2, 5, 1, 5, 5, 2, 4, 3],
    ]
    
    def __init__(self, pickup_location: Tuple[int, int] = (1, 1), 
                 dropoff_location: Tuple[int, int] = (3, 7), game_map: List[List[int]] = default_game_map, omnidrive: bool = True):
        """
        Initialize the pathfinding robot.
        
        Args:
            game_map: 2D grid representing the environment
            pickup_location: (row, col) of pickup point
            dropoff_location: (row, col) of dropoff point
            omnidrive: If True, robot can move sideways without rotating
        """
        self.game_map = game_map
        self.pickup_location = pickup_location
        self.dropoff_location = dropoff_location
        self.omnidrive = omnidrive
        self.carrying_load = False
        
    def set_load_status(self, carrying: bool):
        """Set whether the robot is currently carrying a load."""
        self.carrying_load = carrying
        
    def set_pickup_location(self, row: int, col: int):
        """Set the pickup location."""
        self.pickup_location = (row, col)
        
    def set_dropoff_location(self, row: int, col: int):
        """Set the dropoff location."""
        self.dropoff_location = (row, col)
        
    def _in_bounds(self, r: int, c: int) -> bool:
        """Check if coordinates are within map bounds."""
        return 0 <= r < len(self.game_map) and 0 <= c < len(self.game_map[0])
    
    def _traversable(self, r: int, c: int) -> bool:
        """Check if a cell is traversable (not type 4 - enclosed)."""
        return self.game_map[r][c] != 4
    
    def _get_neighbors(self, r: int, c: int) -> List[Tuple[int, int]]:
        """Get valid neighboring cells."""
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if self._in_bounds(nr, nc) and self._traversable(nr, nc):
                neighbors.append((nr, nc))
        return neighbors
    
    def _manhattan_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        """Calculate Manhattan distance between two points."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from A* search."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        Find optimal path from start to goal using A*.
        
        Args:
            start: Starting position (row, col)
            goal: Goal position (row, col)
            
        Returns:
            List of positions forming the path, or None if no path exists
        """
        if not self._traversable(start[0], start[1]) or not self._traversable(goal[0], goal[1]):
            return None

        open_heap = []
        heappush(open_heap, (0 + self._manhattan_distance(start, goal), 0, start))
        came_from = {}
        g_score = {start: 0}
        closed = set()

        while open_heap:
            _, g, current = heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor in self._get_neighbors(current[0], current[1]):
                tentative = g + 1
                if tentative < g_score.get(neighbor, 1e9):
                    g_score[neighbor] = tentative
                    came_from[neighbor] = current
                    f = tentative + self._manhattan_distance(neighbor, goal)
                    heappush(open_heap, (f, tentative, neighbor))

        return None
    
    def _delta_to_action(self, dr: int, dc: int, orientation: int) -> Optional[str]:
        """Map desired cell delta to a high-level action for the robot."""
        fdr, fdc = self.FORWARD_VECS[orientation]
        ldr, ldc = self.LEFT_VECS[orientation]
        rdr, rdc = self.RIGHT_VECS[orientation]
        bdr, bdc = -fdr, -fdc

        if self.omnidrive:
            # Can translate in any of the 4-neighbor directions without rotating
            if (dr, dc) == (fdr, fdc):
                return "forward"
            if (dr, dc) == (bdr, bdc):
                return "backward"
            if (dr, dc) == (ldr, ldc):
                return "left"  # lateral left
            if (dr, dc) == (rdr, rdc):
                return "right"  # lateral right
            return None
        else:
            # Must rotate to face the desired direction, then move forward
            if (dr, dc) == (fdr, fdc):
                return "forward"
            # If opposite, choose backward as a single action (cheaper than two turns)
            if (dr, dc) == (bdr, bdc):
                return "backward"
            # Need to rotate toward the desired delta
            # Determine which orientation matches target delta
            for target_ori, vec in enumerate(self.FORWARD_VECS):
                if (dr, dc) == vec:
                    diff = (target_ori - orientation) % 4
                    if diff == 1:
                        return "right"
                    elif diff == 3:
                        return "left"
                    elif diff == 2:
                        # ambiguous: either two rights or two lefts; choose one step
                        return "right"
            return None
    
    def get_next_action(self, current_row: int, current_col: int, current_orientation: int, 
                       target_row: int, target_col: int) -> Tuple[str, Optional[List[Tuple[int, int]]]]:
        """
        Get the next action to take towards a target.
        
        Args:
            current_row: Current row position
            current_col: Current column position  
            current_orientation: Current orientation (0=N, 1=E, 2=S, 3=W)
            target_row: Target row position
            target_col: Target column position
            
        Returns:
            Tuple of (action, path) where action is one of:
            'forward', 'backward', 'left', 'right', 'arrived', 'wait'
            and path is the full path to target or None
        """
        start = (current_row, current_col)
        goal = (target_row, target_col)

        if start == goal:
            return ("arrived", [start])

        path = self.find_path(start, goal)
        if not path or len(path) < 2:
            return ("wait", path)

        # Next waypoint is the second node in the path
        next_r, next_c = path[1]
        dr, dc = next_r - current_row, next_c - current_col
        action = self._delta_to_action(dr, dc, current_orientation)
        if action is None:
            # Fallback when mapping fails
            return ("wait", path)
        return (action, path)
    
    def get_current_objective(self) -> Tuple[int, int]:
        """Get the current objective location based on load status."""
        return self.dropoff_location if self.carrying_load else self.pickup_location
    
    def get_next_action_to_objective(self, current_row: int, current_col: int, 
                                   current_orientation: int) -> Tuple[str, Optional[List[Tuple[int, int]]]]:
        """
        Get the next action towards the current objective (pickup or dropoff).
        
        Args:
            current_row: Current row position
            current_col: Current column position
            current_orientation: Current orientation (0=N, 1=E, 2=S, 3=W)
            
        Returns:
            Tuple of (action, path) where action includes:
            'forward', 'backward', 'left', 'right', 'pickup', 'dropoff', 'wait'
        """
        target = self.get_current_objective()
        
        if (current_row, current_col) == target:
            return ("dropoff" if self.carrying_load else "pickup", [target])

        return self.get_next_action(current_row, current_col, current_orientation, 
                                  target[0], target[1])


# Example usage
if __name__ == "__main__":
    # Example game map
    game_map = [
        [2, 1, 5, 2, 4, 3, 4, 3],
        [1, 2, 4, 2, 5, 0, 5, 1],
        [5, 4, 3, 4, 4, 5, 4, 5],
        [2, 5, 1, 5, 5, 2, 4, 3],
    ]
    
    # Create pathfinding robot
    robot = PathfindingRobot(
        game_map=game_map,
        pickup_location=(1, 1),
        dropoff_location=(3, 7),
        omnidrive=False
    )
    
    # Example: robot at position (0,0) facing East, not carrying load
    current_r, current_c, current_ori = 0, 0, 1
    robot.set_load_status(False)
    
    # Get next action towards current objective
    action, path = robot.get_next_action_to_objective(current_r, current_c, current_ori)
    
    print(f"Robot at ({current_r},{current_c}) facing orientation {current_ori}")
    print(f"Current objective: {robot.get_current_objective()}")
    print(f"Next action: {action}")
    if path:
        print(f"Path length: {len(path)}")
        print(f"Full path: {path}")
        
    # Example: Get path to specific target
    target_r, target_c = 2, 3
    action2, path2 = robot.get_next_action(current_r, current_c, current_ori, target_r, target_c)
    print(f"\nPath to ({target_r},{target_c}): {action2}")
    if path2:
        print(f"Path: {path2}")
