# GameMap Configuration
# Block Types:
# 0 - No Walls
# 1 - One Wall
# 2 - Two Adjacent Walls
# 3 - 3 Walls
# 4 - 4 Walls (Enclosed and Unreachable)
# 5 - Two Opposite Walls
gameMap = [
    [2, 1, 5, 2, 4, 3, 4, 3],
    [1, 2, 4, 2, 5, 0, 5, 1],
    [5, 4, 3, 4, 4, 5, 4, 5],
    [2, 5, 1, 5, 5, 2, 4, 3],
]

load_pick_up_location = [1, 1]  # (row, col)
with_load = False
unload_drop_off_location = [3, 7]  # (row, col)


# ----------------------
# A* Pathfinding and Control
# ----------------------
from heapq import heappush, heappop
from typing import List, Tuple, Optional


# Toggle: prefer omnidrive style actions (translate sideways instead of rotate)
OMNIDRIVE_DEFAULT = False


def in_bounds(rows: int, cols: int, r: int, c: int) -> bool:
    return 0 <= r < rows and 0 <= c < cols


def traversable(grid: List[List[int]], r: int, c: int) -> bool:
    # Treat block type 4 (enclosed) as not traversable; all others are allowed
    return grid[r][c] != 4


def neighbors4(grid: List[List[int]], r: int, c: int) -> List[Tuple[int, int]]:
    rows, cols = len(grid), len(grid[0])
    nbrs = []
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if in_bounds(rows, cols, nr, nc) and traversable(grid, nr, nc):
            nbrs.append((nr, nc))
    return nbrs


def manhattan(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct_path(
    came_from: dict, current: Tuple[int, int]
) -> List[Tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(
    grid: List[List[int]], start: Tuple[int, int], goal: Tuple[int, int]
) -> Optional[List[Tuple[int, int]]]:
    """Simple A* on a 4-connected grid. Returns path including start and goal or None."""
    if not traversable(grid, start[0], start[1]) or not traversable(
        grid, goal[0], goal[1]
    ):
        return None

    open_heap = []
    heappush(open_heap, (0 + manhattan(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    closed = set()

    while open_heap:
        _, g, current = heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if current == goal:
            return reconstruct_path(came_from, current)

        for nbr in neighbors4(grid, current[0], current[1]):
            tentative = g + 1
            if tentative < g_score.get(nbr, 1e9):
                g_score[nbr] = tentative
                came_from[nbr] = current
                f = tentative + manhattan(nbr, goal)
                heappush(open_heap, (f, tentative, nbr))

    return None


# Orientation mapping: 0=N,1=E,2=S,3=W
FORWARD_VECS = [(-1, 0), (0, 1), (1, 0), (0, -1)]
LEFT_VECS = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # lateral left per orientation
RIGHT_VECS = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # lateral right per orientation


def delta_to_action(
    dr: int, dc: int, orientation: int, omnidrive: bool
) -> Optional[str]:
    """Map desired cell delta to a high-level action for the robot."""
    fdr, fdc = FORWARD_VECS[orientation]
    ldr, ldc = LEFT_VECS[orientation]
    rdr, rdc = RIGHT_VECS[orientation]
    bdr, bdc = -fdr, -fdc

    if omnidrive:
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
        for target_ori, vec in enumerate(FORWARD_VECS):
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


def decide_next_action(
    grid: List[List[int]],
    current_row: int,
    current_col: int,
    current_orientation: int,
    target_row: int,
    target_col: int,
    omnidrive: bool = OMNIDRIVE_DEFAULT,
) -> Tuple[str, Optional[List[Tuple[int, int]]]]:
    """
    Compute A* path to target and return the next high-level action.

    Returns: (action, path)
      - action: one of 'forward','backward','left','right','arrived','pickup','dropoff','wait','none'
      - path: list of (r,c) including start and goal, or None if no path
    """
    start = (current_row, current_col)
    goal = (target_row, target_col)

    if start == goal:
        return ("arrived", [start])

    path = astar(grid, start, goal)
    if not path or len(path) < 2:
        return ("wait", path)

    # Next waypoint is the second node in the path
    next_r, next_c = path[1]
    dr, dc = next_r - current_row, next_c - current_col
    action = delta_to_action(dr, dc, current_orientation, omnidrive)
    if action is None:
        # Fallback when mapping fails
        return ("wait", path)
    return (action, path)


def goal_from_state(
    carry_load: bool,
    pickup: Tuple[int, int],
    dropoff: Tuple[int, int],
) -> Tuple[int, int]:
    return tuple(dropoff) if carry_load else tuple(pickup)


def next_action_to_objective(
    current_row: int,
    current_col: int,
    current_orientation: int,
    carry_load: bool,
    omnidrive: bool = OMNIDRIVE_DEFAULT,
    pickup: Tuple[int, int] = tuple(load_pick_up_location),
    dropoff: Tuple[int, int] = tuple(unload_drop_off_location),
) -> Tuple[str, Optional[List[Tuple[int, int]]]]:
    """Convenience wrapper that picks pickup/dropoff as target and returns next action.

    When already at target, returns 'pickup' or 'dropoff' based on carry_load.
    """
    target = goal_from_state(carry_load, pickup, dropoff)
    if (current_row, current_col) == target:
        return ("dropoff" if carry_load else "pickup", [target])

    return decide_next_action(
        gameMap,
        current_row,
        current_col,
        current_orientation,
        target[0],
        target[1],
        omnidrive,
    )


# Optional demo for quick testing
if __name__ == "__main__":
    # Example state from a localizer estimate
    current_r, current_c, current_ori = 0, 0, 1  # facing East
    carry = with_load
    omni = OMNIDRIVE_DEFAULT

    target_rc = goal_from_state(
        carry, tuple(load_pick_up_location), tuple(unload_drop_off_location)
    )
    action, path = decide_next_action(
        gameMap, current_r, current_c, current_ori, target_rc[0], target_rc[1], omni
    )
    print(f"Current: ({current_r},{current_c}) ori={current_ori}  Target: {target_rc}")
    print(f"Next action: {action}")
    if path:
        print(f"Path length: {len(path)}  Path: {path}")
