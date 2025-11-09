from histogram_localization import HistogramLocalization
from pathfinding import *

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

omnidrive_mode = input("Enable omnidrive mode? (y/n): ").strip().lower() == "y"
# Initialize the localization system
localizer = HistogramLocalization(
    gameMap, sensor_accuracy=0.8, omnidrive=omnidrive_mode
)

print("Initial belief state (uniform distribution):")
localizer.print_belief_summary()
localizer.visualize_belief()

# Simulate robot observations and movements
while True:
    obs = input("\nEnter observed block type (0-5) or 'q' to quit: ")
    if obs.lower() == "q":
        break
    try:
        observed_block_type = int(obs)
        if observed_block_type < 0 or observed_block_type > 5:
            raise ValueError
    except ValueError:
        print("Invalid input. Please enter a block type between 0 and 5.")
        continue

    localizer.update_belief(observed_block_type)
    localizer.print_belief_summary()
    localizer.visualize_belief()
    
    # Determine next action using pathfinding
    (current_r, current_c, current_ori) = localizer.get_most_likely_position()
    position_prob = localizer.get_position_probability(current_r, current_c)
    print(position_prob)

    if position_prob < 0.4:
        print(
            "Warning: Low confidence in current position estimate. Just move forward."
        )
    else:
        target_rc = goal_from_state(
            with_load, load_pick_up_location, unload_drop_off_location
        )
        action, path = next_action_to_objective(
            int(current_r),
            int(current_c),
            int(current_ori),
            with_load,
            omnidrive=omnidrive_mode,
            pickup=load_pick_up_location,
            dropoff=unload_drop_off_location,
        )
        print(f"Current: ({current_r},{current_c}) ori={current_ori}  Target: {target_rc}")
        print(f"Next action: {action}")
        if path:
            print(f"Path length: {len(path)}  Path: {path}")

    action = input(
        "\nEnter robot action ('forward'(w), 'backward'(s), 'left'(a), 'right'(d), 'loadAction'(l)) or 'q' to quit: "
    )
    if action.lower() == "q":
        break
    if action not in ["w", "a", "s", "d", "l"]:
        print(
            "Invalid action. Please enter 'forward', 'backward', 'left', 'right', or 'loadAction'."
        )
        continue

    action_map = {
        "w": "forward",
        "s": "backward",
        "a": "left",
        "d": "right",
        "l": "loadAction",
    }
    if action == "l":
        with_load = not with_load
        print(f"Load status changed. Now with_load = {with_load}")
    else:
        action = action_map[action]
        localizer.predict_motion(action)
    localizer.print_belief_summary()
    localizer.visualize_belief()
