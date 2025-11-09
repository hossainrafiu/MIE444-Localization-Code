import numpy as np
import matplotlib.pyplot as plt

# GameMap for Histogram Localization
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


class HistogramLocalization:
    def __init__(self, game_map, sensor_accuracy=0.8, omnidrive=False):
        """
        Initialize the histogram localization filter.

        Args:
            game_map: 2D list representing the maze with block types
            sensor_accuracy: Probability that sensor reading is correct (0.0 to 1.0)
            omnidrive: If True, 'left' and 'right' commands strafe laterally without rotation.
                       If False, 'left' and 'right' rotate the robot in place.
        """
        self.game_map = np.array(game_map)
        self.rows, self.cols = self.game_map.shape
        self.sensor_accuracy = sensor_accuracy
        # If omnidrive=True the robot can translate sideways without changing orientation
        self.omnidrive = omnidrive

        # 4 possible orientations: 0=North, 1=East, 2=South, 3=West
        self.num_orientations = 4

        # Initialize uniform probability distribution over (row, col, orientation)
        # Exclude block type 4 (enclosed/unreachable) from possible positions
        self.belief = np.ones((self.rows, self.cols, self.num_orientations))
        for i in range(self.rows):
            for j in range(self.cols):
                if self.game_map[i, j] == 4:
                    self.belief[i, j, :] = 0.0

        # Normalize the initial belief
        self.normalize_belief()

    def normalize_belief(self):
        """Normalize the probability distribution to sum to 1.0"""
        total = np.sum(self.belief)
        if total > 0:
            self.belief /= total
        else:
            # If all probabilities are zero, reinitialize uniformly
            self.belief = np.ones((self.rows, self.cols, self.num_orientations))
            for i in range(self.rows):
                for j in range(self.cols):
                    if self.game_map[i, j] == 4:
                        self.belief[i, j, :] = 0.0
            self.belief /= np.sum(self.belief)

    def update_belief(self, observed_block_type):
        """
        Update belief based on observed block type.

        Args:
            observed_block_type: The block type observed by the robot (0-5)
        """
        # Calculate likelihood for each position and orientation
        for i in range(self.rows):
            for j in range(self.cols):
                actual_block_type = self.game_map[i, j]

                # Calculate probability of this observation given the position
                if actual_block_type == observed_block_type:
                    # Correct match - high probability
                    likelihood = self.sensor_accuracy
                else:
                    # Incorrect match - distribute remaining probability among other types
                    # There are 6 block types (0-5), so if not matching, probability is distributed
                    likelihood = (1.0 - self.sensor_accuracy) / 5.0

                # Update belief for all orientations at this position
                # P(position,orientation|observation) ∝ P(observation|position) * P(position,orientation)
                self.belief[i, j, :] *= likelihood

        # Normalize to maintain valid probability distribution
        self.normalize_belief()

    def predict_motion(self, action, move_accuracy=0.8, strafe_accuracy=0.8):
        """Predict motion based on robot action with uncertainty.

        Supports two modes:
        - Normal (omnidrive=False): actions = forward/backward/left/right (rotate for left/right)
        - Omnidrive (omnidrive=True): actions = forward/backward/left/right (translate for left/right)

        Args:
            action: movement command
            move_accuracy: probability that forward/backward succeeds
            strafe_accuracy: probability that lateral translation succeeds (omnidrive only)
        """
        new_belief = np.zeros_like(self.belief)

        # Motion vectors for each orientation: [North, East, South, West]
        forward_motions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        # Lateral (left) motions relative to orientation (perpendicular to forward, CCW)
        lateral_left = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # From facing N,E,S,W
        lateral_right = [(0, 1), (1, 0), (0, -1), (-1, 0)]

        if action == "forward":
            for i in range(self.rows):
                for j in range(self.cols):
                    if self.game_map[i, j] == 4:
                        continue
                    for orientation in range(self.num_orientations):
                        dy, dx = forward_motions[orientation]
                        prev_i, prev_j = i - dy, j - dx
                        if 0 <= prev_i < self.rows and 0 <= prev_j < self.cols:
                            new_belief[i, j, orientation] += (
                                self.belief[prev_i, prev_j, orientation] * move_accuracy
                            )
                        new_belief[i, j, orientation] += self.belief[
                            i, j, orientation
                        ] * (1.0 - move_accuracy)

        elif action == "backward":
            for i in range(self.rows):
                for j in range(self.cols):
                    if self.game_map[i, j] == 4:
                        continue
                    for orientation in range(self.num_orientations):
                        dy, dx = forward_motions[orientation]
                        prev_i, prev_j = i + dy, j + dx
                        if 0 <= prev_i < self.rows and 0 <= prev_j < self.cols:
                            new_belief[i, j, orientation] += (
                                self.belief[prev_i, prev_j, orientation] * move_accuracy
                            )
                        new_belief[i, j, orientation] += self.belief[
                            i, j, orientation
                        ] * (1.0 - move_accuracy)

        elif action in ("left", "right"):
            if self.omnidrive:
                # Translate sideways without changing orientation
                lateral_vectors = lateral_left if action == "left" else lateral_right
                for i in range(self.rows):
                    for j in range(self.cols):
                        if self.game_map[i, j] == 4:
                            continue
                        for orientation in range(self.num_orientations):
                            dy, dx = lateral_vectors[orientation]
                            prev_i, prev_j = i - dy, j - dx
                            if 0 <= prev_i < self.rows and 0 <= prev_j < self.cols:
                                new_belief[i, j, orientation] += (
                                    self.belief[prev_i, prev_j, orientation]
                                    * strafe_accuracy
                                )
                            new_belief[i, j, orientation] += self.belief[
                                i, j, orientation
                            ] * (1.0 - strafe_accuracy)
            else:
                # Rotate in place, orientation changes
                for i in range(self.rows):
                    for j in range(self.cols):
                        for orientation in range(self.num_orientations):
                            if action == "left":
                                prev_orientation = (
                                    orientation + 1
                                ) % self.num_orientations
                            else:
                                prev_orientation = (
                                    orientation - 1
                                ) % self.num_orientations
                            new_belief[i, j, orientation] += (
                                self.belief[i, j, prev_orientation] * move_accuracy
                            )
                            new_belief[i, j, orientation] += self.belief[
                                i, j, orientation
                            ] * (1.0 - move_accuracy)
        else:
            print(f"Invalid action: {action}")
            return

        self.belief = new_belief
        self.normalize_belief()

    def get_most_likely_position(self):
        """
        Return the most likely position and orientation of the robot.

        Returns:
            tuple: (row, col, orientation) where orientation is 0=North, 1=East, 2=South, 3=West
        """
        max_idx = np.unravel_index(np.argmax(self.belief), self.belief.shape)
        return max_idx

    def get_position_probability(self, row, col, orientation=None):
        """
        Get the probability of being at a specific position and orientation.

        Args:
            row: Row index
            col: Column index
            orientation: Orientation (0-3), or None to sum over all orientations

        Returns:
            Probability value
        """
        if orientation is None:
            return np.sum(self.belief[row, col, :])
        else:
            return self.belief[row, col, orientation]

    def get_orientation_name(self, orientation):
        """Convert orientation index to name."""
        names = ["North", "East", "South", "West"]
        return names[orientation]

    def visualize_belief(self):
        """Visualize the current belief distribution (summed over all orientations)."""
        # Sum over all orientations to get position probability
        position_belief = np.sum(self.belief, axis=2)

        plt.figure(figsize=(12, 6))

        # Plot 1: Position probability
        plt.subplot(1, 2, 1)
        plt.imshow(position_belief, cmap="copper", interpolation="nearest")
        plt.colorbar(label="Probability")
        plt.title("Robot Position Belief Distribution")
        plt.xlabel("Column")
        plt.ylabel("Row")

        # Add grid
        for i in range(self.rows + 1):
            plt.axhline(i - 0.5, color="black", linewidth=0.5)
        for j in range(self.cols + 1):
            plt.axvline(j - 0.5, color="black", linewidth=0.5)

        # Annotate with probabilities
        for i in range(self.rows):
            for j in range(self.cols):
                text = plt.text(
                    j,
                    i,
                    f"{position_belief[i, j]:.3f}",
                    ha="center",
                    va="center",
                    color="white" if position_belief[i, j] < 0.5 else "black",
                    fontsize=8,
                )

        # Plot 2: Orientation distribution at most likely position
        plt.subplot(1, 2, 2)
        most_likely = self.get_most_likely_position()
        orientation_probs = self.belief[most_likely[0], most_likely[1], :] / np.sum(
            self.belief[most_likely[0], most_likely[1], :]
        )
        orientations = ["North", "East", "South", "West"]
        plt.bar(orientations, orientation_probs)
        plt.title(f"Orientation at ({most_likely[0]}, {most_likely[1]})")
        plt.ylabel("Probability")
        plt.ylim([0, 1])

        plt.tight_layout()
        plt.show()

    def print_belief_summary(self):
        """Print a summary of the current belief state."""
        most_likely_state = self.get_most_likely_position()
        max_prob = self.belief[most_likely_state]

        print("\n=== Histogram Localization Summary ===")
        print(
            f"Most likely position: Row {most_likely_state[0]}, Col {most_likely_state[1]}"
        )
        print(
            f"Most likely orientation: {self.get_orientation_name(most_likely_state[2])}"
        )
        print(f"Confidence: {max_prob * 100:.2f}%")
        print(
            f"Block type at most likely position: {self.game_map[most_likely_state[0], most_likely_state[1]]}"
        )
        print("\nTop 3 most likely states (position + orientation):")

        # Get top 3 states
        flat_belief = self.belief.flatten()
        top_3_indices = np.argsort(flat_belief)[-3:][::-1]

        for idx in top_3_indices:
            state = np.unravel_index(idx, self.belief.shape)
            prob = self.belief[state]
            block_type = self.game_map[state[0], state[1]]
            print(
                f"  Position ({state[0]}, {state[1]}), Orientation {self.get_orientation_name(state[2])}: "
                f"{prob * 100:.2f}% (Block type: {block_type})"
            )


# # Example usage
# if __name__ == "__main__":
#     # Initialize the localization system
#     # Toggle omnidrive here. Set omnidrive=True to enable strafing instead of rotation.
#     localizer = HistogramLocalization(gameMap, sensor_accuracy=0.8, omnidrive=False)

#     print("Initial belief state (uniform distribution):")
#     localizer.print_belief_summary()

#     # Simulate robot observations and movements
#     print("\n\n--- Observation 1: Robot sees block type 2 ---")
#     localizer.update_belief(observed_block_type=2)
#     localizer.print_belief_summary()

#     print("\n\n--- Robot turns right ---")
#     localizer.predict_motion("right", move_accuracy=0.9)

#     print("\n\n--- Robot moves forward ---")
#     localizer.predict_motion("forward", move_accuracy=0.9)

#     print("\n\n--- Observation 2: Robot sees block type 1 ---")
#     localizer.update_belief(observed_block_type=1)
#     localizer.print_belief_summary()

#     print("\n\n--- Robot moves forward ---")
#     localizer.predict_motion("forward", move_accuracy=0.9)

#     print("\n\n--- Observation 3: Robot sees block type 5 ---")
#     localizer.update_belief(observed_block_type=5)
#     localizer.print_belief_summary()

#     # Visualize the final belief distribution
#     localizer.visualize_belief()

# Works with user input
if __name__ == "__main__":

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

        action = input(
            "\nEnter robot action ('forward'(w), 'backward'(s), 'left'(a), 'right'(d)) or 'q' to quit: "
        )
        if action.lower() == "q":
            break
        if action not in ["w", "a", "s", "d"]:
            print(
                "Invalid action. Please enter 'forward', 'backward', 'left', or 'right'."
            )
            continue

        action_map = {"w": "forward", "s": "backward", "a": "left", "d": "right"}
        action = action_map[action]
        localizer.predict_motion(action)
        localizer.print_belief_summary()
        localizer.visualize_belief()
