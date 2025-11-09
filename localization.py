# Creating code for particle filter localization
# Will model the walls of the map as line segments
# Each particle will have a position (x, y) and orientation (theta)
# The particles will be updated based on motion commands and sensor readings from external simulator
# The sensor readings will be distances to the nearest wall in certain directions
# The particles will be weighted based on how well their predicted sensor readings match the actual sensor readings
# The particles will be resampled based on their weights to focus on the most likely positions
# The final estimated position will be the weighted average of the particles' positions
# The code will include functions for initializing particles, updating particles based on motion,
# calculating weights based on sensor readings, resampling particles, and estimating the final position.

import numpy as np
import random
import math
from collections import namedtuple
from shapely.geometry import LineString, Point

# Define a named tuple for particles
Particle = namedtuple("Particle", ["x", "y", "theta", "weight"])

rng = np.random.default_rng()

# Define map walls as line segments
default_map_lines = [
    # Outer rectangle
    [(0, 0), (96, 0)],
    [(96, 0), (96, 48)],
    [(96, 48), (0, 48)],
    [(0, 48), (0, 0)],
    # Inner rectangle from 12,12 to 24,24
    [(12, 12), (24, 12)],
    [(24, 12), (24, 24)],
    [(24, 24), (12, 24)],
    [(12, 24), (12, 12)],
    # Inner rectangle from 24,24 to 36,36
    [(24, 24), (36, 24)],
    [(36, 24), (36, 36)],
    [(36, 36), (24, 36)],
    [(24, 36), (24, 24)],
    # Inner rectangle from 36,12 to 60,24
    [(36, 12), (60, 12)],
    [(60, 12), (60, 24)],
    [(60, 24), (36, 24)],
    [(36, 24), (36, 12)],
    # Inner rectangle from 48,36 to 60,48
    [(48, 36), (60, 36)],
    [(60, 36), (60, 48)],
    [(60, 48), (48, 48)],
    [(48, 48), (48, 36)],
    # Inner rectangle from 72,36 to 84,48
    [(72, 36), (84, 36)],
    [(84, 36), (84, 48)],
    [(84, 48), (72, 48)],
    [(72, 48), (72, 36)],
    # Inner rectangle from 72,0 to 84,24
    [(72, 0), (84, 0)],
    [(84, 0), (84, 24)],
    [(84, 24), (72, 24)],
    [(72, 24), (72, 0)],
]

# Split into 12 length segments
default_likely_start_lines = [
    # (6,6) to (6,18)
    LineString([(6, 6), (6, 18)]),
    # (30, 6) to (30, 18)
    LineString([(30, 6), (30, 18)]),
    # (24,42) to (42,42)
    LineString([(18, 42), (30, 42)]),
    LineString([(30, 42), (42, 42)]),
    # (42,42) to (42,30) to (66,30) to (66,6) to (6,6)
    LineString([(42, 42), (42, 30)]),
    LineString([(42, 30), (54, 30)]),
    LineString([(54, 30), (66, 30)]),
    LineString([(66, 30), (66, 18)]),
    LineString([(66, 18), (66, 6)]),
    LineString([(66, 6), (54, 6)]),
    LineString([(54, 6), (42, 6)]),
    LineString([(42, 6), (30, 6)]),
    LineString([(30, 6), (18, 6)]),
    LineString([(18, 6), (6, 6)]),
    LineString([(66, 30), (66, 42)]),
    # (66,30) to (90,30)
    LineString([(66, 30), (78, 30)]),
    LineString([(78, 30), (90, 30)]),
    # (90,42) to (90,6)
    LineString([(90, 42), (90, 30)]),
    LineString([(90, 30), (90, 18)]),
    LineString([(90, 18), (90, 6)]),
]

# Sensor directions (in radians)
default_sensor_directions = [
    0,
    math.pi / 2,
    math.pi,
    3 * math.pi / 2,
]


class ParticleFilter:
    def __init__(
        self,
        num_particles,
        initial_multiplier,
        sensor_range,
        sensor_noise,
        motion_noise,
        map_lines=None,
        likely_start_lines=None,
        sensor_directions=None,
    ):
        self.num_particles = num_particles
        self.original_num_particles = num_particles
        self.initial_multiplier = initial_multiplier
        self.particles = []
        self.map_lines = (
            map_lines if map_lines else default_map_lines
        )  # List of line segments representing walls
        self.likely_start_lines = (
            likely_start_lines if likely_start_lines else default_likely_start_lines
        )  # List of line segments representing likely start positions
        self.sensor_directions = (
            sensor_directions
            if sensor_directions
            else default_sensor_directions  # Directions in which the sensor measures distance
        )
        self.sensor_range = sensor_range  # Maximum range of the sensor
        self.motion_noise = motion_noise  # Noise in motion (std deviation)
        self.sensor_noise = sensor_noise  # Noise in sensor readings (std deviation)

        # Keep track of weights for debugging
        # self.weights_history = []

        # Keep track of stuff
        self.reinitializations = 0
        self.poor_performance_count = 0
        self.just_initialized = False

        self.initialize_particles()

    def initialize_particles(self, oversize=True, reset_particles=True):
        # Initialize particles near particular lines defined in likely_start_lines
        if reset_particles:
            self.particles = []
        self.reinitializations += 1
        self.num_particles = self.original_num_particles
        for line in self.likely_start_lines:
            for _ in range(
                self.original_num_particles
                * (self.initial_multiplier if oversize else 1)
                // len(self.likely_start_lines)
            ):
                x = random.uniform(line.coords[0][0], line.coords[1][0])
                y = random.uniform(line.coords[0][1], line.coords[1][1])
                # decide direction based on line orientation
                if line.coords[0][0] == line.coords[1][0]:  # vertical line
                    theta = random.choice([math.pi / 2, 3 * math.pi / 2])
                else:  # horizontal line
                    theta = random.choice([0, math.pi])
                weight = 1e-1  # small initial weight
                self.particles.append(Particle(x, y, theta, weight))

    # IGNORE
    def move_particles(self, delta_x, delta_y, delta_theta):
        new_particles = []
        for p in self.particles:
            # Apply motion with noise
            new_x = (
                p.x
                + delta_x * math.cos(p.theta)
                - delta_y * math.sin(p.theta)
                + np.random.normal(0, self.motion_noise)
            )
            new_y = (
                p.y
                + delta_x * math.sin(p.theta)
                + delta_y * math.cos(p.theta)
                + np.random.normal(0, self.motion_noise)
            )
            new_theta = (
                p.theta + delta_theta + np.random.normal(0, self.motion_noise / 10)
            )
            if new_x < 0 or new_x > 96 or new_y < 0 or new_y > 48:
                # If out of bounds, remove particle
                continue
            new_particles.append(Particle(new_x, new_y, new_theta, p.weight))
        self.particles = new_particles

    def move_particles_improved(self, delta_x, delta_y, delta_theta):
        """Improved motion model with better noise handling"""
        new_particles = []

        for p in self.particles:
            # Add noise proportional to motion magnitude
            motion_magnitude = np.sqrt(delta_x**2 + delta_y**2)
            noise_scale = max(0.2, motion_magnitude * 0.01)  # Adaptive noise

            # Apply motion with proper coordinate transformation
            cos_theta, sin_theta = np.cos(p.theta), np.sin(p.theta)
            new_x = (
                p.x
                + delta_x * cos_theta
                - delta_y * sin_theta
                + np.random.normal(0, noise_scale)
            )
            new_y = (
                p.y
                + delta_x * sin_theta
                + delta_y * cos_theta
                + np.random.normal(0, noise_scale)
            )
            new_theta = p.theta + delta_theta + np.random.normal(0, self.motion_noise)

            # Boundary checking with reflection instead of removal
            new_x = np.clip(new_x, 0, 96)
            new_y = np.clip(new_y, 0, 48)

            new_particles.append(Particle(new_x, new_y, new_theta, p.weight))

        self.particles = new_particles

    def get_sensor_readings(self, particle, add_noise=False):
        readings = []
        for direction in self.sensor_directions:
            angle = particle.theta + direction
            ray_end_x = particle.x + self.sensor_range * math.cos(angle)
            ray_end_y = particle.y + self.sensor_range * math.sin(angle)
            ray = LineString([(particle.x, particle.y), (ray_end_x, ray_end_y)])
            min_distance = self.sensor_range
            for line in self.map_lines:
                wall = LineString(line)
                if ray.intersects(wall):
                    intersection_point = ray.intersection(wall)
                    dist = Point(particle.x, particle.y).distance(intersection_point)
                    if dist < min_distance:
                        min_distance = dist
            if min_distance == self.sensor_range:
                # No intersection found within range
                readings.append(-1)  # Indicate no reading
            else:
                readings.append(
                    min_distance
                    + (np.random.normal(0, self.sensor_noise) if add_noise else 0)
                )
        return readings

    # Main weight update function
    def update_weights_improved(self, sensor_readings):
        """Simplified and improved weight update"""
        total_weight = 0.0
        max_weight = 0.0

        remove_indices = []
        for i, p in enumerate(self.particles):
            predicted_readings = self.get_sensor_readings(p)
            weight = self.calculate_weight_gaussian(predicted_readings, sensor_readings)
            if weight == 0.0:
                remove_indices.append(i)
                # print(f"Removing particle {i} due to zero weight")
                continue
            self.particles[i] = p._replace(weight=weight)
            total_weight += weight
            max_weight = max(max_weight, weight)

        # Remove particles with zero weight
        for i in reversed(remove_indices):
            self.particles.pop(i)

        if len(self.particles) == 0:
            print("All particles removed, reinitializing")
            self.initialize_particles()
            return

        total_weight = sum(p.weight for p in self.particles)
        print(f"Total weight: {total_weight}")
        max_weight = max(p.weight for p in self.particles)
        print(f"Max weight: {max_weight}")
        if total_weight > 0.1 or max_weight > 0.5:
            if self.num_particles >= self.original_num_particles:
                self.num_particles = int(self.num_particles * 0.8)
                print(f"Reducing particles to {self.num_particles}")
        if total_weight > 0.01 or max_weight > 0.001:
            self.poor_performance_count = 0
            print("Resetting poor performance count")
        if total_weight < 1e-4 or max_weight < 1e-5:
            self.poor_performance_count += 2
            self.initialize_particles(oversize=False, reset_particles=False)
            print(f"Poor performance count: {self.poor_performance_count}")
        elif total_weight < 1e-3 or max_weight < 1e-4:
            self.poor_performance_count += 1
            self.initialize_particles(oversize=False, reset_particles=False)
            print(f"Poor performance count: {self.poor_performance_count}")
        if self.poor_performance_count >= 5:
            print("Reinitializing particles due to poor performance")
            self.poor_performance_count = 0
            self.initialize_particles()
            self.just_initialized = True
            total_weight = sum(p.weight for p in self.particles)

        self.normalize_weights(total_weight)

    def calculate_weight_gaussian(self, predicted, actual):
        """Calculate weight using Gaussian probability distribution"""
        weight = 1.0
        for i in range(len(predicted)):
            p = predicted[i]
            a = actual[i]
            # print(f"Predicted: {p}, Actual: {a}")
            if (a > self.sensor_range and p == -1):
                continue
            if abs(p - a) > 6:
                return 0.0  # Discard particles with large errors
            # Use Gaussian probability density
            error = abs(p - a)
            weight *= np.exp(-(error**2) / (2 * self.sensor_noise**2))
        return weight

    def normalize_weights(self, total_weight):
        print("normalizing...")
        self.particles = [
            Particle(p.x, p.y, p.theta, p.weight / total_weight) for p in self.particles
        ]

    def resample_particles_improved(self):
        """Improved resampling with effective sample size check"""
        weights = np.array([p.weight for p in self.particles])

        # Calculate effective sample size
        weights_normalized = weights / np.sum(weights)
        eff_sample_size = 1.0 / np.sum(weights_normalized**2)

        # Only resample if effective sample size is too low
        if eff_sample_size < self.num_particles / 2:
            # Systematic resampling (better than random choice)
            cumsum = np.cumsum(weights_normalized)
            step = 1.0 / self.num_particles
            start = np.random.uniform(0, step)
            positions = np.arange(self.num_particles) * step + start

            indices = np.searchsorted(cumsum, positions)

            # Create new particles with small perturbation
            new_particles = []
            for idx in indices:
                p = self.particles[idx]
                new_particles.append(
                    Particle(
                        p.x + np.random.normal(0, 1),
                        p.y + np.random.normal(0, 1),
                        p.theta + np.random.normal(0, 0.1),
                        1.0 / self.num_particles,
                    )
                )
            self.particles = new_particles

    def estimate_position(self):
        # Round to 2 decimal places
        x = float(sum(p.x * p.weight for p in self.particles))
        y = float(sum(p.y * p.weight for p in self.particles))
        theta = float(sum(p.theta * p.weight for p in self.particles))
        return (round(x, 2), round(y, 2), round(theta, 2))

    def get_confidence(self):
        """
        Returns confidence based on spatial clustering of particles.
        Tight cluster = high confidence, spread out = low confidence.
        Returns: float between 0.0 and 1.0
        """
        if len(self.particles) < 2:
            return 0.0

        weights = np.array([p.weight for p in self.particles])
        xs = np.array([p.x for p in self.particles])
        ys = np.array([p.y for p in self.particles])

        # Weighted standard deviation
        mean_x = np.sum(xs * weights)
        mean_y = np.sum(ys * weights)

        var_x = np.sum(weights * (xs - mean_x) ** 2)
        var_y = np.sum(weights * (ys - mean_y) ** 2)

        # Combined spatial uncertainty (standard deviation)
        spatial_std = np.sqrt(var_x + var_y)

        # Convert to confidence:
        # std < 2 cm → 100% confidence
        # std > 20 cm → 0% confidence
        confidence = max(0.0, min(1.0, 1.0 - spatial_std / 20.0))

        return confidence * 100.0

    def plot_particles(self, ax):
        xs = [p.x for p in self.particles]
        ys = [p.y for p in self.particles]
        ax.cla()
        ax.scatter(xs, ys, c="blue", s=5, label="Particles")
        for line in self.map_lines:
            ax.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], "k-")
        for i in range(len(self.particles) // 10):
            p = self.particles[i * 10]
            ax.arrow(
                p.x,
                p.y,
                (2 * math.cos(p.theta)),
                (2 * math.sin(p.theta)),
                head_width=0.5,
                head_length=0.5,
                fc="r",
                ec="r",
            )
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 50)
        ax.set_aspect("equal")

    def plot_estimated_position(self, ax, estimated_pos):
        ax.plot(estimated_pos[0], estimated_pos[1], "ro", label="Estimated Position")
        ax.arrow(
            estimated_pos[0],
            estimated_pos[1],
            (5 * math.cos(estimated_pos[2])),
            (5 * math.sin(estimated_pos[2])),
            head_width=1,
            head_length=1,
            fc="r",
            ec="r",
        )

    def plot_actual_position(self, ax, actual_pos):
        ax.plot(actual_pos[0], actual_pos[1], "go", label="Actual Position")
        ax.arrow(
            actual_pos[0],
            actual_pos[1],
            5 * math.cos(actual_pos[2] + math.pi / 2),
            5 * math.sin(actual_pos[2] + math.pi / 2),
            head_width=1,
            head_length=1,
            fc="g",
            ec="g",
        )
        ax.legend()
