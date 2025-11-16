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
import matplotlib.pyplot as plt
from scipy.stats import norm
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

    # Move particles with optimization
    # After moving the particles, we can check some of their sensors with the actual sensor readings
    # and test some small adjustments to the particles to see which gets then closer to the actual readings
    # keep the best adjustment for each particle
    def apply_optimization_to_particles(self, sensor_readings):
        print("Optimizing...")
        adjustments = [
            (0, 0, 0),
            (3, 0, 0),
            (-3, 0, 0),
            (0, 3, 0),
            (0, -3, 0),
            (0, 0, math.pi / 6),
            (0, 0, -math.pi / 6),
        ]  # small adjustments

        new_particles = []
        for p in self.particles:
            best_particle = p
            best_weight = p.weight
            for adj in adjustments:
                adj_x, adj_y, adj_theta = adj
                new_x = p.x + adj_x
                new_y = p.y + adj_y
                new_theta = p.theta + adj_theta
                temp_particle = Particle(new_x, new_y, new_theta, p.weight)
                predicted_readings = self.get_sensor_readings(temp_particle)
                weight = self.calculate_weight(predicted_readings, sensor_readings)
                if weight > best_weight:
                    best_weight = weight
                    best_particle = temp_particle._replace(weight=weight)
            new_particles.append(best_particle)
        self.particles = new_particles

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

    # IGNORE
    def move_particles_with_bounds(self, delta_x, delta_y, delta_theta):
        """Improved motion model with proper bounds handling"""
        new_particles = []

        for p in self.particles:
            # Add noise proportional to motion magnitude
            motion_magnitude = np.sqrt(delta_x**2 + delta_y**2)
            noise_scale = max(0.1, motion_magnitude * 0.1)

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

            # Method 1: Simple clamping (keeps particles at boundaries)
            new_x = np.clip(new_x, 0, 96)
            new_y = np.clip(new_y, 0, 48)

            # Method 2: Reflection (more physically realistic)
            # if new_x < 0:
            #     new_x = -new_x
            # elif new_x > 96:
            #     new_x = 2 * 96 - new_x
            #
            # if new_y < 0:
            #     new_y = -new_y
            # elif new_y > 48:
            #     new_y = 2 * 48 - new_y

            # Method 3: Soft boundary with penalty
            # Keep the particle but reduce its weight if it's out of bounds
            boundary_penalty = 1.0
            if new_x < 0 or new_x > 96 or new_y < 0 or new_y > 48:
                new_x = np.clip(new_x, 0, 96)
                new_y = np.clip(new_y, 0, 48)
                boundary_penalty = 0.1  # Heavily penalize but don't remove

            new_weight = p.weight * boundary_penalty
            new_particles.append(Particle(new_x, new_y, new_theta, new_weight))

        self.particles = new_particles

    # IGNORE
    def move_particles_collision_aware(self, delta_x, delta_y, delta_theta):
        """Advanced bounds checking that considers walls"""
        new_particles = []

        for p in self.particles:
            motion_magnitude = np.sqrt(delta_x**2 + delta_y**2)
            noise_scale = max(0.1, motion_magnitude * 0.1)

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

            # Check if the new position would be inside a wall
            new_point = Point(new_x, new_y)
            collision_penalty = 1.0

            # Check collision with walls
            for line in self.map_lines:
                wall = LineString(line)
                if new_point.distance(wall) < 1.0:  # Too close to wall
                    collision_penalty *= 0.5

            # Boundary checking
            if new_x < 0 or new_x > 96 or new_y < 0 or new_y > 48:
                new_x = np.clip(new_x, 0, 96)
                new_y = np.clip(new_y, 0, 48)
                collision_penalty *= 0.1

            new_weight = p.weight * collision_penalty
            new_particles.append(Particle(new_x, new_y, new_theta, new_weight))

        self.particles = new_particles

    # Main Move function
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

    # IGNORE
    def update_weights(self, sensor_readings):
        print("updating...")
        for i, p in enumerate(self.particles):
            predicted_readings = self.get_sensor_readings(p)
            self.particles[i] = p._replace(
                weight=self.calculate_weight(predicted_readings, sensor_readings)
            )
        # if (
        #     self.particles.__len__() < self.original_num_particles
        #     and sum(p.weight for p in self.particles) > 0.01
        # ):
        #     self.apply_optimization_to_particles(sensor_readings)

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

    # IGNORE
    def calculate_weight(self, predicted, actual):
        weight = 1.0
        for p, a in zip(predicted, actual):
            if a == -1 or p == -1:
                continue  # Skip if no reading
            weight /= (p - a) ** 2 + 1  # Avoid division by zero
        return weight

    def calculate_weight_gaussian(self, predicted, actual):
        """Calculate weight using Gaussian probability distribution"""
        weight = 1.0
        for i in range(len(predicted)):
            p = predicted[i]
            a = actual[i]
            # print(f"Predicted: {p}, Actual: {a}")
            if abs(p - a) > 6:
                return 0.0  # Discard particles with large errors
            if a == -1 or p == -1:
                continue
            # Use Gaussian probability density
            error = abs(p - a)
            weight *= np.exp(-(error**2) / (2 * self.sensor_noise**2))
        return weight

    def calculate_weight_log_likelihood(self, predicted, actual):
        """Calculate weight using log-likelihood for numerical stability"""
        log_likelihood = 0.0
        valid_measurements = 0

        for i, (p, a) in enumerate(zip(predicted, actual)):
            if a == -1 or p == -1:
                continue  # Skip if no reading

            valid_measurements += 1
            error = abs(p - a)

            # Different noise models for different sensors if needed
            sensor_std = self.sensor_noise
            if i in [1, 3, 5, 7]:  # Example: diagonal sensors might be noisier
                sensor_std *= 1.2

            # Log-likelihood calculation (log of Gaussian PDF)
            log_likelihood += -0.5 * (error / sensor_std) ** 2 - 0.5 * np.log(
                2 * np.pi * sensor_std**2
            )

        # If no valid measurements, return very small weight
        if valid_measurements == 0:
            return 1e-10

        # Convert back to probability (but keep it in a reasonable range)
        # Clamp the log-likelihood to prevent overflow
        log_likelihood = np.clip(log_likelihood, -50, 0)  # Prevent extreme values
        return np.exp(log_likelihood)

    def calculate_weight_robust_log(self, predicted, actual):
        """Robust log-likelihood with outlier handling"""
        log_likelihood = 0.0
        valid_measurements = 0

        for p, a in zip(predicted, actual):
            if a == -1 or p == -1:
                continue

            valid_measurements += 1
            error = abs(p - a)

            # Use robust error function (similar to Huber loss)
            if error <= 2 * self.sensor_noise:  # Normal case
                log_likelihood += -0.5 * (error / self.sensor_noise) ** 2
            else:  # Outlier case - use linear penalty instead of quadratic
                log_likelihood += -2 * error / self.sensor_noise + 2

        if valid_measurements == 0:
            return 1e-10

        # Normalize by number of measurements and convert to probability
        log_likelihood /= max(1, valid_measurements)
        log_likelihood = np.clip(log_likelihood, -20, 0)
        return np.exp(log_likelihood)

    def update_weights_log_stable(self, sensor_readings):
        """Weight update using log-likelihood for numerical stability"""
        log_weights = []

        for i, p in enumerate(self.particles):
            predicted_readings = self.get_sensor_readings(p)
            log_weight = self.calculate_log_likelihood(
                predicted_readings, sensor_readings
            )
            log_weights.append(log_weight)

        # Convert to linear weights with numerical stability
        max_log_weight = max(log_weights)
        weights = [np.exp(lw - max_log_weight) for lw in log_weights]

        # Normalize weights
        total_weight = sum(weights)
        if total_weight < 1e-50:
            print("Filter diverged, reinitializing...")
            self.initialize_particles()
            return

        # Update particle weights
        for i, (p, w) in enumerate(zip(self.particles, weights)):
            self.particles[i] = p._replace(weight=w / total_weight)

    def calculate_log_likelihood(self, predicted, actual):
        """Calculate log-likelihood directly (no conversion to probability)"""
        log_likelihood = 0.0
        valid_count = 0

        for p, a in zip(predicted, actual):
            if a == -1 or p == -1:
                continue

            valid_count += 1
            error = abs(p - a)
            log_likelihood += -0.5 * (error / self.sensor_noise) ** 2

        return log_likelihood if valid_count > 0 else -100  # Very small likelihood

    def normalize_weights(self, total_weight):
        print("normalizing...")
        self.particles = [
            Particle(p.x, p.y, p.theta, p.weight / total_weight) for p in self.particles
        ]

    def stochastic_resample_particles(self):
        cumulative_weights = np.cumsum([p.weight for p in self.particles])
        step = 1.0 / self.num_particles
        start = random.uniform(0, step)
        pointers = [start + i * step for i in range(self.num_particles)]
        new_particles = []
        index = 0
        for pointer in pointers:
            while pointer > cumulative_weights[index]:
                index += 1
            new_particles.append(
                Particle(
                    self.particles[index].x,
                    self.particles[index].y,
                    self.particles[index].theta,
                    1.0 / self.num_particles,
                )
            )
        self.particles = new_particles

    def resample_particles(self):
        if self.just_initialized:
            print("Skipping resample after reinitialization")
            self.just_initialized = False
            return
        weights = [p.weight for p in self.particles]
        if sum(weights) > 2:
            print("skipping resampling")
            return
        # check for Neff
        # Neff = 1.0 / sum(w**2 for w in weights)
        # if Neff > self.num_particles / 3:
        #     return  # No need to resample

        # adapted to deal with initialization with 2x particles
        indexes = rng.choice(
            range(self.particles.__len__()),
            size=self.num_particles // 3 * 2,
            replace=True,
            p=weights,
        )
        randomIndexes = rng.integers(
            0, self.particles.__len__(), size=self.num_particles // 3
        )

        new_particles = [self.particles[i] for i in np.hstack((indexes, randomIndexes))]
        # Reset weights after resampling
        self.particles = [
            Particle(
                p.x + random.uniform(-0.5, 0.5),
                p.y + random.uniform(-0.5, 0.5),
                p.theta,
                1.0 / self.num_particles,
            )
            for p in new_particles
        ]

    # Main resample function
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
                        p.x + np.random.normal(0, 0.5),
                        p.y + np.random.normal(0, 0.5),
                        p.theta + np.random.normal(0, 0.1),
                        1.0 / self.num_particles,
                    )
                )
            self.particles = new_particles

    # Not fully implemented
    def adaptive_particle_management(self):
        """Dynamically adjust particle count based on uncertainty"""
        weights = [p.weight for p in self.particles]
        entropy = -sum(w * np.log(w + 1e-100) for w in weights)  # Measure uncertainty

        if entropy > 3.0 and len(self.particles) < self.original_num_particles * 2:
            # High uncertainty, add particles
            self.add_random_particles(self.original_num_particles // 4)
        elif entropy < 1.0 and len(self.particles) > self.original_num_particles // 2:
            # Low uncertainty, reduce particles
            self.remove_low_weight_particles()

    def estimate_position(self):
        # Round to 2 decimal places
        x = float(sum(p.x * p.weight for p in self.particles))
        y = float(sum(p.y * p.weight for p in self.particles))
        theta = float(sum(p.theta * p.weight for p in self.particles))
        return (round(x, 2), round(y, 2), round(theta, 2))

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

    # Plot estimated position with arrow indicating orientation
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


# # Example usage
if __name__ == "__main__":
    # Define map walls as line segments
    map_lines = [
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
    likely_start_lines = [
        # (6,6) to (6,42)
        LineString([(6, 6), (6, 18)]),
        LineString([(6, 18), (6, 30)]),
        LineString([(6, 30), (6, 42)]),
        # (6,42) to (42,42)
        LineString([(6, 42), (18, 42)]),
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
        # (66,42) to (90,42)
        LineString([(66, 42), (78, 42)]),
        LineString([(78, 42), (90, 42)]),
        # (90,42) to (90,6)
        LineString([(90, 42), (90, 30)]),
        LineString([(90, 30), (90, 18)]),
        LineString([(90, 18), (90, 6)]),
    ]

    # Sensor directions (in radians)
    sensor_directions = [
        0,
        # math.pi / 4,
        math.pi / 2,
        # 3 * math.pi / 4,
        math.pi,
        # -math.pi / 4,
        -math.pi / 2,
        # -3 * math.pi / 4,
    ]

    # Simulate a series of movements and sensor readings
    movements = [
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 0, -math.pi / 2),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 0, -math.pi / 2),
        (0, 6, 0),
        (0, 6, 0),
        (0, 0, math.pi / 2),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 0, -math.pi / 2),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 0, -math.pi / 2),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 6, 0),
        (0, 0, -math.pi / 2),
    ]

    # Track performance for different motion noise levels
    list_of_errors = []

    motion_noise_values = np.arange(0, 1.0, 0.1)

    # Test different motion noise levels
    for motion_noise in [1.0]:

        # Create particle filter
        pf = ParticleFilter(
            num_particles=50,
            initial_multiplier=100,
            map_lines=map_lines,
            sensor_directions=sensor_directions,
            sensor_range=100,
            motion_noise=motion_noise,
            sensor_noise=2.0,
            likely_start_lines=likely_start_lines,
        )

        actual_position = [6, 6, 0]  # Starting position

        fig, ax = plt.subplots()

        pf.plot_particles(ax)

        for move in movements:
            delta_x, delta_y, delta_theta = move
            actual_position[0] += delta_x * math.cos(
                actual_position[2]
            ) - delta_y * math.sin(actual_position[2])
            actual_position[1] += delta_x * math.sin(
                actual_position[2]
            ) + delta_y * math.cos(actual_position[2])
            actual_position[2] += delta_theta

            # Get actual sensor readings from the actual position
            actual_particle = Particle(
                actual_position[0], actual_position[1], actual_position[2], 1.0
            )
            sensor_readings = pf.get_sensor_readings(actual_particle)
            print(f"Actual Sensor Readings: {sensor_readings}")

            # Move particles
            # pf.move_particles(delta_x, delta_y, delta_theta)
            # pf.move_particles_with_optimization(delta_x, delta_y, delta_theta, sensor_readings)
            pf.move_particles_improved(delta_x, delta_y, delta_theta)

            # quick plot
            plt.title("Particle Filter Next Motion Estimate")
            pf.plot_particles(ax)
            pf.plot_actual_position(ax, actual_position)
            plt.pause(0.1)
            ax.cla()  # Clear axis for next iteration

            # Update weights based on sensor readings
            # pf.update_weights(sensor_readings)
            # pf.update_weights_improved(sensor_readings)
            pf.update_weights_log_stable(sensor_readings)

            # Plot history of weights
            # plt.figure(2)
            # plt.plot(pf.weights_history)

            # Resample particles
            # pf.resample_particles()
            pf.resample_particles_improved()

            # Estimate position
            estimated_pos = pf.estimate_position()
            print(
                f"Estimated Position: {estimated_pos}, Actual Position: [{actual_position[0]:.2f}, {actual_position[1]:.2f}, {actual_position[2]:.2f}]"
            )
            # Plot particles and estimated position
            plt.title("Particle Filter After Resampling")
            pf.plot_particles(ax)
            pf.plot_estimated_position(ax, estimated_pos)
            pf.plot_actual_position(ax, actual_position)
            plt.pause(0.1)
            ax.cla()  # Clear axis for next iteration
        list_of_errors.append((motion_noise, pf.reinitializations))
        print(
            f"Motion Noise: {motion_noise}, Reinitializations: {pf.reinitializations}"
        )
    # plt.plot([e[0] for e in list_of_errors], [e[1] for e in list_of_errors])
    # plt.xlabel("Motion Noise")
    # plt.ylabel("Reinitializations")
    # plt.title("Particle Filter Performance")
    # plt.show()
