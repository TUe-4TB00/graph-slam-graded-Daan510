import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))

def add_landmark_measurement(graph, initial_estimate, result):
    # Get optimized pose of X(4) and position of L(2)
    pose4 = result.atPose2(X(4))
    landmark2 = result.atPoint2(L(2))

    # Compute bearing and range from X(4) to L(2)
    dx = landmark2[0] - pose4.x()
    dy = landmark2[1] - pose4.y()
    range_ = math.sqrt(dx**2 + dy**2)

    # Bearing is the angle to the landmark relative to the robot's heading
    global_angle = math.atan2(dy, dx)
    bearing = global_angle - pose4.theta()

    graph.add(gtsam.BearingRangeFactor2D(
        X(4), L(2),
        gtsam.Rot2(bearing),
        range_,
        MEASUREMENT_NOISE
    ))

    return graph