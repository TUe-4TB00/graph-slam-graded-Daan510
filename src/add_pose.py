
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    # Odometry: ~45 deg rotation, ~2m forward, ~45 deg more rotation = 90 deg total
    odometry = gtsam.Pose2(2.0, 0.0, math.pi / 2)

    # Add BetweenFactorPose2 between X(3) and X(4)
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), odometry, ODOMETRY_NOISE))

    # Compose X(3)'s current estimate with the odometry to get X(4)'s initial estimate
    pose3 = initial_estimate.atPose2(X(3))
    pose4 = pose3.compose(odometry)
    initial_estimate.insert(X(4), pose4)

    return graph, initial_estimate