
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    odometry = gtsam.Pose2(2.0, 0.0, math.pi / 2)
    
    # Add BetweenFactorPose2 between X(3) and X(4)
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), odometry, ODOMETRY_NOISE))

    # Hardcode X(4) based on clean X(3) = (4, 0, 0) composed with odometry
    pose4 = gtsam.Pose2(4.0, 0.0, 0.0).compose(odometry)
    initial_estimate.insert(X(4), pose4)

    return graph, initial_estimate