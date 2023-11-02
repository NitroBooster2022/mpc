import numpy as np

def transformation_matrix(state1, state2):
    x1, y1, theta1 = state1
    x2, y2, theta2 = state2
    # Rotation matrix to align state1 with x-axis
    R1 = np.array([
        [np.cos(-theta1), -np.sin(-theta1), 0],
        [np.sin(-theta1), np.cos(-theta1), 0],
        [0, 0, 1]
    ])
    # Translation matrix to move origin of state1 to global origin
    T1 = np.array([
        [1, 0, -x1],
        [0, 1, -y1],
        [0, 0, 1]
    ])
    # Translation matrix to move to origin of state2
    T2 = np.array([
        [1, 0, x2],
        [0, 1, y2],
        [0, 0, 1]
    ])
    # Rotation matrix to align with state2
    R2 = np.array([
        [np.cos(theta2), -np.sin(theta2), 0],
        [np.sin(theta2), np.cos(theta2), 0],
        [0, 0, 1]
    ])
    # Combine transformations
    M = np.dot(T2, np.dot(R2, np.dot(T1, R1)))
    return M

def transform_pose(pose, M):
    x, y, theta = pose
    # Convert pose to homogeneous coordinates
    pose_homogeneous = np.array([x, y, 1])
    # Apply the transformation to get new x and y
    transformed_pose_homogeneous = np.dot(M, pose_homogeneous)
    # Compute new theta by adding the orientation difference between frame1 and frame2
    transformed_theta = (theta + (state2[2] - state1[2])) % (2 * np.pi)
    # Return the transformed pose
    return (transformed_pose_homogeneous[0], transformed_pose_homogeneous[1], transformed_theta)

def rotate_point(pt, theta):
    # Create the 2D rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    # Separate x, y, and psi from the waypoints
    xy = pt[:2]
    psi = pt[2]

    # Apply the rotation to x, y coordinates
    rotated_xy = np.dot(xy, rotation_matrix.T)  # The rotation matrix is transposed due to the way np.dot works

    # Apply the rotation to psi (yaw) angles and normalize them
    rotated_psi = (psi + theta) % (2 * np.pi)

    rotated_psi = np.arctan2(np.sin(rotated_psi), np.cos(rotated_psi))

    # Recombine into the complete array and return
    return np.array([rotated_xy[0], rotated_xy[1], rotated_psi])  # None is used to reshape psi from (n,) to (n,1)

def transform_point(pt, frame1, frame2):
    # Unpack the frames and the point
    x1, y1, theta1 = frame1
    x2, y2, theta2 = frame2
    x, y, psi = pt

    # Step 1: Translate to the origin of frame1
    x -= x1
    y -= y1

    # Step 2: Rotate to align frame1 with frame2
    # First, compute the rotation needed
    rotation_angle = theta2 - theta1
    # Create the rotation matrix
    rotation_matrix = np.array([
        [np.cos(rotation_angle), -np.sin(rotation_angle)],
        [np.sin(rotation_angle), np.cos(rotation_angle)]
    ])
    # Apply the rotation
    rotated_xy = np.dot(np.array([x, y]), rotation_matrix.T)

    # Update psi (yaw) and normalize
    rotated_psi = (psi + rotation_angle) % (2 * np.pi)
    rotated_psi = np.arctan2(np.sin(rotated_psi), np.cos(rotated_psi))

    # Step 3: Translate to the origin of frame2
    transformed_xy = rotated_xy + np.array([x2, y2])

    # Recombine into the complete array and return
    return np.array([transformed_xy[0], transformed_xy[1], rotated_psi])

# Example usage:
frame1 = np.array([0, 0, -np.pi])
frame2 = np.array([0, 0, -np.pi])
pt = np.array([1, 2, 0.1])

transformed_pt = transform_point(pt, frame1, frame2)
print(transformed_pt)
