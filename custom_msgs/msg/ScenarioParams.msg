# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# The pose of the scoomatic, structured as [Position, Orientation]:
# [pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w]
float32[] scoomatic_pose

# The pose of the walker and the walking parameters, structured as [Location, Rotation, Walking_Direction, Walk_Speed]:
# [loc_x, loc_y, loc_z, pitch, yaw, roll, walk_x, walk_y, walk_z, walk_speed]
float32[] walker_params

# The goal of the navigation stack that should be driven to, structured as [Position, Orientation]:
# [pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w]
float32[] scoomatic_goal