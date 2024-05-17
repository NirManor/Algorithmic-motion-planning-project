import numpy as np

def compute_closest_point_on_path(point, splines):
    """
    Compute the closest point on the path (defined by splines) to the given point.
    This function should return the minimum distance from the point to the spline path.
    """
    # This is a simplified version; you'll need a more accurate method to find the closest point on a spline.
    t_values = np.linspace(0, 1, 500)  # More samples for higher accuracy
    spline_points = np.array([spline(t_values) for spline in splines]).T
    distances = np.linalg.norm(spline_points - point, axis=1)
    return np.min(distances)

class Building_Blocks(object):
    # Existing initialization and methods...

    def local_planner(self, prev_conf, current_conf, splines):
        """Check for collisions and path deviation between two configurations."""
        dist_prev_curr = np.linalg.norm(current_conf - prev_conf)
        num_intermediate_configs = max(int(np.ceil(dist_prev_curr / self.resolution)), MIN_RESOLUTION)
        intermediate_configs = np.linspace(prev_conf, current_conf, num_intermediate_configs)

        for config in intermediate_configs:
            if self.is_in_collision(config):
                return False

            # Compute end effector position using forward kinematics from Inverse_Kinematics
            transformation_matrix = self.inverse_kinematics.forward_kinematic_solution(self.inverse_kinematics.DH_matrix_UR5e, config)
            end_effector_pos = transformation_matrix[:3, 3]  # Assuming the position is in the last column

            # Calculate distance from the path
            if compute_closest_point_on_path(end_effector_pos, splines) > self.resolution:
                return False

        return True

    def set_inverse_kinematics(self, ik):
        self.inverse_kinematics = ik

# Example usage:
# Assuming 'inverse_kinematics' is an instance of Inverse_Kinematics and 'splines' is defined
bb = Building_Blocks(...)
bb.set_inverse_kinematics(inverse_kinematics)
