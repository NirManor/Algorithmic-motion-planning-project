from LayeredGraph import LayeredGraph as LG
# import LayeredGraph as LG
from building_blocks import Building_Blocks
from kinematics import UR5e_PARAMS, Transform
from environment import Environment
from PathModel import PathModel
import numpy as np
from inverse_kinematics import Inverse_Kinematics


# TODO change the sine wave waypoints to a feasible line in the environment because now it's not feasible
# Sampling Sine wave for testing the path creation mechanism
def create_sine_wave_waypoints(num_points=100, amplitude=5, frequency=1, length=10):
    """Generate waypoints along a sine wave in 3D."""
    x = np.linspace(0, length, num_points)
    y = amplitude * np.sin(frequency * x)
    z = np.zeros_like(x)  # Keep z constant for simplicity
    return np.column_stack((x, y, z))


def main():
    ur_params = UR5e_PARAMS()
    transform = Transform(ur_params)
    env = Environment(env_idx=3)
    bb = Building_Blocks(transform, ur_params, env)
    waypointsArray= create_sine_wave_waypoints()
    path=PathModel(waypointsArray,waypointsArray[0],waypointsArray[-1])
    spline,smooth_path, tangents = path.process_path()


    # Initialize the graph
    graph = LG()
    ik_solutions_per_layer = []
    # Process each waypoint and its corresponding tangent

    # Initialize the inverse kinematics solver with initial parameters
    ik_solver = Inverse_Kinematics(waypointsArray[0][0], waypointsArray[0][1], waypointsArray[0][2], tangents[0])

    # Process each waypoint and its corresponding tangent
    for point, tangent in zip(smooth_path, tangents):
        tx, ty, tz = point
        # Update IK solver target and tangent
        ik_solver.update_target_and_tangent(tx, ty, tz, tangent)

        # Compute IK solutions for the current waypoint
        possible_configs = ik_solver.find_possible_configs()

        # Compute IK solutions for the current waypoint
        possible_configs = ik_solver.find_possible_configs()
        ik_solutions_per_layer.append(possible_configs)

        # Build the graph using valid configurations for each waypoint
    for layer_index, configurations in enumerate(ik_solutions_per_layer):
        graph.add_layer(configurations)
        graph.connect_layers(bb, layer_index)
# TODO create Dijkstra algorithm to find an optimal path



if __name__ == "__main__":
    main()
