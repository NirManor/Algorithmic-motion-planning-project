import numpy as np

class LayeredGraph:
    def __init__(self):
        self.layers = []  # list of layers, each layer is a list of configurations
        self.edges = []   # list of edges, each edge is a tuple (from_node, to_node, cost)

    def add_layer(self, configurations):
        layer_index = len(self.layers)
        self.layers.append(configurations)
        return layer_index

    def add_edge(self, from_node, to_node, cost):
        self.edges.append((from_node, to_node, cost))

    def connect_layers(self, bb, layer_index):
        if layer_index == 0:
            return  # No previous layer to connect to

        current_layer = self.layers[layer_index]
        previous_layer = self.layers[layer_index - 1]

        # Connect nodes within the same layer
        for i in range(len(current_layer)):
            for j in range(i + 1, len(current_layer)):
                if bb.local_planner(current_layer[i], current_layer[j]):
                    cost = bb.edge_cost(current_layer[i], current_layer[j])
                    self.add_edge((layer_index, i), (layer_index, j), cost)
                    self.add_edge((layer_index, j), (layer_index, i), cost)
                else:
                    print("can't extend due to collision")

        # Connect nodes with the previous layer
        for i in range(len(previous_layer)):
            for j in range(len(current_layer)):
                if bb.local_planner(previous_layer[i], current_layer[j]):
                    cost = bb.edge_cost(previous_layer[i], current_layer[j])
                    self.add_edge((layer_index - 1, i), (layer_index, j), cost)
                    self.add_edge((layer_index, j), (layer_index - 1, i), cost)

def build_graph(bb, ik_solutions_per_layer):
    graph = LayeredGraph()
    for layer_index, configurations in enumerate(ik_solutions_per_layer):
        graph.add_layer(configurations)
        graph.connect_layers(bb, layer_index)
    return graph

def main():
    from building_blocks import Building_Blocks
    from kinematics import UR5e_PARAMS, Transform
    from environment import Environment

    ur_params = UR5e_PARAMS()
    transform = Transform(ur_params)
    env = Environment(env_idx=3)
    bb = Building_Blocks(transform, ur_params, env)

    # Example data: list of layers, where each layer is a list of configurations
    ik_solutions_per_layer = [
        [np.random.rand(6) for _ in range(5)],  # Layer 0
        [np.random.rand(6) for _ in range(5)],  # Layer 1
        [np.random.rand(6) for _ in range(5)]   # Layer 2
    ]

    graph = build_graph(bb, ik_solutions_per_layer)
    print("Graph constructed with", len(graph.layers), "layers and", len(graph.edges), "edges")

if __name__ == "__main__":
    main()
