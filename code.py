import heapq
class AgentTreasureScout:
    def __init__(self, map, weights):
        self.map = map
        self.weights = weights

    def distance_manhattan(self, point_a, point_b):
        # Calculate the Manhattan distance between two points.
        return abs(point_a[0] - point_b[0]) + abs(point_a[1] - point_b[1])

    def treasure_search(self, start, target):
        priority_queue = [(0, start)] # Priority queue with (cost + heuristic, node).
        costs = {start: 0} # Dictionary of costs from start to each node
        parent = {} # Dictionary to track the parents of each node

        while priority_queue:
            current_cost, current_node = heapq.heappop(priority_queue)

            if current_node == target:
                # we've found the treasure, we rebuild the path.
                path = [target]
                while current_node in parent:
                    current_node = parent[current_node]
                    path.append(current_node)
                path.reverse()
                return path

            # iterate through the current node's neighbour nodes
            for neighbour in self.map[current_node]:
                new_cost = costs[current_node] + self.weights.get((current_node, neighbour), float('inf'))
                if neighbour not in costs or new_cost < costs[neighbour]:
                    costs[neighbour] = new_cost
                    # Calculate the Manhattan distance from the neighbour to the target
                    heuristic = self.distance_manhattan(neighbour, target)
                    priority = new_cost + heuristic
                    heapq.heappush(priority_queue, (priority, neighbour))
                    parent[neighbour] = current_node

        # If no path is found, return None
        return None

    def has_found_treasure(self, path):
        # Check if the agent has found the treasure.
        return path is not None
if __name__ == "__main__":
    map = {
        (0, 0): [(1, 0), (0, 1)],
        (1, 0): [(0, 0), (2, 0)],
        (0, 1): [(0, 0), (0, 2)],
        (2, 0): [(1, 0), (3, 0)],
        (0, 2): [(0, 1), (1, 2)],
        (3, 0): [(2, 0), (4, 0)],
        (1, 2): [(0, 2), (1, 3)],
        (4, 0): [(3, 0), (4, 1)],
        (1, 3): [(1, 2), (2, 3)],
        (4, 1): [(4, 0), (4, 2)],
        (2, 3): [(1, 3), (3, 3)],
        (4, 2): [(4, 1), (5, 2)],
        (3, 3): [(2, 3), (4, 3)],
        (5, 2): [(4, 2), (5, 3)],
        (4, 3): [(3, 3), (5, 3)],
        (5, 3): [(4, 3)]
    }

    weights = {
        ((0, 0), (1, 0)): 1,
        ((0, 0), (0, 1)): 1,
        # Define the weights for other neighbouring nodes here
    }

    agent = AgentTreasureScout(map, weights)
    start = (0, 0)
    target = (5, 3)
    path = agent.treasure_search(start, target)

    if agent.has_found_treasure(path):
        print("Path found:")
        for point in path:
            print(point)
    else:
        print("No path to the treasure was found.")
