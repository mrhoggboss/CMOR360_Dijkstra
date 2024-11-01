import heapq

# Define the graph as an adjacency list
graph = {
    1: {2: 2, 3: 7},
    2: {1: 2, 3: 4, 4: 8, 5: 10},
    3: {1: 7, 2: 4, 4: 1, 6: 7},
    4: {2: 8, 3: 1, 5: 1, 7: 3},
    5: {2: 10, 4: 1, 6: 5, 7: 1},
    6: {3: 7, 5: 5, 7: 5},
    7: {4: 3, 5: 1, 6: 5}
}

def dijkstra(graph, start, end):
    '''
    graph: the undirected graph represented as an adjacency list
    start: the integer representing the starting node
    end: the integer representing the ending node

    retuirns: (the shortest distance, the shortest path represented by a list of nodes)
    '''
    # initialize distances with infinity and the start node's distance with 0
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    # Initialize a priority queue with the start node
    priority_queue = [(0, start)]
    
    # predecessors map to reconstruct the shortest path
    predecessors = {node: None for node in graph}

    while priority_queue:
        # get node with smallest distance from heap
        current_distance, current_node = heapq.heappop(priority_queue)

        # stop if end is reached
        if current_node == end:
            break

        # we update nothing if current dist is longer
        if current_distance > distances[current_node]:
            continue

        # Check neighbors and update distances
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            # Only consider this path if it's better
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                predecessors[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    # trace shortest path
    path = []
    node = end
    while node is not None:
        path.append(node)
        node = predecessors[node]
    path.reverse()

    return distances[end], path

start_node = 1
end_node = 7
#sanity check
assert(dijkstra(graph, 1, 7)[0] == dijkstra(graph, 7, 1)[0])
distance, path = dijkstra(graph, start_node, end_node)

# Output the results
print(f"Shortest distance from node {start_node} to node {end_node} is: {distance}")
print(f"Path: {' - '.join(map(str, path))}")
