import heapq
import math

# Step 1: Read coordinates from file
def read_coordinates_from_file(filename):
    coordinates = []
    try:
        with open(filename, 'r') as file:
            for line in file:
                line = line.replace(" ", "")
                parts = line.strip().split(',')
                if len(parts) >= 2:  # Ensure valid data
                    try:
                        lat = float(parts[0])
                        lon = float(parts[1])
                        coordinates.append((lat, lon))
                    except ValueError:
                        print(f"Skipping invalid line: {line.strip()}")
    except FileNotFoundError:
        print(f"File not found: {filename}")
    
    return coordinates


# Step 2: Euclidean distance between two coordinates
def euclidean_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)


# Step 3: Dijkstra algorithm for shortest path
def dijkstra(coords, start_index, end_index):
    n = len(coords)
    graph = {i: [j for j in range(n) if j != i] for i in range(n)}  # fully connected

    pq = [(0, start_index, [start_index])]
    visited = set()

    while pq:
        dist, current, path = heapq.heappop(pq)

        if current in visited:
            continue
        visited.add(current)

        if current == end_index:
            return dist, path  # distance and node indices path

        for neighbor in graph[current]:
            if neighbor not in visited:
                edge_weight = euclidean_distance(coords[current], coords[neighbor])
                heapq.heappush(pq, (dist + edge_weight, neighbor, path + [neighbor]))

    return float("inf"), []


# Step 4: Usage Example
if __name__ == "__main__":
    coords = read_coordinates_from_file("cords.txt")

    # Let's say A = 0th point, B = 4th point (you can change)
    start, end = 0, 4  

    distance, indices_path = dijkstra(coords, start, end)

    # Build final path with coordinates
    path = [coords[i] for i in indices_path]

    print(f"Shortest distance: {distance}")
    print(f"Path: {path}")
