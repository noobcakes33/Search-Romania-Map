# graph is in adjacent list representation
graph = {
        'arad': ['sibiu', 'zerind', 'timisoara'],
        'sibiu': ['oradea', 'fagaras', 'rimnicu', 'arad'],
        'zerind': ['arad', 'oradea'],
        'timisoara': ['arad', 'lugoj'],
        'oradea': ['zerind', 'sibiu'],
        'fagaras': ['sibiu', 'bucharest'],
        'lugoj': ['timisoara', 'mehadia'],
        'mehadia': ['dobreta', 'lugoj'],
        'dobreta': ['lugoj', 'craiova'],
        'craiova': ['pitesti', 'rimnicu', 'dobreta'],
        'rimnicu': ['craiova', 'sibiu', 'pitesti'],
        'pitesti': ['craiova', 'rimnicu', 'bucharest'],
        'bucharest': ['fagaras', 'pitesti', 'giurgiu', 'urziceni'],
        'urziceni': ['bucharest', 'hirsova', 'vaslui'],
        'hirsova': ['eforie', 'urziceni'],
        'vaslui': ['lasi', 'urziceni'],
        'lasi': ['neamt', 'vaslui']

        }


def bfs(graph, start, end):
    # maintain a queue of paths
    queue = []
    # push the first path into the queue
    queue.append([start])
    while queue:
        # get the first path from the queue
        path = queue.pop(0)
        # get the last node from the path
        node = path[-1]
        # path found
        if node == end:
            return path
        # enumerate all adjacent nodes, construct a new path and push it into the queue
        for adjacent in graph.get(node, []):
            new_path = list(path)
            new_path.append(adjacent)
            queue.append(new_path)

def cost(path):
    distance = {
        'arad': ['sibiu', 'zerind', 'timisoara', 140, 75, 118],
        'sibiu': ['oradea', 'fagaras', 'rimnicu', 'arad', 151, 99, 80, 140],
        'zerind': ['arad', 'oradea', 75, 71],
        'timisoara': ['arad', 'lugoj', 118, 111],
        'oradea': ['zerind', 'sibiu', 71, 151],
        'fagaras': ['sibiu', 'bucharest', 99, 211],
        'lugoj': ['timisoara', 'mehadia', 111, 70],
        'mehadia': ['dobreta', 'lugoj', 75, 70],
        'dobreta': ['mehadia', 'craiova', 75, 120],
        'craiova': ['pitesti', 'rimnicu', 'dobreta', 138, 146, 120],
        'rimnicu': ['craiova', 'sibiu', 'pitesti', 146, 80, 97],
        'pitesti': ['craiova', 'rimnicu', 'bucharest', 138, 97, 101],
        'bucharest': ['fagaras', 'pitesti', 'giurgiu', 'urziceni', 211, 101, 90, 85],
        'urziceni': ['bucharest', 'hirsova', 'vaslui', 85, 98, 142],
        'hirsova': ['eforie', 'urziceni', 86, 98],
        'vaslui': ['lasi', 'urziceni', 92, 142],
        'lasi': ['neamt', 'vaslui', 87, 92]
    }

    cost_km = 0

    for city in range(len(path)-1):
        next_city_idx = distance[path[city]].index(path[city+1])
        x = len(distance[path[city]])
        cost_km += distance[path[city]][int(x/2) + next_city_idx]

    return cost_km

shortest_path = bfs(graph, 'arad', 'bucharest')
step_cost = cost(shortest_path)
print(shortest_path)
print(step_cost)
