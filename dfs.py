def dfs(g, start):
    stack, enqueued = [(None, start)], set([start])
    while stack:
        parent, n = stack.pop()
        yield parent, n
        new = set(g[n]) - enqueued
        enqueued |= new
        stack.extend([(n, child) for child in new])

def shortest_path(g, start, end):
    parents = {}
    for parent, child in dfs(g, start):
        parents[child] = parent
        if child == end:
            revpath = [end]
            while True:
                parent = parents[child]
                revpath.append(parent)
                if parent == start:
                    break
                child = parent
            return list(reversed(revpath))
    return None # or raise appropriate exception

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

if __name__ == '__main__':
    # a sample graph
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
    shortestPath = shortest_path(graph, 'arad', 'bucharest')
    step_cost = cost(shortestPath)
    print(shortestPath)
    print(step_cost)
