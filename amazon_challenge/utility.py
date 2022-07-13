import copy # this is very important, since default .copy() still track and modify the original dictionary somehow!
import sys
import subprocess
subprocess.check_call([sys.executable, "-m", "pip", "install", "matplotlib"])
import matplotlib.pyplot as plt
from matplotlib import lines
from itertools import combinations, permutations # to perform combinatorical methods working with some graph theoretical aspects
from math import dist, sqrt # equivalently ()**(0.5)
from random import randint

def dijkstra(original_graph, src):
    
    graph = copy.deepcopy(original_graph)
    
    length = len(graph)
    type_ = type(graph)
    if type_ == list:
        nodes = [i for i in range(length)]
    elif type_ == dict:
        nodes = list(graph.keys())

    visited = [src]
    path = {src:{src:[]}}
    nodes.remove(src)
    distance_graph = {src:0}
    pre = nxt = src

    
    while nodes:
        distance = float("inf")
        for v in visited:
            for d in nodes:
                new_dist = graph[src].get(v, float("inf")) + graph[v].get(d, float("inf"))
                if new_dist <= distance:
                    distance = new_dist
                    nxt = d
                    pre = v
                    graph[src][d] = new_dist


        path[src][nxt] = [i for i in path[src][pre]]
        path[src][nxt].append(nxt)

        distance_graph[nxt] = distance

        visited.append(nxt)
        nodes.remove(nxt)

    return distance_graph, path


def generate_graph(n_nodes=10):
    """
    """
    vertices = {i for i in permutations(range(n_nodes), 2)} | {(i, i) for i in range(n_nodes)}
#     print("Number of vertices:", len(vertices))

    graph = {}
    for (i, j) in vertices:
        if i == 0:
            if j == 0:
                graph[(i, j)] = {(i + 1, j + 1):sqrt(2), (i + 1, j):1, 
                                 (i, j + 1):1}
            elif j == n_nodes - 1:
                graph[(i, j)] = {(i + 1, j):1, (i, j - 1):1, (i + 1, j - 1):sqrt(2)}
            else:
                graph[(i, j)] = {(i + 1, j):1, (i + 1, j - 1):sqrt(2), 
                                 (i + 1, j + 1):sqrt(2), (i, j - 1):1, 
                                 (i, j + 1):1}
        elif i == n_nodes - 1:
            if j == 0:
                graph[(i, j)] = {(i - 1, j + 1):sqrt(2), (i - 1, j):1, 
                                 (i, j + 1):1}
            elif j == n_nodes - 1:
                graph[(i, j)] = {(i - 1, j):1, (i, j - 1):1, 
                                 (i - 1, j - 1):sqrt(2)}
            else:
                graph[(i, j)] = {(i - 1, j):1, (i - 1, j - 1):sqrt(2), 
                                 (i - 1, j + 1):sqrt(2), (i, j - 1):1, 
                                 (i, j + 1):1}
        else:
            if j == 0:
                graph[(i, j)] = {(i - 1, j):1, (i + 1, j):1, (i, j + 1):1, 
                                 (i - 1, j + 1):sqrt(2), (i + 1, j + 1):sqrt(2)}
            elif j == n_nodes - 1:
                graph[(i, j)] = {(i + 1, j):1, (i - 1, j):1, (i, j - 1):1, 
                                 (i - 1, j - 1):sqrt(2), (i + 1, j - 1):sqrt(2)}
            else:
                graph[(i, j)] = {(i, j - 1):1, (i, j + 1):1, (i - 1, j):1, (i + 1, j):1, 
                                 (i - 1, j - 1):sqrt(2), (i - 1, j + 1):sqrt(2), 
                                 (i + 1, j - 1):sqrt(2), (i + 1, j + 1):sqrt(2)}
                
        graph[(i, j)][(i, j)] = 0
    return graph

def put_obstacle(graph, vertex):
    
    (i, j) = vertex
    n = int(sqrt(len(graph))) - 1
    graph[vertex] = {}
    
    if i == 0:
        if j == 0:
            graph[(i + 1, j)].pop(vertex, None)
            graph[(i, j + 1)].pop(vertex, None)
            graph[(i + 1, j + 1)].pop(vertex, None)
            graph[(i + 1, j)].pop((i, j + 1), None)
            graph[(i, j + 1)].pop((i + 1, j), None)
        elif j == n:
            graph[(i, j - 1)].pop(vertex, None)
            graph[(i + 1, j)].pop(vertex, None)
            graph[(i + 1, j - 1)].pop(vertex, None)
            graph[(i, j - 1)].pop((i + 1, j) , None)
            graph[(i + 1, j)].pop((i, j - 1) , None)
        else:
            graph[(i, j - 1)].pop(vertex, None)
            graph[(i, j + 1)].pop(vertex, None)
            graph[(i + 1, j - 1)].pop(vertex, None)
            graph[(i + 1, j)].pop(vertex, None)
            graph[(i + 1, j + 1)].pop(vertex, None)
            graph[(i + 1, j)].pop((i, j + 1), None)
            graph[(i, j + 1)].pop((i + 1, j), None)
            graph[(i, j - 1)].pop((i + 1, j) , None)
            graph[(i + 1, j)].pop((i, j - 1) , None)
    elif i == n:
        if j == 0:
            graph[(i - 1, j)].pop(vertex, None)
            graph[(i, j + 1)].pop(vertex, None)
            graph[(i - 1, j + 1)].pop(vertex, None)
            graph[(i - 1, j)].pop((i, j + 1) , None)
            graph[(i, j + 1)].pop((i - 1, j) , None)
        elif j == n:
            graph[(i, j - 1)].pop(vertex, None)
            graph[(i - 1, j)].pop(vertex, None)
            graph[(i - 1, j - 1)].pop(vertex, None)
            graph[(i, j - 1)].pop((i - 1, j) , None)
            graph[(i - 1, j)].pop((i, j - 1) , None)
        else:
            graph[(i, j - 1)].pop(vertex, None)
            graph[(i, j + 1)].pop(vertex, None)
            graph[(i - 1, j - 1)].pop(vertex, None)
            graph[(i - 1, j)].pop(vertex, None)
            graph[(i - 1, j + 1)].pop(vertex, None)
            graph[(i, j - 1)].pop((i - 1, j) , None)
            graph[(i - 1, j)].pop((i, j - 1) , None)
            graph[(i - 1, j)].pop((i, j + 1) , None)
            graph[(i, j + 1)].pop((i - 1, j) , None)
    else:
        if j == 0:
            graph[(i - 1, j)].pop(vertex, None)
            graph[(i + 1, j)].pop(vertex, None)
            graph[(i - 1, j + 1)].pop(vertex, None)
            graph[(i, j + 1)].pop(vertex, None)
            graph[(i + 1, j + 1)].pop(vertex, None)
            graph[(i - 1, j)].pop((i, j + 1) , None)
            graph[(i, j + 1)].pop((i - 1, j) , None)
            graph[(i, j + 1)].pop((i + 1, j) , None)
            graph[(i + 1, j)].pop((i, j + 1) , None)
        elif j == n:
            graph[(i - 1, j)].pop(vertex, None)
            graph[(i + 1, j)].pop(vertex, None)
            graph[(i - 1, j - 1)].pop(vertex, None)
            graph[(i, j - 1)].pop(vertex, None)
            graph[(i + 1, j - 1)].pop(vertex, None)
            graph[(i, j - 1)].pop((i - 1, j) , None)
            graph[(i - 1, j)].pop((i, j - 1) , None)
            graph[(i, j - 1)].pop((i + 1, j) , None)
            graph[(i + 1, j)].pop((i, j - 1) , None)
        else:
            graph[(i - 1, j)].pop(vertex, None)
            graph[(i + 1, j)].pop(vertex, None)
            graph[(i, j - 1)].pop(vertex, None)
            graph[(i, j + 1)].pop(vertex, None)
            graph[(i + 1, j - 1)].pop(vertex, None)
            graph[(i + 1, j + 1)].pop(vertex, None)
            graph[(i - 1, j - 1)].pop(vertex, None)
            graph[(i - 1, j + 1)].pop(vertex, None)
            graph[(i, j - 1)].pop((i - 1, j) , None)
            graph[(i - 1, j)].pop((i, j - 1) , None)
            graph[(i, j - 1)].pop((i + 1, j) , None)
            graph[(i + 1, j)].pop((i, j - 1) , None)
            graph[(i - 1, j)].pop((i, j + 1) , None)
            graph[(i, j + 1)].pop((i - 1, j) , None)
            graph[(i, j + 1)].pop((i + 1, j) , None)
            graph[(i + 1, j)].pop((i, j + 1) , None)
    return graph

def remove_obstacles(graph, obstacles, start, destination):
    
    graph = copy.deepcopy(graph)

    n_steps, steps, dist = shortest_path(graph, obstacles, start, destination)
    if n_steps: # this will check that if the graph has already have a solution
        return set(),  steps, dist # returns an empty set

    removed_obstacles = None
    dist = float("inf")

    for n_removed in range(1, len(obstacles)): # starting from removing one obstacle up to all of them
        for selected_obstalces in list(combinations(obstacles, n_removed)): # then a brute-force kind of testing whether a group candidate of obstacles are sufficient

            temp_graph = copy.deepcopy(graph)

            for obstacle in obstacles - set(selected_obstalces):
                    temp_graph = put_obstacle(temp_graph, obstacle)

            distance, path = dijkstra(temp_graph, start)

            if distance[destination] < dist:
                dist = distance[destination]
                removed_obstacles = selected_obstalces
        
        if dist < float("inf"):
            break
            
    return set(removed_obstacles), [start] + path[start][destination], dist


def shortest_path(graph, obstacles, start, destination):

    graph = copy.deepcopy(graph)

    for obstacle in obstacles:
        graph = put_obstacle(graph, obstacle,)
    
    distance, path = dijkstra(graph, start)

    if (distance[destination] < float("inf")):
        n_steps = len(path[start][destination])
        steps = [start] + path[start][destination]
    else:
        return None, None, None

    return n_steps, steps, distance

def plot_results(graph, obstacles=None, start=None, destination=None, 
         show_paths=False, show_nodes=False, show_results=False,
         to_be_removed=set()):

    graph = copy.deepcopy(graph)

    fig, ax = plt.subplots()
    major_ticks = [i / 10 for i in range(-5, 105, 10)]
    fig.set_figheight(10)
    fig.set_figwidth(10)
#     plt.axis("off")
    ax.tick_params(axis='x', colors=(0,0,0,0))
    ax.tick_params(axis='y', colors=(0,0,0,0))
    ax.grid(True, which='both', linewidth=4)
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)
    plt.margins(0.05,0.05)
#     ax.set_facecolor((1.0, 0.47, 0.42))
    plt.tight_layout()
    plt.xlim([-0.5, 9.5])
    plt.ylim([-0.5, 9.5])
    
    # handling obstacles (showing and affecting the graph)
    if obstacles:
        for obstacle in obstacles:
            img = plt.imread("img/obstacle-cube.png", format="png")
    #         img[(img[:,:,0] == 1)&(img[:,:,1] == 1)&(img[:,:,2] == 1)] = float('nan')
            frame_height = 0.8
            x_coord = obstacle[0] - 0.4
            y_coord = obstacle[1] + 0.4
            i, j = obstacle[0], 9 - obstacle[1]
            ax.imshow(img, extent=[x_coord , x_coord + frame_height,
                                9 - y_coord, 9 - y_coord + frame_height],)
            if obstacle in to_be_removed:
                ax.plot([i -.4, i + .4],
                [j - .4, j + .4], 'red', linewidth=3)
                ax.plot([i - .4, i + .4],
                [j + .4, j - .4], 'red', linewidth=3)
            else:
                graph = put_obstacle(graph, obstacle,) # removing affected edges
    
    if start:
        img = plt.imread("img/starting.png", format="png")
#         img[(img[:,:,0] == 1)&(img[:,:,1] == 1)&(img[:,:,2] == 1)] = float('nan')
        frame_height = 0.8
        x_coord = start[0] - 0.4
        y_coord = start[1] + 0.40
        ax.imshow(img, extent=[x_coord , x_coord + frame_height,
                              9 - y_coord, 9 - y_coord + frame_height],)
    if destination:
        img = plt.imread("img/destination.png", format="png")
#         img[(img[:,:,0] == 1)&(img[:,:,1] == 1)&(img[:,:,2] == 1)] = float('nan')
        frame_height = 0.8
        x_coord = destination[0] - 0.4
        y_coord = destination[1] + 0.40
        ax.imshow(img, extent=[x_coord , x_coord + frame_height,
                              9 - y_coord, 9 - y_coord + frame_height],)
    
    if show_paths:
        # plotting all the available edges
        for key in graph:
            for value in graph[key]:
                ax.plot([key[0], value[0]], [9 - key[1], 9 - value[1]], '#664b78',)
                pass
            pass
    
    # shortest paths to our source
    if start:
        distance, path = dijkstra(graph, start)
    
    # plotting the optimal path
    if destination and (distance[destination] < float("inf")) and show_results:
        n_steps = len(path[start][destination])
        steps = [start] + path[start][destination]

        ax.plot([elem[0] for elem in steps], [9 - elem[1] for elem in steps], '#1a752e',linewidth=8)
    elif show_results and destination:
        #Adding text inside a rectangular box by using the keyword 'bbox'
        ax.text(2, 4, 'Unable to reach delivery point.', fontsize = 22, 
         bbox = dict(facecolor = 'red', alpha = 0.5))

    # plotting all the vertices
    if show_nodes:
        for key in graph:
            ax.plot(*key, marker='o', markerfacecolor='#ed1379', markersize=10,
                   markeredgewidth=0, )
    
    plt.savefig("img/saved.png", dpi='figure', format=None, metadata=None,
        bbox_inches=None, pad_inches=0.1,
        facecolor='auto', edgecolor='auto',
        backend=None,
       )