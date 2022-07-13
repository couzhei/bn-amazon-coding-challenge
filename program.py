#!/usr/bin/env python3

import time
import copy # this is very important, since default .copy() still track and modify the original dictionary somehow!
import sys
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


# Phase 1
my_graph = generate_graph(n_nodes=10)

start = (0,0)
destination = (9,9)

obstacles = {(9, 7), (8, 7), (6, 7), (6, 8)}

print(shortest_path(my_graph, obstacles, start, destination)[1])

# Phase 2 and Bonus
for i in range(20):
    while True:
        candidate = randint(0, 9), randint(0, 9)
        if candidate in obstacles | {start} | {destination}:
            continue
        else:
            obstacles.add(candidate)
            break

solution = shortest_path(my_graph, obstacles, start, destination)[1]

if solution:
    print(solution)
else:
    print("Unable to reach delivery point.")
    suggestion = remove_obstacles(my_graph, obstacles, start, destination)
    print(list(suggestion[0]))