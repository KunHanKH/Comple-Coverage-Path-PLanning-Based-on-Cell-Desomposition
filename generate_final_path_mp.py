import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
from helpers.graph import *
from helpers.geometry import *
from helpers.sweep import *
from helpers.util_mp import *
from helpers.util import *
import matplotlib.pyplot as plt
from tsp_solver.greedy import solve_tsp
import csv
import scipy.io as sio
import os
import multiprocessing
from multiprocessing import Process, Pool, Manager


def generate_final_path(img_file_name, output_file_name, width, step, safeWidth, num_processes=4, unit=2):

    t_total = time.time()

    pool = Pool(num_processes)
    manager = Manager()

    t_total_without_process_start_end = time.time()

    # img is the input image, approxes are the generated polygon
    img, approxes = generate_polygon_countour(img_file_name)

    # genenrate basic
    [y_limit_lower, y_limit_upper, x_limit_lower, x_limit_upper], boundary_basic, obstacles_basic = generate_baisc(approxes)

    t = time.time()
    all_cell_manager_list, boundary, sorted_vertices, obstacles = \
    generate_cells_mp([y_limit_lower, y_limit_upper, x_limit_lower, x_limit_upper], boundary_basic, obstacles_basic, manager, pool, unit)
    print('generate_cells_mp')
    print("time:", time.time() - t)


    # genenrate node set, inside path without step
    t = time.time()
    print('generate_node_set_mp')
    # nodes = generate_node_set(all_cell, width, step, safeWidth=20)
    nodes = generate_node_set_mp(all_cell_manager_list, pool, manager, width, step, safeWidth, unit=1)
    nodes_manager_list = manager.list(nodes)
    print("time:", time.time() - t)


    # generate left_tri_matrix using graph algorithm for generating the shortest path
    left_tri_matrix, st_path_matrix = generate_left_tri_matrix(nodes)


    # use the tsp package to get the shortest path to visit all the nodes
    shortest_path_node = solve_tsp(left_tri_matrix, endpoints=(0, len(nodes) - 1))
    # go back to the origin node 0
    shortest_path_node.append(shortest_path_node[0])


    # generate the path to travel through all node in shortest_path_node
    t = time.time()
    print('generate_path_mp')
    st_path_matrix_manager_list = manager.list(st_path_matrix)
    # new_path_node = generate_path(shortest_path_node, st_path_matrix, nodes, step)
    new_path_node = generate_path_mp(shortest_path_node, st_path_matrix_manager_list, nodes_manager_list, pool, manager, step)
    # new_path_node here has a little difference from the serial version
    # first value is the index based on the shortest_path
    # [0, [...]]
    # [1, [...]]

    print("time:", time.time() - t)

    # final_path, include the inside path of each polygon
    final_path = []
    for i, node in enumerate(shortest_path_node):
        # go back to the origin node 0
        if (i < len(nodes)):
            # final_path = final_path + nodes[node].inside_path + new_path_node[i]
            final_path = final_path + nodes[node].inside_path + new_path_node[i][1]

    print('Total time without process start and end: ', time.time() - t_total_without_process_start_end)

    pool.close()
    pool.join()

    print('Total time: ', time.time() - t_total)

    # print image
    plt.figure(figsize=(20, 10))
    draw_node(nodes, boundary, obstacles, fill=None)
    x = [pnt.x for pnt in final_path]
    y = [pnt.y for pnt in final_path]
    #     print("x-y")
    #     print(x)
    #     print(y)
    plt.plot(x, y)
    for pnt in final_path:
        plt.plot(pnt.x, pnt.y, marker='o')
    plt.show()


def main(argv):
    assert len(argv) == 7, "need 7 parameters: img_file_name, output_file_name, width, step, safeWidth, num_processes, unit"
    img_file_name = argv[0]
    output_file_name = argv[1]
    width = int(argv[2])

    step = int(argv[3])
    safeWidth = int(argv[4])
    num_processes = int(argv[5])
    unit = int(argv[6])

    if step == -1:
        step = None

    if safeWidth == -1:
        safeWidth = None


    generate_final_path(img_file_name, output_file_name, width, step, safeWidth, num_processes, unit)
    print("path generated")


if __name__ == "__main__":
    main(sys.argv[1:])
