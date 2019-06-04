import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
from helpers.graph import *
from helpers.geometry import *
from helpers.sweep import *
from helpers.util import *
import matplotlib.pyplot as plt
from tsp_solver.greedy import solve_tsp
import csv
import scipy.io as sio
import time


def generate_final_path(img_file_name, output_file_name, width, step, safeWidth):

    t_total = time.time()

    # img is the input image, approxes are the generated polygon
    img, approxes = generate_polygon_countour(img_file_name)

    # genenrate basic
    [y_limit_lower, y_limit_upper, x_limit_lower, x_limit_upper], boundary_basic, obstacles_basic = generate_baisc(
        approxes)


    t = time.time()
    print('extract_vertex_se')
    boundary, sorted_vertices, obstacles = extract_vertex(boundary_basic, obstacles_basic)
    print("time:", time.time() - t)

    # generate vertical line
    t = time.time()
    print('get_vertical_line_se')
    open_line_segments = get_vertical_line(sorted_vertices, obstacles,  y_limit_lower, y_limit_upper)
    print("time:", time.time() - t)

    # Find Polygon cells naiively. Will improve next. 
    # open_line_segments and sorted_vertices has the same order of points, based on the x_value
    t = time.time()
    print('generate_naive_polygon_se')
    quad_cells, left_tri_cells, right_tri_cells = generate_naive_polygon(open_line_segments, sorted_vertices, obstacles)
    print("time:", time.time() - t)

    # Merge overlapping Polygons
    t = time.time()
    print('refine_quad_cells_se')
    refine_quad_cells(quad_cells)
    print("time:", time.time() - t)

    # add the boundary cells to the quad_cells
    add_boundary_cells(boundary, sorted_vertices, quad_cells, y_limit_lower, y_limit_upper)


    # combine all the cells
    all_cell = quad_cells + left_tri_cells + right_tri_cells
    # sort the cell based on teh x-value of the first point
    ################-----   IMPORTANT  -----##################
    all_cell.sort(key = lambda pnt: pnt[0].x)


    # genenrate node set, inside path without step
    t = time.time()
    print('generate_node_set_se')
    nodes = generate_node_set(all_cell, width, step, safeWidth)
    print("time:", time.time() - t)


    # generate left_tri_matrix using graph algorithm for generating the shortest path
    left_tri_matrix, st_path_matrix = generate_left_tri_matrix(nodes)
    
    # use the tsp package to get the shortest path to visit all the nodes
    shortest_path_node = solve_tsp( left_tri_matrix, endpoints = (0, len(nodes)-1) )
    # go back to the origin node 0
    shortest_path_node.append(shortest_path_node[0])
    
    # generate the path to travel through all node in shortest_path_node
    t = time.time()
    print('generate_path_se')
    new_path_node = generate_path(shortest_path_node, st_path_matrix, nodes, step)
    print("time:", time.time() - t)

    # final_path, include the inside path of each polygon
    final_path = []
    for i, node in enumerate(shortest_path_node):
        # go back to the origin node 0
        if(i < len(nodes)):
            final_path = final_path + nodes[node].inside_path + new_path_node[i]

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
    ###########################
    # write to csv file so that the simulink could import the data
    # # represent with the list
    # list_final_path = []
    # list_final_path_x = []
    # list_final_path_y = []
    # for pnt in final_path:
    #     list_final_path.append([int(pnt.x), int(pnt.y)])
    #     list_final_path_x.append(int(pnt.x))
    #     list_final_path_y.append(int(pnt.y))
    #
    # # write the csv file
    # with open(output_file_name,'w', newline='') as resultFile:
    #     wr = csv.writer(resultFile, delimiter=',')
    #     wr.writerows(list_final_path)


def main(argv):
	assert len(argv) == 5, "need 5 parameters: img_file_name, output_file_name, width, step, safeWidth"
	img_file_name = argv[0]
	output_file_name = argv[1]
	width = int(argv[2])

	step = int(argv[3])
	safeWidth = int(argv[4])

	if step == -1:
		step = None

	if safeWidth == -1:
		safeWidth = None

	generate_final_path(img_file_name, output_file_name, width, step, safeWidth)
	print("path generated")

if __name__ == "__main__":

	main(sys.argv[1:])
