from helpers.geometry import *
import matplotlib.pyplot as plt
from helpers.sweep import *
import cv2
import multiprocessing
import time
import os
from multiprocessing import Manager


# use cv2.pyrMeanShiftFiltering if filter = True
# sp – The spatial window radius.
# sr – The color window radius.

def generate_polygon_countour(image_name, filter=None, sp=None, sr=None):
    img = cv2.imread(image_name)
    if (filter):
        img = cv2.pyrMeanShiftFiltering(img, sp, sr)
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, threshold = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    countours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    approxes = []
    for i, cnt in enumerate(countours):
        episilon = 0.01 * cv2.arcLength(cnt, True)
        approxes.append(cv2.approxPolyDP(cnt, episilon, True))
    return [img, approxes]


# generate vertex node
def generate_vertex_point_basic(obstacle):
    list_vertex = []
    for index, pnt in enumerate(obstacle):
        list_vertex.append(point(pnt[0], pnt[1], index))
    return list_vertex

def generate_obstacle_basic(obstacle):
	obstacle_vertex_list = []
	for index, pnt in enumerate(obstacle):
		obstacle_vertex_list.append(point(pnt[0], pnt[1], index))
	return obstacle_vertex_list

# Use point object to represent the boundary and the src/dest
def extract_vertex_mp(boundary, obstacles_basic_manager_list, pool):

    boundary = [point(i[0], i[1]) for i in boundary]

    # # generate new sorted vertices based on the x value
    new_sorted_vertices= pool.map(generate_vertex_point_basic, obstacles_basic_manager_list)
    new_sorted_vertices = [point for obs in new_sorted_vertices for point in obs]
    new_sorted_vertices.sort(key=lambda pnt: pnt.x)

    # reconstruct the obstacle using the point object using mp
    new_obstacles = pool.map(generate_obstacle_basic, obstacles_basic_manager_list)

    return boundary, new_sorted_vertices, new_obstacles


# Draw the obstacles and point the source and the destination----------------------------------------------
def draw_problem(boundary, obstacles):
    bnd_x = [i.x for i in boundary]
    # add the x-value of the start point
    bnd_x.append(boundary[0].x)
    bnd_y = [i.y for i in boundary]
    # add the y-value of the start point
    bnd_y.append(boundary[0].y)

    # Draw the boundary
    plt.plot(bnd_x, bnd_y)

    poly_x = []
    poly_y = []
    for index, i in enumerate(obstacles):
        poly_x.append([p.x for p in i])
        poly_y.append([p.y for p in i])

        plt.fill(poly_x[index], poly_y[index], color="#512DA8")


# plt.show()

def draw_cell(cells, boundary, obstacles):
    for i, cell in enumerate(cells):
        #     plt.figure(i)
        x = [pnt.x for pnt in cell]
        y = [pnt.y for pnt in cell]
        x.append(x[0])
        y.append(y[0])
        draw_problem(boundary, obstacles)
        plt.plot(x, y)


def draw_node(nodes, boundary, obstacles, fill=None):
    for index, i in enumerate(nodes):
        draw_problem(boundary, obstacles)

        x = [j.x for j in i.polygon]
        x.append(x[0])
        y = [j.y for j in i.polygon]
        y.append(y[0])
        plt.plot(x, y)

        if fill is not None:
            plt.fill(x, y)

        center = i.centroid
        plt.plot(center.x, center.y, marker="o")
        plt.annotate('cell-{}'.format(index), xy=(center.x, center.y))

def get_vertical_line_basic(index_mp, vertex, obstacles, y_limit_lower, y_limit_upper):

    curr_line_segment = [point(vertex.x, y_limit_lower), point(vertex.x, y_limit_upper)]
    lower_obs_pt = curr_line_segment[0]
    upper_obs_pt = curr_line_segment[1]
    # upper_gone is True if there is no line above the current vertex
    # lower_gone is True if there is no line below the current vertex
    upper_gone = False
    lower_gone = False
    # break_now is True if already find the vertical line
    break_now = False

    intersections = []

    # Find intersection points with the vertical proposed lines. the intersection function returns false if
    # segments are same, so no need to worry about same segment checking
    for index, obs in enumerate(obstacles):
        # Add the first point again for the last line segment of a polygon.
        obs.append(obs[0])

        for vertex_index in range(len(obs) - 1):
            # compare curr_line and segment from obstacle check whether the two section is intersected or
            # colinear since the curr_line is the segment [point(pt.x, y_limit_lower), point(pt.x,
            # y_limit_upper)], so that if the points are colinear, it must intersect return the intersection point
            #  if the line intersect, and -1 if not.
            res = segment_intersection(curr_line_segment[0], curr_line_segment[1], obs[vertex_index],
                                       obs[vertex_index + 1])

            # if there is an intersection between current_line_seg and current obstacle edge
            if (res != -1):
                # make sure the intersection has the same x-value as the current vertical line
                res.x = vertex.x
                if vertex.equals(res) == False:
                    intersections.append(res)
    # among all the intersection from the current vertical line, choose the closest points that above or below the current vertex
    # For closest_lower point, if the point between it and the current vertex is in the obstacle, then lower_gone is True
    # For cloeset_upper point, if the point between it and the current vertex is in the obstacle, then upper_gone is True
    closest_lower, closest_upper = get_closest_intersection(vertex, intersections)
    if (closest_lower != None):
        # check if the middle point pf current point and the intersection is inside theb polygon, lower_gone
        if centroid([vertex, closest_lower]).inside_polygon(obstacles):
            lower_gone = True
        else:
            lower_obs_pt = closest_lower

    if (closest_upper != None):
        # check if the middle point pf current point and the intersection is inside theb polygon, lower_gone
        if centroid([vertex, closest_upper]).inside_polygon(obstacles):
            upper_gone = True
        else:
            upper_obs_pt = closest_upper

    # # Draw the vertical cell lines
    # if(lower_gone is False):
    # 	plt.plot( [lower_obs_pt.x, pt.x],  [lower_obs_pt.y, pt.y] )

    # if(upper_gone is False):
    # 	plt.plot( [pt.x, upper_obs_pt.x],  [pt.y, upper_obs_pt.y] )

    # Add to the global segment list
    # lock_line.acquire()
    if (lower_gone and upper_gone):
        return [index_mp, [None, None]]
    elif (lower_gone):
        return [index_mp, [None, upper_obs_pt]]
    # plt.plot( [pt.x, upper_obs_pt.x],  [pt.y, upper_obs_pt.y] )
    elif (upper_gone):
        return [index_mp, [lower_obs_pt, None]]
    # plt.plot( [lower_obs_pt.x, pt.x],  [lower_obs_pt.y, pt.y] )
    else:
        return [index_mp, [lower_obs_pt, upper_obs_pt]]
    # lock_line.release()

def get_vertical_line_mp(sorted_vertices, y_limit_lower, y_limit_upper,
                         obstacles_manager_list, pool):
    # -----------------------------------------------------------
    # Find vertical lines
    # Make sure all the vertical line has the same x-value as the current vertex
    # set the pool parameter

    # open_line_segments = [[]] * len_vertex
    open_line_segments = []

    multi_results = [pool.apply_async(func=get_vertical_line_basic,
                                     args=(index, vertex, obstacles_manager_list, y_limit_lower, y_limit_upper))
                     for index, vertex in enumerate(sorted_vertices)]

    for res in multi_results:
        open_line_segments.append(res.get())

    open_line_segments.sort(key = lambda line: line[0])

    return open_line_segments

def generate_naive_polyfon_basic(index, open_line_segments, sorted_vertices, obstacles,
                                 queue_quad_cells, queue_left_tri_cells, queue_right_tri_cells):
    # Find Polygon cells naiively. Will improve next.
    # cells = []
    # Important:
    # Don't change the value of element in new_sorted_vertices and open_line_segments

    curr_segment = open_line_segments[index][1]
    curr_vertex = sorted_vertices[index]
    # plt.plot(curr_vertex.x, curr_vertex.y, marker='o')
    break_now = False
    done = [False, False, True]

    # if the lower pt is the vertice
    # done[0] is True if there is no polygon below the vertex, or already get the lower polygon
    if curr_segment[0] is None:
        done[0] = True
    # if the upper vertice is the vertice
    # done[1] is True if there is no polygon above the vertex, or already get the upper polygon
    if (curr_segment[1] is None):
        done[1] = True
    # if both lower and upper pts are the vertice, which also means there is no vertical line through this line
    # if done[3] is True if there is no upper or lower polygon for this vertex and find the polygon for this vertex
    if (curr_segment[0] is None and curr_segment[1] is None):
        done[2] = False

    # index2 the following sorted vertices
    for index2 in range(index + 1, len(open_line_segments)):
        next_segment = open_line_segments[index2][1]
        next_vertex = sorted_vertices[index2]

        if (done[0] is False):
            if (next_segment[0] is not None):
                # check the upper polygon
                if (check_quad_polygon(curr_vertex, curr_segment[0], next_vertex, next_segment[0], obstacles)):
                    # lock_polygon.acquire()
                    queue_quad_cells.append([curr_segment[0], next_segment[0], next_vertex, curr_vertex])
                    # lock_polygon.release()
                    done[0] = True

            if (next_segment[1] is not None):
                # check the lower polygon
                if (check_quad_polygon(curr_vertex, curr_segment[0], next_vertex, next_segment[1], obstacles)):
                    # lock_polygon.acquire()
                    queue_quad_cells.append([curr_segment[0], next_vertex, next_segment[1], curr_vertex])
                    # lock_polygon.release()
                    done[0] = True

            if (next_segment[0] is None and next_segment[1] is None):
                if (check_right_tri_polygon(curr_vertex, curr_segment[0], next_vertex, obstacles)):
                    # lock_polygon.acquire()
                    queue_right_tri_cells.append([curr_segment[0], next_vertex, curr_vertex])
                    # lock_polygon.release()
                    done[0] = True

        # check whether there is a polygon use the [current_vertex, current_segment[1]] as boundary
        if (done[1] is False):
            if (next_segment[1] is not None):
                # check the lowerer polygon
                if (check_quad_polygon(curr_vertex, curr_segment[1], next_vertex, next_segment[1], obstacles)):
                    # lock_polygon.acquire()
                    queue_quad_cells.append([curr_vertex, next_vertex, next_segment[1], curr_segment[1]])
                    # lock_polygon.release()
                    done[1] = True
            # print(current_vertex)

            if (next_segment[0] is not None):
                # check the lower polygon
                if (check_quad_polygon(curr_vertex, curr_segment[1], next_vertex, next_segment[0], obstacles)):
                    # lock_polygon.acquire()
                    queue_quad_cells.append([curr_vertex, next_segment[0], next_vertex, curr_segment[1]])
                    # lock_polygon.release()
                    done[1] = True
            # print(curr_vertex)

            if (next_segment[0] is None and next_segment[1] is None):
                if (check_right_tri_polygon(curr_vertex, curr_segment[1], next_vertex, obstacles)):
                    # lock_polygon.acquire()
                    queue_right_tri_cells.append([curr_vertex, next_vertex, curr_segment[1]])
                    # lock_polygon.release()
                    done[1] = True

        if (done[2] is False):
            if (next_segment[0] is not None):
                if (check_left_tri_polygon(curr_vertex, next_segment[0], next_vertex, obstacles)):
                    # lock_polygon.acquire()
                    queue_left_tri_cells.append([curr_vertex, next_segment[0], next_vertex])
                    # lock_polygon.release()
                    done[2] = True

            if (next_segment[1] is not None):
                if (check_left_tri_polygon(curr_vertex, next_segment[1], next_vertex, obstacles)):
                    # lock_polygon.acquire()
                    queue_left_tri_cells.append([curr_vertex, next_vertex, next_segment[1]])
                    # lock_polygon.release()
                    done[2] = True

        if done[0] == True and done[1] == True and done[2] == True:
            break

# generate naive polygon
def generate_naive_polygon_mp(open_line_segments_manager_list,
                              sorted_vertices_manager_list,
                              obstacles_manager_list,
                              pool, manager):

    quad_cells_manager_list = manager.list()
    left_tri_cells_manager_list = manager.list()
    right_tri_cells_manager_list = manager.list()

    len_vertex = len(open_line_segments_manager_list)

    multi_results = [pool.apply_async(func=generate_naive_polyfon_basic,
                                      args=(index, open_line_segments_manager_list, sorted_vertices_manager_list, obstacles_manager_list,
                                            quad_cells_manager_list, left_tri_cells_manager_list, right_tri_cells_manager_list))
                     for index in range(len_vertex)]

    for res in multi_results:
        res.get()

    return quad_cells_manager_list, left_tri_cells_manager_list, right_tri_cells_manager_list

def refine_quad_cells_mp(quad_cells_manager_list, pool, manager):
    merge_overlapping_quad_cell_mp(quad_cells_manager_list, pool, manager)
    remove_duplicate_cell_mp(quad_cells_manager_list, pool, manager)
    remove_new_added_merged_cell_mp(quad_cells_manager_list, pool, manager)

def merge_overlapping_quad_cell_baisc(index, quad_cells, queue_quad_cells_remove, queue_quad_cells_add):
    # quad_cells = [i for i in cells if len(i)>3]
    # tri_cells = [i for i in cells if len(i)==3]
    # others = [i for i in cells if len(i)<3]

    for index_cell2, cell in enumerate(quad_cells):
        if (index != index_cell2):
            # if two quad_cell has the same x-direction location, then check wether they could merge
            if (quad_cells[index][0].x == cell[0].x and quad_cells[index][1].x == cell[1].x):
                temp1 = list(quad_cells[index])
                temp1.append(temp1[0])
                temp2 = list(cell)
                temp2.append(temp2[0])
                area1 = quad_polygon_area(temp1)
                area2 = quad_polygon_area(temp2)

                # construct new polygon
                new_quad = []

                new_quad.append(point(temp1[0].x, min(temp1[0].y, temp2[0].y)))
                new_quad.append(point(temp1[1].x, min(temp1[1].y, temp2[1].y)))
                new_quad.append(point(temp1[1].x, max(temp1[2].y, temp2[2].y)))
                new_quad.append(point(temp1[0].x, max(temp1[3].y, temp2[3].y)))
                area3 = quad_polygon_area(new_quad)

                if (area1 + area2 >= area3):
                    # merge
                    queue_quad_cells_remove.append(index)
                    queue_quad_cells_remove.append(index_cell2)
                    queue_quad_cells_add.append(new_quad)

def merge_overlapping_quad_cell_mp(quad_cells_manager_list, pool, manager):
    # quad_cells = [i for i in cells if len(i)>3]
    # tri_cells = [i for i in cells if len(i)==3]
    # others = [i for i in cells if len(i)<3]

    quads_to_remove = set()
    len_cells = len(quad_cells_manager_list)

    quad_cells_remove_manager_list = manager.list()
    quads_cells_add_manager_list = manager.list()

    multi_results = [pool.apply_async(func=merge_overlapping_quad_cell_baisc,
                                              args=(index, quad_cells_manager_list,
                                                    quad_cells_remove_manager_list, quads_cells_add_manager_list))
                     for index in range(len_cells)]

    for res in multi_results:
        res.get()

    for remove_index in quad_cells_remove_manager_list:
        quads_to_remove.add(remove_index)

    # generate the reversed array, so that deleting one element will not affect the element before
    for index in sorted(quads_to_remove, reverse=True):
        del quad_cells_manager_list[index]

    for add_polygon in quads_cells_add_manager_list:
        quad_cells_manager_list.append(add_polygon)


def remove_duplicate_cell_basic(index, quad_cells, queue_quad_cells_remove):
    # # Remove duplicates
    for index2 in range(index + 1, len(quad_cells)):
        duplicate = True
        for k, m in zip(quad_cells[index], quad_cells[index2]):
            if k.equals(m) is False:
                duplicate = False
                break
        if (duplicate is True):
                queue_quad_cells_remove.append(index2)



def remove_duplicate_cell_mp(quad_cells_manager_list, pool, manager):

    quads_to_remove = set()
    len_cells = len(quad_cells_manager_list)
    quad_cells_remove_manager_list = manager.list()

    multi_results = [pool.apply_async(func=remove_duplicate_cell_basic,
                                      args=(index, quad_cells_manager_list, quad_cells_remove_manager_list))
                     for index in range(len_cells)]

    for res in multi_results:
        res.get()

    for remove_index in quad_cells_remove_manager_list:
        quads_to_remove.add(remove_index)

    # generate the reversed array, so that deleting one element will not affect the element before


    for index in sorted(quads_to_remove, reverse=True):
        del quad_cells_manager_list[index]

    # print(len(quad_cells_manager_list))
    # for index, cell in enumerate(quad_cells_manager_list):
    #     print('index ', index)
    #     for pnt in cell:
    #         print(pnt)


def remove_new_added_merged_cell_basic(index, quad_cells, queue_quad_cells_remove):
    # since in the previous procedure, we add new quad_cell, then will be a chance that we introduce new overlapping cell
    for index2 in range(len(quad_cells)):
        if (index != index2 and quad_cells[index][0].x == quad_cells[index2][0].x and quad_cells[index][1].x ==
                quad_cells[index2][1].x):

            if ((quad_cells[index][0].y <= quad_cells[index2][0].y) and (
                    quad_cells[index][1].y <= quad_cells[index2][1].y)
                    and (quad_cells[index][2].y >= quad_cells[index2][2].y) and (
                            quad_cells[index][3].y >= quad_cells[index2][3].y)):
                # lock_quad_cells.acquire()
                queue_quad_cells_remove.append(index2)
                # lock_quad_cells.release()


def remove_new_added_merged_cell_mp(quad_cells_manager_list, pool, manager):
    quads_to_remove = set()
    len_cells = len(quad_cells_manager_list)
    quad_cells_remove_manager_list = manager.list()

    multi_results = [pool.apply_async(func=remove_new_added_merged_cell_basic,
                                      args=(index, quad_cells_manager_list, quad_cells_remove_manager_list))
                     for index in range(len_cells)]

    for res in multi_results:
        res.get()

    for remove_index in quad_cells_remove_manager_list:
        quads_to_remove.add(remove_index)

    # generate the reversed array, so that deleting one element will not affect the element before

    for index in sorted(quads_to_remove, reverse=True):
        del quad_cells_manager_list[index]

    # print(len(quad_cells_manager_list))
    # for index, cell in enumerate(quad_cells_manager_list):
    #     print('index ', index)
    #     for pnt in cell:
    #         print(pnt)


def generate_node_set_basic(index, nodes, width, queue_nodes, step=None, safeWidth=None):
    curr_node = nodes[index]
    for next_node in nodes:
        if (next_node.index != curr_node.index):
            # define the type
            # 1: quad_cell
            # 2: left_tri_cell: left side has only one point
            # 3: right_tri_cell: right side has only one point

            # get adjacent nodes
            if ((curr_node.type == 1 or curr_node.type == 2) and
                    (next_node.type == 1 or next_node.type == 3)):
                if (curr_node.polygon[1].x == next_node.polygon[0].x):
                    if ((max(curr_node.polygon[2].y, next_node.polygon[-1].y) - min(curr_node.polygon[1].y,
                                                                                    next_node.polygon[0].y))
                            < (abs(curr_node.polygon[2].y - curr_node.polygon[1].y) + abs(
                                next_node.polygon[-1].y - next_node.polygon[0].y))):
                        curr_node.add_adjacent(next_node)
    total_adjacent = curr_node.get_adjacent()

    # add middle point and path to adjacent node
    if (len(total_adjacent) == 1):
        curr_node.add_middle_point(centroid([curr_node.polygon[1], curr_node.polygon[2]]))
        curr_node.add_path_to_adjacency(
            [curr_node.centroid, curr_node.middle_point[0], curr_node.adjacent[0].centroid])

    elif (len(total_adjacent) >= 1):
        for i, ad_node in enumerate(total_adjacent):
            curr_node.add_middle_point(centroid([ad_node.polygon[0], ad_node.polygon[-1]]))
            curr_node.add_path_to_adjacency(
                [curr_node.centroid, curr_node.middle_point[i], curr_node.adjacent[i].centroid])

    # calculate the distance to the adjacent node
    curr_node.calculate_distance()
    curr_node.inside_path = sweep(curr_node.polygon, width, step, safeWidth)

    # lock_nodes.acquire()
    return curr_node
    # lock_nodes.release()
    # print(os.getpid(), 'add node done')


def generate_node_set_mp(all_cell_manager_list, pool, manager, width, step=None, safeWidth=None):
    init_nodes = []

    initialize_node_set(init_nodes, all_cell_manager_list)
    # print('length of nodes', len(nodes))
    len_node = len(init_nodes)

    nodes_manager_list = manager.list()

    multi_results = [pool.apply_async(func=generate_node_set_basic,
                                      args=(index, init_nodes, width, nodes_manager_list, step, safeWidth))
                     for index in range(len_node)]
    nodes = []
    for res in multi_results:
        nodes.append(res.get())

    ##########################################
    # Because the size of queue is limited, so that we choose to append the list once we join a process
    # print('add to list')
    # while queue_nodes.empty() is False:
    #     nodes.append(queue_nodes.get())
    nodes.sort(key=lambda node: node.polygon[0].x)
    return nodes


def initialize_node_set(nodes, cells):
    # define the type
    # 1: quad_cell
    # 2: left_tri_cell: left side has only one point
    # 3: right_tri_cell: right side has only one point
    for index, cell in enumerate(cells):
        if (len(cell)) == 4:
            nodes.append(node(index, cell, 1))
        elif (len(cell) == 3 and cell[1].x == cell[2].x):
            nodes.append(node(index, cell, 2))
        elif (len(cell) == 3 and cell[0].x == cell[2].x):
            nodes.append(node(index, cell, 3))

#     print("{}-th polygon:\n\ttype: {}\n\tcentroid: {}".format(index, nodes[index].type, nodes[index].centroid))


def get_middle_pnt(nodes, n1, n2):
    for index, ad_node in enumerate(nodes[n1].adjacent):
        if ad_node.index == n2:
            return nodes[n1].middle_point[index]
    for index, ad_node in enumerate(nodes[n2].adjacent):
        if ad_node.index == n1:
            return nodes[n2].middle_point[index]
    print("No middle pnt")
    return None


def generate_two_node_path(index, temp_path, nodes, step=None):
    medium_path = []
    for i in range(len(temp_path) - 1):
        start_pnt = nodes[temp_path[i]].centroid
        end_pnt = nodes[temp_path[i + 1]].centroid

        medium_path.append(start_pnt)

        middle_pnt = get_middle_pnt(nodes, temp_path[i], temp_path[i + 1])

        if step is not None:
            step_path = stepsBetwn(start_pnt, middle_pnt, step)
            medium_path = medium_path + step_path

        medium_path.append(middle_pnt)

        if step is not None:
            step_path = stepsBetwn(middle_pnt, end_pnt, step)
            medium_path = medium_path + step_path

    medium_path.append(nodes[temp_path[-1]].centroid)
    # lock_path.acquire()
    return [index, medium_path]
    # lock_path.release()


def generate_path_mp(st_path, st_path_matrix_manager_list, nodes_manager_list, pool, manager, step=None):


    path_manager_list = manager.list()
    len_path = len(st_path) - 1
    paths = []
    multi_results = [pool.apply_async(func=generate_two_node_path,
                                      args=(index, st_path_matrix_manager_list[st_path[index]][st_path[index + 1]],
                                            nodes_manager_list, step))
                     for index in range(len_path)]

    for res in multi_results:
        paths.append(res.get())

    paths.sort(key = lambda path: path[0])

    return paths
