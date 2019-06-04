from helpers.geometry import *
import matplotlib.pyplot as plt
from helpers.sweep import *
from helpers.graph import *
import numpy as np
import cv2
import multiprocessing

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
# use cv2.pyrMeanShiftFiltering if filter = True
# sp – The spatial window radius.
# sr – The color window radius.


def generate_baisc(approxes):
    # generate the boundary by getting the min/max value of all polygon
    polygons = [np.squeeze(x) for x in approxes]

    y_limit_lower = min([pt[1] for pt in polygons[0]])
    y_limit_upper = max([pt[1] for pt in polygons[0]])

    x_limit_lower = min([pt[0] for pt in polygons[0]])
    x_limit_upper = max([pt[0] for pt in polygons[0]])

    # boundary_basic certex order
    boundary_basic = [[x_limit_lower, y_limit_lower], [x_limit_upper, y_limit_lower], [x_limit_upper, y_limit_upper],
                      [x_limit_lower, y_limit_upper]]

    # Among all the polygon cv2 generated, [1:] are the inner obstacles
    obstacles_basic = polygons[1:]

    return [y_limit_lower, y_limit_upper, x_limit_lower,x_limit_upper], boundary_basic, obstacles_basic


def add_boundary_cells(boundary, sorted_vertices, quad_cells, y_limit_lower, y_limit_upper):
    if (boundary[0].x != sorted_vertices[0].x):
        quad_cells.append(
            [boundary[0], point(sorted_vertices[0].x, y_limit_lower), point(sorted_vertices[0].x, y_limit_upper),
             boundary[3]]);
    if (boundary[1].x != sorted_vertices[len(sorted_vertices) - 1].x):
        quad_cells.append([point(sorted_vertices[len(sorted_vertices) - 1].x, y_limit_lower), boundary[1], boundary[2],
                           point(sorted_vertices[len(sorted_vertices) - 1].x, y_limit_upper)]);


def generate_left_tri_matrix(nodes):
    # get the adjacency matrix
    adjacency_matrix = get_adjacency_matrix(nodes)

    # Dijkstra’s shortest path algorithm to get the shortest distance from the root to the target given adjacency matrix
    # use each node as root node iteratively to generate the distance matrix

    # generate a fully connected graph
    num_node = len(nodes)
    g = Graph(num_node)
    g.graph = adjacency_matrix
    g.generate_distance_st_matrix()

    distance_matrix = np.array(g.distance_matrix)

    # in order to use the solve_tsp command, we need to convert the distance matrix to a left_triangular_matrix
    left_tri_matrix = []
    for i in range(0, num_node):
        temp = list(np.split(distance_matrix[i], [i])[0])
        left_tri_matrix.append(temp)

    return left_tri_matrix, g.st_path_matrix


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


# Use point object to represent the boundary and the src/dest
def extract_vertex(boundary, obstacles):
    boundary = [point(i[0], i[1]) for i in boundary]

    # generate new sorted vertices based on the x value
    new_sorted_vertices = []
    for index, pnts in enumerate(obstacles):
        for pnt in pnts:
            new_sorted_vertices.append(point(pnt[0], pnt[1], index))
    new_sorted_vertices.sort(key=lambda pnt: pnt.x)

    # reconstruct the obstacle using the point object
    new_obstacles = []
    for index, i in enumerate(obstacles):
        temp_obs = []
        for j in i:
            temp = point(j[0], j[1], index)
            temp_obs.append(temp)
        new_obstacles.append(temp_obs)

    return boundary, new_sorted_vertices, new_obstacles





def get_vertical_line(sorted_vertices, obstacles, y_limit_lower, y_limit_upper):
    # -----------------------------------------------------------
    # Find vertical lines
    # Make sure all the vertical line has the same x-value as the current vertex
    open_line_segments = []

    for pt in sorted_vertices:
        curr_line_segment = [point(pt.x, y_limit_lower), point(pt.x, y_limit_upper)]
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
                    res.x = pt.x
                    if pt.equals(res) == False:
                        intersections.append(res)
        # among all the intersection from the current vertical line, choose the closest points that above or below the current vertex
        # For closest_lower point, if the point between it and the current vertex is in the obstacle, then lower_gone is True
        # For cloeset_upper point, if the point between it and the current vertex is in the obstacle, then upper_gone is True
        closest_lower, closest_upper = get_closest_intersection(pt, intersections)
        if (closest_lower != None):
            # check if the middle point pf current point and the intersection is inside theb polygon, lower_gone
            if centroid([pt, closest_lower]).inside_polygon(obstacles):
                lower_gone = True
            else:
                lower_obs_pt = closest_lower

        if (closest_upper != None):
            # check if the middle point pf current point and the intersection is inside theb polygon, lower_gone
            if centroid([pt, closest_upper]).inside_polygon(obstacles):
                upper_gone = True
            else:
                upper_obs_pt = closest_upper

        # # Draw the vertical cell lines
        # if(lower_gone is False):
        # 	plt.plot( [lower_obs_pt.x, pt.x],  [lower_obs_pt.y, pt.y] )

        # if(upper_gone is False):
        # 	plt.plot( [pt.x, upper_obs_pt.x],  [pt.y, upper_obs_pt.y] )

        # Add to the global segment list
        if (lower_gone and upper_gone):
            open_line_segments.append([None, None])
        elif (lower_gone):
            open_line_segments.append([None, upper_obs_pt])
        # plt.plot( [pt.x, upper_obs_pt.x],  [pt.y, upper_obs_pt.y] )
        elif (upper_gone):
            open_line_segments.append([lower_obs_pt, None])
        # plt.plot( [lower_obs_pt.x, pt.x],  [lower_obs_pt.y, pt.y] )
        else:
            open_line_segments.append([lower_obs_pt, upper_obs_pt])
    # plt.plot( [lower_obs_pt.x, upper_obs_pt.x],  [lower_obs_pt.y, upper_obs_pt.y] )
    return open_line_segments


# generate naive polygon
def generate_naive_polygon(open_line_segments, sorted_vertices, obstacles):
    # Find Polygon cells naiively. Will improve next.
    # cells = []
    # trapezoids = []
    quad_cells = []
    left_tri_cells = []
    right_tri_cells = []
    # Important:
    # Don't change the value of element in new_sorted_vertices and open_line_segments
    for index1 in range(len(open_line_segments)):
        curr_segment = open_line_segments[index1]
        curr_vertex = sorted_vertices[index1]
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
        for index2 in range(index1 + 1, len(open_line_segments)):
            next_segment = open_line_segments[index2]
            next_vertex = sorted_vertices[index2]

            # double_index1 = -2;
            # double_index2 = -2;
            # lines_to_check = [];

            # double_check = False;

            # # double_check is True if there are lines below and above the vertex
            # if (next_segment[0] is not None and next_segment[1] is not None ):
            #     double_check = True;

            # if the lower pt isn't the vertice
            # check whether there is a polygon use the [current_vertex, current_segment[0]] as boundary
            if (done[0] is False):
                if (next_segment[0] is not None):
                    # check the upper polygon
                    if (check_quad_polygon(curr_vertex, curr_segment[0], next_vertex, next_segment[0], obstacles)):
                        quad_cells.append([curr_segment[0], next_segment[0], next_vertex, curr_vertex])
                        done[0] = True

                if (next_segment[1] is not None):
                    # check the lower polygon
                    if (check_quad_polygon(curr_vertex, curr_segment[0], next_vertex, next_segment[1], obstacles)):
                        quad_cells.append([curr_segment[0], next_vertex, next_segment[1], curr_vertex])
                        done[0] = True

                if (next_segment[0] is None and next_segment[1] is None):
                    if (check_right_tri_polygon(curr_vertex, curr_segment[0], next_vertex, obstacles)):
                        right_tri_cells.append([curr_segment[0], next_vertex, curr_vertex])
                        done[0] = True

            # check whether there is a polygon use the [current_vertex, current_segment[1]] as boundary
            if (done[1] is False):
                if (next_segment[1] is not None):
                    # check the lowerer polygon
                    if (check_quad_polygon(curr_vertex, curr_segment[1], next_vertex, next_segment[1], obstacles)):
                        #                     print("add")
                        quad_cells.append([curr_vertex, next_vertex, next_segment[1], curr_segment[1]])
                        done[1] = True
                # print(current_vertex)

                if (next_segment[0] is not None):
                    # check the lower polygon
                    if (check_quad_polygon(curr_vertex, curr_segment[1], next_vertex, next_segment[0], obstacles)):
                        #                     print("add")
                        quad_cells.append([curr_vertex, next_segment[0], next_vertex, curr_segment[1]])
                        done[1] = True
                # print(curr_vertex)

                if (next_segment[0] is None and next_segment[1] is None):
                    if (check_right_tri_polygon(curr_vertex, curr_segment[1], next_vertex, obstacles)):
                        #                     print("add")
                        right_tri_cells.append([curr_vertex, next_vertex, curr_segment[1]])
                        done[1] = True

            if (done[2] is False):
                if (next_segment[0] is not None):
                    if (check_left_tri_polygon(curr_vertex, next_segment[0], next_vertex, obstacles)):
                        #                     print("add")
                        left_tri_cells.append([curr_vertex, next_segment[0], next_vertex])
                        done[2] = True

                if (next_segment[1] is not None):
                    if (check_left_tri_polygon(curr_vertex, next_segment[1], next_vertex, obstacles)):
                        #                     print("add")
                        left_tri_cells.append([curr_vertex, next_vertex, next_segment[1]])
                        done[2] = True

            if done[0] == True and done[1] == True and done[2] == True:
                break
    return quad_cells, left_tri_cells, right_tri_cells

def refine_quad_cells(quad_cells):
    merge_overlapping_quad_cell(quad_cells)
    remove_duplicate_cell(quad_cells)
    remove_new_added_merged_cell(quad_cells)


def merge_overlapping_quad_cell(quad_cells):
    # quad_cells = [i for i in cells if len(i)>3]
    # tri_cells = [i for i in cells if len(i)==3]
    # others = [i for i in cells if len(i)<3]
    quads_to_remove = []
    quads_to_add = []

    quads_to_remove = []
    quads_to_add = []
    for index_cell in range(len(quad_cells)):
        for index_cell2, cell in enumerate(quad_cells):
            if (index_cell != index_cell2):
                # if two quad_cell has the same x-direction location, then check wether they could merge
                if (quad_cells[index_cell][0].x == cell[0].x and quad_cells[index_cell][1].x == cell[1].x):
                    temp1 = list(quad_cells[index_cell])
                    temp1.append(temp1[0])
                    temp2 = list(cell)
                    temp2.append(temp2[0])
                    area1 = quad_polygon_area(temp1)
                    area2 = quad_polygon_area(temp2)

                    # construct new polygon
                    new_quad = [];
                    new_quad.append(point(temp1[0].x, min(temp1[0].y, temp2[0].y)))
                    new_quad.append(point(temp1[1].x, min(temp1[1].y, temp2[1].y)))
                    new_quad.append(point(temp1[1].x, max(temp1[2].y, temp2[2].y)))
                    new_quad.append(point(temp1[0].x, max(temp1[3].y, temp2[3].y)))
                    area3 = quad_polygon_area(new_quad);

                    if (area1 + area2 >= area3):
                        # merge
                        quads_to_remove.append(index_cell)
                        quads_to_remove.append(index_cell2)
                        quads_to_add.append(new_quad)

    # delete the duplicated index
    quads_to_remove = list(set(quads_to_remove))
    # generate the reversed array, so that deleting one element will not affect the element before
    for index in sorted(quads_to_remove, reverse=True):
        del quad_cells[index]

    for i in quads_to_add:
        quad_cells.append(i)



def remove_duplicate_cell(quad_cells):
    # # Remove duplicates
    to_remove = []
    for index1 in range(len(quad_cells)):
        for index2 in range(index1 + 1, len(quad_cells)):
            duplicate = True
            for k, m in zip(quad_cells[index1], quad_cells[index2]):
                if k.equals(m) is False:
                    duplicate = False
                    break
            if (duplicate is True):
                if index2 not in to_remove:
                    to_remove.append(index2)

    for index in sorted(to_remove, reverse=True):
        del quad_cells[index]


def remove_new_added_merged_cell(quad_cells):
    # since in the previous procedure, we add new quad_cell, then will be a chance that we introduce new overlapping cell
    quads_to_remove = [];
    for index1 in range(len(quad_cells)):
        for index2 in range(len(quad_cells)):
            if (index1 != index2 and quad_cells[index1][0].x == quad_cells[index2][0].x and quad_cells[index1][1].x ==
                    quad_cells[index2][1].x):

                if ((quad_cells[index1][0].y <= quad_cells[index2][0].y) and (
                        quad_cells[index1][1].y <= quad_cells[index2][1].y)
                        and (quad_cells[index1][2].y >= quad_cells[index2][2].y) and (
                                quad_cells[index1][3].y >= quad_cells[index2][3].y)):
                    quads_to_remove.append(index2)

    quads_to_remove = list(set(quads_to_remove))
    for index in sorted(quads_to_remove, reverse=True):
        del quad_cells[index]


def generate_node_set(cells, width, step=None, safeWidth=None):
    nodes = []

    initialize_node_set(nodes, cells)

    for curr_node in nodes:
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


def generate_two_node_path(n1, n2, st_path_matrix, nodes, step=None):
    temp_path = st_path_matrix[n1][n2]
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
    return medium_path


def generate_path(st_path, st_path_matrix, nodes, step=None):
    path = []
    for i in range(len(st_path) - 1):
        path.append(generate_two_node_path(st_path[i], st_path[i + 1], st_path_matrix, nodes, step))
    return path
