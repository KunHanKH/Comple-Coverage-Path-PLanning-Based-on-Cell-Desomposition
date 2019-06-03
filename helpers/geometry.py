# Helpful geometry functions
# Author -- Shikhar Dev Gupta

from math import *
from random import *
# import matplotlib.pyplot as plt


# Define a point class 
class point:
    def __init__(self, x, y, obstacle = -1, test=-1):
        self.x = x;
        self.y = y;
        self.obstacle = obstacle;
        # Kind of a radioactive tracker! See where your point came from
        self.test = test;

    def __str__(self):
        return ( "x = " + str(self.x) + ", y = " + str(self.y) + ", obs = " + str(self.obstacle) + " and test:"+str(self.test) );


    def __hash__(self):
        return self.x.__hash__() + self.y.__hash__()
    
    def __eq__(self, other):
        if isinstance(other, point):
            if self.x == other.x and self.y == other.y:
                return True
        else:
            return False
    
    def __repr__(self):      
        return str(self.x) + ";"  + str(self.y)


    # Are the two points the same
    def equals(self, other):
        if( self.x == other.x and self.y == other.y):
            return True;
        else:
            return False;

    # Return index of a point from a list of points
    def find_point(self, point_list):
        for i in range(len(point_list)):
            if( self.x == point_list[i].x and self.y == point_list[i].y ):
                return i;
        return -1;

    # Euclidean distance between two points
    def find_dist(self, pt2):
        return int( sqrt(pow((self.x - pt2.x),2) + pow(self.y-pt2.y, 2)) )	;		

    # Check if the point is inside an obstacle
    def inside_polygon(self, obstacles):
        for polygon in obstacles:
            x = self.x; y = self.y; 
            points = [];
            for i in polygon:
                points.append([i.x, i.y]);
            n = len(points);
            inside = False;
            p1x, p1y = points[0];
            # print(points[0])
            # print(p1x)
            # print(p1y)
            for i in range(1, n + 1):
                p2x, p2y = points[i % n];
                if y > min(p1y, p2y):
                    if y <= max(p1y, p2y):
                        if x <= max(p1x, p2x):
                            if p1y != p2y:
                                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
                            if p1x == p2x or x <= xinters:
                                inside = not inside;
                p1x, p1y = p2x, p2y;
            # print(points);
            if(inside is True):
                return True;
        return False;

    # Find the point closest to a given point
    def find_closest_point(self, list_of_vertices):
        min_dist = 99999999; min_index = -1;
        for index, i in enumerate( list_of_vertices):
            dist = find_dist(self, i);
            if(dist<min_dist):
                min_dist = dist;
                min_index = index;
        return min_index;


# define a graph node class
class node:
    def __init__(self, index, polygon, type):

        # define node index
        self.index = index

        # define the polygon
        self.polygon = polygon

        # define the type
        # 1 : quad_cell
        # 2: left_cell
        # 3: right_cell
        # 4: left_right_cell
        self.type = type

        # define the centroid point of each polygon
        self.centroid = centroid(polygon)

        # define end points of trajectory inside each node
        self.start = None
        self.end = None

        # define the adjacent cell that is on the right side
        self.adjacent = []

        # define the middle point from the current cell to the adjacent cell
        # middle point has the same order as adjacent cells
        self.middle_point = []

        # define the path to adjacent polygon
        self.path_to_adjaceny = []

        # define the distance from the current node to the adjacent node:
        # current_cell_centroid -> middle -> point -> adjacent_cell_centroid
        self.distance = []


        # define the inside simple path
        self.inside_path = None


    def add_adjacent(self, node):
        self.adjacent.append(node)

    def get_adjacent(self):
        return self.adjacent

    def add_middle_point(self, point):
        self.middle_point.append(point)

    def get_middle_point(self):
        return self.middle_point

    def add_path_to_adjacency(self, points):
        self.path_to_adjaceny.append(points)

    def get_path_to_adjacency(self):
        return self.path_to_adjaceny

    def calculate_distance(self):
        for index, ad_node in enumerate(self.adjacent):
            distance = find_dist(self.centroid, self.middle_point[index]) + find_dist(self.middle_point[index], self.adjacent[index].centroid)
            self.distance.append(distance)

    def get_distance(self):
        return self.distance



# General procedures---------------------------------------

# Pull a random point from a given range
def random_point(x_range, y_range):
    return point( randint(x_range[0], x_range[1]), randint(y_range[0], y_range[1]) );

# See if three points are counter-clockwise in direction
# See https://www.geeksforgeeks.org/orientation-3-ordered-points/ 
def counter_clockwise(A,B,C):
    # return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
    # if the return value < 0 -> counter clockwise
    # if the return value > 0 -> clockwise
    return (B.y-A.y) * (C.x-B.x) - (C.y-B.y) * (B.x-A.x) < 0

# Return true if line segments AB and CD intersect, or colinear.
def intersect(A,B,C,D):
    # Check if any three points are co-linear 
    # just check wether the three point are colinear, actually need to check the point is on the segment
    # improve later
    if( ( (A.x * (B.y - C.y) ) + (B.x * (C.y - A.y) ) + (C.x * (A.y - B.y) ) )== 0 ):
        return True;
    if( ( (A.x * (B.y - D.y) ) + (B.x * (D.y - A.y) ) + (D.x * (A.y - B.y) ) )== 0 ):
        return True;
    if( ( (A.x * (C.y - D.y) ) + (C.x * (D.y - A.y) ) + (D.x * (A.y - C.y) ) )== 0 ):
        return True;
    if( ( (B.x * (C.y - D.y) ) + (C.x * (D.y - B.y) ) + (D.x * (B.y - C.y) ) )== 0 ):
        return True;

    return counter_clockwise(A,C,D) != counter_clockwise(B,C,D) and counter_clockwise(A,B,C) != counter_clockwise(A,B,D);


# A generic line intersection function. Only returns -1 if lines are parallel
def line_intersection(segment1, segment2):
    line1 = [[segment1[0].x, segment1[0].y], [segment1[1].x, segment1[1].y]]
    line2 = [[segment2[0].x, segment2[0].y], [segment2[1].x, segment2[1].y]]

    x_diff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0]);
    y_diff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]);

    def determinant(a, b):
        return ( (a[0] * b[1]) - (a[1] * b[0]) );

    div = determinant(x_diff, y_diff);
    if div == 0:
        #parallel lines
        return -1;

    d = ( determinant(line1[0] , line1[1]) , determinant(line2[0], line2[1]));
    x = determinant(d, x_diff) / float(div);
    y = determinant(d, y_diff) / float(div);
    x = int(x + 0.5);
    y = int(y + 0.5);
    return point(x, y);


# Final Segment Intersection function
# segment [a, b] and segment [c, d]
def segment_intersection(a, b, c, d):
    if( intersect(a, b, c, d) == True):
        return line_intersection([a, b], [c, d]);
    else:
        return -1;	

# Find centroid of list of vertices
def centroid(vertices):
    n = len(vertices);
    if( n==0 ):
        return -1;	
    sum_x = 0;
    sum_y = 0;
    for i in vertices:
        sum_x = sum_x + i.x;
        sum_y = sum_y + i.y;
    centr_x = int(0.5 + sum_x/float(n) );
    centr_y = int(0.5 + sum_y/float(n) );

    return point(centr_x, centr_y);	


# Find area of a polygon
def quad_polygon_area(vertices):

    # n = number_of_vertices;
    area = 0;
    # area = (vertices[3].y - vertices[0].y + vertices[2].y - vertices[1].y) * (vertices[2].x - vertices[1].x) / 2
    # for i in range(0, n, 2 ):
    #     area += vertices[i+1].x*(vertices[i+2].y-vertices[i].y) + vertices[i+1].y*(vertices[i].x-vertices[i+2].x);
    return int(area);


# Max point in a list of points
def point_max(lst, cmp2 = 1):
    #1 for x and 2 for y
    if(cmp2 == 1):
        max2 = -1; max2_ind = -1;
        tmp = [i.x for i in lst];
        for index,j in enumerate(tmp):
            if(j>max2):
                max2 = j;
                max2_ind = index;
    elif(cmp2 == 2):
        max2 = -1; max2_ind = -1;
        tmp = [i.y for i in lst];
        for index,j in enumerate(tmp):
            if(j>max2):
                max2 = j;
                max2_ind = index;
    return max2_ind;


# Min point in a list of points
def point_min(lst, cmp2 = 1):
    #1 for x and 2 for y
    if(cmp2 == 1):
        min2 = 999999999; min2_ind = -1;
        tmp = [i.x for i in lst];
        for index,j in enumerate(tmp):
            if(j<min2):
                min2 = j;
                min2_ind = index;
    elif(cmp2 == 2):
        min2 = 999999999; min2_ind = -1;
        tmp = [i.y for i in lst];
        for index,j in enumerate(tmp):
            if(j<min2):
                min2 = j;
                min2_ind = index;
    return min2_ind;		

# Distance between two points
def find_dist(pt1, pt2):
    return int( sqrt(pow((pt1.x - pt2.x),2) + pow(pt1.y-pt2.y, 2)) )	;


# Check if the given segment is obstructed by given list of obstacles
# Returns True if segment is clear of obstructions
def check_obstruction(obstacles, segment):

    res = True; break_out = False;
    for obs in obstacles:
        # Add the last line to make closed polygon
        n = len(obs)-1;
        if ( obs[n].equals(obs[0]) is False):
            obs.append(obs[0]);
        for index in range(len(obs)-1):
            if (segment_intersection( segment[0], segment[1],  obs[index],  obs[index+1]) != -1):
                res = False;
                break_out = True;
                break;	
        if(break_out is True):
            break;
    return res;

# Find a point in some distance from the end of a line segment
def find_point_on_line(a, b, step):
    if(a.equals(b)):
        print("sdfsdfsdfsdf");
    v_vector_x = (b.x-a.x);
    v_vector_y = (b.y-a.y);
    vector_norm_x = v_vector_x/float( sqrt( pow(v_vector_x,2) + pow(v_vector_y,2)) );
    vector_norm_y = v_vector_y/float( sqrt( pow(v_vector_x,2) + pow(v_vector_y,2)) );

    result_x = b.x + (step*vector_norm_x);
    result_y = b.y + (step*vector_norm_y);

    return point( int(round(result_x) ), int(round(result_y) ) );


def step_from_to(p1,p2, step_size):
    if find_dist(p1,p2) < step_size:
        return p2;
    else:
        theta = atan2(p2.y-p1.y,p2.x-p1.x)
        return point(p1.x + step_size*cos(theta), p1.y + step_size*sin(theta) );	

def get_closest_intersection(point, intersections):

    min_upper = 99999
    min_lower = 9999
    closest_upper = None
    closest_lower = None
    for i, intersection in enumerate(intersections):
        if (intersection.y - point.y) < min_upper and intersection.y > point.y :
            min_upper = intersection.y - point.y
            closest_upper = intersection

        elif (point.y - intersection.y) < min_lower and intersection.y < point.y :
            min_lower = point.y - intersection.y
            closest_lower = intersection

    return closest_lower, closest_upper


def check_line_obstacle_intersection(line, obstacles):
    for obs in obstacles:
        # Add the last line to make closed polygon
        obs.append( obs[0] )
        for index in range(len(obs)-1):
            if (segment_intersection( line[0], line[1],  obs[index],  obs[index+1]) != -1):
                return True
    return False


def check_quad_polygon(current_vertex, current_segment_point, next_vertex, next_segment_point, obstacles):
    check_line = [centroid([current_segment_point, current_vertex]), centroid([next_segment_point, next_vertex])]
    check_line = refine_check_line(check_line)
    # x = [p.x for p in check_line]
    # y = [p.y for p in check_line]

    if check_line_obstacle_intersection(check_line, obstacles):
        # print("get_intersection")
        return False
    # plt.plot(x, y)
    # print("no intersection")
    return True

# if the right side of the polygon is one point
def check_right_tri_polygon(current_vertex, current_segment_point, next_vertex, obstacles):
    check_line = [centroid([current_segment_point, current_vertex]), next_vertex]
    # check_line = refine_check_line(check_line);
    # x = [p.x for p in check_line]
    # y = [p.y for p in check_line]
    
    if check_line_obstacle_intersection(check_line, obstacles):
        return False
    # plt.plot(x, y)
    # print("no intersection")
    return True

# if the left side of the polygon is one point
def check_left_tri_polygon(current_vertex, next_vertex, next_segment_point, obstacles):
    check_line = [current_vertex, centroid([next_segment_point, next_vertex])]
    check_line = refine_check_line(check_line)
    # x = [p.x for p in check_line]
    # y = [p.y for p in check_line]
    

    if check_line_obstacle_intersection(check_line, obstacles):
        # print("left intersection")
        return False
    # plt.plot(x, y)
    # print("no intersection")
    return True

def refine_check_line(line):
    mean_x = sum([p.x for p in line]) / 2
    mean_y = sum([p.y for p in line]) / 2

    # deep copy the line, in case change the sharing data
    new_line = []
    new_line.append(point(line[0].x, line[0].y))
    new_line.append(point(line[1].x, line[1].y))
    for p in new_line:
        if p.x > mean_x:
            p.x = p.x - 0.5
        else:
            p.x = p.x + 0.5
        # only care about the x direction
        # if p.y > mean_y:
        #     p.y = p.y - 0.5
        # else:
        #     p.y = p.y + 0.5
    return new_line


def merge_polygon(points):
    points = list(set(points))
    points.sort(key = lambda point: point.x)
    upper_points_index = []
    upper_points = []
    for index in range(len(points) - 1):
        if points[index].x == points[index+1].x:
            if points[index].y > points[index+1].y:
                upper_points_index.append(index+1)
                upper_points.append(points[index+1])
            else:
                upper_points_index.append(index)
                upper_points.append(points[index])
    upper_points.sort(key = lambda point: -point.x)
    points = points + upper_points
    upper_points_index = sorted(upper_points_index, reverse=True)
    for index in upper_points_index:
        points.pop(index)
    return points

