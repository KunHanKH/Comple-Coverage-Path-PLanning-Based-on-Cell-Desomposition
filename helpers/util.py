from helpers.geometry import *
import matplotlib.pyplot as plt
from helpers.sweep import *
import cv2
# use cv2.pyrMeanShiftFiltering if filter = True
# sp – The spatial window radius.
# sr – The color window radius.

def generate_polygon_countour(image_name, filter=None, sp=None, sr=None):
	img = cv2.imread(image_name)
	if(filter):
		img = cv2.pyrMeanShiftFiltering(img, sp, sr)
	gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	ret, threshold = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	_, countours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	approxes = []
	for i, cnt in enumerate(countours):
		episilon = 0.01 * cv2.arcLength(cnt, True)
		approxes.append(cv2.approxPolyDP(cnt, episilon, True))
	return [img, approxes]

# Use point object to represent the boundary and the src/dest
def extract_vertex(boundary, source, dest, obstacles):
	boundary = [point(i[0], i[1]) for i in boundary]
	source = point(source[0], source[1])
	dest = point(dest[0], dest[1])

	# generate new sorted vertices based on the x value
	new_sorted_vertices = []
	for index, pnts in enumerate(obstacles):
		for pnt in pnts:
			new_sorted_vertices.append(point(pnt[0], pnt[1], index))
	new_sorted_vertices.sort(key = lambda pnt: pnt.x)

	# reconstruct the obstacle using the point object
	new_obstacles = []
	for index, i in enumerate(obstacles):
		temp_obs = []
		for j in i:
			temp = point(j[0], j[1], index)
			temp_obs.append(temp)
		new_obstacles.append(temp_obs)

	return boundary, source, dest, new_sorted_vertices, new_obstacles



# Draw the obstacles and point the source and the destination----------------------------------------------
def draw_problem(boundary, obstacles, source, dest):
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

		plt.fill( poly_x[index], poly_y[index], color="#512DA8")

	plt.plot(source.x, source.y, marker="o")
	plt.plot(dest.x, dest.y, marker="o")
	plt.annotate('Source', xy=(source.x, source.y), xytext=(source.x+5, source.y-6) )
	plt.annotate('Destination', xy=(dest.x, dest.y), xytext=(dest.x-4, dest.y-10) )
	# plt.show()

def draw_node(nodes, boundary, obstacles, source, dest, fill=None):
	for index, i in enumerate(nodes):
	    draw_problem(boundary, obstacles, source, dest)

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


def get_vertical_line(sorted_vertices, obstacles,  y_limit_lower, y_limit_upper):
	#-----------------------------------------------------------
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

		# Find intersection points with the vertical proposed lines. the intersection function returns false if segments are same, so no need to worry about same segment checking
		for index,obs in enumerate(obstacles):
			# Add the first point again for the last line segment of a polygon.
			obs.append(obs[0])

			for vertex_index in range(len(obs)-1 ):
				# compare curr_line and segment from obstacle
				# check whether the two section is intersected or colinear
				# since the curr_line is the segment [point(pt.x, y_limit_lower), point(pt.x, y_limit_upper)], so that if the points are colinear, it must intersect
				# return the intersection point if the line intersect, and -1 if not.
				res = segment_intersection( curr_line_segment[0], curr_line_segment[1], obs[vertex_index],  obs[vertex_index+1])
				
				# if there is an intersection between current_line_seg and current obstacle edge
				if (res!=-1):
					# make sure the intersection has the same x-value as the current vertical line
					res.x = pt.x
					if pt.equals( res ) == False:
						intersections.append(res)
		# among all the intersection from the current vertical line, choose the closest points that above or below the current vertex
		# For closest_lower point, if the point between it and the current vertex is in the obstacle, then lower_gone is True
		# For cloeset_upper point, if the point between it and the current vertex is in the obstacle, then upper_gone is True
		closest_lower, closest_upper = get_closest_intersection(pt, intersections)
		if(closest_lower != None):
			# check if the middle point pf current point and the intersection is inside theb polygon, lower_gone
			if centroid([pt, closest_lower]).inside_polygon(obstacles):
				lower_gone = True
			else:
				lower_obs_pt = closest_lower
		
		if(closest_upper != None):
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
			plt.plot( [pt.x, upper_obs_pt.x],  [pt.y, upper_obs_pt.y] )
		elif (upper_gone):
			open_line_segments.append([lower_obs_pt, None])
			plt.plot( [lower_obs_pt.x, pt.x],  [lower_obs_pt.y, pt.y] )
		else:
			open_line_segments.append([lower_obs_pt, upper_obs_pt])
			plt.plot( [lower_obs_pt.x, upper_obs_pt.x],  [lower_obs_pt.y, upper_obs_pt.y] )
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
	for index1 in range(len(open_line_segments) ):
		curr_segment = open_line_segments[index1];
		curr_vertex = sorted_vertices[index1];
		plt.plot(curr_vertex.x, curr_vertex.y, marker='o')
		break_now = False;
		done = [False, False, True];

		# if the lower pt is the vertice
		# done[0] is True if there is no polygon below the vertex, or already get the lower polygon
		if( curr_segment[0] is None ):
			done[0] = True; 
		# if the upper vertice is the vertice
		# done[1] is True if there is no polygon above the vertex, or already get the upper polygon
		if( curr_segment[1] is None ):
			done[1] = True;
		# if both lower and upper pts are the vertice, which also means there is no vertical line through this line
		# if done[3] is True if there is no upper or lower polygon for this vertex and find the polygon for this vertex
		if( curr_segment[0] is None and curr_segment[1] is None):
			done[2] = False;

		# index2 the following sorted vertices
		for index2 in range(index1+1,  len(open_line_segments)):
			next_segment = open_line_segments[index2];
			next_vertex = sorted_vertices[index2];

			# double_index1 = -2;
			# double_index2 = -2;
			# lines_to_check = [];
			
			# double_check = False;

			# # double_check is True if there are lines below and above the vertex
			# if (next_segment[0] is not None and next_segment[1] is not None ):
			#     double_check = True;


			# if the lower pt isn't the vertice
			# check whether there is a polygon use the [current_vertex, current_segment[0]] as boundary
			if( done[0] is False ):
				if ( next_segment[0] is not None ):
					# check the upper polygon
					if(check_quad_polygon(curr_vertex, curr_segment[0], next_vertex, next_segment[0], obstacles)):
						quad_cells.append([ curr_segment[0], next_segment[0], next_vertex, curr_vertex ])
						done[0] = True
				
				if( next_segment[1] is not None ):
					# check the lower polygon
					if(check_quad_polygon(curr_vertex, curr_segment[0], next_vertex, next_segment[1], obstacles)):
						quad_cells.append([ curr_segment[0], next_vertex, next_segment[1], curr_vertex ])
						done[0] = True
					
				if(next_segment[0] is None and next_segment[1] is None):
					if(check_right_tri_polygon(curr_vertex, curr_segment[0], next_vertex, obstacles)):
						right_tri_cells.append([ curr_segment[0], next_vertex, curr_vertex ])
						done[0] = True
				   
			# check whether there is a polygon use the [current_vertex, current_segment[1]] as boundary
			if( done[1] is False ):
				if ( next_segment[1] is not None ):
					# check the lowerer polygon
					if(check_quad_polygon(curr_vertex, curr_segment[1], next_vertex, next_segment[1], obstacles)):
	#                     print("add")
						quad_cells.append([ curr_vertex, next_vertex, next_segment[1], curr_segment[1] ])
						done[1] = True
						# print(current_vertex)
			
				if( next_segment[0] is not None ):
					# check the lower polygon
					if(check_quad_polygon(curr_vertex, curr_segment[1], next_vertex, next_segment[0], obstacles)):
	#                     print("add")
						quad_cells.append([ curr_vertex, next_segment[0], next_vertex , curr_segment[1]] )
						done[1] = True
						# print(curr_vertex)
			
				if( next_segment[0] is None and next_segment[1] is None):
					if(check_right_tri_polygon(curr_vertex, curr_segment[1], next_vertex, obstacles)):
	#                     print("add")
						right_tri_cells.append([ curr_vertex, next_vertex, curr_segment[1] ])
						done[1] = True

			
			if( done[2] is False ):
				if ( next_segment[0] is not None ):
					if(check_left_tri_polygon(curr_vertex, next_segment[0], next_vertex, obstacles)):
	#                     print("add")
						left_tri_cells.append([ curr_vertex,next_segment[0], next_vertex ])
						done[2] = True
						
				if( next_segment[1] is not None ):
					if(check_left_tri_polygon(curr_vertex, next_segment[1], next_vertex, obstacles)):
	#                     print("add")
						left_tri_cells.append([ curr_vertex, next_vertex, next_segment[1] ])
						done[2] = True

			if( done[0] == True and done[1] == True and done[2] == True ):
				break
	return quad_cells, left_tri_cells, right_tri_cells


def merge_overlapping_quad_cell(quad_cells):
	# quad_cells = [i for i in cells if len(i)>3]
	# tri_cells = [i for i in cells if len(i)==3]
	# others = [i for i in cells if len(i)<3]
	quads_to_remove = []
	quads_to_add = []

	quads_to_remove = []
	quads_to_add = []
	for index_cell in range(len(quad_cells)):
		for index_cell2,cell in enumerate(quad_cells):
			if(index_cell != index_cell2):
				# if two quad_cell has the same x-direction location, then check wether they could merge
				if(quad_cells[index_cell][0].x == cell[0].x and quad_cells[index_cell][1].x == cell[1].x):
						temp1 = list(quad_cells[index_cell])
						temp1.append(temp1[0])
						temp2 = list(cell)
						temp2.append(temp2[0])
						area1 = quad_polygon_area(temp1)
						area2 = quad_polygon_area(temp2)
						
						# construct new polygon
						new_quad=[];
						new_quad.append( point(temp1[0].x, min(temp1[0].y, temp2[0].y)) )
						new_quad.append( point(temp1[1].x, min(temp1[1].y, temp2[1].y)) )
						new_quad.append( point(temp1[1].x, max(temp1[2].y, temp2[2].y)) )
						new_quad.append( point(temp1[0].x, max(temp1[3].y, temp2[3].y)) )
						area3 = quad_polygon_area(new_quad);
						
						if( area1 + area2 >= area3):
							#merge
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

	remove_duplicate_cell(quad_cells)

	remove_new_added_merged_cell(quad_cells)


	


def remove_duplicate_cell(quad_cells):
	# # Remove duplicates
	to_remove = []
	for index1 in range(len(quad_cells)):
		for index2 in range(index1+1, len(quad_cells)):
			duplicate = True;
			for k,m in zip(quad_cells[index1], quad_cells[index2]):
				if k.equals(m) is False:
					duplicate = False
					break
			if(duplicate is True):
				if index2 not in to_remove:
					to_remove.append(index2)

	for index in sorted(to_remove, reverse=True):
		del quad_cells[index]

def remove_new_added_merged_cell(quad_cells):
	# since in the previous procedure, we add new quad_cell, then will be a chance that we introduce new overlapping cell
	quads_to_remove = [];
	for index1 in range(len(quad_cells)):
		for index2 in range(len(quad_cells)):
			if(index1 != index2 and quad_cells[index1][0].x == quad_cells[index2][0].x and quad_cells[index1][1].x == quad_cells[index2][1].x):

				if( (quad_cells[index1][0].y<= quad_cells[index2][0].y) and  (quad_cells[index1][1].y<= quad_cells[index2][1].y)
					and (quad_cells[index1][2].y>= quad_cells[index2][2].y) and (quad_cells[index1][3].y >= quad_cells[index2][3].y)):
					quads_to_remove.append(index2)
					
	quads_to_remove = list(set(quads_to_remove))
	for index in sorted(quads_to_remove, reverse=True):
		del quad_cells[index]


def get_middle_pnt(nodes, n1, n2):
    for index, ad_node in enumerate(nodes[n1].adjacent):
        if ad_node.index == n2:
            return nodes[n1].middle_point[index]
    for index, ad_node in enumerate(nodes[n2].adjacent):
        if ad_node.index == n1:
            return nodes[n2].middle_point[index]
    print("No middle pnt")
    return None

def generate_two_node_path(n1, n2, st_path_matrix, nodes):
    temp_path = st_path_matrix[n1][n2]
    medium_path = []
    for i in range(len(temp_path)-1):
        start_pnt = nodes[temp_path[i]].centroid
        end_pnt = nodes[temp_path[i+1]].centroid
        
        medium_path.append(start_pnt)
        
        middle_pnt = get_middle_pnt(nodes, temp_path[i], temp_path[i+1])
        
        step_path = stepsBetwn(start_pnt, middle_pnt, 30)
        medium_path = medium_path + step_path
        
        medium_path.append(middle_pnt)
        
        step_path = stepsBetwn(middle_pnt, end_pnt, 30)
        medium_path = medium_path + step_path
    
    medium_path.append(nodes[temp_path[-1]].centroid)
    return medium_path

def generate_path(st_path, st_path_matrix, nodes):
    path = []
    for i in range(len(st_path)-1):
        path.append(generate_two_node_path(st_path[i], st_path[i+1], st_path_matrix, nodes))
    return path