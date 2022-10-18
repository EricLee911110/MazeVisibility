/************************************************************************
     File:        Maze.cpp

     Author:     
                  Stephen Chenney, schenney@cs.wisc.edu
     Modifier
                  Yu-Chi Lai, yu-chi@cs.wisc.edu

     Comment:    
						(c) 2001-2002 Stephen Chenney, University of Wisconsin at Madison

						Class header file for Maze class. Manages the maze.
		

     Platform:    Visio Studio.Net 2003 (converted to 2005)

*************************************************************************/

#include "Maze.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <FL/Fl.h>
#include <FL/fl_draw.h>

// I added these includes from MazeWindow.cpp
#include <Fl/math.h>
#include <Fl/gl.h>
#include <GL/glu.h>
#include <stdio.h>
#include <iostream>
#include "LineSeg.h"
using namespace std;

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>


using namespace std;
void myPerspective(float fovInDegrees, float aspect, float znear, float zfar);		// identifier not found
void myFrustum(float* matrix, float left, float right, float bottom, float top, float znear, float zfar);		// identifier not found

float projection[16];
float view[16];
float transform[16];
float aspect = 0.0f;

float my_near = 0.01f;
float my_far = 200.0f;



const char Maze::X = 0;
const char Maze::Y = 1;
const char Maze::Z = 2;

const float Maze::BUFFER = 0.1f;


//**********************************************************************
//
// * Constructor for the maze exception
//======================================================================
MazeException::
MazeException(const char *m)
//======================================================================
{
	message = new char[strlen(m) + 4];
	strcpy(message, m);
}


//**********************************************************************
//
// * Constructor to create the default maze
//======================================================================
Maze::
Maze(const int nx, const int ny, const float sx, const float sy)
//======================================================================
{
	// Build the connectivity structure.
	Build_Connectivity(nx, ny, sx, sy);

	// Make edges transparent to create a maze.
	Build_Maze();

	// Set the extents of the maze
	Set_Extents();

	// Default values for the viewer.
	viewer_posn[X] = viewer_posn[Y] = viewer_posn[Z] = 0.0;
	viewer_dir = 0.0;
	viewer_fov = 45.0;

	// Always start on the 0th frame.
	frame_num = 0;
}


//**********************************************************************
//
// * Construtor to read in precreated maze
//======================================================================
Maze::
Maze(const char *filename)
//======================================================================
{
	char    err_string[128];
	FILE    *f;
	int	    i;

	// Open the file
	if ( ! ( f = fopen(filename, "r") ) )
		throw new MazeException("Maze: Couldn't open file");

	// Get the total number of vertices
	if ( fscanf(f, "%d", &num_vertices) != 1 )
		throw new MazeException("Maze: Couldn't read number of vertices");

	// Read in each vertices
	vertices = new Vertex*[num_vertices];
	for ( i = 0 ; i < num_vertices ; i++ ) {
		float x, y;
		if ( fscanf(f, "%g %g", &x, &y) != 2 )	{
			sprintf(err_string, "Maze: Couldn't read vertex number %d", i);
			throw new MazeException(err_string);
		}
		vertices[i] = new Vertex(i, x, y);
	}

	// Get the number of edges
	if ( fscanf(f, "%d", &num_edges) != 1 )
		throw new MazeException("Maze: Couldn't read number of edges");

	// read in all edges
	edges = new Edge*[num_edges];
	for ( i = 0 ; i < num_edges ; i++ ){
		int     vs, ve, cl, cr, o;
		float	r, g, b;
		if ( fscanf(f, "%d %d %d %d %d %g %g %g",
						&vs, &ve, &cl, &cr, &o, &r, &g, &b) != 8) {
			sprintf(err_string, "Maze: Couldn't read edge number %d", i);
			throw new MazeException(err_string);
		}
		edges[i] = new Edge(i, vertices[vs], vertices[ve], r, g, b);
		edges[i]->Add_Cell((Cell*)cl, Edge::LEFT);
		edges[i]->Add_Cell((Cell*)cr, Edge::RIGHT);
		edges[i]->opaque = o ? true : false;
	}

	// Read in the number of cells
	if ( fscanf(f, "%d", &num_cells) != 1 )
		throw new MazeException("Maze: Couldn't read number of cells");


	// Read in all cells
	cells = new Cell*[num_cells];
	for ( i = 0 ; i < num_cells ; i++ )	{
		int epx, epy, emx, emy;
		if ( fscanf(f, "%d %d %d %d", &epx, &epy, &emx, &emy) != 4 ){
			sprintf(err_string, "Maze: Couldn't read cell number %d", i);
			throw new MazeException(err_string);
		}
		cells[i] = new Cell(i, epx >= 0 ? edges[epx] : NULL,
									epy >= 0 ? edges[epy] : NULL,
									emx >= 0 ? edges[emx] : NULL,
									emy >= 0 ? edges[emy] : NULL);
		if ( cells[i]->edges[0] ) {
			if ( cells[i]->edges[0]->neighbors[0] == (Cell*)i )
				cells[i]->edges[0]->neighbors[0] = cells[i];
			else if ( cells[i]->edges[0]->neighbors[1] == (Cell*)i )
				cells[i]->edges[0]->neighbors[1] = cells[i];
			else	{
				sprintf(err_string,
						  "Maze: Cell %d not one of edge %d's neighbors",
							i, cells[i]->edges[0]->index);
				throw new MazeException(err_string);
			}
		}

		if ( cells[i]->edges[1] )	{
			if ( cells[i]->edges[1]->neighbors[0] == (Cell*)i )
				cells[i]->edges[1]->neighbors[0] = cells[i];
			else if ( cells[i]->edges[1]->neighbors[1] == (Cell*)i )
				cells[i]->edges[1]->neighbors[1] = cells[i];
			else {
				sprintf(err_string,
							"Maze: Cell %d not one of edge %d's neighbors",
							i, cells[i]->edges[1]->index);
				throw new MazeException(err_string);
			}
		}
		if ( cells[i]->edges[2] ) {
			if ( cells[i]->edges[2]->neighbors[0] == (Cell*)i )
				cells[i]->edges[2]->neighbors[0] = cells[i];
			else if ( cells[i]->edges[2]->neighbors[1] == (Cell*)i )
				cells[i]->edges[2]->neighbors[1] = cells[i];
			else	{
				sprintf(err_string,
							"Maze: Cell %d not one of edge %d's neighbors",
							i, cells[i]->edges[2]->index);
				throw new MazeException(err_string);
			}
		}
		if ( cells[i]->edges[3] ) {
			if ( cells[i]->edges[3]->neighbors[0] == (Cell*)i )
				cells[i]->edges[3]->neighbors[0] = cells[i];
			else if ( cells[i]->edges[3]->neighbors[1] == (Cell*)i )
				cells[i]->edges[3]->neighbors[1] = cells[i];
			else	{
				sprintf(err_string,
							"Maze: Cell %d not one of edge %d's neighbors",
							i, cells[i]->edges[3]->index);
				throw new MazeException(err_string);
			}
		}
	}

	if ( fscanf(f, "%g %g %g %g %g",
					 &(viewer_posn[X]), &(viewer_posn[Y]), &(viewer_posn[Z]),
					 &(viewer_dir), &(viewer_fov)) != 5 )
		throw new MazeException("Maze: Error reading view information.");

	// Some edges have no neighbor on one side, so be sure to set their
	// pointers to NULL. (They were set at -1 by the save/load process.)
	for ( i = 0 ; i < num_edges ; i++ )	{
		if ( edges[i]->neighbors[0] == (Cell*)-1 )
			edges[i]->neighbors[0] = NULL;
		if ( edges[i]->neighbors[1] == (Cell*)-1 )
			edges[i]->neighbors[1] = NULL;
	}

	fclose(f);

	Set_Extents();

	// Figure out which cell the viewer is in, starting off by guessing the
	// 0th cell.
	Find_View_Cell(cells[0]);

	frame_num = 0;
}


//**********************************************************************
//
// * Destructor must free all the memory allocated.
//======================================================================
Maze::
~Maze(void)
//======================================================================
{
	int i;

	for ( i = 0 ; i < num_vertices ; i++ )
		delete vertices[i];
	delete[] vertices;

	for ( i = 0 ; i < num_edges ; i++ )
		delete edges[i];
	delete[] edges;

	for ( i = 0 ; i < num_cells ; i++ )
		delete cells[i];
	delete[] cells;
}


//**********************************************************************
//
// * Randomly generate the edge's opaque and transparency for an empty maze
//======================================================================
void Maze::
Build_Connectivity(const int num_x, const int num_y,
                   const float sx, const float sy)
//======================================================================
{
	int	i, j, k;
	int edge_i;

	// Ugly code to allocate all the memory for a new maze and to associate
	// edges with vertices and faces with edges.

	// Allocate and position the vertices.
	num_vertices = ( num_x + 1 ) * ( num_y + 1 );
	vertices = new Vertex*[num_vertices];
	k = 0;
	for ( i = 0 ; i < num_y + 1 ; i++ ) {
		for ( j = 0 ; j < num_x + 1 ; j++ )	{
			vertices[k] = new Vertex(k, j * sx, i * sy);
			k++;
		}
	}

	// Allocate the edges, and associate them with their vertices.
	// Edges in the x direction get the first num_x * ( num_y + 1 ) indices,
	// edges in the y direction get the rest.
	num_edges = (num_x+1)*num_y + (num_y+1)*num_x;
	edges = new Edge*[num_edges];
	k = 0;
	for ( i = 0 ; i < num_y + 1 ; i++ ) {
		int row = i * ( num_x + 1 );
		for ( j = 0 ; j < num_x ; j++ ) {
			int vs = row + j;
			int ve = row + j + 1;
			edges[k] = new Edge(k, vertices[vs], vertices[ve],
			rand() / (float)RAND_MAX * 0.5f + 0.25f,
			rand() / (float)RAND_MAX * 0.5f + 0.25f,
			rand() / (float)RAND_MAX * 0.5f + 0.25f);
			k++;
		}
	}

	edge_i = k;
	for ( i = 0 ; i < num_y ; i++ ) {
		int row = i * ( num_x + 1 );
		for ( j = 0 ; j < num_x + 1 ; j++ )	{
			int vs = row + j;
			int ve = row + j + num_x + 1;
			edges[k] = new Edge(k, vertices[vs], vertices[ve],
			rand() / (float)RAND_MAX * 0.5f + 0.25f,
			rand() / (float)RAND_MAX * 0.5f + 0.25f,
			rand() / (float)RAND_MAX * 0.5f + 0.25f);
			k++;
		}
	}

	// Allocate the cells and associate them with their edges.
	num_cells = num_x * num_y;
	cells = new Cell*[num_cells];
	k = 0;
	for ( i = 0 ; i < num_y ; i++ ) {
		int row_x = i * ( num_x + 1 );
		int row_y = i * num_x;
		for ( j = 0 ; j < num_x ; j++ )	{
			int px = edge_i + row_x + 1 + j;
			int py = row_y + j + num_x;
			int mx = edge_i + row_x + j;
			int my = row_y + j;
			cells[k] = new Cell(k, edges[px], edges[py], edges[mx], edges[my]);
			edges[px]->Add_Cell(cells[k], Edge::LEFT);
			edges[py]->Add_Cell(cells[k], Edge::RIGHT);
			edges[mx]->Add_Cell(cells[k], Edge::RIGHT);
			edges[my]->Add_Cell(cells[k], Edge::LEFT);
			k++;
		}
	}
}


//**********************************************************************
//
// * Add edges from cell to the set that are available for removal to
//   grow the maze.
//======================================================================
static void
Add_To_Available(Cell *cell, int *available, int &num_available)
//======================================================================
{
	int i, j;

	// Add edges from cell to the set that are available for removal to
	// grow the maze.

	for ( i = 0 ; i < 4 ; i++ ){
		Cell    *neighbor = cell->edges[i]->Neighbor(cell);

		if ( neighbor && ! neighbor->counter )	{
			int candidate = cell->edges[i]->index;
			for ( j = 0 ; j < num_available ; j++ )
				if ( candidate == available[j] ) {
					printf("Breaking early\n");
					break;
			}
			if ( j == num_available )  {
				available[num_available] = candidate;
				num_available++;
			}
		}
	}

	cell->counter = 1;
}


//**********************************************************************
//
// * Grow a maze by removing candidate edges until all the cells are
//   connected. The edges are not actually removed, they are just made
//   transparent.
//======================================================================
void Maze::
Build_Maze()
//======================================================================
{
	Cell    *to_expand;
	int     index;
	int     *available = new int[num_edges];
	int     num_available = 0;
	int	    num_visited;
	int	    i;

	srand(time(NULL));

	// Choose a random starting cell.
	index = (int)floor((rand() / (float)RAND_MAX) * num_cells);
	to_expand = cells[index];
	Add_To_Available(to_expand, available, num_available);
	num_visited = 1;

	// Join cells up by making edges opaque.
	while ( num_visited < num_cells && num_available > 0 ) {
		int ei;

		index = (int)floor((rand() / (float)RAND_MAX) * num_available);
		to_expand = NULL;

		ei = available[index];

		if ( edges[ei]->neighbors[0] && 
			 !edges[ei]->neighbors[0]->counter )
			to_expand = edges[ei]->neighbors[0];
		else if ( edges[ei]->neighbors[1] && 
			 !edges[ei]->neighbors[1]->counter )
			to_expand = edges[ei]->neighbors[1];

		if ( to_expand ) {
			edges[ei]->opaque = false;
			Add_To_Available(to_expand, available, num_available);
			num_visited++;
		}

		available[index] = available[num_available-1];
		num_available--;
	}

	for ( i = 0 ; i < num_cells ; i++ )
		cells[i]->counter = 0;
}


//**********************************************************************
//
// * Go through all the vertices looking for the minimum and maximum
//   extents of the maze.
//======================================================================
void Maze::
Set_Extents(void)
//======================================================================
{
	int i;

	min_xp = vertices[0]->posn[Vertex::X];
	max_xp = vertices[0]->posn[Vertex::X];
	min_yp = vertices[0]->posn[Vertex::Y];
	max_yp = vertices[0]->posn[Vertex::Y];
	for ( i = 1 ; i < num_vertices ; i++ ) {
		if ( vertices[i]->posn[Vertex::X] > max_xp )
			 max_xp = vertices[i]->posn[Vertex::X];
		if ( vertices[i]->posn[Vertex::X] < min_xp )
			 min_xp = vertices[i]->posn[Vertex::X];
		if ( vertices[i]->posn[Vertex::Y] > max_yp )
			 max_yp = vertices[i]->posn[Vertex::Y];
		if ( vertices[i]->posn[Vertex::Y] < min_yp )
			 min_yp = vertices[i]->posn[Vertex::Y];
    }
}


//**********************************************************************
//
// * Figure out which cell the view is in, using seed_cell as an
//   initial guess. This procedure works by repeatedly checking
//   whether the viewpoint is in the current cell. If it is, we're
//   done. If not, Point_In_Cell returns in new_cell the next cell
//   to test. The new cell is the one on the other side of an edge
//   that the point is "outside" (meaning that it might be inside the
//   new cell).
//======================================================================
void Maze::
Find_View_Cell(Cell *seed_cell)
//======================================================================
{
	Cell    *new_cell;

	// 
	while ( ! ( seed_cell->Point_In_Cell(viewer_posn[X], viewer_posn[Y],
													 viewer_posn[Z], new_cell) ) ) {
		if ( new_cell == 0 ) {
			// The viewer is outside the top or bottom of the maze.
			throw new MazeException("Maze: View not in maze\n");
		}

		seed_cell = new_cell;
    }
    
    view_cell = seed_cell;
}


//**********************************************************************
//
// * Move the viewer's position. This method will do collision detection
//   between the viewer's location and the walls of the maze and prevent
//   the viewer from passing through walls.
//======================================================================
void Maze::
Move_View_Posn(const float dx, const float dy, const float dz)
//======================================================================
{
	Cell    *new_cell;
	float   xs, ys, zs, xe, ye, ze;

	// Move the viewer by the given amount. This does collision testing to
	// prevent walking through walls. It also keeps track of which cells the
	// viewer is in.

	// Set up a line segment from the start to end points of the motion.
	xs = viewer_posn[X];
	ys = viewer_posn[Y];
	zs = viewer_posn[Z];
	xe = xs + dx;
	ye = ys + dy;
	ze = zs + dz;

	// Fix the z to keep it in the maze.
	if ( ze > 1.0f - BUFFER )
		ze = 1.0f - BUFFER;
	if ( ze < BUFFER - 1.0f )
		ze = BUFFER - 1.0f;

	// Clip_To_Cell clips the motion segment to the view_cell if the
	// segment intersects an opaque edge. If the segment intersects
	// a transparent edge (through which it can pass), then it clips
	// the motion segment so that it _starts_ at the transparent edge,
	// and it returns the cell the viewer is entering. We keep going
	// until Clip_To_Cell returns NULL, meaning we've done as much of
	// the motion as is possible without passing through walls.
	while ( ( new_cell = view_cell->Clip_To_Cell(xs, ys, xe, ye, BUFFER) ) )
		view_cell = new_cell;

	// The viewer is at the end of the motion segment, which may have
	// been clipped.
	viewer_posn[X] = xe;
	viewer_posn[Y] = ye;
	viewer_posn[Z] = ze;
}

//**********************************************************************
//
// * Set the viewer's location 
//======================================================================
void Maze::
Set_View_Posn(float x, float y, float z)
//======================================================================
{
	// First make sure it's in some cell.
	// This assumes that the maze is rectangular.
	if ( x < min_xp + BUFFER )
		x = min_xp + BUFFER;
	if ( x > max_xp - BUFFER )
		x = max_xp - BUFFER;
	if ( y < min_yp + BUFFER )
		y = min_yp + BUFFER;
	if ( y > max_yp - BUFFER )
		y = max_yp - BUFFER;
	if ( z < -1.0f + BUFFER )
		z = -1.0f + BUFFER;
	if ( z > 1.0f - BUFFER )
		z = 1.0f - BUFFER;

	viewer_posn[X] = x;
	viewer_posn[Y] = y;
	viewer_posn[Z] = z;

	// Figure out which cell we're in.
	Find_View_Cell(cells[0]);
}


//**********************************************************************
//
// * Set the angle in which the viewer is looking.
//======================================================================
void Maze::
Set_View_Dir(const float d)
//======================================================================
{
	viewer_dir = d;
}


//**********************************************************************
//
// * Set the horizontal field of view.
//======================================================================
void Maze::
Set_View_FOV(const float f)
//======================================================================
{
	viewer_fov = f;
}


//**********************************************************************
//
// * Draws the map view of the maze. It is passed the minimum and maximum
//   corners of the window in which to draw.
//======================================================================
void Maze::
Draw_Map(int min_x, int min_y, int max_x, int max_y)
//======================================================================
{
	int	    height;
	float   scale_x, scale_y, scale;
	int	    i;

	// Figure out scaling factors and the effective height of the window.
	scale_x = ( max_x - min_x - 10 ) / ( max_xp - min_xp );
	scale_y = ( max_y - min_y - 10 ) / ( max_yp - min_yp );
	scale = scale_x > scale_y ? scale_y : scale_x;
	height = (int)ceil(scale * ( max_yp - min_yp ));

	min_x += 5;
	min_y += 5;

	// Draw all the opaque edges.
	for ( i = 0 ; i < num_edges ; i++ )
		if ( edges[i]->opaque )	{
			float   x1, y1, x2, y2;

			x1 = edges[i]->endpoints[Edge::START]->posn[Vertex::X];
			y1 = edges[i]->endpoints[Edge::START]->posn[Vertex::Y];
			x2 = edges[i]->endpoints[Edge::END]->posn[Vertex::X];
			y2 = edges[i]->endpoints[Edge::END]->posn[Vertex::Y];

			fl_color((unsigned char)floor(edges[i]->color[0] * 255.0),
					 (unsigned char)floor(edges[i]->color[1] * 255.0),
					 (unsigned char)floor(edges[i]->color[2] * 255.0));
			fl_line_style(FL_SOLID);
			fl_line(min_x + (int)floor((x1 - min_xp) * scale),
					  min_y + height - (int)floor((y1 - min_yp) * scale),
					  min_x + (int)floor((x2 - min_xp) * scale),
					  min_y + height - (int)floor((y2 - min_yp) * scale));
		}
}


void Maze::
set_aspect_from_MazeWindow(float aspect_from_MazeWindow) {
	aspect = aspect_from_MazeWindow;

	// cout << "aspect set!!: " << aspect << endl;
}

bool clip(LineSeg frustum_side, float* start, float* end) {
	cout << "you just called clip function" << endl;

	int s_side = frustum_side.Point_Side(start[0], start[2]);
	int e_side = frustum_side.Point_Side(end[0], end[2]);
	cout << "start_side: " << s_side << " end_side: " << e_side << endl;
	if (s_side == 1) {		// 1 if right, start at right
		if (e_side == 0) {	// 0 if left  // start right, end left
			float percent = frustum_side.Cross_Param(LineSeg(start[0], start[2], end[0], end[2]));
			end[0] = frustum_side.start[0] + (frustum_side.end[0] - frustum_side.start[0]) * percent;
			end[2] = frustum_side.start[1] + (frustum_side.end[1] - frustum_side.start[1]) * percent;
			
		}
	}
	else if (e_side == 1) {		// start is left, end is right
		float percent = frustum_side.Cross_Param(LineSeg(start[0], start[2], end[0], end[2]));
		start[0] = frustum_side.start[0] + (frustum_side.end[0] - frustum_side.start[0]) * percent;
		start[2] = frustum_side.start[1] + (frustum_side.end[1] - frustum_side.start[1]) * percent;
	}
	else {						// start left, end left
		return false;
	}
	return true;
	
}

void myMatrixMul(float* vec, float* matrix) {
	float result[4];

	result[0] = matrix[0] * vec[0] + matrix[4] * vec[1] + matrix[8] * vec[2] + matrix[12] * vec[3];
	result[1] = matrix[1] * vec[0] + matrix[5] * vec[1] + matrix[9] * vec[2] + matrix[13] * vec[3];
	result[2] = matrix[2] * vec[0] + matrix[6] * vec[1] + matrix[10] * vec[2] + matrix[14] * vec[3];
	result[3] = matrix[3] * vec[0] + matrix[7] * vec[1] + matrix[11] * vec[2] + matrix[15] * vec[3];


	vec[0] = result[0];
	vec[1] = result[1];
	vec[2] = result[2];
	vec[3] = result[3];



}

void Maze::
Draw_Wall(float* start, float* end, float* color) {
	myMatrixMul(start, transform);
	myMatrixMul(start, view);


	myMatrixMul(end, transform);
	myMatrixMul(end, view);


	LineSeg left_frustum(my_near * tan(To_Radians(viewer_fov * 0.5f)), -my_near, my_far * tan(To_Radians(viewer_fov * 0.5f)), -my_far);
	LineSeg right_frustum(-my_far * tan(To_Radians(viewer_fov * 0.5f)), -my_far, -my_near * tan(To_Radians(viewer_fov * 0.5f)), -my_near);

	cout << "left_frustum: " << left_frustum.start[0] << " " << left_frustum.start[1] << " " << left_frustum.end[0] << " " << left_frustum.end[1] << endl;
	cout << "right_frustum: " << right_frustum.start[0] << " " << right_frustum.start[1] << " " << right_frustum.end[0] << " " << right_frustum.end[1] << endl;
	cout << "start: " << start[0] << " " << start[1] << " " << start[2] << endl;
	cout << "end: " << end[0] << " " << start[1] << " " << end[2] << endl;

	if (clip(left_frustum, start, end) && clip(right_frustum, start, end)) {
		cout << "start clipping: " << start[0] << " " << start[1] << " " << start[2] << endl;
		cout << "end clipping: " << end[0] << " " << start[1] << " " << end[2] << endl;

		myMatrixMul(start, projection);
		myMatrixMul(end, projection);
		cout << "start projection: " << start[0] << " " << start[1] << " " << start[2] << endl;
		cout << "end projection: " << end[0] << " " << start[1] << " " << end[2] << endl;

		if (start[3] < my_far && end[3] < my_far) {
			divideW(start);
			divideW(end);
			cout << "start divideW: " << start[0] << " " << start[1] << " " << start[2] << endl;
			cout << "end divideW: " << end[0] << " " << start[1] << " " << end[2] << endl;
			glBegin(GL_POLYGON);
			glColor3f(color[0], color[1], color[2]);

			glVertex2f(start[0], start[1]);
			glVertex2f(end[0], end[1]);
			glVertex2f(end[0], -end[1]);
			glVertex2f(start[0], -start[1]);
			glEnd();
		}
	}

}






// for more detail how this matrix work: https://www.khronos.org/opengl/wiki/GluPerspective_code
void Maze::
myPerspective(float fovInDegrees, float aspect, float znear, float zfar) {

	float matrix[16];
	float ymax, xmax;

	ymax = znear * tanf(fovInDegrees * M_PI / 360.0);
	xmax = ymax * aspect;

	myFrustum(matrix, -xmax, xmax, -ymax, ymax, znear, zfar);
}

void myFrustum(float* matrix, float left, float right, float bottom, float top, float znear, float zfar) {
	float temp, temp2, temp3, temp4;
	temp = 2.0 * znear;
	temp2 = right - left;
	temp3 = top - bottom;
	temp4 = zfar - znear;
	projection[0] = temp / temp2;
	projection[1] = 0.0f;
	projection[2] = 0.0f;
	projection[3] = 0.0f;
	projection[4] = 0.0f;
	projection[5] = temp / temp3;
	projection[6] = 0.0f;
	projection[7] = 0.0f;
	projection[8] = (right + left) / temp2;
	projection[9] = (top + bottom) / temp3;
	projection[10] = (-zfar - znear) / temp4;
	projection[11] = -1.0;
	projection[12] = 0.0;
	projection[13] = 0.0;
	projection[14] = (-temp * zfar) / temp4;
	projection[15] = 0.0;
}


void NormalizeVector(float x[3]) {
	float Norm = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
	x[0] /= Norm;
	x[1] /= Norm;
	x[2] /= Norm;
}

void myCross(float result[3], float A[3], float B[3]) {
	result[0] = A[1] * B[2] - A[2] * B[1];
	result[1] = A[2] * B[0] - A[0] * B[2];
	result[2] = A[0] * B[1] - A[1] * B[0];
}

void Maze::
myLookAt(float camPosition3Dx, float camPosition3Dy, float camPosition3Dz,
	float center3Dx, float center3Dy, float center3Dz,
	float upVector3Dx, float upVector3Dy, float upVector3Dz)
{
	cout << endl;
	cout << "camPosition: " << camPosition3Dx << " " << camPosition3Dy << " " << camPosition3Dz << endl;
	// cout << "center: " << center3Dx << " " << center3Dy << " " << center3Dz << endl;

	float forward[3], side[3], up[3];
	
	forward[0] = center3Dx - camPosition3Dx;
	forward[1] = center3Dy - camPosition3Dy;
	forward[2] = center3Dz - camPosition3Dz;
	
	
	NormalizeVector(forward);

	// cout << "forward: " << forward[0] << " " << forward[1] << " " << forward[2] << endl;

	float tmp[3] = { upVector3Dx ,upVector3Dy, upVector3Dz };
	myCross(side, forward, tmp);
	NormalizeVector(side);

	// cout << "side: " << side[0] << " " << side[1] << " " << side[2] << endl;

	myCross(up, side, forward);

	// cout << "up: " << up[0] << " " << up[1] << " " << up[2] << endl;
	
	view[0] = side[0];
	view[4] = side[1];
	view[8] = side[2];
	view[12] = 0.0f;
	view[1] = up[0];
	view[5] = up[1];
	view[9] = up[2];
	view[13] = 0.0f;
	view[2] = -forward[0];
	view[6] = -forward[1];
	view[10] = -forward[2];
	view[14] = 0.0f;
	view[3] = 0.0f;
	view[7] = 0.0f;
	view[11] = 0.0f;
	view[15] = 1.0f;
	

	
	transform[0] = 1.0f;
	transform[4] = 0.0f;
	transform[8] = 0.0f;
	transform[12] = -camPosition3Dx;
	transform[1] = 0.0f;
	transform[5] = 1.0f;
	transform[9] = 0.0f;
	transform[13] = -camPosition3Dy;
	transform[2] = 0.0f;
	transform[6] = 0.0f;
	transform[10] = 1.0f;
	transform[14] = -camPosition3Dz;
	transform[3] = 0.0f;
	transform[7] = 0.0f;
	transform[11] = 0.0f;
	transform[15] = 1.0f;
	
}

void Maze::
divideW(float* A) {
	A[0] = A[0] / A[3];
	A[1] = A[1] / A[3];
	A[2] = A[2] / A[3];
	A[3] = A[3] / A[3];
}


//**********************************************************************
//
// * Draws the first-person view of the maze. It is passed the focal distance.
//   THIS IS THE FUINCTION YOU SHOULD MODIFY.
//======================================================================
void Maze::
Draw_View(const float focal_dist)
//======================================================================
{
	frame_num++;

	//###################################################################
	// TODO
	// The rest is up to you!
	//###################################################################

	// cout << "enable depth test" << endl;
	// glEnable(GL_DEPTH_TEST);
	// glLoadIdentity();
	// glMultMatrixf((GLfloat*)&projection);
	// glMultMatrixf((GLfloat*)&view);
	// glMatrixMode(GL_PROJECTION);
	// glMatrixMode(GL_MODELVIEW);

	float viewer_pos[3] = { viewer_posn[Maze::Y], 0.0f, viewer_posn[Maze::X] };

	
	// glLoadMatrix(GL_PROJECTION);
	// glLoadIdentity();
	glLoadIdentity();

	myPerspective(viewer_fov, aspect, my_near, my_far);
	//glMultMatrixf(projection);

	//glLoadMatrixf(projection);
	
	/*
	gluLookAt(viewer_pos[0], viewer_pos[1], viewer_pos[2],
		viewer_pos[0] + sin(To_Radians(viewer_dir)),
		viewer_pos[1], 
		viewer_pos[2] + cos(To_Radians(viewer_dir)),
		0.0, 1.0, 0.0);

	*/

	myLookAt(viewer_pos[0], viewer_pos[1], viewer_pos[2],
		viewer_pos[0] + sin(To_Radians(viewer_dir)),
		viewer_pos[1],
		viewer_pos[2] + cos(To_Radians(viewer_dir)),
		0.0, 1.0, 0.0);
	//glMultMatrixf(view);		// view is actually rotate
	//glMultMatrixf(transform);
	
	
	/*
	cout << view[0] << " " << view[1] << " " << view[2] << " " << view[3] << endl;
	cout << view[4] << " " << view[5] << " " << view[6] << " " << view[7] << endl;
	cout << view[8] << " " << view[9] << " " << view[10] << " " << view[11] << endl;
	cout << view[12] << " " << view[13] << " " << view[14] << " " << view[15] << endl;
	*/

	for (int i = 0; i < num_edges; i++) {

		float start[4] = { edges[i]->endpoints[0]->posn[Y], 1.0f, edges[i]->endpoints[0]->posn[X], 1.0f };
		float end[4] = { edges[i]->endpoints[1]->posn[Y], 1.0f, edges[i]->endpoints[1]->posn[X], 1.0f };
		float color[3] = { edges[i]->color[0], edges[i]->color[1], edges[i]->color[2] };



		if (edges[i]->opaque) {

			//std::cout << glm::to_string(view) << std::endl;

			Draw_Wall(start, end, color);
		}



	}

	/* clipping successfully

	float start[4] = { 6, 1, 0, 1 };
	float end[4] = { 6, 1, 10, 1 };

	myMatrixMul(start, transform);
	myMatrixMul(start, view);
	

	myMatrixMul(end, transform);
	myMatrixMul(end, view);

	
	LineSeg left_frustum(my_near * tan(To_Radians(viewer_fov * 0.5f)), -my_near, my_far * tan(To_Radians(viewer_fov * 0.5f)), -my_far);
	LineSeg right_frustum(-my_far * tan(To_Radians(viewer_fov * 0.5f)), -my_far, -my_near * tan(To_Radians(viewer_fov * 0.5f)), -my_near);

	cout << "left_frustum: " << left_frustum.start[0] << " " << left_frustum.start[1] << " " << left_frustum.end[0] << " " << left_frustum.end[1] << endl;
	cout << "right_frustum: " << right_frustum.start[0] << " " << right_frustum.start[1] << " " << right_frustum.end[0] << " " << right_frustum.end[1] << endl;
	cout << "start: " << start[0] << " " << start[1] << " " << start[2] << endl;
	cout << "end: " << end[0] << " " << start[1] << " " << end[2] << endl;

	if (clip(left_frustum, start, end) && clip(right_frustum, start, end)) {
		cout << "start clipping: " << start[0] << " " << start[1] << " " << start[2] << endl;
		cout << "end clipping: " << end[0] << " " << start[1] << " " << end[2] << endl;

		myMatrixMul(start, projection);
		myMatrixMul(end, projection);
		cout << "start projection: " << start[0] << " " << start[1] << " " << start[2] << endl;
		cout << "end projection: " << end[0] << " " << start[1] << " " << end[2] << endl;

		if (start[3] < my_far && end[3] < my_far) {
			divideW(start);
			divideW(end);
			cout << "start divideW: " << start[0] << " " << start[1] << " " << start[2] << endl;
			cout << "end divideW: " << end[0] << " " << start[1] << " " << end[2] << endl;
			glBegin(GL_POLYGON);
			glColor3f(0.0f, 1.0f, 0.0f);

			glVertex2f(start[0], start[1]);
			glVertex2f(end[0], end[1]);
			glVertex2f(end[0], -end[1]);
			glVertex2f(start[0], -start[1]);
			glEnd();
		}
	}

	*/
	

	



	

	/*
	if (!clip(left_frustum, start, end) || !clip(right_frustum, start, end)) {
		glBegin(GL_POLYGON);
		glColor3f(0.0f, 1.0f, 0.0f);

		glVertex2f(start[0], start[1]);
		glVertex2f(end[0], end[1]);
		glVertex2f(end[0], -end[1]);
		glVertex2f(start[0], -start[1]);
		glEnd();
	}
	*/


	
	

	
	/*
	LineSeg left_frustum(0, 0, cos(To_Radians(90 + viewer_fov / 2)), sin(To_Radians(90 + viewer_fov / 2)));
	LineSeg right_frustum(0, 0, cos(To_Radians(90 - viewer_fov / 2)), sin(To_Radians(90 - viewer_fov / 2)));
	cout << "left_frustum: " << left_frustum.end[0] << " " << left_frustum.end[1] << endl;
	cout << "right_frustum: " << right_frustum.end[0] << " " << right_frustum.end[1] << endl;

	
	float color[3] = { 0.0f, 1.0f, 0.0f };
	LineSeg wall(0, 0, 0, 6);
	Draw_Wall(wall, left_frustum, right_frustum);
	*/
	
	/*  Basic drawing wall using 3f
	for (int i = 0; i < num_edges; i++) {

		glm::vec4 edge_start(edges[i]->endpoints[0]->posn[Y], 1.0f, edges[i]->endpoints[0]->posn[X], 1.0f);
		glm::vec4 edge_end( edges[i]->endpoints[1]->posn[Y], 1.0f, edges[i]->endpoints[1]->posn[X], 1.0f);
		float color[3] = { edges[i]->color[0], edges[i]->color[1], edges[i]->color[2]};
		

		
		if (edges[i]->opaque) {

			//std::cout << glm::to_string(view) << std::endl;

			Draw_Wall(edge_start, edge_end, color);
		}



	}
	*/

	

	


	// std::cout << cells[0]->edges[0]->endpoints[0] << cells[0]->edges[0]->endpoints[1] << std::endl;

	
	

}



//**********************************************************************
//
// * Draws the frustum on the map view of the maze. It is passed the
//   minimum and maximum corners of the window in which to draw.
//======================================================================
void Maze::
Draw_Frustum(int min_x, int min_y, int max_x, int max_y)
//======================================================================
{
	int	  height;
	float   scale_x, scale_y, scale;
	float   view_x, view_y;

	// Draws the view frustum in the map. Sets up all the same viewing
	// parameters as draw().
	scale_x	= ( max_x - min_x - 10 ) / ( max_xp - min_xp );
	scale_y	= ( max_y - min_y - 10 ) / ( max_yp - min_yp );
	scale		= scale_x > scale_y ? scale_y : scale_x;
	height	= (int)ceil(scale * ( max_yp - min_yp ));

	min_x += 5;
	min_y += 5;

	view_x = ( viewer_posn[X] - min_xp ) * scale;
	view_y = ( viewer_posn[Y] - min_yp ) * scale;
	fl_line(min_x + (int)floor(view_x + 
			  cos(To_Radians(viewer_dir+viewer_fov / 2.0)) * scale),
			  min_y + height- 
			  (int)floor(view_y + 
							 sin(To_Radians(viewer_dir+viewer_fov / 2.0)) * 
							 scale),
				min_x + (int)floor(view_x),
				min_y + height - (int)floor(view_y));
	fl_line(min_x + (int)floor(view_x + 
										cos(To_Radians(viewer_dir-viewer_fov / 2.0))	* 
										scale),
				min_y + height- 
				(int)floor(view_y + sin(To_Radians(viewer_dir-viewer_fov / 2.0)) *
				scale),
				min_x + (int)floor(view_x),
				min_y + height - (int)floor(view_y));
	}


//**********************************************************************
//
// * Draws the viewer's cell and its neighbors in the map view of the maze.
//   It is passed the minimum and maximum corners of the window in which
//   to draw.
//======================================================================
void Maze::
Draw_Neighbors(int min_x, int min_y, int max_x, int max_y)
//======================================================================
{
	int	    height;
	float   scale_x, scale_y, scale;
	int	    i, j;

	// Draws the view cell and its neighbors in the map. This works
	// by drawing just the neighbor's edges if there is a neighbor,
	// otherwise drawing the edge. Every edge is shared, so drawing the
	// neighbors' edges also draws the view cell's edges.

	scale_x = ( max_x - min_x - 10 ) / ( max_xp - min_xp );
	scale_y = ( max_y - min_y - 10 ) / ( max_yp - min_yp );
	scale = scale_x > scale_y ? scale_y : scale_x;
	height = (int)ceil(scale * ( max_yp - min_yp ));

	min_x += 5;
	min_y += 5;

	for ( i = 0 ; i < 4 ; i++ )   {
		Cell	*neighbor = view_cell->edges[i]->Neighbor(view_cell);

		if ( neighbor ){
			for ( j = 0 ; j < 4 ; j++ ){
				Edge    *e = neighbor->edges[j];

				if ( e->opaque )	{
					float   x1, y1, x2, y2;

					x1 = e->endpoints[Edge::START]->posn[Vertex::X];
					y1 = e->endpoints[Edge::START]->posn[Vertex::Y];
					x2 = e->endpoints[Edge::END]->posn[Vertex::X];
					y2 = e->endpoints[Edge::END]->posn[Vertex::Y];

					fl_color((unsigned char)floor(e->color[0] * 255.0),
							  (unsigned char)floor(e->color[1] * 255.0),
							  (unsigned char)floor(e->color[2] * 255.0));
					fl_line_style(FL_SOLID);
					fl_line( min_x + (int)floor((x1 - min_xp) * scale),
							 min_y + height - (int)floor((y1 - min_yp) * scale),
							 min_x + (int)floor((x2 - min_xp) * scale),
							 min_y + height - (int)floor((y2 - min_yp) * scale));
				}
			}
		}
		else {
			Edge    *e = view_cell->edges[i];

			if ( e->opaque ){
				float   x1, y1, x2, y2;

				x1 = e->endpoints[Edge::START]->posn[Vertex::X];
				y1 = e->endpoints[Edge::START]->posn[Vertex::Y];
				x2 = e->endpoints[Edge::END]->posn[Vertex::X];
				y2 = e->endpoints[Edge::END]->posn[Vertex::Y];

				fl_color((unsigned char)floor(e->color[0] * 255.0),
							 (unsigned char)floor(e->color[1] * 255.0),
							 (unsigned char)floor(e->color[2] * 255.0));
				fl_line_style(FL_SOLID);
				fl_line(min_x + (int)floor((x1 - min_xp) * scale),
							min_y + height - (int)floor((y1 - min_yp) * scale),
							min_x + (int)floor((x2 - min_xp) * scale),
							min_y + height - (int)floor((y2 - min_yp) * scale));
			 }
		}
	}
}


//**********************************************************************
//
// * Save the maze to a file of the given name.
//======================================================================
bool Maze::
Save(const char *filename)
//======================================================================
{
	FILE    *f = fopen(filename, "w");
	int	    i;

	// Dump everything to a file of the given name. Returns false if it
	// couldn't open the file. True otherwise.

	if ( ! f )  {
		return false;
   }

	fprintf(f, "%d\n", num_vertices);
	for ( i = 0 ; i < num_vertices ; i++ )
		fprintf(f, "%g %g\n", vertices[i]->posn[Vertex::X],
			      vertices[i]->posn[Vertex::Y]);

		fprintf(f, "%d\n", num_edges);
	for ( i = 0 ; i < num_edges ; i++ )
	fprintf(f, "%d %d %d %d %d %g %g %g\n",
				edges[i]->endpoints[Edge::START]->index,
				edges[i]->endpoints[Edge::END]->index,
				edges[i]->neighbors[Edge::LEFT] ?
				edges[i]->neighbors[Edge::LEFT]->index : -1,
				edges[i]->neighbors[Edge::RIGHT] ?
				edges[i]->neighbors[Edge::RIGHT]->index : -1,
				edges[i]->opaque ? 1 : 0,
				edges[i]->color[0], edges[i]->color[1], edges[i]->color[2]);

	fprintf(f, "%d\n", num_cells);
	for ( i = 0 ; i < num_cells ; i++ )
		fprintf(f, "%d %d %d %d\n",
					cells[i]->edges[0] ? cells[i]->edges[0]->index : -1,
					cells[i]->edges[1] ? cells[i]->edges[1]->index : -1,
					cells[i]->edges[2] ? cells[i]->edges[2]->index : -1,
					cells[i]->edges[3] ? cells[i]->edges[3]->index : -1);

	   fprintf(f, "%g %g %g %g %g\n",
					viewer_posn[X], viewer_posn[Y], viewer_posn[Z],
					viewer_dir, viewer_fov);

	fclose(f);

	return true;
}
