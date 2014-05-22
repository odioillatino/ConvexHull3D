/*
 *  Convex Hull 3D
 *  Copyright 2013 Simone Barbieri 
 * 
 *  This file is part of Convex Hull 3D.
 *
 *  Convex Hull 3D is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Convex Hull 3D is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Convex Hull 3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "engine.h"
#include <assert.h>
#include "timer.h"

Engine::Engine(QObject *parent) : QObject(parent)
{
	reset();
}


void Engine::reset(void)
{
	state = INPUT;
	meshes.clear();
}


void Engine::calculate_ch(void)
{
	Timer timer("3D Convex Hull");
    
    srand (time(NULL));
    
    DCEL convex_hull;
    
    // Get all the vertex from the mesh
    QVector<vertex> vertex_list = meshes[0].get_all_vertexes();
    
    // Shuffle the vertexes in the list
    random_shuffle(vertex_list.begin(), vertex_list.end());
    
    // Find the first four points to obtain a tethraedron
    int vert0, vert1, vert2, vert3;
    int v0, v1, v2, v3;
    vertex vertex0, vertex1, vertex2, vertex3;
    
    // Get the first point
    vert0 = rand() % vertex_list.size();
    v0 = convex_hull.add_vertex(vertex_list[vert0]);
    vertex0 = convex_hull.get_vertex(v0);
    swap_vertex(vertex_list, 0, vert0);
    
    // Get the second point
    do {
        
        vert1 = rand() % vertex_list.size();
    } while (vert0 == vert1);
    
    v1 = convex_hull.add_vertex(vertex_list[vert1]);
    vertex1 = convex_hull.get_vertex(v1);
    swap_vertex(vertex_list, 1, vert1);
    
    // Get the third point
    while (convex_hull.get_vertex_n() < 3) {
        
        do {
            
            vert2 = rand() % vertex_list.size();
        } while (vert0 == vert2 || vert1 == vert2);
        vertex2 = vertex_list[vert2];
                
        // Check if the three point are colinear
        double colinearity = vertex0.get_coord().getX() * (vertex1.get_coord().getY() - vertex2.get_coord().getY()) +
                          vertex1.get_coord().getX() * (vertex2.get_coord().getY() - vertex0.get_coord().getY()) +
                          vertex2.get_coord().getX() * (vertex0.get_coord().getY() - vertex1.get_coord().getY());
                
        // If they aren't colienar, then add the point to the convex hull, else choose another one
        if (colinearity != 0) {
            
            v2 = convex_hull.add_vertex(vertex2);
            swap_vertex(vertex_list, 2, vert2);
        }
    }
    
    double determinant;
    
    // Get the fourth point
    while (convex_hull.get_vertex_n() < 4) {
        
        do {
            
            vert3 = rand() % vertex_list.size();
        } while (vert0 == vert3 || vert1 == vert3 || vert2 == vert3);
        vertex3 = vertex_list[vert3];
        
        // Check if the four points are coplanar
        QMatrix4x4 coplanarity(vertex0.get_coord().getX(), vertex0.get_coord().getY(), vertex0.get_coord().getZ(), 1,
                               vertex1.get_coord().getX(), vertex1.get_coord().getY(), vertex1.get_coord().getZ(), 1,
                               vertex2.get_coord().getX(), vertex2.get_coord().getY(), vertex2.get_coord().getZ(), 1,
                               vertex3.get_coord().getX(), vertex3.get_coord().getY(), vertex3.get_coord().getZ(), 1);
        
        determinant = coplanarity.determinant();
        
        if (determinant != 0) {
            
            v3 = convex_hull.add_vertex(vertex3);
            swap_vertex(vertex_list, 3, vert3);
        }
    }
    
    // If the determinant is positive, the fourh point see the face in counterclockwise sense, and the face from outside would be in clockwise sense.
    // So, if the determinant is positive, the tethraedron will be constructed with the points in the reverse order.
    if (determinant > 0) {
        
        create_tethraedron(convex_hull, v0, v2, v1, v3);
    } else {
        
        create_tethraedron(convex_hull, v0, v1, v2, v3);
    }
        
    conflict_graph conf_graph;
    
    // Add face nodes in the conflict graph for the tethraedron faces
    for (int i = 0; i < convex_hull.get_faces_n(); i++) {
        
        conf_graph.add_face_node(i);
    }
    
    // Check if each face of the tethraedron is visible from each vertex in the list
    for (int i = STARTING_POINTS; i < vertex_list.size(); i++) {
        
        // Add a node in the conflict graph
        conf_graph.add_vertex_node(i);
        
        for (int j = 0; j < convex_hull.get_faces_n(); j++) {
            
            // If the face is visible from the vertex, create an arch in the conflict grapf between them
            if (is_face_visible(convex_hull, j, vertex_list[i])) {
                
                conf_graph.create_arch(j, i);
            }
        }
    }
    
    // For each point, check visible faces and replace them with new faces
    for (int i = STARTING_POINTS; i < vertex_list.size(); i++) {
        
        // Obtain the list of visible faces from the vertex i
        QVector<int> visible_faces = conf_graph.get_visible_faces(i);
        
        // If there are visible faces, then the vertex is outside the convex hull
        if (visible_faces.size() > 0) {
            
            QVector<int> horizon_edges;
            QVector<int> edges_to_set_twin;
            QVector<int> vertex_to_check;
            QVector<int> horizon_vertexes;
            
            // Add the vertex to convex hull
            int new_vertex = convex_hull.add_vertex(vertex_list[i]);
            
            // Find horizon edges
            check_horizon_edges(convex_hull, horizon_edges, horizon_vertexes, vertex_to_check, visible_faces);
            
            // Find vertexes to hide
            check_vertex_to_hide(convex_hull, vertex_to_check, horizon_vertexes);
            
            // For each horizon edge, create a new face and check visibility
            for (int j = 0; j < horizon_edges.size(); j++) {
                
                int horizon_edge = horizon_edges[j];
                half_edge real_horizon_edge = convex_hull.get_half_edge(horizon_edge);
                
                int old_face = real_horizon_edge.get_face();
                int old_face_twin = convex_hull.get_half_edge(real_horizon_edge.get_twin()).get_face();
                
                QVector<int> visible_vertexes = conf_graph.get_visible_vertexes(old_face);
                QVector<int> visible_vertexes_help = conf_graph.get_visible_vertexes(old_face_twin);
                
                // Find the vertexes which could see the face incident to the horizon edge and the one incident on its twin
                for (int k = 0; k < visible_vertexes_help.size(); k++) {
                    
                    if (!visible_vertexes.contains(visible_vertexes_help[k])) {
                        
                        visible_vertexes.append(visible_vertexes_help[k]);
                    }
                }
                
                // Create two new half-edges
                int new_half_edge_1 = create_half_edge(convex_hull, real_horizon_edge.to_vertex, new_vertex);
                int new_half_edge_2 = create_half_edge(convex_hull, new_vertex, real_horizon_edge.from_vertex);
                
                edges_to_set_twin.append(new_half_edge_1);
                edges_to_set_twin.append(new_half_edge_2);
                
                // Set half-edges nexts
                convex_hull.set_half_edge_next(horizon_edge, new_half_edge_1);
                convex_hull.set_half_edge_next(new_half_edge_1, new_half_edge_2);
                convex_hull.set_half_edge_next(new_half_edge_2, horizon_edge);
                
                // Create the new face
                int new_face = create_face(convex_hull, horizon_edge);
                
                // Set the face visible in the convex hull
                convex_hull.set_face_in_convex_hull(new_face, true);
                
                // Set half-edges' incident face
                convex_hull.set_half_edge_face(horizon_edge, new_face);
                convex_hull.set_half_edge_face(new_half_edge_1, new_face);
                convex_hull.set_half_edge_face(new_half_edge_2, new_face);
                
                // Create a new node in the conflict graph for the new face
                conf_graph.add_face_node(new_face);
                
                // Check the visibility of the new faces with the vertexes that we found before
                for (int k = 0; k < visible_vertexes.size(); k++) {
                    
                    if (is_face_visible(convex_hull, new_face, vertex_list[visible_vertexes[k]])) {
                        
                        conf_graph.create_arch(new_face, visible_vertexes[k]);
                    }
                }
            }
            
            // Check the new edges to set the correct twins
            set_correct_twin(convex_hull, edges_to_set_twin);
            
            // Remove the old faces from the conflict graph
            for (int j = 0; j < visible_faces.size(); j++) {
                
                conf_graph.remove_face_node(visible_faces[j]);
            }
            
            // Remove the new vertex from the conflict graph
            conf_graph.remove_vertex_node(new_vertex);
        }
    }
        
    meshes.push_back(convex_hull);
    send_dcel(meshes);

	timer.stop_and_print();
}

// Swap two vertex in the vertex list
void Engine::swap_vertex(QVector<vertex> &vertex_list, int vertex1, int vertex2)
{
    vertex tmp;
    
    tmp = vertex_list[vertex1];
    vertex_list[vertex1] = vertex_list[vertex2];
    vertex_list[vertex2] = tmp;
}

// Declare a new half-edge, set its next and from vertexes and insert it into the convex hull. Return the position of the half-edge in the convex hull.
int Engine::create_half_edge(DCEL &convex_hull, int from_vertex, int to_vertex)
{
    half_edge new_half_edge;
    new_half_edge.set_from_vertex(from_vertex);
    new_half_edge.set_to_vertex(to_vertex);
    
    return convex_hull.add_half_edge(new_half_edge);
}

// Declare a new face, set its inner edge and insert it into the convex hull. Return the position of the face in the convex hull.
int Engine::create_face(DCEL &convex_hull, int inner_edge)
{
    face new_face;
    new_face.set_inner_half_edge(inner_edge);
    
    return convex_hull.add_face(new_face);
}

// Create the tethraedron in the convex hull DCEL.
void Engine::create_tethraedron(DCEL &convex_hull, int v0, int v1, int v2, int v3)
{
    // Create edges and set their twins
    int h0 = create_half_edge(convex_hull, v2, v1);
    int h1 = create_half_edge(convex_hull, v1, v2);
    convex_hull.set_half_edge_twin(h0, h1);
    convex_hull.set_half_edge_twin(h1, h0);
    
    int h2 = create_half_edge(convex_hull, v2, v0);
    int h3 = create_half_edge(convex_hull, v0, v2);
    convex_hull.set_half_edge_twin(h2, h3);
    convex_hull.set_half_edge_twin(h3, h2);
    
    int h4 = create_half_edge(convex_hull, v2, v3);
    int h5 = create_half_edge(convex_hull, v3, v2);
    convex_hull.set_half_edge_twin(h4, h5);
    convex_hull.set_half_edge_twin(h5, h4);
    
    int h6 = create_half_edge(convex_hull, v1, v0);
    int h7 = create_half_edge(convex_hull, v0, v1);
    convex_hull.set_half_edge_twin(h6, h7);
    convex_hull.set_half_edge_twin(h7, h6);
    
    int h8 = create_half_edge(convex_hull, v1, v3);
    int h9 = create_half_edge(convex_hull, v3, v1);
    convex_hull.set_half_edge_twin(h8, h9);
    convex_hull.set_half_edge_twin(h9, h8);
    
    int h10 = create_half_edge(convex_hull, v0, v3);
    int h11 = create_half_edge(convex_hull, v3, v0);
    convex_hull.set_half_edge_twin(h10, h11);
    convex_hull.set_half_edge_twin(h11, h10);
    
    // Set half-edges' nexts
    convex_hull.set_half_edge_next(h0, h8);
    convex_hull.set_half_edge_next(h8, h5);
    convex_hull.set_half_edge_next(h5, h0);
    
    convex_hull.set_half_edge_next(h1, h2);
    convex_hull.set_half_edge_next(h2, h7);
    convex_hull.set_half_edge_next(h7, h1);
    
    convex_hull.set_half_edge_next(h6, h10);
    convex_hull.set_half_edge_next(h10, h9);
    convex_hull.set_half_edge_next(h9, h6);
    
    convex_hull.set_half_edge_next(h4, h11);
    convex_hull.set_half_edge_next(h11, h3);
    convex_hull.set_half_edge_next(h3, h4);
    
    // Create faces
    int f0 = create_face(convex_hull, h0);
    int f1 = create_face(convex_hull, h1);
    int f2 = create_face(convex_hull, h6);
    int f3 = create_face(convex_hull, h4);

    convex_hull.set_face_in_convex_hull(f0, true);
    convex_hull.set_face_in_convex_hull(f1, true);
    convex_hull.set_face_in_convex_hull(f2, true);
    convex_hull.set_face_in_convex_hull(f3, true);
    
    // Set half-edges' incident faces
    convex_hull.set_half_edge_face(h0, f0);
    convex_hull.set_half_edge_face(h1, f1);
    convex_hull.set_half_edge_face(h2, f1);
    convex_hull.set_half_edge_face(h3, f3);
    convex_hull.set_half_edge_face(h4, f3);
    convex_hull.set_half_edge_face(h5, f0);
    convex_hull.set_half_edge_face(h6, f2);
    convex_hull.set_half_edge_face(h7, f1);
    convex_hull.set_half_edge_face(h8, f0);
    convex_hull.set_half_edge_face(h9, f2);
    convex_hull.set_half_edge_face(h10, f2);
    convex_hull.set_half_edge_face(h11, f3);
}

// Check if a face is visible from the vertex "point".
bool Engine::is_face_visible(DCEL &convex_hull, int face_id, vertex &point)
{
    face face_to_check = convex_hull.get_face(face_id);
    
    vertex vertex0, vertex1, vertex2;
    
    // Get the three vertex of the face
    int h0 = face_to_check.get_inner_half_edge();
    half_edge real_h0 = convex_hull.get_half_edge(h0);
    vertex0 = convex_hull.get_vertex(real_h0.get_from_vertex());
    
    int h1 = real_h0.get_next();
    half_edge real_h1 = convex_hull.get_half_edge(h1);
    vertex1 = convex_hull.get_vertex(real_h1.get_from_vertex());
    
    int h2 = real_h1.get_next();
    half_edge real_h2 = convex_hull.get_half_edge(h2);
    vertex2 = convex_hull.get_vertex(real_h2.get_from_vertex());
    
    QMatrix4x4 visibility(vertex0.get_coord().getX(), vertex0.get_coord().getY(), vertex0.get_coord().getZ(), 1,
                          vertex1.get_coord().getX(), vertex1.get_coord().getY(), vertex1.get_coord().getZ(), 1,
                          vertex2.get_coord().getX(), vertex2.get_coord().getY(), vertex2.get_coord().getZ(), 1,
                          point.get_coord().getX(), point.get_coord().getY(), point.get_coord().getZ(), 1);
    
    // Compute the determinant
    double determinant = visibility.determinant();
    
    // If the determinant is positive, the face is visible
    if (determinant > 0) {
        
        return true;
    }
    
    return false;
}

// Analyse all the visible faces to find horizon edges
void Engine::check_horizon_edges(DCEL &convex_hull, QVector<int> &horizon_edges, QVector<int> &horizon_vertixes, QVector<int> &vertexes_to_check, QVector<int> &visible_faces)
{
    // For each visible face, analyse its half-edges to find horizon edges
    for (int i = 0; i < visible_faces.size(); i++) {
        
        face face_to_check = convex_hull.get_face(visible_faces[i]);
        
        // Get the half-edges of the face
        int h0 = face_to_check.get_inner_half_edge();
        half_edge real_h0 = convex_hull.get_half_edge(h0);
        
        int h1 = real_h0.get_next();
        half_edge real_h1 = convex_hull.get_half_edge(h1);
        
        int h2 = real_h1.get_next();
        half_edge real_h2 = convex_hull.get_half_edge(h2);
        
        // Get the twins of the face's half-edges
        half_edge h0_twin = convex_hull.get_half_edge(real_h0.get_twin());
        half_edge h1_twin = convex_hull.get_half_edge(real_h1.get_twin());
        half_edge h2_twin = convex_hull.get_half_edge(real_h2.get_twin());
        
        // Appends all face's vertexes to a list
        vertexes_to_check.append(real_h0.get_from_vertex());
        vertexes_to_check.append(real_h1.get_from_vertex());
        vertexes_to_check.append(real_h2.get_from_vertex());
        
        // For each half-edges' twin, check if the incident face is in the list of the visible faces.
        // If the face isn't in the list, then the half-edge is an horizon edge, and appends the vertexes to a list. Else, the edges has to be hidden.
        if (!visible_faces.contains(h0_twin.get_face())) {
            
            horizon_edges.append(h0);
            
            if (!horizon_vertixes.contains(real_h0.get_from_vertex())) {
                
                horizon_vertixes.append(real_h0.get_from_vertex());
            }
            
            if (!horizon_vertixes.contains(real_h0.get_to_vertex())) {
                
                horizon_vertixes.append(real_h0.get_to_vertex());
            }
        } else {
            
            convex_hull.set_half_edge_in_convex_hull(h0, false);
        }
        
        if (!visible_faces.contains(h1_twin.get_face())) {
            
            horizon_edges.append(h1);
            
            if (!horizon_vertixes.contains(real_h1.get_from_vertex())) {
                
                horizon_vertixes.append(real_h1.get_from_vertex());
            }
            
            if (!horizon_vertixes.contains(real_h1.get_to_vertex())) {
                
                horizon_vertixes.append(real_h1.get_to_vertex());
            }
        } else {
            
            convex_hull.set_half_edge_in_convex_hull(h1, false);
        }
        
        
        if (!visible_faces.contains(h2_twin.get_face())) {
            
            horizon_edges.append(h2);
            
            if (!horizon_vertixes.contains(real_h2.get_from_vertex())) {
                
                horizon_vertixes.append(real_h2.get_from_vertex());
            }
            
            if (!horizon_vertixes.contains(real_h2.get_to_vertex())) {
                
                horizon_vertixes.append(real_h2.get_to_vertex());
            }
        } else {
            
            convex_hull.set_half_edge_in_convex_hull(h2, false);
        }
        
        // Hide the face from convex hull
        convex_hull.set_face_in_convex_hull(visible_faces[i], false);
    }
}

// Check visible faces' vertexes to find out the ones to be deleted.
void Engine::check_vertex_to_hide(DCEL &convex_hull, QVector<int> &vertex_to_check, QVector<int> &horizon_vertexes)
{
    // For each point to check
    for (int i = 0; i < vertex_to_check.size(); i++) {
        
        // If the point isn't in the horizon, then hide it
        if (!horizon_vertexes.contains(vertex_to_check[i])) {
            
            convex_hull.set_vertex_in_convex_hull(vertex_to_check[i], false);
        }
    }
}

// For each edge, check the others to find his twin. Then, remove both from list.
void Engine::set_correct_twin(DCEL &convex_hull, QVector<int> &half_edge_list)
{
    while (half_edge_list.size() != 0) {
        
        // Get the first edge
        half_edge real_half_edge = convex_hull.get_half_edge(half_edge_list[0]);
        
        // Check the other vertex
        for (int i = 1; i < half_edge_list.size(); i++) {
            
            half_edge half_edge_to_check = convex_hull.get_half_edge(half_edge_list[i]);
            
            // If they have from and to vertexes inverted, then they are twins
            if (real_half_edge.get_from_vertex() == half_edge_to_check.get_to_vertex() &&
                real_half_edge.get_to_vertex() == half_edge_to_check.get_from_vertex()) {
                
                // Set them as twins
                convex_hull.set_half_edge_twin(half_edge_list[0], half_edge_list[i]);
                convex_hull.set_half_edge_twin(half_edge_list[i], half_edge_list[0]);
                
                // Remove them from list
                half_edge_list.remove(i);
                half_edge_list.remove(0);
                
                break;
            }
        }
    }
}


// Let the open file dialog show up and call the OFF loader
//
void Engine::open_file(void)
{
	QString filename = QFileDialog::getOpenFileName( NULL, "Open mesh", "~", "3D meshes (*.off)");

	if (!filename.isNull())
	{
		if(!create_from_file(filename)) return;
		state = COMPUTED;
	}
}


// Save a DCEL object to a .off file. This may be useful if you want to store
// your convex hulls into the file system...
//
void Engine::save_file(void)
{
	QFileDialog saveDialog( NULL, "Save mesh", "~", "3D meshes (*.off)");
	saveDialog.setDefaultSuffix("off");
	saveDialog.setAcceptMode(QFileDialog::AcceptSave);
	saveDialog.setConfirmOverwrite(true);
	saveDialog.setFileMode(QFileDialog::AnyFile);
	
	if (saveDialog.exec())
	{
		QString filename = saveDialog.selectedFiles().takeFirst();
	
		QFile file(filename);

		if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			std::cout << "Error saving file " << file.error() << std::endl;
			return;
		}

		QTextStream out(&file);
		
		out << "OFF\n";
		
		int last = meshes.size() - 1;
		int nv = meshes[last].get_vertex_n(), nf = meshes[last].get_faces_n();
		out << nv << " " << nf << " 0\n";

		for(int i=0; i<nv; i++)
		{
			CGPointf v = meshes[last].get_vertex(i).get_coord();
			out << v.x << " " << v.y << " " << v.z << "\n";
		}

		for(int i=0; i<nf; i++)
		{
			int v1, v2, v3, h;
			h = meshes[last].get_face(i).get_inner_half_edge();
			v1 = meshes[last].get_half_edge(h).get_from_vertex();
			v2 = meshes[last].get_half_edge(h).get_to_vertex();
			v3 = meshes[last].get_half_edge(meshes[last].get_half_edge(h).get_next()).get_to_vertex();
			out << "3 " << v1 << " " << v2 << " " << v3 << "\n";
		}
	}
}


// Takes in input a .off file and fills the DCEL up
//
bool Engine::create_from_file(QString filename)
{
	int nv, // number of vertices
	    nh, // number of half-edges
	    nf; // number of facets

	DCEL tmp_d;	
	meshes.push_back(tmp_d);
	int last = meshes.size() - 1;

	std::cout << meshes.size() << " models loaded" << std::endl;
	
	QFile file(filename);
	if (!file.open(QIODevice::ReadWrite | QIODevice::Text)) return false;

	QByteArray line = file.readLine();
	char *tmp = line.data();
	char t[5];
	
	//
	// check the .off header
	//

	int res = sscanf(tmp, "%s", t);
	if (res != 1 || strcmp(t, "OFF"))
	{
		std::cout << res << " element read, first is " << t << std::endl;
		file.close();
		return false;
	}
	line = file.readLine();
	tmp = line.data();
	res = sscanf(tmp, "%d %d %d", &nv, &nf, &nh); //ignorare nh
	if( res < 2 )
	{
		std::cout << "ERROR IN SECOND LINE (expected nv nf [ne])" << std::endl;
		file.close();
		return false;
	}

	//
	// load vertices
	//

	for( int i=0; i < nv; i++ )
	{
		float x, y, z;

		line = file.readLine();
		char *tmp = line.data();
		res = sscanf(tmp, "%f %f %f", &x, &y, &z);
		
		if( res != 3 )
		{
			std::cout << "Error in vertex " << i << ": " << res << " elements read, first is " << x << std::endl;
			file.close();
			return false;
		}

		vertex v;
		v.set_coord( CGPointf( x, y, z ) );
		v.set_incident_half_edge( -1 );

		meshes[last].add_vertex( v );
	}

	QVector<QVector<QPair<int, int> > > record_table(nv);

	//
	// load facets
	//

	for( int i=0; i < nf; i++ )
	{
		int v1, v2, v3, nvf;
		line = file.readLine();
		char *tmp = line.data();
		res = sscanf(tmp, "%d %d %d %d", &nvf, &v1, &v2, &v3);
	
		if( res != 4 || nvf != 3 )
		{
			std::cout << "Error in face " << i << ": " << res << " elements read, first is " << nvf << std::endl;
			file.close();
			return false;
		}
		face f;

		//
		// make connectivity
		//

		half_edge he1, he2, he3; //he1 da v1, he2 da v2, he3 da v3
		int he1_id, he2_id, he3_id;		
		he1.set_from_vertex( v1 );
		he1.set_to_vertex( v2 );
		he2.set_from_vertex( v2 );
		he2.set_to_vertex( v3 );
		he3.set_from_vertex( v3 );

		he1_id = meshes[last].add_half_edge( he1 );
		he2_id = meshes[last].add_half_edge( he2 );
		he3_id = meshes[last].add_half_edge( he3 );

		meshes[last].set_half_edge_prev( he1_id, he3_id );
		meshes[last].set_half_edge_prev( he3_id, he2_id );
		meshes[last].set_half_edge_prev( he2_id, he1_id );
		meshes[last].set_half_edge_next( he1_id, he2_id );
		meshes[last].set_half_edge_next( he2_id, he3_id );
		meshes[last].set_half_edge_next( he3_id, he1_id );

		QVector<QPair<int,int> > list = record_table.at(v2);

		int position = -1;
		for(int seek=0; seek<list.size(); seek++)
		{
			QPair<int,int> record = list.at(seek);
			if( record.first == v1 ) //v2->v1 messo
			{
				position = seek;
				seek = list.size()+1;				
			}
		}

		if( position != -1 )
		{
			//recupera record
			QPair<int,int> record = list.at(position);
			int twin_id = record.second;
			//setta twin (v2->v1)
			meshes[last].set_half_edge_twin( twin_id, he1_id );
			meshes[last].set_half_edge_twin( he1_id, twin_id );
			//rimuovi elemento
			list.remove(position);
			record_table.replace(v2, list);
		}
		else
		{
			//list diventa la lista per v1 (aggiungiamo v1->v2)
			list = record_table.at(v1);
			//creiamo il record v1->v2
			QPair<int,int> record( v2, he1_id );
			list.push_back(record);
			record_table.replace(v1, list);
		}

		//PER V2->V3
		list = record_table.at(v3);

		//cercare v2 in list
		position = -1;
		for(int seek=0; seek<list.size(); seek++)
		{
			QPair<int,int> record = list.at(seek);
			if( record.first == v2 ) //v3->v2 messo
			{
				position = seek;
				seek = list.size()+1;				
			}
		}

		if( position != -1 )
		{
			//recupera record
			QPair<int,int> record = list.at(position);
			int twin_id = record.second;
			//setta twin (v3->v2)
			meshes[last].set_half_edge_twin( twin_id, he2_id );
			meshes[last].set_half_edge_twin( he2_id, twin_id );
			//rimuovi elemento
			list.remove(position);
			record_table.replace(v3, list);
		}
		else
		{
			//list diventa la lista per v2 (aggiungiamo v2->v3)
			list = record_table.at(v2);
			//creiamo il record v2->v3
			QPair<int,int> record( v3, he2_id );
			list.push_back(record);
			record_table.replace(v2, list);
		}

		//PER V3->V1
		list = record_table.at(v1);

		//cercare v3 in list
		position = -1;
		for(int seek=0; seek<list.size(); seek++)
		{
			QPair<int,int> record = list.at(seek);
			if( record.first == v3 ) //v1->v3 messo
			{
				position = seek;
				seek = list.size()+1;				
			}
		}

		if( position != -1 )
		{
			//recupera record
			QPair<int,int> record = list.at(position);
			int twin_id = record.second;
			//setta twin (v1->v3)
			meshes[last].set_half_edge_twin( twin_id, he3_id );
			meshes[last].set_half_edge_twin( he3_id, twin_id );
			//rimuovi elemento
			list.remove(position);
			record_table.replace(v1, list);
		}
		else
		{
			//list diventa la lista per v3 (aggiungiamo v3->v1)
			list = record_table.at(v3);
			//creiamo il record v3->v1
			QPair<int,int> record( v1, he3_id );
			list.push_back(record);
			record_table.replace(v3, list);
		}

		if( meshes[last].get_vertex( v1 ).get_incident_half_edge() == -1 ) meshes[last].set_vertex_incident( v1, he1_id );
		if( meshes[last].get_vertex( v2 ).get_incident_half_edge() == -1 ) meshes[last].set_vertex_incident( v2, he2_id );
		if( meshes[last].get_vertex( v3 ).get_incident_half_edge() == -1 ) meshes[last].set_vertex_incident( v3, he3_id );

		//
		// compute face normals
		//

		CGPointf vec1, vec2, norm;
		vec1 = meshes[last].get_vertex(v2).get_coord() - meshes[last].get_vertex(v1).get_coord();
		vec2 = meshes[last].get_vertex(v3).get_coord() - meshes[last].get_vertex(v1).get_coord();
		norm = cross(vec1, vec2);
		float norma = sqrt(pow(norm.x,2)+pow(norm.y,2)+pow(norm.z,2));
		norm.x /= norma;
		norm.y /= norma;
		norm.z /= norma;

		f.set_normal(norm);

		f.set_inner_half_edge( he1_id );
		int face_id = meshes[last].add_face( f );

		meshes[last].set_half_edge_face( he1_id, face_id );
		meshes[last].set_half_edge_face( he2_id, face_id );
		meshes[last].set_half_edge_face( he3_id, face_id );
	}
	file.close();

	//
	// compute vertex normals
	//

	for(int i=0; i<nv && meshes[last].get_faces_n(); i++)
	{
		vertex v_i = meshes[last].get_vertex(i);
		CGPointf normal_i;
		int start_he_id = v_i.get_incident_half_edge();
		int circulator = start_he_id;
		int faces_n = 0;
		do
		{
			half_edge circ_h = meshes[last].get_half_edge(circulator);
			face f = meshes[last].get_face( circ_h.get_face() );
			normal_i = normal_i + f.get_normal();
			faces_n ++;

			circulator = meshes[last].get_half_edge( circ_h.get_twin() ).get_next();

		}
		while (circulator != start_he_id);

		normal_i = normal_i / (float)faces_n;
		float normal_norm = sqrt(pow(normal_i.x,2)+pow(normal_i.y,2)+pow(normal_i.z,2));
		normal_i = normal_i / normal_norm;
		meshes[last].set_vertex_normal( i, normal_i );
	}

	//
	// update GUI
	//

	emit send_dcel(meshes);

    return true;
}
