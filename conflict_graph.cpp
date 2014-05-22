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

#include "conflict_graph.h"

conflict_graph::conflict_graph()
{
    conflict_node null_node(-1);
    
    // Fill f_conflict with four null nodes. In this way, we can access directly f_conflict with the id of the vertex in the convex hull.
    f_conflict.fill(null_node, 4);
}

// Add a face node to the conflict graph.
void conflict_graph::add_face_node(int face_id)
{
    conflict_node new_node(face_id);
    
    p_conflict.append(new_node);
}

// Add a vertex node to the conflict graph.
void conflict_graph::add_vertex_node(int vertex_id)
{
    conflict_node new_node(vertex_id);
    
    f_conflict.append(new_node);
}

// Create an arch between a face node and a vertex node.
void conflict_graph::create_arch(int face_id, int vertex_id)
{
    f_conflict[vertex_id].add_arch(face_id);
    p_conflict[face_id].add_arch(vertex_id);
}

// Get all the faces visible from the "vertex_id" vertex.
QVector<int> conflict_graph::get_visible_faces(int vertex_id)
{
    return f_conflict[vertex_id].get_arch_list();
}

// Get all the vertexes which can see the "face_id" face.
QVector<int> conflict_graph::get_visible_vertexes(int face_id)
{
    return p_conflict[face_id].get_arch_list();
}

// Delete all arches with the face to delete.
void conflict_graph::remove_face_node(int face_id)
{
    QVector<int> vertex_list = p_conflict[face_id].get_arch_list();
    
    for (int i = 0; i < vertex_list.size(); i++) {
        
        f_conflict[vertex_list[i]].remove_arch(face_id);
    }
}

// Delete all arches with the vertex to delete.
void conflict_graph::remove_vertex_node(int vertex_id)
{
    QVector<int> face_list = f_conflict[vertex_id].get_arch_list();
    
    for (int i = 0; i < face_list.size(); i++) {
        
        p_conflict[face_list[i]].remove_arch(vertex_id);
    }
}
