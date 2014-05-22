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

#ifndef CONFLICT_GRAPH_H
#define CONFLICT_GRAPH_H

#include "conflict_node.h"

class conflict_graph
{
    public:
        
        conflict_graph();
        
        void add_face_node(int face_id);
        void add_vertex_node(int vertex_id);
        
        void create_arch(int face_id, int vertex_id);
        
        QVector<int> get_visible_faces(int vertex_id);
        QVector<int> get_visible_vertexes(int face_id);
        
        void remove_face_node(int face_id);
        void remove_vertex_node(int vertex_id);
        
    private:
        
        QVector<conflict_node> f_conflict;
        QVector<conflict_node> p_conflict;
};

#endif // CONFLICT_GRAPH_H
