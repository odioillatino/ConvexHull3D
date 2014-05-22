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

#include "conflict_node.h"

conflict_node::conflict_node()
{
}

conflict_node::conflict_node(int node_id)
{
    // Set the id of the node
    this->node_id = node_id;
}

// Add an arch with a certain node with "node_id" id.
void conflict_node::add_arch(int node_id)
{
    if (!arch_list.contains(node_id)) {
        
        arch_list.append(node_id);
    }
}

QVector<int> conflict_node::get_arch_list()
{
    return arch_list;
}

// Remove an arch between two nodes.
void conflict_node::remove_arch(int node_id)
{
    int position = arch_list.indexOf(node_id);
    
    if (position != -1) {
        
        arch_list.remove(position);
    }
}
