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

#ifndef CONFLICT_NODE_H
#define CONFLICT_NODE_H

#include <QVector>

class conflict_node
{
    public:
        
        conflict_node();
        conflict_node(int node_id);
        
        void add_arch(int node_id);
        QVector<int> get_arch_list();
        void remove_arch(int node_id);
        
    private:
        
        int node_id;
        QVector<int> arch_list;
};

#endif // CONFLICT_NODE_H
