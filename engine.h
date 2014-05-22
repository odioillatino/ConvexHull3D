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

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>

#include <QtGui>
#include <QFile>
#include <QFileDialog>
#include <QByteArray>

#include "dcel/DCEL.hh"
#include "conflict_graph.h"

#define STARTING_POINTS 4

enum State { INPUT, COMPUTED };

class Engine : public QObject
{

	Q_OBJECT


 	public:

                Engine(QObject *parent = 0);



	signals:

		void send_dcel(QVector<DCEL>& dc); // update the GUI



	public slots:

		void reset(void);
		void open_file(void);
		void save_file(void);
		void calculate_ch(void);		// <= you are expected to fill this method in with
						//    the 3D Convex Hull Algorithm.
						//
						//    Every new variable/method/class is welcome as
						//    long as it is useful for the completion of the
						//    exercise.
						//
						//    Try to keep the logical structure of the program
						//    clean (if you need a data structure create a separate
						//    class; if you want to add a functionality to this
						//    class think if that should be either public or private...)


	private:

        bool create_from_file(QString filename);
        
        void swap_vertex(QVector<vertex> &vertex_list, int vertex1, int vertex2);
        
        int create_half_edge(DCEL &convex_hull, int from_vertex, int to_vertex);
        int create_face(DCEL &convex_hull, int inner_edge);
        void create_tethraedron(DCEL &convex_hull, int v0, int v1, int v2, int v3);
        
        bool is_face_visible(DCEL &convex_hull, int face_id, vertex &point);
        
        void check_horizon_edges(DCEL &convex_hull, QVector<int> &horizon_edges, QVector<int> &horizon_vertixes, QVector<int> &vertexes_to_check, QVector<int> &visible_faces);
        void check_vertex_to_hide(DCEL &convex_hull, QVector<int> &vertex_to_check, QVector<int> &horizon_vertexes);
        
        void set_correct_twin(DCEL &convex_hull, QVector<int> &half_edge_list);
        
		State state;
		QVector<DCEL> meshes;
};
