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

#include <QtOpenGL/QGLWidget>
#include <QKeyEvent>
#include <QtOpenGL/QtOpenGL>
#include <iostream>
#include <cmath>
#include <glu.h>
#include "dcel/DCEL.hh" 

#define THETA M_PI/36.0

enum Drawmode { CGPOINTS, WIRE, FLAT, SMOOTH };
// CGPOINTS     draw vertices
// WIRE         draw half-edges
// FLAT         draw facets (flat shaded);
// SMOOTH       draw facets (smooth shaded);


class Window_gl : public QGLWidget
{

	Q_OBJECT

	public:

		Window_gl(QWidget *parent = 0);



	public slots:

		void add_dcel(QVector<DCEL>& dc);
		void reset( void );

		void set_p_drawmode( void );
		void set_w_drawmode( void );
		void set_f_drawmode( void );
		void set_s_drawmode( void );



	protected:

		void initializeGL(void);		// init GL environment (lights, shading, clipping planes, ...)
		void resizeGL(int w, int h);	// handle canvas' resizing
		void paintGL(void);		// render DCELs

		void keyPressEvent( QKeyEvent *event );
		void mousePressEvent( QMouseEvent *event );



	private:

		Drawmode style;

		CGPointf eye,	 // camera position
			 center, // point observed by the camera (tipically the DCEL's centroid)
			 up;	 // up direction

		QVector<DCEL> mesh;

		double diagonal; // bbox diagonal

		QVector<CGPointf> centroid; // DCEL centroids

		float cos_theta;
		float sin_theta;

		int last; // pointer to the last DCEL in the vector
};
