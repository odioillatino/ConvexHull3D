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

/**
 * \mainpage DCEL
 *
 * In questa sezione viene presentata una implementazione semplice
 * ma comunque funzionale di una \b double-connected-edge-list (più semplicemente, DCEL).\n
 * La DCEL è una particolare struttura dati, qui modellata mediante l'apposita classe \link DCEL \endlink,
 * che si occupa di implementare e gestire gli elementi necessari a modellare una mesh
 * bidimensionale, introdotti nella sezione \link strutture_dati Strutture dati \endlink. Il compito
 * della classe DCEL è quello di mettere insieme tutti i componenti, modellarne le relazioni
 * di adiacenza e topologia sulla base di quanto specificato dall'utente, rendere disponibili,
 * quando necessario, le informazioni relative a un singolo componente (sia esso un vertice,
 * un half-edge o una faccia) e consentirne la modifica ed eventualmente la rimozione.\n
 * Il modello di ispirazione è quello classico, presentato anche in \e Computational \e Geometry: 
 * \e Algorithms \e and \e Applications; i vertici, gli half-edge e le facce sono inserite in tre
 * liste separate (\b NON accessibili direttamente), e i collegamenti tra elementi appartenenti 
 * alle liste (es: un vertice che funge da origine per l'half-edge) sono gestiti utilizzando
 * esclusivamente gli indici delle liste (nessun puntatore).\n
 *
 * \note La DCEL utilizza alcune strutture dati proprie delle librerie QT (in particolare, QPoint
 * e QVector) la cui documentazione può essere reperita presso http://doc.trolltech.com/4.3/ .
 *
 * \author Stefano Marras (mr.marras@gmail.com)
 * \author Fabio Guggeri (guga2112@gmail.com)
 * \version 0.20
 ******************************************************/

#ifndef DCEL_HH
#define DCEL_HH

#include <QVector>
#include <QVectorIterator>
#include "structures.hh"

/**
 * \brief Implementazione della DCEL.
 *
 * La classe DCEL implementa una double connected edge list.
 * La modellazione viene effettuata seguendo le linee guida indicate nella pagina principale.\n
 * Si noti che, all'interno dei metodi DCEL, è possibile accedere direttamente alle liste
 * di vertici, half-edge e facce memorizzate, che possono quindi essere modificati utilizzando
 * i metodi definiti nelle relative strutture dati. Se invece è necessario modificare la DCEL
 * dall'esterno, ad esempio da una classe che si occupa del disegno, non è possibile
 * utilizzare direttamente i metodi sulle strutture dati, ma si rende necessario utilizzare
 * gli appositi metodi direttamente sulla DCEL. Ad esempio, quando si vuole impostare una nuova relazione
 * next-prev, dentro la DCEL è possibile richiamare il metodo \e set_next sull'half-edge di interesse,
 * mentre una classe che utilizza la DCEL dovrà utilizzare il metodo \e set_half_edge_next specificando
 * gli indici degli half-edge coinvolti nella relazione, mentre l'esecuzione dell'istruzione 
 * \e dcel.get_vertex(i).set_incident_half_edge(he) non sortirà alcun effetto per i noti meccanismi del C e del passaggio
 * dei parametri alle funzioni. Questo meccanismo che può apparire una
 * inutile complicazione è stato implementato in modo che una classe esterna non possa accedere direttamente
 * alle liste di elementi, così da poter implementare queste liste nella maniera più opportuna (nel caso
 * specifico, attraverso dei QVector).
 */
class DCEL	{

	public:

		void describe(void);

	/** \brief Rende il vertice con indice i */
	vertex get_vertex( int i );

	/** \brief Rende l'half-edge con indice i */
	half_edge get_half_edge( int i );

	/** \brief Rende la faccia con indice i */
	face get_face( int i );

	/** \brief Aggiunge un vertice e ne rende l'indice */
	int add_vertex( vertex v );

	/** \brief Aggiunge un half-edge e ne rende l'indice */
	int add_half_edge( half_edge he );
	
	/** \brief Aggiunge una faccia e ne rende l'indice */
	int add_face( face f );

	/** \brief Rende il numero di vertici */
	int get_vertex_n ( void );
    
    /** \brief Rende tutti i vertici in un QVector */
    QVector<vertex> get_all_vertexes( void );
    
    /** \brief Rende vero se il vertice si trova sul convex hull */
    bool is_vertex_in_convex_hull( int v );
    
    /** \brief Rende vero se l'half-edge si trova sul convex hull */
    bool is_half_edge_in_convex_hull( int he );
    
    /** \brief Rende vero se la faccia si trova sul convex hull */
    bool is_face_in_convex_hull( int f );

	/** \brief Rende il numero di half_edge */
	int get_half_edge_n ( void );

	/** \brief Rende il numero di facce */
	int get_faces_n ( void );

	int get_vertex_card( int v );
	
	int dec_vertex_card( int v );

	bool set_vertex_card( int v , int card );

	/** \brief Imposta l'incident_half_edge di un vertice */
	bool set_vertex_incident ( int v, int he );
    
    /** \brief Indica se il vertice si trova sul convex hull */
    bool set_vertex_in_convex_hull ( int v, bool b );

	/** \brief Imposta la normale ad un vertice */
	bool set_vertex_normal ( int v, CGPointf normal );

	/** \brief Imposta il from vertex di un half-edge */
	bool set_half_edge_from ( int he, int from );

	/** \brief Imposta il to vertex di un half-edge */
	bool set_half_edge_to ( int he, int to );

	/** \brief Imposta il twin di un half-edge */
	bool set_half_edge_twin ( int he, int twin );
	
	/** \brief Imposta il prev di un half-edge */
	bool set_half_edge_prev ( int he, int prev );

	/** \brief Imposta il next di un half-edge */
	bool set_half_edge_next ( int he, int next );

	/** \brief Imposta la faccia incidente di un half_edge */
	bool set_half_edge_face ( int he, int f );
    
    /** \brief Indica se l'edge si trova sul convex hull */
    bool set_half_edge_in_convex_hull ( int he, bool b );

	/** \brief Imposta l'inner half-edge di una faccia */
	bool set_face_inner_half_edge ( int f, int he );

	/** \brief Imposta l'outer half-edge di una faccia */
	bool set_face_outer_half_edge ( int f, int he );
    
    /** \brief Indica se la faccia si trova sul convex hull */
    bool set_face_in_convex_hull ( int f, bool b );

	/** \brief Rimuove il vertice specificato dalla lista */
	bool delete_vertex ( int v );

	/** \brief Rimuove l'half-edge specificato dalla lista */
	bool delete_half_edge ( int he );

	/** \brief Rimuove la faccia specificata dalla lista */
	bool delete_face ( int f );

	/** \brief Cancella la DCEL precedentemente creata */
	void reset( void );

	private:

	///Lista di vertici
	QVector < vertex > V;	
	///Lista di Half-edge
	QVector < half_edge > HE;
	///Lista di Facce
	QVector < face > F;		
};

#endif //DCEL_HH
