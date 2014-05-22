TEMPLATE = app

TARGET = ConvexHull

QT += opengl

DEPENDPATH += . dcel
INCLUDEPATH += . dcel

HEADERS +=  \
            engine.h \
            window_gl.h \
            dcel/DCEL.hh \
            dcel/structures.hh \
    timer.h \
    conflict_node.h \
    conflict_graph.h

SOURCES +=  \
            engine.cpp \
            main.cpp \
            window_gl.cpp \
            dcel/DCEL.cpp \
    conflict_node.cpp \
    conflict_graph.cpp

OTHER_FILES +=


