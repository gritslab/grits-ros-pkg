#
#  ********************************************************************
#  *                         Makefile                                 *
#  * Convex Hull                                                      *
#  *                                                                  *
#  * Author: Arash Partow - 2001                                      *
#  * URL: http://www.partow.net                                       *
#  *                                                                  *
#  * Copyright Notice:                                                *
#  * Free use of this library is permitted under the guidelines and   *
#  * in accordance with the most current version of the Common Public *
#  * License.                                                         *
#  * http://www.opensource.org/licenses/cpl.php                       *
#  *                                                                  *
#  ********************************************************************
#


CC               = -c++
OPTIONS          = -pedantic -ansi -Wall -O3 -o
OPTIONS_LIBS     = -pedantic -ansi -Wall -O3 -c


CPP_SRC = ConvexHull.cpp \
	GrahamScanConvexHull.cpp \
	ConvexHullEPSWriter.cpp


OBJECTS = $(CPP_SRC:.cpp=.o)


%.o: %.h %.cpp
	$(CC) $(OPTIONS_LIBS) $*.cpp -o $@


all: $(OBJECTS) GSCHDemo

GSCHDemo: GSCHDemo.cpp $(OBJECTS)
	$(CC) $(OPTIONS) GSCHDemo GSCHDemo.cpp $(OBJECTS) $(LIBS) 
	
clean:
	rm -f core *.o *.bak *stackdump *#
 

#
# The End !
#
  