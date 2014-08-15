/*
 **************************************************************************
 * Class: Extended Postscript Writer                                      *
 * By Arash Partow - 2001                                                 *
 * URL: http://www.partow.net                                             *
 *                                                                        *
 * Copyright Notice:                                                      *
 * Free use of this library is permitted under the guidelines and         *
 * in accordance with the most current version of the Common Public       *
 * License.                                                               *
 * http://www.opensource.org/licenses/cpl.php                             *
 *                                                                        *
 **************************************************************************
*/


#ifndef INCLUDE_EXTENDEDPOSTSCRIPTWRITER_H
#define INCLUDE_EXTENDEDPOSTSCRIPTWRITER_H

#include <math.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "ConvexHull.h"


class ConvexHullEPSWriter
{
public:

   ConvexHullEPSWriter(){};
  ~ConvexHullEPSWriter(){};

   bool operator ()(const std::string& file_name, const std::vector<PointVDG>& point, const std::vector<PointVDG>& hull)
   {
      std::vector < std::string > output;

      double xmin = point[0].x;
      double ymin = point[0].y;
      double xmax = point[0].x;
      double ymax = point[0].y;

      for (unsigned int i = 0; i < point.size(); i++)
      {
         if      (point[i].x > xmax) xmax = point[i].x;
         else if (point[i].x < xmin) xmin = point[i].x;

         if      (point[i].y > ymax) ymax = point[i].y;
         else if (point[i].y < ymin) ymin = point[i].y;
      }

      std::ofstream fout(file_name.c_str(),std::ios::trunc);

      fout << "%!PS"<< std::endl;
      fout << "%%Creator: Convex Hull EPS Writer by Arash Partow (2001)"<< std::endl;
      fout << "%%BoundingBox: " + convert_to_string<int>((int)floor(xmin)) + " " + convert_to_string<int>((int)floor(ymin)) + " " + convert_to_string<int>((int)floor(xmax)) + " " + convert_to_string<int>((int)floor(ymax))<< std::endl;
      fout << "%%EndComments"<< std::endl;
      fout << ".00 .00 setlinewidth"<< std::endl;


      if (point.size() > 0)
      {

         fout << "newpath"   << std::endl;
         fout << "%%Points:" << std::endl;

         for(unsigned int i = 0; i < point.size(); i++)
         {
            fout << convert_to_string<int>((int)floor(point[i].x)) + " " + convert_to_string<int>((int)floor(point[i].y)) + " 3 0 360 arc stroke" << std::endl;
         }

         fout << "closepath" << std::endl;
      }


      if (hull.size() > 0)
      {
         fout << "%%Hull:" << std::endl;
         fout << "newpath" << std::endl;

         fout << convert_to_string<int>((int)floor(hull[0].x)) + " " + convert_to_string<int>((int)floor(hull[0].y)) + " moveto" << std::endl;

         for(unsigned int i = 0; i < hull.size(); i++)
         {
            fout << convert_to_string<int>((int)floor(hull[i].x)) + " " + convert_to_string<int>((int)floor(hull[i].y)) + " lineto" << std::endl;
         }

         fout << "closepath stroke" << std::endl;
         fout << "showpage"         << std::endl;
         fout << "%%EOF"            << std::endl;
      }

      fout.close();

      return true;

   }

private:

   template <typename T>
   std::string convert_to_string(const T & value)
   {
      std::stringstream stream;
      stream << value;
      return stream.str();
   }

};

#endif
