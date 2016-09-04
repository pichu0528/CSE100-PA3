/*****************************P3 FILE HEADER********************************/
/* Name: Pin Chu A98041513, Jessica Tran A11358012
* Instructor: Debashis Sahoo
* Filename: Vertex.cpp
* Date: 05/14/2015 
* Description:
* Vertex class that will be used in UndirectedGraph files to implement. *
* **/
/***************************************************************************/



#include "Vertex.hpp"
#include <math.h>
using namespace std;
Vertex::Vertex(const string &name ){
  this->name = name;
  this->distance = INFINITY;
  this->visited = false;
}


