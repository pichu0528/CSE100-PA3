/*****************************P3 FILE HEADER********************************/
/* Name: Pin Chu A98041513, Jessica Tran A11358012
* Instructor: Debashis Sahoo
* Filename: Edge.cpp
* Date: 05/14/2015 
* Description: Edges which connect vertices in the UndirectedGraph data
*              structure.
*
* **/
/***************************************************************************/


#include "Edge.hpp"

// Method implementations here

/*
 * Compares this Edge to another Edge. Suitable for
 * use with a priority queue where Edges with the lowest
 * weight have the highest priority.
 */
bool Edge::operator<(const Edge &right) const{
    if(cost != right.cost){
       return cost > right.cost;
    }
    else{
        return true;
    }     
}
