/*****************************P3 FILE HEADER********************************/
/* Name: Pin Chu A98041513, Jessica Tran A11358012
* Instructor: Debashis Sahoo
* Filename: Edge.hpp
* Date: 05/14/2015 
* Description: Edges connecting vertices in Undirected Graph data structure.
*
* **/
/***************************************************************************/


#ifndef EDGE_HPP
#define EDGE_HPP

class Vertex;

/**
 * Represents an edge in a graph.
 *
 * Maintains pointers to the vertices that the edge originates 
 * from and terminates at. Edges have both a cost and a length,
 * which are both non-negative integers.
 *
 * Follows value semantics, so can be copy constructed.
 */
class Edge {
  friend class UndirectedGraph;
  friend class Vertex;
  public:
    /**
     * Constructs an Edge from the given parameters.
     */
    Edge(Vertex *from = nullptr, Vertex *to = nullptr,
            unsigned int cost = 1,
            unsigned int length = 1): from(from), to(to),
                          cost(cost), length(length){}

    /*
     * Compares this Edge to another Edge. Suitable for
     * use with a priority queue where Edges with the lowest
     * weight have the highest priority.
     */
    bool operator<(const Edge &right) const;




    
  private:
    /**
     * Vertex that this Edge originates from.
     */
    Vertex *from;

    /**
     * Vertex that this Edge terminates at.
     */
    Vertex *to;

    /**
     * Cost of this Edge.
     */
    unsigned int cost;

    /**
     * Length of this Edge.
     */
    unsigned int length;
};

#endif
