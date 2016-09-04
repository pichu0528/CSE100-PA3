/*****************************P3 FILE HEADER********************************/
/* Name: Pin Chu A98041513, Jessica Tran A11358012
* Instructor: Debashis Sahoo
* Filename: Vertex.hpp
* Date: 05/14/2015 
* Description:
* Vertex class that will be used in UndirectedGraph files to implement. 
* 
* **/
/***************************************************************************/


#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <string>
#include <unordered_map>

#include "Edge.hpp"

/**
 * Represents a Vertex in a graph.
 *
 * Vertices are connected to other Vertices via Edges. Each Vertex
 * maintains a collection of all Edges that originate from it.
 */
using namespace std;
class Vertex {
    // Graph needs access to Edge map for Dijkstra/Prim algorithms.
    friend class UndirectedGraph;
    
  public:
    /**
     * Initialize the Vertex with the given name.
     */
    Vertex(const std::string &name);

    
  private:

    /**
     * Name of this Vertex.
     */
    std::string name;
    
    /**
     * Distance of this Vertex from initial Vertex.
     * Used by Dijkstra's algorithm.
     */
    unsigned int distance;
    
    /**
     * Whether this node has been visited.
     * Used by Prim's algorithm.
     */
    bool visited;


    /**
     * Map of adjacent Vertex name to Edge describing the adjacency.
     */
    std::unordered_map<std::string, Edge> edges;
};

#endif
