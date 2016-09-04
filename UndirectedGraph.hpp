/*****************************P3 FILE HEADER********************************/
/* Name: Pin Chu A98041513, Jessica Tran A11358012
* Instructor: Debashis Sahoo
* Filename: UndirectedGraph.hpp
* Date: 05/14/2015 
* Description:
* This file contains the methods that we are going to use in netplan.cpp to 
* get the calculations for 1-6. Those methods including building an MST, 
* obtaining total time cost for difference case, Prim's algorithm, and 
* Dijkstra Algorithm.
* **/
/***************************************************************************/

#ifndef UNDIRECTEDGRAPH_HPP
#define UNDIRECTEDGRAPH_HPP

#include <string>
#include <unordered_map>
#include <vector>
#include "Vertex.hpp"

/**
 * Implements an undirected graph. Any edge in the graph
 * represents a bidirectional connection between two vertices.
 * 
 * Implements methods for producing a minimum spanning tree of the
 * graph, as well as calculating the total length of the shortest
 * paths between each pair of vertices.
 *
 */
using namespace std;

class EdgePtrComp{
  public:
    bool operator()(Edge &e, Edge &other)const{
      return e < other;
    }
};

class UndirectedGraph {
  public:
    /**
     * Constructs an empty UndirectedGraph with no vertices and
     * no edges.
     */
    UndirectedGraph();

    /**
     * Destructs an UndirectedGraph.
     */
    ~UndirectedGraph();

    /* adds vertex to graph*/
    Vertex* addVertex(const string &name);

    /* adds Edge to graph*/
    void addEdge(string v1, string v2, Vertex* vertex1, Vertex* vertex2,
                  unsigned int cost, unsigned int time);

    /* print the total cost of original graph*/
    unsigned int totalCost();

    /* create the minimum cost MST */
    UndirectedGraph* buildCostMST();

    /* Dijkstra Algorithm - greedy algorithm */
    unsigned int Dijkstra(Vertex* ver);

    /* total time */
    unsigned int totalTransit();
 
    /* reset all the parents, distances, and vertices */
    void reset();

  private:
    class DijkstraVertexComparator{
      public:
        bool operator()(const std::pair<Vertex*, unsigned int> &left,
                         const std::pair<Vertex*, unsigned int> &right){ 
          return left.second > right.second;
      }
    };
   
    /**
     * Map of vertex name to Vertex.
     */
    std::unordered_map<std::string, Vertex*> vertices;
};

#endif
