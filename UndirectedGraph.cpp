/*****************************P3 FILE HEADER********************************/
/* Name: Pin Chu A98041513, Jessica Tran A11358012
* Instructor: Debashis Sahoo
* Filename: UndirectedGraph.cpp
* Date: 05/14/2015 
* Description:
* This file contains the methods that we are going to use in netplan.cpp to 
* get the calculations for 1-6. Those methods including building an MST, 
* obtaining total time cost for difference case, Prim's algorithm, and 
* Dijkstra Algorithm.
* **/
/***************************************************************************/
#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"
#include <queue>
#include <vector>
#include <iostream>
#include <cmath>
using namespace std;
// Method implementations here

/* graph destructor: deallocate memory for all vertices*/
UndirectedGraph::~UndirectedGraph(){

  for(auto i = vertices.begin(); i != vertices.end(); i++){
    Vertex * v =  i->second;
    delete v;
    v = 0;

  }
}

/* empty graph constructor*/
UndirectedGraph::UndirectedGraph(){
}

Vertex* UndirectedGraph::addVertex(const string &name){
  //to check for duplicate elements
  unordered_map<string, Vertex*>::iterator found;

  Vertex* vertex;
  
  //check for duplicates with map find function
  found = vertices.find(name);

  if(found == vertices.end()){
    //create new vertex if it was not found in the map
    vertex = new Vertex(name);
    vertices.insert(make_pair(name, vertex));
  }
  else{
   //skip allocation if vertex exists 
   vertex = found->second;
  }

  return vertex;

}
void UndirectedGraph::addEdge(string v1, string v2, Vertex* vertex1, Vertex* vertex2, 
                               unsigned int cost, unsigned int time){

  //create a new edges for each vertex
  Edge edge1 = Edge(vertex1, vertex2, cost, time);
  Edge edge2 = Edge(vertex2, vertex1, cost, time);
    
  //add these edges to the vertex's adjacency list
  vertex1->edges.insert(make_pair(v2, edge1));
  vertex2->edges.insert(make_pair(v1, edge2));

    
}

unsigned int  UndirectedGraph::totalCost(){
  unsigned int totalCost = 0;

  /* traverse graph to add up costs*/
  for(auto i = vertices.begin(); i != vertices.end(); i++){
    //check all of the edges in each vertex in the graph
    Vertex* v = i->second;
    for (auto j = v->edges.begin(); j != v->edges.end(); j++){
      //add up the costs
      totalCost+=(j->second.cost);
    }

  }

  //divide by two since each edge was accounted for twice
  totalCost = totalCost/2;

  return totalCost;

}

UndirectedGraph* UndirectedGraph::buildCostMST(){
  UndirectedGraph* MST = new UndirectedGraph(); //graph to return
  priority_queue<Edge, vector<Edge>, EdgePtrComp> edgePQ;
  Edge edge;

  //pick arbitrary start vertex and mark as visited and distance 0
  auto vertexIT = vertices.begin();
  Vertex * start = vertexIT->second;
  start->visited = true;

  //iterate through adjacency list of v and put edges in priority queue
  for(auto adjList = start->edges.begin(); adjList != start->edges.end(); adjList++){

    //push edge to pq
    edgePQ.push(adjList->second);

  }

  //if priority que is empty you are done
  while(!edgePQ.empty()){
    //remove from pq edge with smallest cost
    edge = edgePQ.top();
    edgePQ.pop();
    //if to vertex visited go back to beginning to avoid cycle
    if(edge.to-> visited && edge.from->visited){continue;}
    //else add the vertex and edge to the MST
    Vertex * from = MST ->addVertex(edge.from->name);
    Vertex* to = MST -> addVertex(edge.to->name);
    MST->addEdge(from->name, to->name, from, to, 
                  edge.cost, edge.length);
     
    //mark to as visited
    edge.to-> visited = true;
    //go through the to's adjacency list, putting each edge in pq
    for( auto toList = edge.to->edges.begin(); 
         toList != edge.to->edges.end(); toList++){
      edgePQ.push(toList->second);
    }
  }
  return MST;
}


/* Dijkstra Algorithm - greedy algorithm */
unsigned int UndirectedGraph::Dijkstra(Vertex* ver){
    // Reset everything before calling Dijkstra
    reset();
    
    Vertex* adjacent = 0;
    priority_queue<std::pair<Vertex*, unsigned int>, 
                   std::vector<std::pair<Vertex*, unsigned int>>, 
                   UndirectedGraph::DijkstraVertexComparator> queue;
    unsigned int totalDistance = 0; //shortest distance to return

    //give start vertex distance of zero
    ver->distance = 0;

    //enqueue start vertex
    queue.push(make_pair(ver, ver->distance));
   
    //stop when priority queue is empty
    while(!queue.empty()){
      //dequeue the vertex with smallest cost
      pair <Vertex*, unsigned int> p = queue.top();
      Vertex* v = p.first;
      queue.pop();
      
      //skip v if already visited
      if(v->visited){continue;}

      //else mark as visited: we found the shortest path
      v->visited = true;
      totalDistance += v->distance;


      // for each of vertex's unvisited adjacent nodes
      for(auto j = v->edges.begin(); j != v->edges.end(); j++){
  
        //find the adjacent vertices
        adjacent = vertices.find(j->first)->second;
        //calculate best cost from v to its adjacent vertex
        unsigned int bestCost = (j->second).length + v->distance;

        //if the weight of edge is less than distance and vertex is still 
        //in graph update the distance and push into queue
        if((adjacent->visited == false) && 
            (bestCost < adjacent->distance) ){
          adjacent->distance = bestCost;
          queue.push(make_pair(adjacent, adjacent->distance));
        }
      }

    }

  return totalDistance;

}

/* total time */
unsigned int UndirectedGraph::totalTransit(){
  int totalTime = 0;
  auto begin = vertices.begin();
  // Add up all the lengths from all the edges
  for(; begin != vertices.end(); begin++){
     unsigned int transit = Dijkstra(begin->second);
     totalTime += transit;
  }

  return totalTime;
}

/* reset all the  distances, and vertices. */
void UndirectedGraph::reset(){
    auto start = vertices.begin();
    for(; start != vertices.end(); start++){
     start->second -> visited = false;
     start->second -> distance = INFINITY;   
    }
}
