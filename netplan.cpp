/*****************************P3 FILE HEADER********************************/
/* Name: Pin Chu A98041513, Jessica Tran A11358012
* Instructor: Debashis Sahoo
* Filename: netplan.cpp
* Date: 05/14/2015 
* Description: Main driver for PA3. Reads in a file containing data on
*              networks (vertices) and outputs total cost, 
*              cost of MST, total time,
*              and total time using the MST, as well as their differences.
*
* **/
/***************************************************************************/




#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "UndirectedGraph.hpp"

/**
 * Entry point into the netplan program.
 *
 * Usage:
 *   ./netplan infile
 *
 */
using namespace std;
int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " infile" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "Unable to open file for reading." << std::endl;
        return EXIT_FAILURE;
    }
    
    // Implementation here
    
    UndirectedGraph graph;
    string v1;
    string v2;
    unsigned int cost = 0;
    unsigned int time = 0;
    unsigned int totalCost = 0; //total cost of networks using original graph
    unsigned int totalTime = 0; //shortest time to traverse networks w/ original
    unsigned int totalMSTTime = 0; //shortest time using MST
    unsigned int minCost = 0; //min cost of original graph
    unsigned int moneySaved = 0; //amount saved with MST
    unsigned int difference = 0; //time saved using Dijkstra's on original 
    Vertex* vertex1; 
    Vertex* vertex2;
    UndirectedGraph* mst; //MST based on cost using Prim's


    
    //read in file info line by line
    while(in.good()){

      in >> v1 >> v2 >> cost >> time;

      if(!in.good()){ break; } //exit if read in error or EOF

      //add vertices v1 and v2 to the graph
      vertex1 = graph.addVertex(v1);
      vertex2 = graph.addVertex(v2);

      
      //add the edge with cost and time to the graph
      graph.addEdge(v1, v2, vertex1, vertex2, cost, time);

    }

    //print out total cost
    totalCost = graph.totalCost();
    cout << totalCost << endl;

    //print min cost
    mst = graph.buildCostMST();
    minCost = mst->totalCost();
    cout << minCost <<endl;

    //print out money saved with min cost network
    moneySaved = totalCost-minCost;
    cout << moneySaved <<endl;

    //print shortest time traversal using original graph
    totalTime = graph.totalTransit();
    cout <<  totalTime << endl;

    //print time traversal using MST
    totalMSTTime = mst->totalTransit();
    cout << totalMSTTime << endl;

    //the difference between total transit and total MST transit
    difference = totalMSTTime - totalTime;
    cout << difference << endl;

    //deallocate memory
    delete mst;
    mst = 0;

    return EXIT_SUCCESS;
}
