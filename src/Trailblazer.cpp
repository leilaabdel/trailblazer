// This is the CPP file you will edit and turn in.
// Also remove these comments here and add your own, along with
// comments on every function and on complex code sections.
// TODO: write comment header for this file; remove this comment

#include "Trailblazer.h"
#include "set.h"
#include "queue.h"
#include "priorityqueue.h"
//#include <iostream>
using namespace std;

static const double SUFFICIENT_DIFFERENCE = 0.2;

void printPath(Path p)
{
    for(RoadNode* node : p)
    {
        cout << node->nodeName() << "  ";
    }

    cout << endl;

}

bool isVisited(Path & completePath , RoadNode* node) {
    for(int i = 0 ; i < completePath.size() ; i++) {
        if (completePath[i] == node) return true;
    }

    return false;
}


Path breadthFirstSearch(const RoadGraph& graph, RoadNode* start, RoadNode* end) {

    Queue<Path> pathsQueue;
    Path startPath;
    startPath.add(start); // add the first roadNode to the currPath;




    pathsQueue.enqueue(startPath);
    Set<RoadNode*> visited;

    while(!pathsQueue.isEmpty()) { // added condition of end node // && !visited.contains(end)

        Path currPath = pathsQueue.dequeue();
        RoadNode* last = currPath[currPath.size()-1];


        last->setColor(Color::GREEN);
        visited.add(last); // if it is already added nothing will happen as it is set



        // check if last is the end node
        if(last == end)
        {
            return currPath;
        }


        // explore the neighbors
        Set<RoadNode*> neighbors = graph.neighborsOf(last);


        // check

        if(neighbors.isEmpty() && pathsQueue.isEmpty())
        {
            // from description
            // if no path is found, return an empty path

            return {};
        }

        // if there are no neighbors but queue is not empty then continue
        if(neighbors.isEmpty())
        {
            continue;
        }


        // form new paths

        for(RoadNode* currNeighbor : neighbors) {

            if(!visited.contains(currNeighbor)) {
                Path newPath = currPath;

                // add the new neighbor to the new path
                newPath.add(currNeighbor);

                // add the new path to the pathsQueue
                pathsQueue.enqueue(newPath);

            }
        }


    }

    return {};

}


double getDistance(const RoadGraph& graph, Path & currPath) {

    double dist = 0;

    for(int i = 0 ; i < currPath.size() ; i++) {
        RoadEdge* edge = graph.edgeBetween(currPath[i] , currPath[(i+1)%currPath.size()]);
        double currDist = edge->cost();
        dist += currDist;
    }

    return dist;
}






Path dijkstrasAlgorithm(const RoadGraph& graph, RoadNode* start, RoadNode* end) {

    PriorityQueue<Path> pathsQueue;
    Path startPath;
    startPath.add(start); // add the first roadNode to the currPath;
    pathsQueue.enqueue(startPath, 0);
    Set<RoadNode*> visited;
    double cost;


    while(!pathsQueue.isEmpty()) { // added condition of end node // && !visited.contains(end)

        // get current cost
        cost = pathsQueue.peekPriority();

        Path currPath = pathsQueue.dequeue();
        RoadNode* last = currPath[currPath.size()-1];


        last->setColor(Color::GREEN);
        visited.add(last); // if it is already added nothing will happen as it is set



        // check if last is the end node
        if(last == end)
        {
            printPath(currPath);

            return currPath;
        }


        // explore the neighbors
        Set<RoadNode*> neighbors = graph.neighborsOf(last);


        // check

        if(neighbors.isEmpty() && pathsQueue.isEmpty())
        {
            // from description
            // if no path is found, return an empty path

            return {};
        }

        // if there are no neighbors but queue is not empty then continue
        if(neighbors.isEmpty())
        {
            continue;
        }


        // form new paths


        for(RoadNode* currNeighbor : neighbors) {

            if(!visited.contains(currNeighbor)) {
                Path newPath = currPath;

                // add the new neighbor to the new path
                newPath.add(currNeighbor);

                // update cost
                RoadEdge* edge = graph.edgeBetween(last , currNeighbor);

                // add the new path to the pathsQueue
                pathsQueue.enqueue(newPath, cost + edge->cost()); // accumulative cost

            }
        }


    }

    return {};

}



Path aStar(const RoadGraph& graph, RoadNode* start, RoadNode* end) {

    /**
    Your A* search algorithm should always return a path with the same length and cost as the path found by Dijkstra's
    algorithm. If you find that the algorithms give paths of different costs, it probably indicates a bug in your solution.
    **/

    PriorityQueue<Path> pathsQueue;
    Path startPath;
    startPath.add(start); // add the first roadNode to the currPath;


    double cost;
    double heuristic;
    double totalCost;

    // initially
    cost = 0;
    heuristic = graph.crowFlyDistanceBetween(start, end)/graph.maxRoadSpeed();
    totalCost = cost + heuristic;

    pathsQueue.enqueue(startPath, totalCost);
    Set<RoadNode*> visited;



    while(!pathsQueue.isEmpty()) { // added condition of end node // && !visited.contains(end)

        // get current totalCost
        totalCost = pathsQueue.peekPriority();


        Path currPath = pathsQueue.dequeue();
        RoadNode* last = currPath[currPath.size()-1];


        last->setColor(Color::GREEN);
        visited.add(last); // if it is already added nothing will happen as it is set


        // check if last is the end node
        if(last == end)
        {
            printPath(currPath);
            return currPath;
        }


        // old heuristic
        heuristic = graph.crowFlyDistanceBetween(last, end)/graph.maxRoadSpeed();

        // old cost
        cost = totalCost - heuristic;


        // explore the neighbors
        Set<RoadNode*> neighbors = graph.neighborsOf(last);


        // check

        if(neighbors.isEmpty() && pathsQueue.isEmpty())
        {
            // from description
            // if no path is found, return an empty path

            return {};
        }

        // if there are no neighbors but queue is not empty then continue
        if(neighbors.isEmpty())
        {
            continue;
        }


        // form new paths

        for(RoadNode* currNeighbor : neighbors) {

            if(!visited.contains(currNeighbor)) {
                Path newPath = currPath;

                // add the new neighbor to the new path
                newPath.add(currNeighbor);

                // update cost
                RoadEdge* edge = graph.edgeBetween(last , currNeighbor);

                // new heuristic
                heuristic = graph.crowFlyDistanceBetween(currNeighbor, end)/graph.maxRoadSpeed();

                // new totalCost
                totalCost = (cost + edge->cost()) + heuristic;

                // add the new path to the pathsQueue
                pathsQueue.enqueue(newPath, totalCost); // accumulative cost

            }
        }


    }

    return {};

}




Path dijkstra2(const RoadGraph& graph, RoadNode* start, RoadNode* end, RoadNode* edgeStart, RoadNode* edgeEnd, double& pathCost) {


    cout << "remove " << edgeStart->nodeName() << " " << edgeEnd->nodeName() << endl;

    PriorityQueue<Path> pathsQueue;
    Path startPath;
    startPath.add(start); // add the first roadNode to the currPath;


    double cost;

    // initially
    cost = 0;

    pathsQueue.enqueue(startPath, cost);
    Set<RoadNode*> visited;


    while(!pathsQueue.isEmpty()) { // added condition of end node // && !visited.contains(end)

        // get current totalCost
        cost = pathsQueue.peekPriority();


        Path currPath = pathsQueue.dequeue();
        RoadNode* last = currPath[currPath.size()-1];


        last->setColor(Color::GREEN);
        visited.add(last); // if it is already added nothing will happen as it is set



        // check if last is the end node
        if(last == end)
        {

            pathCost = cost; // at end node totalCost is purely cost as heuristic will be 0
            return currPath;
        }


        // explore the neighbors
        Set<RoadNode*> neighbors = graph.neighborsOf(last);


        // check

        if(neighbors.isEmpty() && pathsQueue.isEmpty())
        {
            // from description
            // if no path is found, return an empty path

            pathCost = INFINITY;
            return {};
        }

        // if there are no neighbors but queue is not empty then continue
        if(neighbors.isEmpty())
        {
            continue;
        }


        // form new paths


        for(RoadNode* currNeighbor : neighbors) {

            if(!visited.contains(currNeighbor)) {

                // excluded path check

                if( last == edgeStart && currNeighbor == edgeEnd)
                {
                    cout << "skipped path " << last->nodeName() <<  " " << currNeighbor->nodeName()<< endl;

                    continue; // do not add path
                }


                Path newPath = currPath;

                // add the new neighbor to the new path
                newPath.add(currNeighbor);

                // update cost
                RoadEdge* edge = graph.edgeBetween(last , currNeighbor);


                // add the new path to the pathsQueue
                pathsQueue.enqueue(newPath, cost + edge->cost()); // accumulative cost

            }
        }


    }


    return {};
}


Path aStar2(const RoadGraph& graph, RoadNode* start, RoadNode* end, RoadNode* edgeStart, RoadNode* edgeEnd, double& pathCost) {

    /**
    Your A* search algorithm should always return a path with the same length and cost as the path found by Dijkstra's
    algorithm. If you find that the algorithms give paths of different costs, it probably indicates a bug in your solution.
    **/

    PriorityQueue<Path> pathsQueue;
    Path startPath;
    startPath.add(start); // add the first roadNode to the currPath;


    double cost;
    double heuristic;
    double totalCost;

    // initially
    cost = 0;
    heuristic = graph.crowFlyDistanceBetween(start, end)/graph.maxRoadSpeed();
    totalCost = cost + heuristic;

    pathsQueue.enqueue(startPath, totalCost);
    Set<RoadNode*> visited;


    while(!pathsQueue.isEmpty()) { // added condition of end node // && !visited.contains(end)

        // get current totalCost
        totalCost = pathsQueue.peekPriority();


        Path currPath = pathsQueue.dequeue();
        RoadNode* last = currPath[currPath.size()-1];


        last->setColor(Color::GREEN);
        visited.add(last); // if it is already added nothing will happen as it is set



        // check if last is the end node
        if(last == end)
        {

            pathCost = totalCost; // at end node totalCost is purely cost as heuristic will be 0
            return currPath;
        }


        // old heuristic
        heuristic = graph.crowFlyDistanceBetween(last, end)/graph.maxRoadSpeed();

        // old cost
        cost = totalCost - heuristic;


        // explore the neighbors
        Set<RoadNode*> neighbors = graph.neighborsOf(last);


        // check

        if(neighbors.isEmpty() && pathsQueue.isEmpty())
        {
            // from description
            // if no path is found, return an empty path

            pathCost = INFINITY;
            return {};
        }

        // if there are no neighbors but queue is not empty then continue
        if(neighbors.isEmpty())
        {
            continue;
        }


        // form new paths


        for(RoadNode* currNeighbor : neighbors) {

            if(!visited.contains(currNeighbor)) {

                // excluded path check

                if( last == edgeStart && currNeighbor == edgeEnd)
                {
                    continue; // do not add path
                }


                Path newPath = currPath;

                // add the new neighbor to the new path
                newPath.add(currNeighbor);

                // update cost
                RoadEdge* edge = graph.edgeBetween(last , currNeighbor);

                // new heuristic
                heuristic = graph.crowFlyDistanceBetween(currNeighbor, end)/graph.maxRoadSpeed();

                // new totalCost
                totalCost = (cost + edge->cost()) + heuristic;

                // add the new path to the pathsQueue
                pathsQueue.enqueue(newPath, totalCost); // accumulative cost

            }
        }


    }

    return {};
}





Path aStarAlt(const RoadGraph& graph, RoadNode* start, RoadNode* end, RoadNode* edgeStart, RoadNode* edgeEnd)
{

    /**
    Your A* search algorithm should always return a path with the same length and cost as the path found by Dijkstra's
    algorithm. If you find that the algorithms give paths of different costs, it probably indicates a bug in your solution.
    **/

    PriorityQueue<Path> pathsQueue;
    Path startPath;
    startPath.add(start); // add the first roadNode to the currPath;


    double cost;
    double heuristic;
    double totalCost;

    // initially
    cost = 0;
    heuristic = graph.crowFlyDistanceBetween(start, end)/graph.maxRoadSpeed();
    totalCost = cost + heuristic;

    pathsQueue.enqueue(startPath, totalCost);
    Set<RoadNode*> visited;


    while(!pathsQueue.isEmpty()) { // added condition of end node // && !visited.contains(end)

        // get current totalCost
        totalCost = pathsQueue.peekPriority();


        Path currPath = pathsQueue.dequeue();
        RoadNode* last = currPath[currPath.size()-1];


        last->setColor(Color::GREEN);
        visited.add(last); // if it is already added nothing will happen as it is set



        // check if last is the end node
        if(last == end)
        {

            return currPath;
        }


        // old heuristic
        heuristic = graph.crowFlyDistanceBetween(last, end)/graph.maxRoadSpeed();

        // old cost
        cost = totalCost - heuristic;

        // explore the neighbors
        Set<RoadNode*> neighbors = graph.neighborsOf(last);


        // check

        if(neighbors.isEmpty() && pathsQueue.isEmpty())
        {
            // from description
            // if no path is found, return an empty path
            return {};
        }

        // if there are no neighbors but queue is not empty then continue
        if(neighbors.isEmpty())
        {
            continue;
        }


        // form new paths


        for(RoadNode* currNeighbor : neighbors) {

            if(!visited.contains(currNeighbor)) {

                // excluded path check

                if( last == edgeStart && currNeighbor == edgeEnd)
                {
                    continue; // do not add path
                }


                Path newPath = currPath;

                // add the new neighbor to the new path
                newPath.add(currNeighbor);

                // update cost
                RoadEdge* edge = graph.edgeBetween(last , currNeighbor);

                // new heuristic
                heuristic = graph.crowFlyDistanceBetween(currNeighbor, end)/graph.maxRoadSpeed();

                // new totalCost
                totalCost = (cost + edge->cost()) + heuristic;

                // add the new path to the pathsQueue
                pathsQueue.enqueue(newPath, totalCost); // accumulative cost

            }
        }


    }

    return {};
}



bool findNode(Path p, RoadNode* n)
{
    for(RoadNode* node : p)
    {
        if(node == n) return true;
    }

    return false;
}

double getSufficientDifference(Path best, Path alt)
{

    int len = best.size();
    int count = 0;

    for(RoadNode* node : best)
    {
        if(!findNode(alt, node)) count++;

    }

    return double(count)/len;

}


Path alternativeRoute(const RoadGraph& graph, RoadNode* start, RoadNode* end) {

    Path best = aStar(graph, start, end);


    Path alternative = {};
    double suffDiff  = 0; // initially no difference

    Path alt_temp;
    double diff_temp ;


    for(int i=0; i<best.size()-1; i++)
    {
        alt_temp = aStarAlt(graph, start, end, best[i], best[i+1]);
        diff_temp = getSufficientDifference(best, alternative);

        // continue if no path
        if(alt_temp.isEmpty())
        {
            continue;
        }


        // difference

        if(diff_temp >= SUFFICIENT_DIFFERENCE) // diff_temp > suffDiff
        {
            alternative = alt_temp;
            suffDiff = diff_temp;
        }
    }

    return alternative;
}


Path alternativeRoute1(const RoadGraph& graph, RoadNode* start, RoadNode* end) {

    //Path best = dijkstrasAlgorithm(graph, start, end);
    Path best = aStar(graph, start, end);

    //


    printPath(best);

    Path alternative = {};
    double cost = INFINITY;

    Path tempPath;
    double tempCost;

    for(int i=0; i<best.size()-1; i++)
    {

        //tempPath = dijkstra2(graph, start, end, best[i], best[i+1], tempCost);
        tempPath = aStar2(graph, start, end, best[i], best[i+1], tempCost);

        if(tempPath.isEmpty())
        {
            continue;
        }

        if(tempCost < cost)
        {
            alternative = tempPath;
            cost = tempCost;
        }
    }

    return alternative;
}



