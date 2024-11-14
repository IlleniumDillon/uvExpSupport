#ifndef GRAPH_SEARCH_HPP
#define GRAPH_SEARCH_HPP

#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <chrono>

#include "uvs_tools/graph_search/graph.hpp"

template <typename TraceType>
class GraphSearchResultBase
{
public:
    GraphSearchResultBase() = default;
    virtual ~GraphSearchResultBase() = default;
public:
    bool success = false;
    int iterations = 0;
    double planTime = 0;
    double cost = 0;
    std::vector<TraceType> trace;
};

template <typename GraphType, typename NodeType, typename ResultType>
class GraphSearchBase
{
public:
    GraphSearchBase() = default;
    virtual ~GraphSearchBase() = default;

    virtual void updateGraph(GraphType& graph) = 0;
    virtual void reset() = 0;
    virtual GraphNodeCost heuristic(NodeType* node1, NodeType* node2) = 0;
    virtual void getNeighbors(NodeType* node, std::vector<NodeType*>& neighbors, std::vector<GraphNodeCost>& costs) = 0;
    
    virtual bool endSearch(NodeType* node1, NodeType* node2) {return *node1 == *node2;};
    virtual ResultType search(NodeType* start, NodeType* goal);
public:
    GraphType graph;
    ResultType result;
    std::multimap<GraphNodeCost, void*> openSet;
    std::vector<NodeType*> closeSet;
};

template <typename GraphType, typename NodeType, typename ResultType>
inline ResultType GraphSearchBase<GraphType, NodeType, ResultType>::search(NodeType *start, NodeType *goal)
{
    reset();
    auto start_time = std::chrono::high_resolution_clock::now();
    
    start->g = GraphNodeCost(0, 0);
    start->h = heuristic(start, goal);
    start->f = start->g + start->h;
    start->flag = IN_OPENSET;
    start->it = openSet.insert(std::make_pair(start->f, start));
    while (!openSet.empty())
    {
        result.iterations++;
        auto current = (NodeType*)openSet.begin()->second;
        // std::cout << "current: " << current->index << std::endl;

        if (endSearch(current, goal))
        {
            result.success = true;
            result.cost = current->g.distance;
            while (current != nullptr)
            {
                result.trace.push_back(current->state);
                current = (NodeType*)current->parent;
            }
            std::reverse(result.trace.begin(), result.trace.end());
            break;
        }
        openSet.erase(current->it);
        closeSet.push_back(current);
        current->flag = IN_CLOSESET;

        std::vector<NodeType*> neighbors;
        std::vector<GraphNodeCost> costs;
        getNeighbors(current, neighbors, costs);
        for (int i = 0; i < neighbors.size(); i++)
        {
            auto neighbor = neighbors[i];
            // if (neighbor->flag == IN_CLOSESET)
            // {
            //     continue;
            // }
            GraphNodeCost tentative_g = current->g + costs[i];
            if (neighbor->flag != IN_OPENSET)
            {
                neighbor->h = heuristic(neighbor, goal);
                neighbor->g = tentative_g;
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                neighbor->flag = IN_OPENSET;
                neighbor->it = openSet.insert(std::make_pair(neighbor->f, neighbor));
            }
            else if (tentative_g < neighbor->g)
            {
                openSet.erase(neighbor->it);
                neighbor->g = tentative_g;
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                neighbor->it = openSet.insert(std::make_pair(neighbor->f, neighbor));
            }
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    result.planTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    return result;
}

#endif // GRAPH_SEARCH_HPP
