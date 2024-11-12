#ifndef UVS_TOOLS_GRAPH_SEARCH_GRAPH_HPP
#define UVS_TOOLS_GRAPH_SEARCH_GRAPH_HPP

#include <vector>
#include <map>
#include <unordered_map>
#include <list>
#include <memory>
#include <iostream>
#include <tuple>
#include <set>

#include "eigen3/Eigen/Eigen"

constexpr int GRAPH_TYPE_GRID = 0;
constexpr int GRAPH_TYPE_ABSTRACT = 1;

template <int graph_type>
class GraphBase
{
public:
    GraphBase() = default;
    virtual ~GraphBase() = default;
public:
    int type = graph_type;
};

enum GraphNodeFlag
{
    NOT_VISITED,
    IN_OPENSET,
    IN_CLOSESET
};

class GraphNodeCost
{
public:
    double distance = 0;
    double rotation = 0;
public:
    GraphNodeCost(){};
    GraphNodeCost(double distance, double rotation) : distance(distance), rotation(rotation){};
    bool operator==(const GraphNodeCost& cost) const
    {
        return distance == cost.distance && rotation == cost.rotation;
    }
    bool operator!=(const GraphNodeCost& cost) const
    {
        return distance != cost.distance || rotation != cost.rotation;
    }
    bool operator<(const GraphNodeCost& cost) const
    {
        return distance < cost.distance || (distance == cost.distance && rotation < cost.rotation);
    }
    bool operator>(const GraphNodeCost& cost) const
    {
        return distance > cost.distance || (distance == cost.distance && rotation > cost.rotation);
    }
    GraphNodeCost operator+(const GraphNodeCost& cost) const
    {
        return GraphNodeCost(distance + cost.distance, rotation + cost.rotation);
    }
    GraphNodeCost operator-(const GraphNodeCost& cost) const
    {
        return GraphNodeCost(distance - cost.distance, rotation - cost.rotation);
    }
};

template <int Dimension>
class GraphNodeBase
{
// public:
//     struct GridState
//     {
//         GraphNodeBase* node = nullptr;
//         bool occupied = false;
//         std::set<int>* collision = nullptr;
//     };
public:
    GraphNodeBase() = default;
    virtual ~GraphNodeBase() = default;

    GraphNodeBase(GraphNodeBase& other) = default;
    GraphNodeBase(const GraphNodeBase& other) = default;
    GraphNodeBase& operator=(GraphNodeBase& other) = default;

    virtual bool operator==(GraphNodeBase& other) const
    {
        return index == other.index;
    }
public:
    int dimension = Dimension;
    Eigen::Matrix<int, 1, Dimension> index;
    Eigen::Matrix<double, 1, Dimension> state;
    GraphNodeCost g,h,f;
    void* parent = nullptr;
    typename std::multimap<GraphNodeCost, void*>::iterator it;
    GraphNodeFlag flag = NOT_VISITED;
    // GridState grid_state;
};

template <typename NodeType, int Dimension>
class GridGraph : public GraphBase<GRAPH_TYPE_GRID>
{
public:
    GridGraph(){}
    GridGraph(Eigen::Matrix<double, 1, Dimension>& size, Eigen::Matrix<double, 1, Dimension>& resolution, Eigen::Matrix<double, 1, Dimension>& origin)
    {
        constructGraph(size, resolution, origin);
    }
    ~GridGraph() override 
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            delete nodes[i];
        }
    }

    GridGraph(GridGraph &other)
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            delete nodes[i];
        }
        constructGraph(other.size, other.resolution, other.origin);
    }
    GridGraph(const GridGraph &other)
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            delete nodes[i];
        }
        constructGraph(other.size, other.resolution, other.origin);
    }
    GridGraph& operator=(GridGraph &other)
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            delete nodes[i];
        }
        constructGraph(other.size, other.resolution, other.origin);
        return *this;
    }

    virtual void constructGraph(Eigen::Matrix<double, 1, Dimension>& size, Eigen::Matrix<double, 1, Dimension>&resolution, Eigen::Matrix<double, 1, Dimension>& origin)
    {
        this->size = size;
        this->resolution = resolution;
        this->origin = origin;
        for (int i = 0; i < Dimension; i++)
        {
            grid_size(i) = size(i) / resolution(i);
        }
        nodes.resize(grid_size.prod());
        for (int i = 0; i < nodes.size(); i++)
        {
            nodes[i] = new NodeType();
            
            for (int j = 0; j < Dimension; j++)
            {
                nodes[i]->index(j) = i / grid_size.block(0, 0, 1, j).prod() % grid_size(j);
                nodes[i]->state(j) = origin(j) + nodes[i]->index(j) * resolution(j);
            }

        }
    }

    virtual NodeType* operator[](Eigen::Matrix<int, 1, Dimension> index) 
    {
        if ((index.minCoeff() < 0) || ((grid_size - index).minCoeff() <= 0))
        {
            return nullptr;
        }
        int i = index(Dimension - 1);
        for (int j = Dimension - 2; j >= 0; j--)
        {
            i = i * grid_size(j) + index(j);
        }
        return nodes[i];
    }
    virtual NodeType* operator()(std::vector<int> index) 
    {
        for (int i = 0; i < Dimension; i++)
        {
            if (index[i] >= grid_size(i) || index[i] < 0)
            {
                return nullptr;
            }
        }
        int i = index[Dimension - 1];
        for (int j = Dimension - 2; j >= 0; j--)
        {
            i = i * grid_size[j] + index[j];
        }
        return nodes[i];
    }

public:
    Eigen::Matrix<double, 1, Dimension> size;   //x,y,z
    Eigen::Matrix<double, 1, Dimension> resolution; //x,y,z
    Eigen::Matrix<double, 1, Dimension> origin; //x,y,z

    Eigen::Matrix<int, 1, Dimension> grid_size; //width(x), length(y), height(z)

    std::vector<NodeType*> nodes;
};

template <typename NodeType, typename EdgeType>
class AbstractGraph : public GraphBase<GRAPH_TYPE_ABSTRACT>
{
public:
    AbstractGraph(){}
    ~AbstractGraph() override 
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            delete nodes[i];
        }
        for (int i = 0; i < edges.size(); i++)
        {
            delete edges[i];
        }
    }

    AbstractGraph(AbstractGraph &other)
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            delete nodes[i];
        }
        for (int i = 0; i < edges.size(); i++)
        {
            delete edges[i];
        }
        for (auto & node : other.nodes)
        {
            nodes.push_back(new NodeType(*node));
        }
        for (auto & edge : other.edges)
        {
            edges.push_back(new EdgeType(*edge));
        }
        adjacency_matrix = other.adjacency_matrix;
    }
    AbstractGraph(const AbstractGraph &other)
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            delete nodes[i];
        }
        for (int i = 0; i < edges.size(); i++)
        {
            delete edges[i];
        }
        for (auto & node : other.nodes)
        {
            nodes.push_back(new NodeType(*node));
        }
        for (auto & edge : other.edges)
        {
            edges.push_back(new EdgeType(*edge));
        }
        adjacency_matrix = other.adjacency_matrix;
    }
    AbstractGraph& operator=(AbstractGraph &other)
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            delete nodes[i];
        }
        for (int i = 0; i < edges.size(); i++)
        {
            delete edges[i];
        }
        for (auto & node : other.nodes)
        {
            nodes.push_back(new NodeType(*node));
        }
        for (auto & edge : other.edges)
        {
            edges.push_back(new EdgeType(*edge));
        }
        adjacency_matrix = other.adjacency_matrix;
    }

    virtual void addNode(NodeType* node)
    {
        adjacency_matrix.conservativeResize(adjacency_matrix.rows() + 1, adjacency_matrix.cols() + 1);
        nodes.push_back(node);
    }
    virtual void addEdge(int index1, int index2, EdgeType* edge)
    {
        if (index1 >= nodes.size() || 
            index2 >= nodes.size() ||
            index1 < 0 ||
            index2 < 0)
        {
            return;
        }
        adjacency_matrix(index1, index2) = 1;
        adjacency_matrix(index2, index1) = 1;
        edge_map[std::make_pair(index1, index2)] = edges.size();
        edges.push_back(edge);
    }

    virtual NodeType* operator[](int index)
    {
        if (index >= nodes.size() || index < 0)
        {
            return nullptr;
        }
        return nodes[index];
    }

    virtual EdgeType* operator()(int index1, int index2)
    {
        if (index1 >= nodes.size() || 
            index2 >= nodes.size() ||
            index1 < 0 ||
            index2 < 0)
        {
            return nullptr;
        }
        auto it = edge_map.find(std::make_pair(index1, index2));
        if (it == edge_map.end())
        {
            return nullptr;
        }
        else
        {
            return edges[it->second];
        }
    }

public:
    std::vector<NodeType*> nodes;
    Eigen::MatrixXi adjacency_matrix;
    std::vector<EdgeType*> edges;
    std::map<std::pair<int,int>,int> edge_map;
};

#endif // UVS_TOOLS_GRAPH_SEARCH_GRAPH_HPP