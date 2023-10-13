#ifndef WEIGHTED_GRAPH_ADJ_LIST_
#define WEIGHTED_GRAPH_ADJ_LIST_

#include "../includes.h"

/**
 * @class WGraphList
 * Represents a weighted directed graph using an adjacency list.
 */
class WGraphList {    
public:
    /**
     * Constructor to create a weighted graph with a specified number of vertices.
     *
     * @param v The number of vertices in the graph.
     */
    WGraphList(std::size_t v);

    /**
     * Copy constructor for creating a deep copy of a graph.
     */
    WGraphList(const WGraphList&);

    /**
     * Copy assignment operator for creating a deep copy of a graph.
     */
    WGraphList& operator=(const WGraphList&);

    /**
     * Move constructor for efficiently transferring ownership of graph data.
     */
    WGraphList(WGraphList&&);

    /**
     * Move assignment operator for efficiently transferring ownership of graph data.
     */
    WGraphList& operator=(WGraphList&&);

    /**
     * Add a directed edge from source to destination with a specified weight.
     *
     * @param s The source vertex.
     * @param d The destination vertex.
     * @param w The weight of the edge.
     */
    void addEdge(std::size_t s, std::size_t d, int w);

    /**
     * Add a new vertex to the graph.
     */
    void addVertex();

    /**
     * Get the number of vertices in the graph.
     */
    constexpr std::size_t size() noexcept;

    /**
     * Perform a topological sort of the graph and return the result as a vector of vertices.
     *
     * Complexity: O(V + E), where V is the number of vertices and E is the number of edges.
     */
    std::vector<std::size_t> topSort();

    /**
     * Compute the Single-Source Shortest Path (SSSP) in a Directed Acyclic Graph (DAG).
     *
     * @param s The source vertex.
     * @return A vector of distances from the source vertex to all other vertices.
     *
     * Complexity: O(V + E), where V is the number of vertices and E is the number of edges.
     */
    std::vector<int> SSSPDAG(std::size_t s);

    /**
     * Compute the Single-Source Shortest Path (SSSP) using Dijkstra's algorithm.
     *
     * @param s The source vertex.
     * @return A vector of distances from the source vertex to all other vertices.
     *
     * Complexity: O((V + E) * log(V)), where V is the number of vertices and E is the number of edges.
     */
    std::vector<int> SSSPDijkstra(std::size_t s);

    /**
     * Compute the Single-Source Shortest Path (SSSP) using the Bellman-Ford algorithm with adjacency lists.
     *
     * @param s The source vertex.
     * @return A vector of distances from the source vertex to all other vertices.
     *
     * Complexity: O(V * E), where V is the number of vertices and E is the number of edges.
     */
    std::vector<int> SSSPBellmanFordList(std::size_t s);

    /**
     * Compute the Single-Source Shortest Path (SSSP) using the Bellman-Ford algorithm with edge list representation.
     *
     * @param s The source vertex.
     * @return A vector of distances from the source vertex to all other vertices.
     *
     * Complexity: O(V * E), where V is the number of vertices and E is the number of edges.
     */
    std::vector<int> SSSPBellmanFordEdge(std::size_t s);

    /**
     * Find the maximum flow in the graph using the Ford-Fulkerson algorithm.
     *
     * @param source The source vertex.
     * @param sink The sink (target) vertex.
     * @return The maximum flow from the source to the sink.
     *
     * Complexity: O(F * E), where F is the maximum flow and E is the number of edges.
     */
    int maxFlowFordFulkerson(std::size_t source, std::size_t sink);

    /**
     * Benchmark the maximum flow algorithm with a specified function and input vertices.
     *
     * @param algorithm A function pointer to the maximum flow algorithm to benchmark.
     * @param source The source vertex.
     * @param sink The sink (target) vertex.
     */
    void benchMarkMaxFlow(std::function<int(std::size_t, std::size_t)>, std::size_t, std::size_t);

    /**
     * Benchmark the Single-Source Shortest Path (SSSP) algorithm with a specified function and a source vertex.
     *
     * @param algorithm A function pointer to the SSSP algorithm to benchmark.
     * @param s The source vertex.
     */
    void benchMarkSSSP(std::function<std::vector<int>(std::size_t)>, std::size_t);

private: 
    /**
     * Breadth-First Search (BFS) to find a path from source to sink in the graph.
     *
     * @param s The source vertex.
     * @param t The sink (target) vertex.
     * @param parent A vector to store the parent vertices.
     * @return True if a path exists, false otherwise.
     */
    bool bfs(std::size_t s, std::size_t t, std::vector<int>& parent);

    /**
     * Convert the adjacency list to an edge list representation.
     *
     * @return A vector of edge representations (source, destination, weight).
     */
    std::vector<std::vector<int>> adjList2Edge();

private:
    std::size_t V;
    std::vector<std::vector<std::pair<std::size_t, int>>> adjList;
};

#endif   //WEIGHTED_GRAPH_ADJ_LIST_
