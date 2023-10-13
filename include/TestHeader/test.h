#ifndef TEST_H
#define TEST_H

#include "../ConvexHullHeader/convexHull.h"
#include "../GraphHeader/wgraphList.h"
#include "../PatternMatchHeader/patternMatch.h"

/**
 * @class Test
 * Provides methods for testing various algorithms and data structures.
 */
class Test {
public:
    Test() = default;
    ~Test() = default;

    /**
     * Test the Graham Scan algorithm for computing the convex hull of a set of points.
     *
     * @param input A vector of Point objects representing the input points.
     * @param expected A vector of Point objects representing the expected convex hull vertices.
     */
    void testConvexHullGrahamScan(std::vector<ch::Point>& input, std::vector<ch::Point>& expected);

    /**
     * Test the Jarvis March algorithm for computing the convex hull of a set of points.
     *
     * @param input A vector of Point objects representing the input points.
     * @param expected A vector of Point objects representing the expected convex hull vertices.
     */
    void testConvexHullJarvishMarch(std::vector<ch::Point>& input, std::vector<ch::Point>& expected);

    /**
     * Test the Graham Scan algorithm for computing the convex hull from a file.
     *
     * @param fileName The name of the file containing the input points and expected result.
     */
    void testConvexHullGrahamScan(const std::string& fileName);

    /**
     * Test the Jarvis March algorithm for computing the convex hull from a file.
     *
     * @param fileName The name of the file containing the input points and expected result.
     */
    void testConvexHullJarvishMarch(const std::string& fileName);

    /**
     * Test the Single-Source Shortest Path (SSSP) in a Directed Acyclic Graph (DAG).
     *
     * @param graph The graph for which to compute SSSP.
     * @param inputSource The source vertex for SSSP.
     * @param expected A vector of expected distances from the source vertex to all other vertices.
     */
    void testSSSPDAG(WGraphList&, std::size_t inputSource, std::vector<int>& expected);

    /**
     * Test the Single-Source Shortest Path (SSSP) using Dijkstra's algorithm.
     *
     * @param graph The graph for which to compute SSSP.
     * @param inputSource The source vertex for SSSP.
     * @param expected A vector of expected distances from the source vertex to all other vertices.
     */
    void testSSSPDijkstra(WGraphList&, std::size_t inputSource, std::vector<int>& expected);

    /**
     * Test the Single-Source Shortest Path (SSSP) using the Bellman-Ford algorithm with adjacency lists.
     *
     * @param graph The graph for which to compute SSSP.
     * @param inputSource The source vertex for SSSP.
     * @param expected A vector of expected distances from the source vertex to all other vertices.
     */
    void testSSSPBellmanFordList(WGraphList&, std::size_t inputSource, std::vector<int>& expected);

    /**
     * Test the Single-Source Shortest Path (SSSP) using the Bellman-Ford algorithm with edge list representation.
     *
     * @param graph The graph for which to compute SSSP.
     * @param inputSource The source vertex for SSSP.
     * @param expected A vector of expected distances from the source vertex to all other vertices.
     */
    void testSSSPBellmanFordEdge(WGraphList&, std::size_t inputSource, std::vector<int>& expected);

    /**
     * Test the Single-Source Shortest Path (SSSP) in a Directed Acyclic Graph (DAG) from a file.
     *
     * @param fileName The name of the file containing the graph edge representation, source vertex and expected result.
     */
    void testSSSPDAG(const std::string& fileName);

    /**
     * Test the Single-Source Shortest Path (SSSP) using Dijkstra's algorithm from a file.
     *
     * @param fileName The name of the file containing the graph edge representation, source vertex and expected result.
     */
    void testSSSPDijkstra(const std::string& fileName);

    /**
     * Test the Single-Source Shortest Path (SSSP) using the Bellman-Ford algorithm with adjacency lists from a file.
     *
     * @param fileName The name of the file containing the graph edge representation, source vertex and expected result.
     */
    void testSSSPBellmanFordList(const std::string& fileName);

    /**
     * Test the Single-Source Shortest Path (SSSP) using the Bellman-Ford algorithm with edge list representation from a file.
     *
     * @param fileName The name of the file containing the graph edge representation, source vertex and expected result.
     */
    void testSSSPBellmanFordEdge(const std::string& fileName);

    /**
     * Test the Ford-Fulkerson algorithm for finding the maximum flow in a graph.
     *
     * @param graph The graph for which to compute the maximum flow.
     * @param source The source vertex for the maximum flow.
     * @param sink The sink (target) vertex for the maximum flow.
     * @param expected The expected maximum flow value.
     */
    void testMaxFlowFordFulkerson(WGraphList&, std::size_t source, std::size_t sink, int expected);

    /**
     * Test the Ford-Fulkerson algorithm for finding the maximum flow from a file.
     *
     * @param fileName The name of the file containing the graph edge representation, source and sink vertices, and expected result.
     */
    void testMaxFlowFordFulkerson(const std::string& fileName);

    /**
     * Test the Knuth-Morris-Pratt (KMP) string matching algorithm.
     *
     * @param text The text in which to search for the pattern.
     * @param pattern The pattern to search for within the text.
     * @param expected A vector of expected starting indices of pattern occurrences in the text.
     */
    void testKMP(const std::string& text, const std::string& pattern, std::vector<int>& expected);

    /**
     * Test the Knuth-Morris-Pratt (KMP) string matching algorithm from a file.
     *
     * @param fileName The name of the file containing the text, pattern, and expected indices.
     */
    void testKMP(const std::string& fileName);
};

#endif  //TEST_H
