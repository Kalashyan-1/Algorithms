#include "include/ConvexHullHeader/convexHull.h"
#include "include/GraphHeader/wgraphList.h"
#include "include/PatternMatchHeader/patternMatch.h"
#include "include/TestHeader/test.h"

int main() {
    Test t;
    t.testConvexHullGrahamScan("convexHullTests.json");
    t.testConvexHullJarvishMarch("convexHullTests.json");
    t.testKMP("patternMatchingTests.json");
    t.testSSSPDAG("testSSSPDAG.json");
    t.testSSSPDijkstra("testSSSPDijkstra.json");
    t.testSSSPBellmanFordList("testSSSPBellmanFord.json");
    t.testMaxFlowFordFulkerson("testMaxFlow.json");

    ch::ConvexHull c;
    c.visualize();
}
