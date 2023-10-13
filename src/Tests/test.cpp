#include "../../include/TestHeader/test.h"

void Test::testConvexHullGrahamScan(std::vector<ch::Point>& input, std::vector<ch::Point>& expected) {
    ch::ConvexHull c;

    auto GrahamScanFunction = [&c](std::vector<ch::Point>& input) -> std::vector<ch::Point> {
        return c.convexHullGrahamScan(input);
    };

    auto output = GrahamScanFunction(input);

    if (output.size() != expected.size()) {
        std::cout << "Graham Scan algorithm does not pass the test." << std::endl;
        return;
    } 

    for(const auto& p : output) {
        if (std::find(expected.begin(), expected.end(), p) == expected.end()) {
            std::cout << "Graham Scan algorithm does not pass the test." << std::endl;
            return;
        }
    }

    std::cout << "Graham Scan";
    c.benchmarkConvexHullAlgorithm(GrahamScanFunction, input);

    std::cout << "Graham Scan algorithm pass the test." << std::endl;
}


void Test::testConvexHullJarvishMarch(std::vector<ch::Point>& input, std::vector<ch::Point>& expected) {
    ch::ConvexHull c;

    auto jarvisMarchFunction = [&c](std::vector<ch::Point>& input) -> std::vector<ch::Point> {
        return c.convexHullGrahamScan(input);
    };

    auto output = jarvisMarchFunction(input);

    if (output.size() != expected.size()) {
        std::cout << "Jarvish March algorithm does not pass the test." << std::endl;
        return;
    } 

    for(const auto& p : output) {
        if (std::find(expected.begin(), expected.end(), p) == expected.end()) {
            std::cout << "Jarvish March algorithm does not pass the test." << std::endl;
            return;
        }
    }

    std::cout << "Jarvish March";
    c.benchmarkConvexHullAlgorithm(jarvisMarchFunction, input);

    std::cout << "Jarvish March algorithm pass the test." << std::endl;
}


void Test::testConvexHullGrahamScan(const std::string& fileName) {
    std::filesystem::path filePath = std::filesystem::current_path().parent_path() / "src/Tests" / fileName;
    std::fstream input(filePath , std::ios::in);

    if (!input.is_open()) {
        std::cerr << "Failed to open JSON file: " << filePath << std::endl;
        return;
    }
    nlohmann::json tests;
    input >> tests;

    ch::ConvexHull convexHull; 

    for (const auto& test : tests) {
        const nlohmann::json& inputJson = test["input"];
        const nlohmann::json& expectedJson = test["expected"];

        std::vector<ch::Point> points = convexHull.deserialize(inputJson, "points");
        std::vector<ch::Point> expectedHull = convexHull.deserialize(expectedJson, "hull");

        testConvexHullGrahamScan(points, expectedHull);
    }
}


void Test::testConvexHullJarvishMarch(const std::string& fileName) {
    std::filesystem::path filePath = std::filesystem::current_path().parent_path() / "src/Tests" / fileName;
    std::fstream input(filePath , std::ios::in);

    if (!input.is_open()) {
        std::cerr << "Failed to open JSON file: " << filePath << std::endl;
        return;
    }
    nlohmann::json tests;
    input >> tests;

    ch::ConvexHull convexHull; 

    for (const auto& test : tests) {
        const nlohmann::json& inputJson = test["input"];
        const nlohmann::json& expectedJson = test["expected"];

        std::vector<ch::Point> points = convexHull.deserialize(inputJson, "points");
        std::vector<ch::Point> expectedHull = convexHull.deserialize(expectedJson, "hull");

        testConvexHullJarvishMarch(points, expectedHull);
    }
}



void Test::testSSSPDAG(WGraphList& graph, std::size_t inputSource, std::vector<int>& expected) {
    auto result = graph.SSSPDAG(inputSource);

    if (result.size() != expected.size()) {
        std::cout << "Algorithm to find SSSP for DAG does not pass the test." << std::endl;
        return;
    } 

    for(int i = 0; i < expected.size(); ++i) {
        if (result[i] != expected[i]) {
             std::cout << "Algorithm to find SSSP for DAG does not pass the test." << std::endl;
            return;
        }
    }    

    auto func = [&graph](std::size_t inputSource) {
        return graph.SSSPDAG(inputSource);
    };

    std::cout << "SSSP for DAG ";
    graph.benchMarkSSSP(func, inputSource);

    std::cout << "SSSP for DAG algorithm pass the test." << std::endl;
}


void Test::testSSSPDijkstra(WGraphList& graph, std::size_t inputSource, std::vector<int>& expected) {
    auto result = graph.SSSPDijkstra(inputSource);

    if (result.size() != expected.size()) {
        std::cout << "Dijkstra's Algorithm to find SSSP does not pass the test." << std::endl;
        return;
    } 

    for(int i = 0; i < expected.size(); ++i) {
        if (result[i] != expected[i]) {
             std::cout << "Dijkstra's Algorithm to find SSSP does not pass the test." << std::endl;
            return;
        }
    }    

    auto func = [&graph](std::size_t inputSource) {
        return graph.SSSPDijkstra(inputSource);
    };

    std::cout << "Dijkstra's ";
    graph.benchMarkSSSP(func, inputSource);

    std::cout << "Dijkstra's  algorithm pass the test." << std::endl;
}


void Test::testSSSPBellmanFordList(WGraphList& graph, std::size_t inputSource, std::vector<int>& expected) {
    auto result = graph.SSSPBellmanFordList(inputSource);

    if (result.size() != expected.size()) {
        std::cout << "Bellman-Ford's Algorithm to find SSSP in adjacency list representation does not pass the test." << std::endl;
        return;
    } 

    for(int i = 0; i < expected.size(); ++i) {
        if (result[i] != expected[i]) {
             std::cout << "Bellman-Ford's Algorithm to find SSSP in adjacency list representation does not pass the test." << std::endl;
            return;
        }
    }    

    auto func = [&graph](std::size_t inputSource) {
        return graph.SSSPBellmanFordList(inputSource);
    };

    std::cout << "Bellman-Ford's ";
    graph.benchMarkSSSP(func, inputSource);

    std::cout << "Bellman-Ford's algorithm pass the test." << std::endl;
}


void Test::testSSSPBellmanFordEdge(WGraphList& graph, std::size_t inputSource, std::vector<int>& expected) {
    auto result = graph.SSSPBellmanFordEdge(inputSource);

    if (result.size() != expected.size()) {
        std::cout << "Bellman-Ford's Algorithm to find SSSP in edge representation does not pass the test." << std::endl;
        return;
    } 

    for(int i = 0; i < expected.size(); ++i) {
        if (result[i] != expected[i]) {
             std::cout << "Bellman-Ford's Algorithm to find SSSP in edge representation does not pass the test." << std::endl;
            return;
        }
    }    

    auto func = [&graph](std::size_t inputSource) {
        return graph.SSSPBellmanFordEdge(inputSource);
    };

    std::cout << "Bellman-Ford's ";
    graph.benchMarkSSSP(func, inputSource);

    std::cout << "Bellman-Ford's algorithm pass the test." << std::endl;
}


void Test::testMaxFlowFordFulkerson(WGraphList& graph, std::size_t source, std::size_t sink, int expected) {
    auto result = graph.maxFlowFordFulkerson(source, sink);

    if (result != expected) {
        std::cout << "Ford-Fulkerson Algorithm to find maximum flow does not pass the test." << std::endl;
        return;
    }

    auto func = [&graph](std::size_t source, std::size_t sink) {
        return graph.maxFlowFordFulkerson(source, sink);
    };

    std::cout << "Ford-Fulkerson ";
    graph.benchMarkMaxFlow(func, source, sink);

    std::cout << "Ford-Fulkerson algorithm pass the test." << std::endl;
}





void Test::testSSSPDAG(const std::string& fileName) {
    std::filesystem::path filePath = std::filesystem::current_path().parent_path() / "src/Tests" / fileName;
    std::fstream input(filePath , std::ios::in);

    if (!input.is_open()) {
        std::cerr << "Failed to open JSON file: " << filePath << std::endl;
        return;
    }
    nlohmann::json tests;
    input >> tests;
    for (const auto& jsonData : tests) {
        WGraphList graph(jsonData["num_vertices"]);
        const std::vector<std::vector<int>>& edges = jsonData["edges"];
        for (const auto& edge : edges) {
            graph.addEdge(edge[0], edge[1], edge[2]);
        }
        const std::size_t source = jsonData["source"];
        std::vector<int> expected = jsonData["expected"];

        testSSSPDAG(graph, source, expected);
    }
}


void Test::testSSSPDijkstra(const std::string& fileName) {
    std::filesystem::path filePath = std::filesystem::current_path().parent_path() / "src/Tests" / fileName;
    std::fstream input(filePath , std::ios::in);

    if (!input.is_open()) {
        std::cerr << "Failed to open JSON file: " << filePath << std::endl;
        return;
    }

    nlohmann::json tests;
    input >> tests;
    for (const auto& jsonData : tests) {
        WGraphList graph(jsonData["num_vertices"]);
        const std::vector<std::vector<int>>& edges = jsonData["edges"];
        for (const auto& edge : edges) {
            graph.addEdge(edge[0], edge[1], edge[2]);
        }
        const std::size_t source = jsonData["source"];
        std::vector<int> expected = jsonData["expected"];

        testSSSPDijkstra(graph, source, expected);
    }
}


void Test::testSSSPBellmanFordList(const std::string& fileName) {
    std::filesystem::path filePath = std::filesystem::current_path().parent_path() / "src/Tests" / fileName;
    std::fstream input(filePath , std::ios::in);

    if (!input.is_open()) {
        std::cerr << "Failed to open JSON file: " << filePath << std::endl;
        return;
    }

    nlohmann::json tests;
    input >> tests;
    for (const auto& jsonData : tests) {
        WGraphList graph(jsonData["num_vertices"]);
        const std::vector<std::vector<int>>& edges = jsonData["edges"];
        for (const auto& edge : edges) {
            graph.addEdge(edge[0], edge[1], edge[2]);
        }
        const std::size_t source = jsonData["source"];
        std::vector<int> expected = jsonData["expected"];

        testSSSPBellmanFordList(graph, source, expected);
    }
}


void Test::testSSSPBellmanFordEdge(const std::string& fileName) {
    std::filesystem::path filePath = std::filesystem::current_path().parent_path() / "src/Tests" / fileName;
    std::fstream input(filePath , std::ios::in);

    if (!input.is_open()) {
        std::cerr << "Failed to open JSON file: " << filePath << std::endl;
        return;
    }

    nlohmann::json tests;
    input >> tests;
    for (const auto& jsonData : tests) {
        WGraphList graph(jsonData["num_vertices"]);
        const std::vector<std::vector<int>>& edges = jsonData["edges"];
        for (const auto& edge : edges) {
            graph.addEdge(edge[0], edge[1], edge[2]);
        }
        const std::size_t source = jsonData["source"];
        std::vector<int> expected = jsonData["expected"];

        testSSSPBellmanFordEdge(graph, source, expected);
    }

}


void Test::testMaxFlowFordFulkerson(const std::string& fileName) {
    std::filesystem::path filePath = std::filesystem::current_path().parent_path() / "src/Tests" / fileName;
    std::fstream input(filePath , std::ios::in);

    if (!input.is_open()) {
        std::cerr << "Failed to open JSON file: " << filePath << std::endl;
        return;
    }

    nlohmann::json tests;
    input >> tests;

    for (const auto& testCase : tests) {
        std::size_t source = testCase["source"];
        std::size_t sink = testCase["sink"];
        int expected = testCase["expected"];

        WGraphList graph(testCase["num_vertices"]);
        for (const auto& edge : testCase["edges"]) {
            graph.addEdge(edge[0], edge[1], edge[2]);
        }

        testMaxFlowFordFulkerson(graph, source, sink, expected);
    }
}



void Test::testKMP(const std::string& text, const std::string& pattern, std::vector<int>& expected) {
    PatternMatch p;
    auto result = p.KMP(text, pattern);

    if (result.size() != expected.size()) {
        std::cout << "KMP pattern matching algorithm does not pass the test." << std::endl;
        return;
    } 

    for(int i = 0; i < expected.size(); ++i) {
        if (result[i] != expected[i]) {
            std::cout << "KMP pattern matching algorithm does not pass the test." << std::endl;
            return;
        }
    }    

    auto func = [&p](const std::string& text, const std::string& pattern) {
        return p.KMP(text, pattern);
    };

    std::cout << "KMP ";
    p.benchmark(func, text, pattern);

    std::cout << "KMP algorithm pass the test." << std::endl;
}


void Test::testKMP(const std::string& fileName) {
    std::filesystem::path filePath = std::filesystem::current_path().parent_path() / "src/Tests" / fileName;
    std::fstream input(filePath , std::ios::in);

    if (!input.is_open()) {
        std::cerr << "Failed to open JSON file: " << filePath << std::endl;
        return;
    }
    PatternMatch p;
    nlohmann::json tests;
    input >> tests;

    for (const auto& test : tests) {
        std::string text = test["input"]["text"];
        std::string pattern = test["input"]["pattern"];
        std::vector<int> expectedIndices = test["expected"]["indices"];

        testKMP(text, pattern, expectedIndices);
    }
}


