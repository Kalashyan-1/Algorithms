#include "../../include/GraphHeader/wgraphList.h"
WGraphList::WGraphList(std::size_t v) 
    : V{v}, adjList(V)
{    
}

WGraphList::WGraphList(const WGraphList& oth) {
    V = oth.V;
    adjList = oth.adjList;
}


WGraphList& WGraphList::operator=(const WGraphList& rhs) {
    if (this == &rhs) {return *this;}
    V = rhs.V;
    adjList = rhs.adjList;
    return *this;
}

WGraphList::WGraphList(WGraphList&& rhs) {
    V = rhs.V;
    adjList = std::move(rhs.adjList);
}

WGraphList& WGraphList::operator=(WGraphList&& rhs) {
    V = rhs.V;
    adjList = std::move(rhs.adjList);
    return *this;
}


void WGraphList::addEdge(std::size_t s, std::size_t d, int w) {
    if (s >= V || d >= V) {throw std::invalid_argument("Given sours or destination is grather then vertex degree.");}
    adjList[s].push_back({d, w});
}


void WGraphList::addVertex() {
    adjList.push_back({});
    ++V;
}


constexpr std::size_t WGraphList::size() noexcept {
    return V;
}


std::vector<std::size_t> WGraphList::topSort() {
    std::vector<int> indegree(V, 0);
    std::queue<std::size_t> qu;
    std::vector<std::size_t> res(V);
    for (int i = 0; i < V; ++i) {
        for (auto v : adjList[i]) {
            ++indegree[v.first];
        }
    }

    for (std::size_t i = 0; i < V; ++i) {
        if (indegree[i] == 0) {
            qu.push(i);
        }
    }
    int index = 0;
    while (!qu.empty()) {
        auto u = qu.front();
        qu.pop();
        res[index++] = u;
        for (auto v : adjList[u]) {
            if (--indegree[v.first] == 0) {
                qu.push(v.first);
            }
        }
    }
    if (index != V) {return {};}
    return res;
}


std::vector<int> WGraphList::SSSPDAG(std::size_t s) {
    const int INF = std::numeric_limits<int>::max();
    if (s > V) {throw std::invalid_argument("Given sours is grather then vertex degree.");}
    std::vector<int> dist(V,  INF);
    auto topSorted = topSort();
    dist[s] = 0;

    for (auto i : topSorted) {
        if (dist[i] != INF) {
            for (auto v : adjList[i]) {
                if (dist[v.first] > dist[i] + v.second) {
                    dist[v.first] = dist[i] + v.second;
                }
            }
        }
    }
    return dist;
}


std::vector<int> WGraphList::SSSPDijkstra(std::size_t s) {
    const int INF = std::numeric_limits<int>::max();
    std::vector<int> dist(V, INF);
    std::priority_queue<std::pair<std::size_t, int>, std::vector<std::pair<std::size_t, int>>, std::greater<std::pair<std::size_t, int>>> pq;

    std::vector<bool> visited(V, false);
    dist[s] = 0;
    pq.push({s, 0});

    while (!pq.empty()) {
        auto u = pq.top().first;
        auto w = pq.top().second;
        pq.pop();

        if (dist[u] < w) continue;

        visited[u] = true;

        for (auto v : adjList[u]) {
            if (!visited[v.first]) {
                if (dist[v.first] > dist[u] + v.second) {
                    dist[v.first] = dist[u] + v.second;
                    pq.push({v.first, dist[v.first]});
                }
            }
        }
    }
    return dist;    
}


std::vector<int> WGraphList::SSSPBellmanFordList(std::size_t s) {
    const int INF = std::numeric_limits<int>::max();
    std::vector<int> dist(V, INF);
    dist[s] = 0;
    for (int i = 0; i < V - 1; ++i) {
        for (int u = 0; u < V; ++u) {
            for (auto v : adjList[u]) {
                if (dist[u] != INF && dist[v.first] > dist[u] + v.second) {
                    dist[v.first] = dist[u] + v.second;
                } 
            }
        }
    }

    for (int u = 0; u < V; ++u) {
        for (auto v : adjList[u]) {
            if (dist[u] != INF && dist[v.first] > dist[u] + v.second) {
                std::cout << "Negative cycle is detected\n";
                return {};
            } 
        }
    }
    return dist;
}

std::vector<int> WGraphList::SSSPBellmanFordEdge(std::size_t s) {
    const int INF = std::numeric_limits<int>::max();
    std::vector<int> dist(V, INF);
    dist[s] = 0;
    auto edges = adjList2Edge();
    for (int i = 0; i < V - 1; ++i) {
        for (const auto& edge : edges) {
            auto u = edge[0];
            auto v = edge[1];
            auto w = edge[2];
            if (dist[u] != INF && dist[v] > dist[u] + w) {
                dist[v] = dist[u] + w;
            } 
        }
    }

    for (const auto& edge : edges) {
        auto u = edge[0];
        auto v = edge[1];
        auto w = edge[2];
        if (dist[u] != INF && dist[v] > dist[u] + w) {
            return {};
        } 
    }
    return dist;
}

std::vector<std::vector<int>> WGraphList::adjList2Edge() {
    std::vector<std::vector<int>> res;
    for (int i = 0; i < V; ++i) {
        for (auto v : adjList[i]) {
            std::vector<int> tmp;
            tmp.push_back(i);
            tmp.push_back(v.first);
            tmp.push_back(v.second);
            res.push_back(tmp);
        }
    }
    return res;
}


bool WGraphList::bfs(std::size_t s, std::size_t t, std::vector<int>& parent) {
    std::queue<std::size_t> qu;
    qu.push(s);
    while (!qu.empty()) {
        auto u = qu.front();
        qu.pop();
        for (auto edge : adjList[u]) {
            auto v = edge.first;
            auto cap = edge.second;
            if (parent[v] == -1 && cap > 0) {
                parent[v] = u;
                qu.push(v);
            } 
        }
    }
    return (parent[t] == -1);
}


int WGraphList::maxFlowFordFulkerson(std::size_t source, std::size_t sink) {
    int flow = 0;
    auto adj = adjList;
    while (true) {
    std::vector<int> parent(V, -1);
        
        if (bfs(source, sink, parent)) break;

        int bottleneck = std::numeric_limits<int>::max();
        
        for (std::size_t v = sink; v != source; v = parent[v]) {
            auto u = parent[v];
            for (const auto& edge : adjList[u]) {
                if (edge.first == v) {
                   bottleneck = (bottleneck < edge.second)? bottleneck : edge.second;
                   break;
                }
            }
        }

        flow += bottleneck;
        for (std::size_t v = sink; v != source; v = parent[v]) {
            auto u = parent[v];
            for (auto& edge : adjList[u]) {
                if (edge.first == v) {
                    edge.second -= bottleneck;
                    break;
                }
            }
            adjList[v].emplace_back(u, bottleneck);
        }
    }
    adjList = adj;
    return flow;
}


void WGraphList::benchMarkMaxFlow(std::function<int(std::size_t, std::size_t)> algorithm, std::size_t source, std::size_t sink) {
    auto start = std::chrono::steady_clock::now();
    int flow = algorithm(source, sink);
    auto end = std::chrono::steady_clock::now();
    std::cout << flow;
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Algorithm execution time: " << duration.count() << " microseconds\n";

}

void WGraphList::benchMarkSSSP(std::function<std::vector<int>(std::size_t)> algorithm, std::size_t source) {
    auto start = std::chrono::steady_clock::now();
    auto res = algorithm(source);
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Algorithm execution time: " << duration.count() << " microseconds\n";
}

