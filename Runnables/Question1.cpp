#include <iostream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <fstream>

using Graph = std::vector<std::vector<std::pair<int, int>>>; 

void initializeGraphAndWeightFunction(const vector<vector<Edge>>& graph) {
    
}

#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

// Define a structure to represent edges and their weights
struct Edge {
    int to;
    int weight;
    Edge(int _to, int _weight) : to(_to), weight(_weight) {}
};

// Define a custom comparison function for the priority queue
struct Compare {
    bool operator()(const Edge& a, const Edge& b) {
        return a.weight > b.weight;
    }
};

int Ddijkstra(const vector<vector<Edge>>& graph, int source, int target) {
    int n = graph.size();
    vector<int> dist(n, numeric_limits<int>::max());
    priority_queue<Edge, vector<Edge>, Compare> pq;

    dist[source] = 0;
    pq.push(Edge(source, 0));

    while (!pq.empty()) {
        Edge u = pq.top();
        pq.pop();

        if (u.to == target) {
            return dist[u.to]; // Shortest path found
        }

        for (const Edge& neighbor : graph[u.to]) {
            int alt = dist[u.to] + neighbor.weight;
            if (alt < dist[neighbor.to]) {
                dist[neighbor.to] = alt;
                pq.push(Edge(neighbor.to, alt));
            }
        }
    }

    return -1; // No path from source to target
}

int main() {
    // Initialize the Florida graph and other necessary data structures
     vector<vector<Edge>>floridaGraph;
    initializeGraphAndWeightFunction(floridaGraph);

    std::ifstream inputFile("USA-road-d.FLA.gr"); 

    if (!inputFile.is_open()) {
        std::cerr << "Error: Unable to open the input file." << std::endl;
        return 1;
    }

    // Read the graph data from the input file
    int n, m;
    inputFile >> n >> m;
    floridaGraph.resize(n);

    for (int i = 0; i < m; ++i) {
        int u, v, w;
        inputFile >> u >> v >> w;
        floridaGraph[u].emplace_back(v, w);
    }

    inputFile.close();

 
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    const int numPairs = 200;

    
    double totalRuntime = 0.0;

    
    for (int i = 0; i < numPairs; ++i) {
    
        int source = std::rand() % n; 
        int target = std::rand() % n;

        clock_t startTime = clock();

        
        int shortestPathLength = Ddijkstra(floridaGraph,source,target);

    
        clock_t endTime = clock();

        double runtimeInSeconds = static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC;

      
        totalRuntime += runtimeInSeconds;

        
        std::cout << "Shortest path length for pair " << i + 1 << ": " << shortestPathLength << std::endl;
    }

    double averageRuntime = totalRuntime / numPairs;
    std::cout << "Total runtime in seconds for 200 random Dijkstra: " << averageRuntime << std::endl;

    return 0;
}