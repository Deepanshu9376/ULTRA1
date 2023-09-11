#include <iostream>
#include <vector>
#include <queue>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <limits>

using namespace std;

struct Edge {
    int to;
    double weight;
};

const int numNodes = 1000; //I took default count 
vector<vector<Edge>> graph(numNodes);

double heuristic(int node, int target) {
    return sqrt(pow(node - target, 2));
}

// A* Search
double astar(int source, int target) {
    vector<double> distance(numNodes, numeric_limits<double>::infinity());
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

    distance[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        int u = pq.top().second;
        double dist_u = pq.top().first;
        pq.pop();

        if (u == target) {
            return dist_u; // Found the shortest path to the target
        }

        for (const Edge& edge : graph[u]) {
            int v = edge.to;
            double weight = edge.weight;

            // A* modification: Add heuristic value to the distance
            double new_dist = dist_u + weight + heuristic(v, target);

            if (new_dist < distance[v]) {
                distance[v] = new_dist;
                pq.push({new_dist, v});
            }
        }
    }

    return numeric_limits<double>::infinity(); // No path found
}

int main() {
    srand(time(NULL));

    int numPairs = 200;
    int totalPairs = 0;
    double totalTime = 0;

    for (int i = 0; i < numPairs; i++) {
        int source = rand() % numNodes;
        int target = rand() % numNodes;

        // Ensure source and target are different
        while (source == target) {
            target = rand() % numNodes;
        }

        clock_t start = clock();
        double shortestDistance = astar(source, target);
        clock_t end = clock();

        double elapsedTime = double(end - start) / CLOCKS_PER_SEC;
        totalTime += elapsedTime;

        totalPairs++;

        cout << "Pair " << totalPairs << ": Shortest distance from " << source << " to " << target << " is " << shortestDistance << endl;
    }

    double averageRuntime = totalTime / totalPairs;
    cout << "Total runtime in seconds for " << totalPairs << " random modified A* Search: " << averageRuntime << endl;

    return 0;
}
