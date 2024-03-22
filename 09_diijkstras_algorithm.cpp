#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <limits>
#include <ostream>

using vertex_t = std::string;
using edges_t = std::unordered_map<vertex_t, unsigned>;
using cost_table_t = std::unordered_map<vertex_t, unsigned>;
using processed_table_t = std::unordered_map<vertex_t, bool>;
using graph_t = std::unordered_map<vertex_t, edges_t>;
using parent_graph_t = std::unordered_map<vertex_t, vertex_t>;
const auto max_cost = std::numeric_limits<unsigned>::max();

std::ostream& operator<<(std::ostream &os, const std::vector<vertex_t>& vec) {
    int first = 1;

    for (auto& v: vec) {
        if (first) {
            os << v;
            first = 0;
        } else {
            os << " -> " << v;
        }
    }

    return os;
}

std::ostream& operator<<(std::ostream &os, const cost_table_t& cost_table) {
    int first = 1;

    for (auto& p: cost_table) {
        if (first) {
            os << "{" << p.first << " = " << p.second << "}";
            first = 0;
        } else {
            os << ", {" << p.first << " = " << p.second << "}";
        }
    }

    return os;
}

parent_graph_t setup_parents_graph(graph_t& graph, vertex_t& start, vertex_t& finish) {
    parent_graph_t parents;

    for (auto& p : graph) {
        auto &v = p.first;

        if (v != start || v != finish) {
            parents[v] = start;
        }
    }

    return parents;
}

cost_table_t setup_cost_table(graph_t& graph, vertex_t& start) {
    cost_table_t table_cost;

    for (auto& p : graph) {
        auto &v = p.first;

        if (v != start) {
            table_cost[v] = max_cost;
        }
    }

    // Set real cost for nodes connected to start node
    for (auto& p : graph[start]) {
        table_cost[p.first] = p.second;
    }

    return table_cost;
}

vertex_t find_lowest_cost_node(const cost_table_t& table, processed_table_t& processed) {
    auto lowest_cost = max_cost;
    vertex_t lowest_cost_node{};

    for (const auto& pair : table) {
        const auto& node = pair.first;
        const auto& cost = pair.second;
        bool isProcessed = processed[node];

        if (isProcessed)
            continue;

        if (cost <= lowest_cost) {
            lowest_cost = cost;
            lowest_cost_node = node;
        }

    }

    return lowest_cost_node; 
}

void dijkstra_search(graph_t& graph, vertex_t& start, vertex_t& finish) {
    parent_graph_t parents = setup_parents_graph(graph, start, finish);
    cost_table_t cost_table = setup_cost_table(graph, start);
    processed_table_t processed;

    auto vertex = find_lowest_cost_node(cost_table, processed);
    while (!vertex.empty()) {
        const auto costSoFar = cost_table[vertex];
        const auto& neighbours = graph[vertex];

        // Loop through all the nodes
        for (const auto& neighbour : neighbours) {
            const auto newCost = costSoFar + neighbour.second;
            const auto& currentNeighbourName = neighbour.first;
        
            // If it is cheaper than the cost registered in the costs graph, update the costs graph
            if (newCost < cost_table[currentNeighbourName]) {
                cost_table[currentNeighbourName] = newCost;
                parents[currentNeighbourName] = vertex;
            }
        }
        
        // Mark the current node as processed
        processed[vertex] = true;
        // Find the next node to process. If they are all processed, this will return an empty string.
        vertex = find_lowest_cost_node(cost_table, processed);
    }

    std::cout << "Cost from the start to each node: " << cost_table << std::endl;
}

int main(int argc, char **argv) {
    graph_t graph;
    vertex_t start = "START";
    vertex_t a = "A";
    vertex_t b = "B";
    vertex_t finish = "FINISH";

    graph.reserve(4U);
    graph.emplace(start, edges_t{{a, 6}, {b, 2}});
    graph.emplace(a, edges_t{{finish, 1}});
    graph.emplace(b, edges_t{{a, 3},{finish, 5}});
    graph.emplace(finish, edges_t{});

    dijkstra_search(graph, start, finish);

    return 0;
}
