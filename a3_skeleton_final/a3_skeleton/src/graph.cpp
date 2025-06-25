#include "graph.h"
#include <queue>
#include <algorithm>
#include <iostream>

Graph::Graph(size_t num_nodes) : num_nodes_(num_nodes) {
    adjacency_list_.resize(num_nodes_);
}

void Graph::addEdge(size_t from, size_t to, double cost) {
    if (from >= num_nodes_ || to >= num_nodes_) {
        return;  // Invalid node indices
    }
    
    // Add bidirectional edge
    adjacency_list_[from].emplace_back(to, cost);
    adjacency_list_[to].emplace_back(from, cost);
}

const std::vector<Edge>& Graph::getEdges(size_t node) const {
    static const std::vector<Edge> empty_edges;
    if (node >= num_nodes_) {
        return empty_edges;
    }
    return adjacency_list_[node];
}

size_t Graph::getNumNodes() const {
    return num_nodes_;
}

bool Graph::hasEdge(size_t from, size_t to) const {
    if (from >= num_nodes_ || to >= num_nodes_) {
        return false;
    }
    
    const auto& edges = adjacency_list_[from];
    return std::any_of(edges.begin(), edges.end(), 
                      [to](const Edge& edge) { return edge.to == to; });
}

double Graph::getEdgeCost(size_t from, size_t to) const {
    if (!hasEdge(from, to)) {
        return std::numeric_limits<double>::infinity();
    }
    
    const auto& edges = adjacency_list_[from];
    auto it = std::find_if(edges.begin(), edges.end(), 
                          [to](const Edge& edge) { return edge.to == to; });
    
    return (it != edges.end()) ? it->cost : std::numeric_limits<double>::infinity();
}

std::vector<size_t> Graph::findShortestPath(size_t start, size_t end) const {
    if (start >= num_nodes_ || end >= num_nodes_) {
        return {};  // Invalid nodes
    }
    
    if (start == end) {
        return {start};  // Same node
    }
    
    // Dijkstra's algorithm
    std::vector<double> distances(num_nodes_, std::numeric_limits<double>::infinity());
    std::vector<size_t> previous(num_nodes_, SIZE_MAX);
    std::priority_queue<std::pair<double, size_t>, 
                       std::vector<std::pair<double, size_t>>, 
                       std::greater<std::pair<double, size_t>>> pq;
    
    distances[start] = 0.0;
    pq.push({0.0, start});
    
    while (!pq.empty()) {
        double current_dist = pq.top().first;
        size_t current_node = pq.top().second;
        pq.pop();
        
        if (current_node == end) {
            break;  // Found shortest path to end
        }
        
        if (current_dist > distances[current_node]) {
            continue;  // Already found a better path
        }
        
        for (const auto& edge : adjacency_list_[current_node]) {
            double new_dist = current_dist + edge.cost;
            
            if (new_dist < distances[edge.to]) {
                distances[edge.to] = new_dist;
                previous[edge.to] = current_node;
                pq.push({new_dist, edge.to});
            }
        }
    }
    
    // Reconstruct path
    std::vector<size_t> path;
    if (distances[end] == std::numeric_limits<double>::infinity()) {
        return {};  // No path found
    }
    
    size_t current = end;
    while (current != SIZE_MAX) {
        path.push_back(current);
        current = previous[current];
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

double Graph::calculatePathCost(const std::vector<size_t>& path) const {
    if (path.size() < 2) {
        return 0.0;
    }
    
    double total_cost = 0.0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        double edge_cost = getEdgeCost(path[i], path[i + 1]);
        if (edge_cost == std::numeric_limits<double>::infinity()) {
            return std::numeric_limits<double>::infinity();  // Invalid path
        }
        total_cost += edge_cost;
    }
    
    return total_cost;
}

std::vector<std::vector<size_t>> Graph::getAllPaths(size_t start, size_t end, size_t max_depth) const {
    std::vector<std::vector<size_t>> all_paths;
    std::vector<size_t> current_path;
    std::set<size_t> visited;
    
    if (start >= num_nodes_ || end >= num_nodes_) {
        return all_paths;  // Invalid nodes
    }
    
    current_path.push_back(start);
    visited.insert(start);
    
    findAllPathsRecursive(start, end, current_path, visited, all_paths, max_depth);
    
    return all_paths;
}

void Graph::findAllPathsRecursive(size_t current, size_t end, std::vector<size_t>& current_path, 
                                 std::set<size_t>& visited, std::vector<std::vector<size_t>>& all_paths, 
                                 size_t max_depth) const {
    if (current == end) {
        all_paths.push_back(current_path);
        return;
    }
    
    if (current_path.size() >= max_depth) {
        return;  // Prevent infinite loops
    }
    
    for (const auto& edge : adjacency_list_[current]) {
        if (visited.find(edge.to) == visited.end()) {
            current_path.push_back(edge.to);
            visited.insert(edge.to);
            
            findAllPathsRecursive(edge.to, end, current_path, visited, all_paths, max_depth);
            
            current_path.pop_back();
            visited.erase(edge.to);
        }
    }
}

void Graph::printGraphInfo(const rclcpp::Logger& logger) const {
    RCLCPP_INFO(logger, "Graph Information:");
    RCLCPP_INFO(logger, "  Number of nodes: %zu", num_nodes_);
    
    size_t total_edges = 0;
    for (size_t i = 0; i < num_nodes_; i++) {
        total_edges += adjacency_list_[i].size();
    }
    total_edges /= 2;  // Undirected graph, so each edge is counted twice
    
    RCLCPP_INFO(logger, "  Number of edges: %zu", total_edges);
    
    for (size_t i = 0; i < num_nodes_; i++) {
        std::string connections = "Node " + std::to_string(i) + " connections: ";
        for (const auto& edge : adjacency_list_[i]) {
            connections += std::to_string(edge.to) + "(cost:" + 
                          std::to_string(edge.cost) + ") ";
        }
        RCLCPP_INFO(logger, "  %s", connections.c_str());
    }
}

std::vector<std::vector<double>> Graph::getAdjacencyMatrix() const {
    std::vector<std::vector<double>> matrix(num_nodes_, 
                                           std::vector<double>(num_nodes_, 
                                           std::numeric_limits<double>::infinity()));
    
    // Set diagonal to 0 (distance from node to itself)
    for (size_t i = 0; i < num_nodes_; i++) {
        matrix[i][i] = 0.0;
    }
    
    // Fill in edge costs
    for (size_t i = 0; i < num_nodes_; i++) {
        for (const auto& edge : adjacency_list_[i]) {
            matrix[i][edge.to] = edge.cost;
        }
    }
    
    return matrix;
}