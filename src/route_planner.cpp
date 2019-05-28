#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {

    const float scaling_factor = 0.01;
    start_x *= scaling_factor;
    start_y *= scaling_factor;
    end_x *= scaling_factor;
    end_y *= scaling_factor;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node){
    // Create vector of nodes path_found
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node next_node;
    distance = 0.0;

    while(current_node->parent != nullptr){
        path_found.push_back(*current_node);
        next_node = *(current_node->parent);
        distance += current_node->distance(next_node);
        current_node = current_node->parent;
    }
    // Make sure we include the first node (with no parent)
    path_found.push_back(*current_node);
    distance *= m_Model.MetricScale();
    return path_found;
}


float RoutePlanner::CalculateHValue(RouteModel::Node const * node) {
    return end_node->distance(*node);
}

//RouteModel::Node * RoutePlanner::NextNode(){
//    std::sort(open_list.begin(), open_list.end(), RoutePlanner::CompareFValues);
//    RouteModel::Node * next = open_list.front();
//    open_list.erase(open_list.begin());
//    return next;
//}
RouteModel::Node* RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const auto &first, const auto &second){
        return (first->GetFValue() < second->GetFValue());
    });
    RouteModel::Node * next = open_list.front();
    open_list.erase(open_list.begin());
    return next;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node * neighbour: current_node->neighbors){
        neighbour->parent = current_node;
        neighbour->g_value = current_node->g_value + current_node->distance(*neighbour);
        neighbour->h_value = CalculateHValue(neighbour);
        open_list.push_back(neighbour);
        neighbour->visited = true;
    }
}

void RoutePlanner::AStarSearch(){
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;
    while( open_list.size() > 0){
        current_node = NextNode();
        if ( current_node->distance(*end_node) == 0){
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}

//void RoutePlanner::AStarSearch() {
//    start_node->visited = true;
//    open_list.push_back(start_node);
//    std::cout << "Open list size: " << open_list.size() << "\n";
//    RouteModel::Node *current_node = nullptr;
//    current_node = NextNode();
//    AddNeighbors(current_node);
//    std::cout << "Size of 'neighbors' vector: " << current_node->neighbors.size() << "\n";
//    end_node->parent = start_node;
//    m_Model.path = ConstructFinalPath(end_node);
//}