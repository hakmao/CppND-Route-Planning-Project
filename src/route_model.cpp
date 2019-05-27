#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    int node_index = 0;
    for(Model::Node node: this->Nodes()){
        m_Nodes.push_back(Node(node_index, this, node));
        node_index++;
    }
    CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap(){
    for(const Model::Road &road: Roads()){
        if (road.type != Model::Road::Type::Footway){
            for (auto node_idx: Ways()[road.way].nodes){
                if (node_to_road.find(node_idx) == node_to_road.end()){
                    node_to_road[node_idx] = std::vector<const Model::Road *>();
                }
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

RouteModel::Node* RouteModel::Node::FindNeighbour(std::vector<int> node_indices){
    Node* closest_node = nullptr;
    Node node;
    for (int node_idx: node_indices){
        node = parent_model->SNodes()[node_idx];
        if (!node.visited && distance(node) != 0.0){
            if (closest_node == nullptr || distance(node) < distance(*closest_node)){
                closest_node = &node;
            }
        }
    }
    return closest_node;
}

void RouteModel::Node::FindNeighbours(){
    auto roads = parent_model->node_to_road[this->index];
    // 'roads' is of type std::vector<const Model::Road*>>, so each 'road' is of type const Model::Road*
    // we need the set of indices related to each road, in order to use FindNeighbour().
    // FindNeighbour() returns a pointer to the closest node (if any)
    std::vector<int> node_indices;
    RouteModel::Node * p_closest;
    for(auto & road: roads){
        // Each Model::Way object has an attribute '.nodes', a vector of node indices.
        // So, for a given road object, we can find all the node indices related to it with Ways()[road.way].nodes
        node_indices = parent_model->Ways()[road->way].nodes;
        // Now we can use FindNeighbour()
        p_closest = this->FindNeighbour(node_indices);
        if (p_closest){
            this->neighbours.push_back(p_closest);
        }
    }
}

RouteModel::Node & RouteModel::FindClosestNode(double x, double y){
    RouteModel::Node target_node;
    target_node.x = x;
    target_node.y = y;
    float min_dist = std::numeric_limits<float>::max();
    float new_dist;
    int closest_idx;
    Node node;
    for (auto &road: this->Roads()){
        if (road.type != Model::Road::Type::Footway){
            for (int node_idx: Ways()[road.way].nodes){
                node = SNodes()[node_idx];
                new_dist = node.distance(target_node);
                if ( new_dist < min_dist){
                    min_dist = new_dist;
                    closest_idx = node_idx;
                }
            }
        }
    }
    return SNodes()[closest_idx];
}