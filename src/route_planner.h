#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance(){ return distance; }
    void AStarSearch();
    static bool CompareFValues(const RouteModel::Node * n1, const RouteModel::Node * n2){ return (n1->GetFValue() < n2->GetFValue()); }

  private:
    // Add private variables or methods declarations here.
    RouteModel &m_Model;
    RouteModel::Node * start_node;
    RouteModel::Node *end_node;
    float distance;
    std::vector<RouteModel::Node *> open_list;
    RouteModel::Node * NextNode();
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node* current_node);
    float CalculateHValue(RouteModel::Node const * node);
    void AddNeighbors(RouteModel::Node *current_node);
};
