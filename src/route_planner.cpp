#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates. _/
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.                               _/
    
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.                                                          _/
// - Node objects have a distance method to determine the distance to another node.                                     _/

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors. _/
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value.                                   _/
// - Use CalculateHValue below to implement the h-Value calculation.                                                      _/
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true. _/

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    

    for (const auto &neighbors : current_node->neighbors){
        neighbors->g_value += current_node->distance(*neighbors);
        neighbors->h_value = CalculateHValue(neighbors);
        neighbors->parent = current_node;
        neighbors->visited = true;
        open_list.emplace_back(neighbors);
    }
                                  
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.                                   _/
// Tips:                                                                                                                  _/
// - Sort the open_list according to the sum of the h value and g value.                                                  _/
// - Create a pointer to the node in the list with the lowest sum.                                                        _/
// - Remove that node from the open_list.                                                                                 _/                                                                                                 
// - Return the pointer.                                                                                                  _/


bool Compare(const RouteModel::Node* x,const RouteModel::Node* y)
    {   
    return (x->g_value + x->h_value) < (y->g_value + y->h_value);
    }

RouteModel::Node *RoutePlanner::NextNode() 
    {
    std::sort(open_list.begin(),open_list.end(),Compare); 
    RouteModel::Node *lowest_sum = open_list.back(); 
    open_list.pop_back(); // lowest sum node is removed here
    return lowest_sum;
    }


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.                     _/
// Tips:                                                                                        
// - This method should take the current (final) node as an argument and iteratively follow the                             
//   chain of parents of nodes until the starting node is found.                                                          _/
// - For each node in the chain, add the distance from the node to its parent to the distance variable.                   _/
// - The returned vector should be in the correct order: the start node should be the first element                       _/
//   of the vector, the end node should be the last element.                                                              _/

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    while(current_node != start_node){  
    path_found.emplace_back(*current_node);  //places current_node in the path_found vector
    auto parent= *(current_node->parent);         
    distance += current_node -> distance(parent); //adds the distance from the current_node to its parent to the distance variable
    current_node = current_node->parent;     // changes current_node to parent_node of previous current_node
    }
    path_found.emplace_back(*start_node); // finally adds the start_node
    std::reverse(path_found.begin(), path_found.end());   // reverses the path to the correct order
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    
    start_node->visited = true;
    open_list.emplace_back(start_node);
    while(open_list.size() > 0) {
      current_node = NextNode();
      if (current_node == end_node) {
        m_Model.path = ConstructFinalPath(current_node);
        break;
      }
      AddNeighbors(current_node);
    }
    
//     start_node->visited = true;
//     open_list.push_back(start_node);
//     while (open_list.size() > 0) {
//     current_node = NextNode();
//     if (current_node->distance(*end_node) == 0) {
//     m_Model.path = ConstructFinalPath(current_node);
//     return;
//     }
//     AddNeighbors(current_node);
//     }

//     current_node = start_node;
//     current_node->visited = true;
//     while (current_node != end_node) {
//     AddNeighbors(current_node);
//     current_node = NextNode();
//     }
//     m_Model.path = ConstructFinalPath(current_node);
// }
    
}
