#ifndef NET_CODERODDE_PATHFINDING_A_STAR_HPP
#define NET_CODERODDE_PATHFINDING_A_STAR_HPP

#include "child_node_iterator.hpp"
#include "heuristic_function.hpp"
#include "path_not_found_exception.hpp"
#include "weighted_path.hpp"
#include "weight_function.hpp"
#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace net {
namespace coderodde {
namespace pathfinding {
    
    template<typename Node, typename Weight>
    struct node_holder {
        Node* m_node;
        Weight m_f;
        
        node_holder(Node* node, Weight f) : m_node{node}, m_f{f} {}
    };
    
    template<typename Node, typename Weight, typename Cmp>
    void remove_and_delete_all_node_holders(
                std::priority_queue<node_holder<Node, Weight>*,
                                            std::vector<node_holder<Node, Weight>*>,
                                            Cmp>& open) {
        while (!open.empty()) {
            node_holder<Node, Weight>* current_node_holder = open.top();
            open.pop();
            delete current_node_holder;
        }
    }
    
    template<typename Node, typename Weight>
    weighted_path<Node, Weight>
    traceback_path(Node& target,
                   std::unordered_map<Node*, Node*>& parents,
                   weight_function<Node, Weight>& w) {
        std::vector<Node*> path;
        Node* current_node = &target;
        
        while (current_node) {
            path.push_back(current_node);
            current_node = parents[current_node];
        }
        
        std::reverse(path.begin(), path.end());
        
        Weight total_weight {};
        
        for (size_t i = 0; i < path.size() - 1; ++i) {
            total_weight += w(*path[i], *path[i + 1]);
        }
        
        return weighted_path<Node, Weight>(path, total_weight);
    }
    
    template<typename Node, typename Weight>
    weighted_path<Node, Weight> search(Node& source,
                                       Node& target,
                                       weight_function<Node, Weight>& w,
                                       heuristic_function<Node, Weight>& h) {

        auto cmp = [](node_holder<Node, Weight>* nh1,
                      node_holder<Node, Weight>* nh2) {
            return nh1->m_f > nh2->m_f;
        };
        
        std::priority_queue<node_holder<Node, Weight>*,
                            std::vector<node_holder<Node, Weight>*>,
                            decltype(cmp)> open(cmp);
        
        std::unordered_set<Node*> closed;
        std::unordered_map<Node*, Node*> parents;
        std::unordered_map<Node*, Weight> distances;
        
        open.push(new node_holder<Node, Weight>(&source, Weight{}));
        parents[&source] = nullptr;
        distances[&source] = Weight{};
        
        while (!open.empty()) {
            Node* current_node = open.top()->m_node;
            open.pop();
            
            if (*current_node == target) {
                remove_and_delete_all_node_holders(open);
                return traceback_path(*current_node, parents, w);
            }
            
            if (closed.find(current_node) != closed.end()) {
                continue;
            }
            
            closed.insert(current_node);
            
            for (Node& child_node : *current_node) {
                if (closed.find(&child_node) != closed.end()) {
                    continue;
                }
                
                Weight tentative_distance = distances[current_node] +
                w(*current_node, child_node);
                
                if (distances.find(&child_node) == distances.end()
                    || distances[&child_node] > tentative_distance) {
                    open.push(new node_holder<Node, Weight>(
                                        &child_node,
                                        tentative_distance + h(child_node)));
                    distances[&child_node] = tentative_distance;
                    parents[&child_node] = current_node;
                }
            }
        }
        
        remove_and_delete_all_node_holders(open);
        throw path_not_found_exception<Node>(source, target);
    }
} // End of namespace net::coderodde::pathfinding.
} // End of namespace net::coderodde.
} // End of namespace net.

#endif // NET_CODERODDE_PATHFINDING_A_STAR_HPP
