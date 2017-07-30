#ifndef NET_CODERODDE_PATHFINDING_HPP
#define NET_CODERODDE_PATHFINDING_HPP

#include "a_star.hpp"
#include "dijkstra.hpp"
#include "heuristic_function.hpp"
#include "weight_function.hpp"
#include "weighted_path.hpp"

namespace net {
namespace coderodde {
namespace pathfinding {
    
    template<typename Node, typename Weight>
    class heuristic_function_selector {
    public:
        heuristic_function_selector(
                                Node& source,
                                Node& target,
                                weight_function<Node, Weight>* weight_function)
        :
        m_source{source},
        m_target{target},
        m_weight_function{weight_function} {}
        
        weighted_path<Node, Weight> without_heuristic_function() {
            return search(m_source, m_target, *m_weight_function);
        }
        
        weighted_path<Node, Weight>
        with_heuristic_function(
                        heuristic_function<Node, Weight>* heuristic_function) {
            return search(m_source,
                          m_target,
                          *m_weight_function,
                          *heuristic_function);
        }
        
    private:
        Node m_source;
        Node m_target;
        weight_function<Node, Weight>* m_weight_function;
    };
    
    template<typename Node, typename Weight>
    class weight_function_selector {
    public:
        weight_function_selector(Node& source, Node& target) :
        m_source{source},
        m_target{target} {}
        
        heuristic_function_selector<Node, Weight>
        with_weights(weight_function<Node, Weight>* wf) {
            return heuristic_function_selector<Node, Weight>(m_source,
                                                             m_target,
                                                             wf);
        }
        
    private:
        Node m_source;
        Node m_target;
    };
    
    template<typename Node, typename Weight>
    class target_node_selector {
    public:
        target_node_selector(Node source) : m_source{source} {}
        weight_function_selector<Node, Weight> to(Node& target) {
            return weight_function_selector<Node, Weight>(m_source, target);
        }
        
    private:
        Node m_source;
    };
    
    template<typename Node, typename Weight>
    class source_node_selector {
    public:
        target_node_selector<Node, Weight> from(Node& source) {
            return target_node_selector<Node, Weight>{source};
        }
    };
    
    template<typename Node, typename Weight>
    source_node_selector<Node, Weight> find_shortest_path() {
        return source_node_selector<Node, Weight>{};
    }
    
} // End of namespace net::coderodde::pathfinding.
} // End of namespace net::coderodde.
} // End of namespace net.

#endif // NET_CODERODDE_PATHFINDING_HPP
