#ifndef NET_CODERODDE_PATHFINDING_WEIGHTED_PATH_HPP
#define NET_CODERODDE_PATHFINDING_WEIGHTED_PATH_HPP

#include <iostream>

namespace net {
namespace coderodde {
namespace pathfinding {
    
    template<typename Node, typename Weight>
    class weighted_path {
    public:
        weighted_path(std::vector<Node*> path_vector, Weight total_weight)
        :
        m_path_vector{path_vector},
        m_total_weight{total_weight}
        {}
        
        Node& node_at(size_t index) {
            return *m_path_vector->at(index);
        }
        
    private:
        std::vector<Node*> m_path_vector;
        Weight             m_total_weight;
        
        friend std::ostream& operator<<(std::ostream& out, weighted_path& path) {
            std::string separator{};
            out << "[";
            
            for (Node* node : path.m_path_vector) {
                out << separator;
                separator = ", ";
                out << *node;
            }
            
            return out << "]";
        }
    };
    
} // End of namespace net::coderodde::pathfinding.
} // End of namespace net::coderodde.
} // End of namespace net.

#endif // NET_CODERODDE_PATHFINDING_WEIGHTED_PATH_HPP
