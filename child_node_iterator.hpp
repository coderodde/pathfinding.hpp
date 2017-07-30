#ifndef NET_CODERODDE_PATHFINDING_FORWARD_NODE_EXPANDER_HPP
#define NET_CODERODDE_PATHFINDING_FORWARD_NODE_EXPANDER_HPP

namespace net {
namespace coderodde {
namespace pathfinding {
    
    template<typename Node>
    class child_node_iterator {
        
    public:
        virtual child_node_iterator<Node>& operator++() = 0;
        virtual Node& operator*()                       = 0;
    };
    
} // End of namespace net::coderodde::pathfinding.
} // End of namespace net::coderodde.
} // End of namespace net.

#endif // End of NET_CODERODDE_PATHFINDING_FORWARD_NODE_EXPANDER_HPP.
