#ifndef forward_node_expander_h
#define forward_node_expander_h

#include "child_node_iterator.hpp"

namespace net {
namespace coderodde {
namespace pathfinding {
    
    template<typename Node>
    class forward_node_expander {
    public:
        virtual child_node_iterator<Node> begin() const = 0;
        virtual child_node_iterator<Node> end()   const = 0;
    };
}
}
}

#endif /* forward_node_expander_h */
