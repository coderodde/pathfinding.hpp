#ifndef NET_CODERODDE_PATHFINDING_PATH_NOT_FOUND_EXCEPTION_HPP
#define NET_CODERODDE_PATHFINDING_PATH_NOT_FOUND_EXCEPTION_HPP

#include <sstream>
#include <stdexcept>

namespace net {
namespace coderodde {
namespace pathfinding {
    
    template<typename Node>
    class path_not_found_exception : public virtual std::logic_error {
    public:
        path_not_found_exception(const Node& source,
                                 const Node& target)
        :
        std::logic_error{""},
        m_source{&source},
        m_target{&target}
        {}
        
        const char* what() {
            std::stringstream ss;
            ss << "A path from source {" << *m_source << "} to target {"
               << *m_target << "} not found.";
            return ss.str().c_str();
        }
        
    private:
        const Node* m_source;
        const Node* m_target;
    };
    
} // End of namespace net::coderodde::pathfinding.
} // End of namespace net::coderodde.
} // End of namespace net.

#endif // NET_CODERODDE_PATHFINDING_PATH_NOT_FOUND_EXCEPTION_HPP
