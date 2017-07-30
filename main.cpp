#include "a_star.hpp"
#include "child_node_iterator.hpp"
#include "dijkstra.hpp"
#include "heuristic_function.hpp"
#include "path_not_found_exception.hpp"
#include "weight_function.hpp"
#include <cstdlib>
#include <functional>
#include <iostream>
#include <unordered_set>
#include <utility>
#include <vector>

using net::coderodde::pathfinding::child_node_iterator;
using net::coderodde::pathfinding::heuristic_function;
using net::coderodde::pathfinding::weight_function;
using net::coderodde::pathfinding::weighted_path;

// This is just a sample graph node type. The only requirement for coupling it
// with the search algorithms is 'bool operator==(const grid_node& other) const'
// and 'begin()' + 'end()' for iterating over the child nodes.
class grid_node {
private:
    
    class grid_node_neighbor_iterator : public child_node_iterator<grid_node> {
    private:
        std::vector<grid_node*>* m_neighbor_vector;
        std::size_t m_index;
        
    public:
        grid_node_neighbor_iterator(std::vector<grid_node*>* neighbor_vector,
                                    std::size_t index)
        :
        m_neighbor_vector{neighbor_vector},
        m_index{index} {}
        
        ~grid_node_neighbor_iterator() {
            delete m_neighbor_vector;
        }
        
        grid_node_neighbor_iterator& operator++() {
            ++m_index;
            return *this;
        }
        
        bool operator==(grid_node_neighbor_iterator& other) const {
            return m_index == other.m_index;
        }
        
        bool operator!=(grid_node_neighbor_iterator& other) const {
            return m_index != other.m_index;
        }
        
        grid_node& operator*() {
            return *m_neighbor_vector->at(m_index);
        }
    };
    
public:
    
    grid_node(int x, int y, bool traversable);
    
    void set_top_neighbor    (grid_node& neighbor);
    void set_bottom_neighbor (grid_node& neighbor);
    void set_left_neighbor   (grid_node& neighbor);
    void set_right_neighbor  (grid_node& neighbor);
    
    bool operator==(const grid_node& other) {
        return m_x == other.m_x && m_y == other.m_y;
    }
    
    grid_node_neighbor_iterator begin() {
        std::vector<grid_node*>* neighbor_vector =
        new std::vector<grid_node*>;
        
        if (m_top_neighbor && m_top_neighbor->m_traversable) {
            neighbor_vector->push_back(m_top_neighbor);
        }
        
        if (m_bottom_neighbor && m_bottom_neighbor->m_traversable) {
            neighbor_vector->push_back(m_bottom_neighbor);
        }
        
        if (m_left_neighbor && m_left_neighbor->m_traversable) {
            neighbor_vector->push_back(m_left_neighbor);
        }
        
        if (m_right_neighbor && m_right_neighbor->m_traversable) {
            neighbor_vector->push_back(m_right_neighbor);
        }
        
        return grid_node_neighbor_iterator(neighbor_vector, 0);
    }
    
    grid_node_neighbor_iterator end() {
        std::size_t neighbor_count = 0;
        
        if (m_top_neighbor && m_top_neighbor->m_traversable) {
            neighbor_count++;
        }
        
        if (m_bottom_neighbor && m_bottom_neighbor->m_traversable) {
            neighbor_count++;
        }
        
        if (m_left_neighbor && m_left_neighbor->m_traversable) {
            neighbor_count++;
        }
        
        if (m_right_neighbor && m_right_neighbor->m_traversable) {
            neighbor_count++;
        }
        
        return grid_node_neighbor_iterator(nullptr, neighbor_count);
    }
    
    // Heuristic function must know the coordinates:
    friend class grid_node_heuristic_function;
    
    // For printing to, say, cout:
    friend std::ostream& operator<<(std::ostream& out, const grid_node& gn);
    
    // std::hash and std::equal_to so that the internal unordered_* data
    // structures may work with grid nodes via pointers:
    friend class std::hash<grid_node*>;
    friend class std::equal_to<grid_node*>;
    
private:
    
    int m_x;
    int m_y;
    
    bool   m_traversable;
    
    grid_node* m_top_neighbor;
    grid_node* m_bottom_neighbor;
    grid_node* m_left_neighbor;
    grid_node* m_right_neighbor;
};

grid_node::grid_node(int x, int y, bool traversable)
:
m_x{x},
m_y{y},
m_traversable{traversable}
{
    m_top_neighbor    = nullptr;
    m_bottom_neighbor = nullptr;
    m_left_neighbor   = nullptr;
    m_right_neighbor  = nullptr;
}

void grid_node::set_top_neighbor(grid_node& neighbor)
{
    m_top_neighbor = &neighbor;
}

void grid_node::set_bottom_neighbor(grid_node& neighbor)
{
    m_bottom_neighbor = &neighbor;
}

void grid_node::set_left_neighbor(grid_node& neighbor)
{
    m_left_neighbor = &neighbor;
}

void grid_node::set_right_neighbor(grid_node& neighbor)
{
    m_right_neighbor = &neighbor;
}

std::ostream& operator<<(std::ostream& out, const grid_node& gn)
{
    out << "grid_node {x=" << gn.m_x << ", y=" << gn.m_y << ", traversable="
        << std::boolalpha << gn.m_traversable << "}";
    return out;
}

// This class will be used as an EDGE WEIGHT:
class matrix {
public:
    matrix(int a1, int a2, int b1, int b2)
    :
    m_a1{a1},
    m_a2{a2},
    m_b1{b1},
    m_b2{b2}
    {}
    
    matrix() : matrix{0, 0, 0, 0} {}
    
    int determinant() const {
        return m_a1 * m_b2 - m_a2 * m_b1;
    }
    
    matrix operator+(const matrix& other) {
        return matrix{m_a1 + other.m_a1,
                      m_a2 + other.m_a2,
                      m_b1 + other.m_b1,
                      m_b2 + other.m_b2};
    }
    
    matrix& operator+=(const matrix& other) {
        m_a1 += other.m_a1;
        m_a2 += other.m_a2;
        m_b1 += other.m_b1;
        m_b2 += other.m_b2;
        return *this;
    }
    
    bool operator>(const matrix& other) {
        return abs(determinant()) > abs(other.determinant());
    }
    
    friend std::ostream& operator<<(std::ostream& out, const matrix& m) {
        return out << "{{" << m.m_a1 << ", " << m.m_a2 << "}, {"
                   << m.m_b1 << ", " << m.m_b2 << "}}";
    }
    
    friend class grid_node_heuristic_function;
    friend class std::hash<grid_node*>;
    friend class std::equal_to<grid_node*>;
    
private:
    int m_a1;
    int m_a2;
    int m_b1;
    int m_b2;
};

class matrix_node {
private:
    
    class matrix_node_child_iterator :
    public child_node_iterator<matrix_node> {
    
    private:
        std::size_t m_index;
        std::vector<matrix_node*>* m_matrix_node_pointer_vector;
        
    public:
        matrix_node_child_iterator(
                        std::vector<matrix_node*>& matrix_node_pointer_vector,
                        std::size_t index)
        : m_matrix_node_pointer_vector{&matrix_node_pointer_vector},
          m_index{index} {}
        
        matrix_node_child_iterator& operator++() {
            m_index++;
            return *this;
        }
        
        bool operator!=(const matrix_node_child_iterator& other) {
            return m_index != other.m_index;
        }
        
        matrix_node& operator*() {
            return *m_matrix_node_pointer_vector->at(m_index);
        }
    };
    
public:
    
    matrix_node(size_t id) : m_id{id} {}
    
    bool operator==(const matrix_node& other) {
        return m_id == other.m_id;
    }
    
    void add_neighbor(matrix_node& neighbor) {
        m_neighbors.push_back(&neighbor);
    }
    
    matrix_node_child_iterator begin() {
        return matrix_node_child_iterator(m_neighbors, 0);
    }
    
    matrix_node_child_iterator end() {
        return matrix_node_child_iterator(m_neighbors, m_neighbors.size());
    }
    
private:
    
    friend class std::hash<matrix_node>;
    friend class std::equal_to<matrix_node>;
    friend std::ostream& operator<<(std::ostream& out, const matrix_node& n);
    
    std::vector<matrix_node*> m_neighbors;
    size_t m_id;
};

std::ostream& operator<<(std::ostream& out, const matrix_node& node) {
    return out << "{id=" << node.m_id << "}";
}

namespace std {
    template<>
    struct hash<matrix_node> {
        std::size_t operator()(const matrix_node& node) const {
            return node.m_id;
        }
    };
    
    template<>
    struct equal_to<matrix_node> {
        bool operator()(const matrix_node& a, const matrix_node& b) const {
            return a.m_id == b.m_id;
        }
    };
}

class grid_node_weight_function :
public virtual weight_function<grid_node, int>
{
public:
    int operator()(const grid_node& a, const grid_node& b) {
        return 1;
    }
};

class grid_node_heuristic_function :
public virtual heuristic_function<grid_node, int>
{
public:
    grid_node_heuristic_function(const grid_node source)
    :
    m_source{source}
    {}
    
    int operator()(const grid_node& target) const {
        // Manhattan-distance:
        return abs(m_source.m_x - target.m_x) + abs(m_source.m_y - target.m_y);
    }
    
private:
    grid_node m_source;
};

class matrix_node_weight_function :
public virtual weight_function<matrix_node, matrix>
{
public:
    std::unordered_map<matrix_node, matrix>&
    operator[](const matrix_node& node) {
        return m_map[node];
    }
    
    matrix operator()(const matrix_node& tail, const matrix_node& head) {
        return m_map[tail][head];
    }
    
private:
    std::unordered_map<matrix_node,
    std::unordered_map<matrix_node, matrix>> m_map;
};

namespace std {
    
    template<>
    struct hash<grid_node*> {
        std::size_t operator()(const grid_node* gn) const {
            return gn->m_x ^ gn->m_y;
        }
    };
    
    template<>
    struct equal_to<grid_node*> {
        bool operator()(const grid_node* a, const grid_node* b) const {
            return a->m_x == b->m_x && a->m_y == b->m_y;
        }
    };
}

int main(int argc, const char * argv[]) {
    std::vector<std::vector<int>> maze = {
        { 0, 0, 0, 1, 0, 0 },
        { 0, 1, 1, 1, 0, 0 },
        { 0, 0, 0, 1, 0, 0 },
        { 1, 1, 0, 1, 0, 0 },
        { 0, 0, 0, 1, 0, 0 },
        { 0, 1, 1, 1, 0, 0 },
        { 0, 0, 0, 0, 0, 0 },
    };
    
    std::vector<std::vector<grid_node>> grid_node_maze;
    
    for (size_t y = 0; y < maze.size(); ++y) {
        std::vector<grid_node> grid_node_maze_row;
        
        for (size_t x = 0; x < maze[y].size(); ++x) {
            grid_node_maze_row.push_back(grid_node(x, y, maze[y][x] != 1));
        }
        
        grid_node_maze.push_back(grid_node_maze_row);
    }
    
    for (size_t y = 0; y < grid_node_maze.size(); ++y) {
        for (int x = 0; x < grid_node_maze[0].size() - 1; ++x) {
            grid_node_maze[y][x].set_right_neighbor(grid_node_maze[y][x + 1]);
        }
        
        for (int x = 1; x < grid_node_maze[0].size(); ++x) {
            grid_node_maze[y][x].set_left_neighbor(grid_node_maze[y][x - 1]);
        }
    }
    
    for (size_t x = 0; x < grid_node_maze[0].size(); ++x) {
        for (int y = 0; y < grid_node_maze.size() - 1; ++y) {
            grid_node_maze[y][x].set_bottom_neighbor(grid_node_maze[y + 1][x]);
        }
        
        for (int y = 1; y < grid_node_maze.size(); ++y) {
            grid_node_maze[y][x].set_top_neighbor(grid_node_maze[y - 1][x]);
        }
    }
    
    grid_node_weight_function grid_node_wf;
    grid_node_heuristic_function grid_node_hf(grid_node_maze[6][5]);
    
    try {
        net::coderodde::pathfinding::weighted_path<grid_node, int> path
        = net::coderodde::pathfinding::
        search<grid_node, int>(grid_node_maze[0][0],
                               grid_node_maze[6][5],
                               grid_node_wf);
        std::cout << path << "\n";
        std::cout << "Final maze distance: " << path.total_weight() << "\n";
    } catch (net::coderodde::pathfinding::path_not_found_exception<grid_node>& ex) {
        std::cerr << ex.what() << "\n";
    }
    
    ////////// MATRIX DEMO ///////////
    matrix_node a{1};
    matrix_node b{2};
    matrix_node c{3};
    matrix_node d{4};
    matrix_node e{5};
    
    matrix_node_weight_function matrix_wf;
    
    matrix mab{1,   2,  3, 4};
    matrix mac{1,  -2, -3, 4};
    matrix mbc{2,   2,  1, 3};
    matrix mcd{5,  10,  7, 8};
    matrix mce{4,   0,  1, 3};
    matrix mde{1, -10,  9, 2};
    
    matrix_wf[a][b] = mab; a.add_neighbor(b);
    matrix_wf[a][c] = mac; a.add_neighbor(c);
    matrix_wf[b][c] = mbc; b.add_neighbor(c);
    matrix_wf[c][d] = mcd; c.add_neighbor(d);
    matrix_wf[c][e] = mce; c.add_neighbor(e);
    matrix_wf[d][e] = mde; d.add_neighbor(e);
    
    try {
        net::coderodde::pathfinding::weighted_path<matrix_node, matrix> path
        = net::coderodde::pathfinding::search<matrix_node, matrix>(a, e, matrix_wf);
        std::cout << path << "\n";
        std::cout << "Final matrix length: " << path.total_weight() << "\n";
    } catch (...) {
        
    }
}
