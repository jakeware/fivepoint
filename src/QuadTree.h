#ifndef QUADTREE_H
#define QUADTREE_H

#include <TooN/TooN.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <iterator>
#include <queue>
#include <list>
#include <map>
#include <stack>
#include <algorithm>

// Contains region quad tree objects:
// BoundingBox, QuadTreeElement, QuadTreeNode, QuadTree
// QuadTree has two iterator classes to traverse through
// the tree from the root, BFSIter and EquiIter

// A rectangle, represents the region
// occupied by a node in the quad tree
struct BoundingBox
{
    BoundingBox(){}

    BoundingBox(double nTopLeftX_, double nTopLeftY_, double nW_, double nH_)
    {
        nTopLeftX = nTopLeftX_;
        nTopLeftY = nTopLeftY_;
        nW = nW_;
        nH = nH_;
    }

    bool
    Intersects(const BoundingBox& other);

    double nTopLeftX;   // top left corner, x-position
    double nTopLeftY;   // top left corner, y-position
    double nW;          // width
    double nH;          // height
};

// Information contained in a quad tree leaf
struct QuadTreeElement
{
    TooN::Vector<2> v2pos;  // xy position of feature
    int idx;                // index of feature
};

class QuadTreeNode
{
public:

    ~QuadTreeNode();

    QuadTreeNode(){}

    QuadTreeNode(BoundingBox win_);

    // Inserts a new data point.
    //
    // If this node has no data and
    // is the leaf, then the new data point will be added.
    //
    // If the node already has data, then subdivide the
    // region into 4 children, and pass the new and existing
    // data points to the appropriate children.
    void
    Insert(TooN::Vector<2> v2, int idx_);

    void
    Query(const BoundingBox &window,
          std::list<QuadTreeElement> &elemlist);

    bool
    HasChildren();

    // Returns the bounding box. If this node is
    // a leaf, its element is returned as well. Otherwise,
    // the node's children are traversed, and this goes on
    // recursively until you find a leaf node.
    void
    Traverse(std::list<BoundingBox> &boxlist,
             std::list<QuadTreeElement> &elemlist);

//private:

    BoundingBox win;

    // A quadtree node can be subdivided
    // into 4 equally sized region nodes, and
    // these nodes are stored in this Children class
    struct Children
    {
        Children(BoundingBox box);

        void
        Insert(TooN::Vector<2> v2, int idx, BoundingBox box);

        void
        Query(const BoundingBox &window,
              std::list<QuadTreeElement> &elemlist);

        ~Children();

        QuadTreeNode* nW;
        QuadTreeNode* nE;
        QuadTreeNode* sW;
        QuadTreeNode* sE;
    };

    Children *children;
    QuadTreeElement elem;
    bool bHasData;          // Does this node have data?
    bool bVisited;          // Has this node been previously visited?
};

class QuadTree
{
public:
    QuadTree(int nSizeX, int nSizeY);

    // Inserts a new data point into the quadtree.
    // If this new data point's distance to the nearest
    // existing data point in the tree < threshold, it
    // won't be added to the tree. Here v2 is the xy position
    // and idx_ is the index of the keypoint in the vector in
    // which it is stored in.
    //
    // Starting from the root, it keeps recursing through the
    // tree until it finds a leaf and inserts the point there.
    void
    Insert(TooN::Vector<2> v2, int idx_);

    // Given a bounding box, returns all elements that lie
    // within the bounding box
    void
    Query(const BoundingBox &window,
          std::list<QuadTreeElement> &elemlist);

    // Returns a list of bounding boxes and elements
    // in the tree. Used in conjunction with the Visualize()
    // function so that the quad tree can be visualized.
    void
    Traverse(std::list<BoundingBox> &boxlist,
             std::list<QuadTreeElement> &elemlist);

    // Draws the quad tree.
    void
    Visualize(CVD::Image<CVD::byte> &im);

    QuadTreeNode* begin(){return &root;}

    // Traverses quadtree from its root using breadth-first search algorithm.
    class BFSIter: public std::iterator<std::forward_iterator_tag,QuadTreeElement>
    {
    private:
        QuadTreeNode* node_ptr;
        std::queue<QuadTreeNode*> node_queue;
        bool reached_end_;

    public:

        BFSIter(const BFSIter& bfs_it);

        BFSIter(QuadTreeNode* node_ptr_);

        bool IsQueueEmpty();

        bool reached_end();

        BFSIter& operator++();

        BFSIter operator++(int);

        bool operator!=(const BFSIter& rhs);

        QuadTreeElement& operator*();
    };

    // Traverses quadtree from its root using a hybrid of breadth-first search
    // and depth-first search. Starts off with BFS, with the difference that
    // if it is not a leaf node, it performs DFS until a leaf with data point is
    // found. Another difference is that the children are added in random order
    class EquiIter: public std::iterator<std::forward_iterator_tag,QuadTreeElement>
    {
    private:

        QuadTreeNode* node_ptr;
        std::map<int, std::list<QuadTreeNode*> > node_map;
        bool reached_end_;

        QuadTreeNode* GetRandomChild(QuadTreeNode* parent, int idx);

        void AddToList(int level);

        void AddToStack(std::stack<QuadTreeNode*> &node_stack_,
                        QuadTreeNode* node_ptr_);

        bool DFS();

    public:

        EquiIter(QuadTreeNode* node_ptr_);

        EquiIter(const EquiIter &bfs_it);

        EquiIter& operator++();

        EquiIter operator++(int);

        bool reached_end(){return reached_end_;}

        QuadTreeElement& operator*(){return node_ptr->elem;}
    };

private:

    QuadTreeNode root;
};

#endif // QUADTREE_H
