#include "QuadTree.h"
#include <cassert>
#include "GLWindow2.h"
#include <cvd/gl_helpers.h>

bool BoundingBox
::Intersects(const BoundingBox &other)
{
    if(nTopLeftX >= other.nTopLeftX + other.nW)
        return false;
    if(nTopLeftY >= other.nTopLeftY + other.nH)
        return false;
    if(nTopLeftX+nW <= other.nTopLeftX)
        return false;
    if(nTopLeftY+nH <= other.nTopLeftY)
        return false;

    return true;
}

QuadTreeNode
::QuadTreeNode(BoundingBox win_)
    : win(win_)
{
    bHasData = false;
    bVisited = false;
    children = NULL;
}

QuadTreeNode
::~QuadTreeNode()
{
    if(children != NULL)
        delete children;
}

bool QuadTreeNode
::HasChildren()
{
    if(children==NULL)
        return false;
    else
        return true;
}

QuadTreeNode::
Children::Children(BoundingBox box)
{
    double dHalfW = 0.5*box.nW;
    double dHalfH = 0.5*box.nH;

    nW = new QuadTreeNode(BoundingBox(box.nTopLeftX,
                                      box.nTopLeftY,
                                      dHalfW,
                                      dHalfH));
    nE = new QuadTreeNode(BoundingBox(box.nTopLeftX + dHalfW,
                                      box.nTopLeftY,
                                      dHalfW,
                                      dHalfH));
    sW = new QuadTreeNode(BoundingBox(box.nTopLeftX,
                                      box.nTopLeftY + dHalfH,
                                      dHalfW,
                                      dHalfH));
    sE = new QuadTreeNode(BoundingBox(box.nTopLeftX + dHalfW,
                                      box.nTopLeftY + dHalfH,
                                      dHalfW,
                                      dHalfH));
}

QuadTreeNode::Children::~Children()
{
    delete nW;
    delete nE;
    delete sW;
    delete sE;
}

void QuadTreeNode::Children
::Insert(TooN::Vector<2> v2, int idx, BoundingBox box)
{
    TooN::Vector<2> v2RelPos;
    v2RelPos[0] = (v2[0] - box.nTopLeftX) / box.nW;
    v2RelPos[1] = (v2[1] - box.nTopLeftY) / box.nH;

    if(v2RelPos[0]<=0.5 && v2RelPos[1]<=0.5)
        nW->Insert(v2,idx);
    else if(v2RelPos[0]>0.5 && v2RelPos[1]<=0.5)
        nE->Insert(v2,idx);
    else if(v2RelPos[0]<=0.5 && v2RelPos[1]>0.5)
        sW->Insert(v2,idx);
    else if(v2RelPos[0]>0.5 && v2RelPos[1]>0.5)
        sE->Insert(v2,idx);
}

void QuadTreeNode::Children
::Query(const BoundingBox &window,
        std::list<QuadTreeElement> &elemlist)
{
    if(nW->win.Intersects(window))
        nW->Query(window,elemlist);
    if(nE->win.Intersects(window))
        nE->Query(window,elemlist);
    if(sW->win.Intersects(window))
        sW->Query(window,elemlist);
    if(sE->win.Intersects(window))
        sE->Query(window,elemlist);
}

void QuadTreeNode
::Insert(TooN::Vector<2> v2, int idx_)
{
    if(bHasData==false)
    {
        elem.v2pos = v2;
        elem.idx = idx_;
        bHasData=true;
    }
    else
    {
        if(HasChildren())
        {
            TooN::Vector<2> v2RelPos;
            v2RelPos[0] = (double)(v2[0] - win.nTopLeftX) / win.nW;
            v2RelPos[1] = (double)(v2[1] - win.nTopLeftY) / win.nH;

            if(v2RelPos[0]<=0.5 && v2RelPos[1]<=0.5)
                children->nW->Insert(v2,idx_);
            else if(v2RelPos[0]>0.5 && v2RelPos[1]<=0.5)
                children->nE->Insert(v2,idx_);
            else if(v2RelPos[0]<=0.5 && v2RelPos[1]>0.5)
                children->sW->Insert(v2,idx_);
            else if(v2RelPos[0]>0.5 && v2RelPos[1]>0.5)
                children->sE->Insert(v2,idx_);
        }
        else
        {
            if(norm(elem.v2pos - v2) < 1.0)
            {
                return;
            }

            children = new Children(win);
            children->Insert(elem.v2pos,elem.idx,win);
            children->Insert(v2,idx_,win);
        }
    }
}

void QuadTreeNode
::Query(const BoundingBox& window,
            std::list<QuadTreeElement> &elemlist)
{
    if(HasChildren())
        children->Query(window,elemlist);
    else
    {
        if(bHasData)
        {
            if(elem.v2pos[0] > window.nTopLeftX &&
               elem.v2pos[1] > window.nTopLeftY &&
               elem.v2pos[0] < window.nTopLeftX+window.nW &&
               elem.v2pos[1] < window.nTopLeftY+window.nH)
                elemlist.push_back(elem);
        }
    }
}

void QuadTreeNode
::Traverse(std::list<BoundingBox> &boxlist,std::list<QuadTreeElement> &elemlist)
{
    if(HasChildren())
    {
        boxlist.push_back(win);
        children->nE->Traverse(boxlist,elemlist);
        children->nW->Traverse(boxlist,elemlist);
        children->sE->Traverse(boxlist,elemlist);
        children->sW->Traverse(boxlist,elemlist);
    }
    else
    {
        boxlist.push_back(win);
        if(bHasData)
            elemlist.push_back(elem);
    }
}

QuadTree
::QuadTree(int nSizeX, int nSizeY)
    : root(BoundingBox(0,0,nSizeX,nSizeY))
{
}

void QuadTree::
Insert(TooN::Vector<2> v2, int idx_)
{
    root.Insert(v2, idx_);
}

void QuadTree::
Query(const BoundingBox &window,
      std::list<QuadTreeElement> &elemlist)
{
    root.Query(window, elemlist);
}

void QuadTree::
Traverse(std::list<BoundingBox> &boxlist,std::list<QuadTreeElement> &elemlist)
{
    root.Traverse(boxlist,elemlist);
}

void QuadTree::
Visualize(CVD::Image<CVD::byte> &im)
{
    std::list<BoundingBox> boxlist;
    std::list<QuadTreeElement> elemlist;
    Traverse(boxlist,elemlist);

    GLWindow2 glw(CVD::ImageRef(root.win.nW,root.win.nH),"Quadtree");
    glClearColor(1,1,1,1);
    glClear(GL_COLOR_BUFFER_BIT);
    glw.SetupWindowOrtho(CVD::ImageRef(root.win.nW,root.win.nH));
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glDrawPixels(im);

    glColor3f(0,0,1);
    for(std::list<BoundingBox>::iterator it = boxlist.begin();
        it != boxlist.end(); ++it)
    {
        glBegin(GL_LINES);
        glVertex2d(it->nTopLeftX,it->nTopLeftY);
        glVertex2d(it->nTopLeftX+it->nW,it->nTopLeftY);

        glVertex2d(it->nTopLeftX,it->nTopLeftY);
        glVertex2d(it->nTopLeftX,it->nTopLeftY+it->nH);

        glVertex2d(it->nTopLeftX,it->nTopLeftY+it->nH);
        glVertex2d(it->nTopLeftX+it->nW,it->nTopLeftY+it->nH);

        glVertex2d(it->nTopLeftX+it->nW,it->nTopLeftY);
        glVertex2d(it->nTopLeftX+it->nW,it->nTopLeftY+it->nH);
        glEnd();
    }

    glColor3f(1,0,0);
    glPointSize(2);
    for(std::list<QuadTreeElement>::iterator it = elemlist.begin();
        it != elemlist.end(); ++it)
    {
        glBegin(GL_POINTS);
        CVD::glVertex(it->v2pos);
        glEnd();
    }
    glPointSize(1);

    glw.swap_buffers();
    std::cin.get();
}

QuadTree::BFSIter::BFSIter(const BFSIter& bfs_it)
    : reached_end_(bfs_it.reached_end_)
{
}

QuadTree::BFSIter::BFSIter(QuadTreeNode *node_ptr_)
{
    reached_end_=false;
    node_queue.push(node_ptr_);
    operator++();
}

QuadTree::BFSIter&
QuadTree::BFSIter::operator++()
{
    bool bReachedNextElem=false;

    while(!bReachedNextElem && !reached_end_)
    {
        if(node_queue.size() > 0)
        {
            node_ptr = node_queue.front();
            node_queue.pop();

            if(node_ptr->HasChildren())
            {
                node_queue.push(node_ptr->children->nE);
                node_queue.push(node_ptr->children->nW);
                node_queue.push(node_ptr->children->sE);
                node_queue.push(node_ptr->children->sW);
            }
            else
                bReachedNextElem = node_ptr->bHasData;
        }
        else
            reached_end_ = true;
    }
    return *this;
}

QuadTree::BFSIter
QuadTree::BFSIter::operator++(int)
{
    BFSIter tmp(*this);
    operator++();
    return tmp;
}

bool
QuadTree::BFSIter::IsQueueEmpty()
{
    return node_queue.empty();
}

bool
QuadTree::BFSIter::operator!=(const BFSIter& rhs)
{
    return node_ptr != rhs.node_ptr;
}

bool
QuadTree::BFSIter::reached_end()
{
    return reached_end_;
}

QuadTreeElement&
QuadTree::BFSIter::operator*()
{
    return node_ptr->elem;
}

QuadTreeNode*
QuadTree::EquiIter::GetRandomChild(QuadTreeNode* parent, int idx)
{

    QuadTreeNode* child;
    switch(idx)
    {
    case 0: {child = parent->children->nE; break;}
    case 1: {child = parent->children->nW; break;}
    case 2: {child = parent->children->sE; break;}
    case 3: {child = parent->children->sW; break;}
    }

    return child;
}

void
QuadTree::EquiIter::AddToList(int level)
{
    if(node_map.count(level)==0)
        node_map.insert(std::pair<int,std::list<QuadTreeNode*> >(level,std::list<QuadTreeNode*>()));

    std::vector<int> indices_vec;
    indices_vec.reserve(4);
    for(int i=0; i<4; ++i)
        indices_vec.push_back(i);
    std::random_shuffle(indices_vec.begin(),indices_vec.end());

    for(size_t i = 0; i < indices_vec.size(); ++i)
    {
        QuadTreeNode* child = GetRandomChild(node_ptr, indices_vec[i]);
        node_map.at(level).push_back(child);
    }
}

void
QuadTree::EquiIter::AddToStack(std::stack<QuadTreeNode*> &node_stack_,
                               QuadTreeNode* node_ptr_)
{
    std::vector<int> indices_vec;
    indices_vec.reserve(4);
    for(int i=0; i<4; ++i)
        indices_vec.push_back(i);
    std::random_shuffle(indices_vec.begin(),indices_vec.end());

    for(size_t i = 0; i < indices_vec.size(); ++i)
    {
        QuadTreeNode* child = GetRandomChild(node_ptr_, indices_vec[i]);
        node_stack_.push(child);
    }
}

bool
QuadTree::EquiIter::DFS()
{
    std::stack<QuadTreeNode*> node_stack;
    AddToStack(node_stack, node_ptr);

    while(node_stack.size() > 0)
    {
        QuadTreeNode* dfs_node_ptr = node_stack.top();
        node_stack.pop();
        if(dfs_node_ptr->HasChildren())
            AddToStack(node_stack, dfs_node_ptr);
        else
        {
            if(!dfs_node_ptr->bVisited)
            {
                dfs_node_ptr->bVisited = true;
                if(dfs_node_ptr->bHasData)
                {
                    node_ptr = dfs_node_ptr;
                    return true;
                }
            }
        }
    }
    return false;
}

QuadTree::EquiIter::EquiIter(const EquiIter &bfs_it)
    : node_ptr(bfs_it.node_ptr),
      node_map(bfs_it.node_map),
      reached_end_(bfs_it.reached_end_)
{
}

QuadTree::EquiIter::EquiIter(QuadTreeNode* node_ptr_)
{
    node_map[0].push_back(node_ptr_);
    reached_end_ = false;
    operator++();
}

QuadTree::EquiIter&
QuadTree::EquiIter::operator++()
{
    bool bReachedNextElem = false;
    while(!bReachedNextElem && !reached_end_)
    {
        if(node_map.size() > 0)
        {
            int level = node_map.begin()->first;
            if(node_map[level].size()==0)
            {
                node_map.erase(level);
                continue;
            }

            node_ptr = node_map[level].front();
            node_map[level].pop_front();
            if(node_ptr->HasChildren())
            {
                AddToList(level+1);

                // Perform DFS with random child traversal to return single entry
                bool bDFSsuccessful = DFS();
                if(bDFSsuccessful)
                    return *this;
            }
            else
            {
                if(!node_ptr->bVisited)
                {
                    bReachedNextElem = node_ptr->bHasData;
                    node_ptr->bVisited = true;
                }
            }
        }
        else
            reached_end_ = true;
    }
    return *this;
}

QuadTree::EquiIter
QuadTree::EquiIter::operator++(int)
{
    QuadTree::EquiIter tmp(*this);
    operator++();
    return tmp;
}
