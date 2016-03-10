#ifndef _IBFS_H__
#define _IBFS_H__

#include <stdio.h>
#include <string.h>
#include <queue>

#define IB_EXCESSES 1
#define IB_ALLOC_INIT_LEVELS 4096

class IBFSGraph
{
public:
  enum IBFSInitMode { IB_INIT_FAST, IB_INIT_COMPACT };
  IBFSGraph(IBFSInitMode initMode);
  ~IBFSGraph();
  void initSize(int numNodes, int numEdges);
  void addEdge(int nodeIndexFrom, int nodeIndexTo, double capacity, double reverseCapacity);
  void addNode(int nodeIndex, double capFromSource, double capToSink);
  struct Arc;
  void initGraph();
  double computeMaxFlow();
  double computeMaxFlow(bool allowIncrements);
  int isNodeOnSrcSide(int nodeIndex, int freeNodeValue = 0);


  struct Node;

  struct Arc
  {
    Node*    head;
    Arc*    rev;
    int      isRevResidual :1;
    double  rCap;
  };

  struct Node
  {
    bool     m_in_queue;
    int      id;
    int      isParentCurr:1;
    int      isIncremental:1;
    Arc      *firstArc;
    Arc      *parent;
    Node    *firstSon;
    Node    *nextPtr;
    int      label;  // label > 0: distance from s, label < 0: -distance from t
    double   excess;   // excess > 0: capacity from s, excess < 0: -capacity to t
  };

private:
  Arc *arcIter;
  void augment(Arc *bridge);
  template<bool sTree> int augmentPath(Node *x, double push);
  template<bool sTree> int augmentExcess(Node *x, double push);
  template<bool sTree> void augmentExcesses();
  template <bool sTree> void adoption();
  template <bool dirS> void growth();

  double computeMaxFlow(bool trackChanges, bool initialDirS);

  class ActiveList
  {
  public:
    inline ActiveList() {
      list = NULL;
      len = 0;
    }
    inline void init(int numNodes) {
      list = new Node*[numNodes];
      len = 0;
    }
    inline void init(Node **mem) {
      list = mem;
      len = 0;
    }
    inline void free() {
      if (list != NULL) {
        delete []list;
        list = NULL;
      }
    }
    inline void clear() {
      len = 0;
    }
    inline void add(Node* x) {
      list[len] = x;
      len++;
    }
    inline Node* pop() {
      len--;
      return list[len];
    }
    inline Node** getEnd() {
      return list+len;
    }
    inline static void swapLists(ActiveList *a, ActiveList *b) {
      ActiveList tmp = (*a);
      (*a) = (*b);
      (*b) = tmp;
    }
    Node **list;
    int len;
  };

  // members
  Node  *nodes, *nodeEnd;
  Arc    *arcs, *arcEnd;
  Node  **ptrs;
  int   numNodes;
  double flow;
  int topLevelS, topLevelT;
  ActiveList active0, activeS1, activeT1;
  //
  // Orphans
  //
  unsigned int uniqOrphansS, uniqOrphansT;
  template <bool sTree> inline void orphanFree(Node *x) {
    if (IB_EXCESSES && x->excess) {
      x->label = (sTree ? -topLevelT : topLevelS);
      if (sTree) activeT1.add(x);
      else activeS1.add(x);
      x->isParentCurr = 0;
    } else {
      x->label = 0;
    }
  }

  //
  // Initialization
  //
  struct TmpEdge
  {
    int    head;
    int    tail;
    double cap;
    double revCap;
  };
  struct TmpArc
  {
    TmpArc    *rev;
    double cap;
  };
  char  *memArcs;
  TmpEdge  *tmpEdges, *tmpEdgeLast;
  TmpArc  *tmpArcs;
  Node* m_orphanFirst;
  Node* m_orphanLast;
  std::queue<Node*> m_excess_queue;
  bool isInitializedGraph() {
    return memArcs != NULL;
  }
  IBFSInitMode initMode;
  int m_orphan_count;
  int m_path;
  double m_time;
  double m_time2;
  void initGraphFast();
  void initNodes();
};

inline void IBFSGraph::addNode(int nodeIndex, double capSource, double capSink)
{
  int f = nodes[nodeIndex].excess;
  if (f > 0) {
    capSource += f;
  } else {
    capSink -= f;
  }
  if (capSource < capSink) {
    flow += capSource;
  } else {
    flow += capSink;
  }
  nodes[nodeIndex].excess = capSource - capSink;
  nodes[nodeIndex].id = nodeIndex;
  nodes[nodeIndex].m_in_queue = false;
}

inline void IBFSGraph::addEdge(int nodeIndexFrom, int nodeIndexTo, double capacity, double reverseCapacity)
{
  tmpEdgeLast->tail = nodeIndexFrom;
  tmpEdgeLast->head = nodeIndexTo;
  tmpEdgeLast->cap = capacity;
  tmpEdgeLast->revCap = reverseCapacity;
  tmpEdgeLast++;

  // use label as a temporary storage
  // to count the out degree of nodes
  nodes[nodeIndexFrom].label++;
  nodes[nodeIndexTo].label++;
}

inline int IBFSGraph::isNodeOnSrcSide(int nodeIndex, int freeNodeValue)
{
  if (nodes[nodeIndex].label == 0) {
    return freeNodeValue;
  }
  return (nodes[nodeIndex].label > 0 ? 1 : 0);
}
#endif
