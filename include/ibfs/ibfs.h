#ifndef _IBFS_H__
#define _IBFS_H__

#include <stdio.h>
#include <string.h>
#include <algorithm>
#include "include/CountTime.h"

#define IBDEBUG(X) fprintf(stdout, X"\n"); fflush(stdout)
#define IB_ORPHANS_END   ( (Node *) 1 )

// #define ENABLE_DYNAMIC_EDGE

#define HALF_NEIGHBOUR 4
#define NEIGHBOUR 8
#define HALF_NEIGHBOUR_ARR_INDEX FOUR_ARR_INDEX
#define NEIGHBOUR_ARR_INDEX EIGHT_ARR_INDEX
typedef double (*EPF)(int src_node_colour, int dst_node_colour);

const double ibgraph_div_sqrt2 = 1 / sqrt(2.0f);

#define FOUR_ARR_INDEX(node_x, node_y, image_width, image_height) \
{ \
  node_y * image_width + std::max(0, node_x - 1), \
  std::max(node_y - 1, 0) * image_width + std::max(0, node_x - 1), \
  std::max(node_y - 1, 0) * image_width + node_x, \
  std::max(node_y - 1, 0) * image_width + std::min(image_width - 1, node_x + 1) \
}

#define EIGHT_ARR_INDEX(node_x, node_y, image_width, image_height) \
{ \
  node_y * image_width + std::max(0, node_x - 1), \
  std::max(node_y - 1, 0) * image_width + std::max(0, node_x - 1), \
  std::max(node_y - 1, 0) * image_width + node_x, \
  std::max(node_y - 1, 0) * image_width + std::min(image_width - 1, node_x + 1), \
  node_y * image_width + std::min(image_width - 1, node_x + 1), \
  std::min(node_y + 1, image_height - 1) * image_width + std::min(image_width - 1, node_x + 1), \
  std::min(node_y + 1, image_height - 1) * image_width + node_x, \
  std::min(node_y + 1, image_height - 1) * image_width + std::max(0, node_x - 1) \
}

class IBFSGraph
{
public:
  IBFSGraph();
  ~IBFSGraph();
  void initSize(int numNodes, int numEdges);
  void addEdge(int nodeIndexFrom, int nodeIndexTo, double capacity, double reverseCapacity);
  void addNode(int nodeIndex, double capacityFromSource, double capacityToSink);
  void initGraph();
  double computeMaxFlow();
  bool isNodeOnSrcSide(int nodeIndex);
#ifdef ENABLE_DYNAMIC_EDGE
  void initSize(int numNodes, int width, int height, EPF epf);
  void addNode(int nodeIndex, double capacityFromSource, double capacityToSink, int node_colour);
  void AddActiveNodes(int node_x, int node_y);
#endif

  // inline int getFlow() {
  //   return m_flow;
  // }
  // inline int getNumNodes() {
  //   return m_nodeEnd-m_nodes;
  // }
  // inline int getNumArcs() {
  //   return m_arcEnd-m_arcs;
  // }

private:
  struct Node;
  struct Arc;

  struct Arc
  {
    Node*    head;
    Arc*    rev;
    bool    isRevResidual;
    double  rCap;
  };

  class Node
  {
  public:
    Arc      *firstArc;
    Arc      *parent;
    Node    *firstSon;
    Node    *nextPtr;
    int      label;  // label > 0: distance from s, label < 0: -distance from t
    double  excess;   // excess > 0: capacity from s, excess < 0: -capacity to t

    bool m_is_active;
    int m_node_idx;
#ifdef ENABLE_DYNAMIC_EDGE
    Arc m_out_edges[NEIGHBOUR];
    bool m_is_gotten_all_edges;
    int m_out_edges_num;
    int m_node_colour;
#endif

  };

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
    inline void free() {
      if (list != NULL) {
        delete list;
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
    inline static void swapLists(ActiveList *a, ActiveList *b) {
      ActiveList tmp = (*a);
      (*a) = (*b);
      (*b) = tmp;
    }
    Node **list;
    int len;
  };

  // members
  Node  *m_nodes, *m_nodeEnd;
  Arc    *m_arcs, *m_arcEnd;
  int   m_numNodes;
  double m_flow;
  double m_time;
  int m_count;
  CountTime m_ct;
  int m_orphan_count;
  int m_path;
  unsigned int m_uniqOrphansS, m_uniqOrphansT;
  Node* m_orphanFirst;
  Node* m_orphanLast;
  int m_topLevelS, m_topLevelT;
  ActiveList m_active0, m_activeS1, m_activeT1;

  void augment(Arc *bridge);
  template<bool sTree> void augmentTree(Node *x, double bottleneck);
  template <bool sTree> void adoption();
  template <bool dirS> void growth();


  //
  // Initialization
  //
  struct TmpEdge
  {
    Node*    head;
    Node*    tail;
    double   cap;
    double  revCap;
  };
  struct TmpArc
  {
    TmpArc    *rev;
    double cap;
  };
  char  *m_memArcs;
  TmpEdge  *m_tmpEdges, *m_tmpEdgeLast;
  TmpArc  *m_tmpArcs;
  void initGraphFast();
  void initGraphCompact();
#ifdef ENABLE_DYNAMIC_EDGE
  Arc* GetEdge(Node* src_node, Node* dst_node);
  Arc* CreateEdge(Node* src_node, Node* dst_node, double punish_factor);
  void CreateOutEdges(Node* cen_node);
#endif

#ifdef ENABLE_DYNAMIC_EDGE
  int m_image_width;
  int m_image_height;
  EPF m_epf;
#endif
};

inline void IBFSGraph::addNode(int nodeIndex, double capacitySource, double capacitySink)
{
  if (capacitySource < capacitySink) {
    m_flow += capacitySource;
  } else {
    m_flow += capacitySink;
  }
  m_nodes[nodeIndex].excess = capacitySource - capacitySink;
  m_nodes[nodeIndex].m_node_idx = nodeIndex;
  m_nodes[nodeIndex].m_is_active = false;
}

#ifdef ENABLE_DYNAMIC_EDGE
inline void IBFSGraph::addNode(int nodeIndex, double capacitySource, double capacitySink, int node_colour)
{
  addNode(nodeIndex, capacitySource, capacitySink);
  if (m_nodes[nodeIndex].excess == 0) {
    m_nodes[nodeIndex].label = m_numNodes;
  } else if (m_nodes[nodeIndex].excess > 0) {
    m_nodes[nodeIndex].label = 1;
    // m_activeS1.add(&m_nodes[nodeIndex]);
  } else {
    m_nodes[nodeIndex].label = -1;
    // m_activeT1.add(&m_nodes[nodeIndex]);
  }
  m_nodes[nodeIndex].m_node_colour = node_colour;
  m_nodes[nodeIndex].m_is_gotten_all_edges = false;
  m_nodes[nodeIndex].m_out_edges_num = 0;
  m_nodes[nodeIndex].m_node_idx = nodeIndex;
  m_nodes[nodeIndex].m_is_active = false;
}
#endif

inline void IBFSGraph::addEdge(int nodeIndexFrom, int nodeIndexTo, double capacity, double reverseCapacity)
{
  m_tmpEdgeLast->tail = m_nodes + nodeIndexFrom;
  m_tmpEdgeLast->head = m_nodes + nodeIndexTo;
  m_tmpEdgeLast->cap = capacity;
  m_tmpEdgeLast->revCap = reverseCapacity;
  m_tmpEdgeLast++;

  // use label as a temporary storage
  // to count the out degree of nodes
  m_nodes[nodeIndexFrom].label++;
  m_nodes[nodeIndexTo].label++;
}

inline bool IBFSGraph::isNodeOnSrcSide(int nodeIndex)
{
  if (m_nodes[nodeIndex].label == m_numNodes || m_nodes[nodeIndex].label == 0) {
    return m_activeT1.len == 0;
  }
  return (m_nodes[nodeIndex].label > 0);
}

#endif
