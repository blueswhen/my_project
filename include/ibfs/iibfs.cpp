#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <assert.h>
#include "iibfs.h"
#include <algorithm>

#define ORPHAN  ( (Arc *) 2 )
#define EPSILON 0.00000001
#define ABS(value) ((value) > 0 ? (value) : -(value))
#define SOURCE true
#define SINK false

//
// Orphan handling
//
#define ADD_ORPHAN_BACK(n)              \
if (m_orphanFirst != IB_ORPHANS_END)          \
{                          \
  m_orphanLast = (m_orphanLast->nextPtr = (n));  \
}                          \
else                        \
{                          \
  m_orphanLast = (m_orphanFirst = (n));        \
}                          \
(n)->nextPtr = IB_ORPHANS_END

#define ADD_ORPHAN_FRONT(n)              \
if (m_orphanFirst == IB_ORPHANS_END)          \
{                          \
  (n)->nextPtr = IB_ORPHANS_END;          \
  m_orphanLast = (m_orphanFirst = (n));        \
}                          \
else                        \
{                          \
  (n)->nextPtr = m_orphanFirst;            \
  m_orphanFirst = (n);                \
}

IIBFSGraph::IIBFSGraph()
{
  m_numNodes = 0;
  m_uniqOrphansS = m_uniqOrphansT = 0;
  m_topLevelS = m_topLevelT = 0;
  m_flow = 0;
  m_orphan_count = 0;
  m_path = 0;
  m_orphanFirst = m_orphanLast = NULL;
  m_nlink = 0;
}

IIBFSGraph::~IIBFSGraph()
{
  if (m_nodes) delete [] m_nodes;
  m_active0.free();
  m_activeS1.free();
  m_activeT1.free();
}

#ifdef ENABLE_DYNAMIC_EDGE
void IIBFSGraph::initSize(int numNodes, int width, int height, EPF epf) {
  // allocate m_nodes
  m_numNodes = numNodes;
  m_nodes = new Node[numNodes];
  memset(m_nodes, 0, sizeof(Node)*(numNodes));
  m_active0.init(numNodes);
  m_activeS1.init(numNodes);
  m_activeT1.init(numNodes);
  // init members
  m_flow = 0;
  m_image_width = width;
  m_image_height = height;
  m_epf = epf;
}

void IIBFSGraph::AddActiveNodes(int node_x, int node_y) {
  int index = node_y * m_image_width + node_x;
  Node* cen_node = &m_nodes[index];
  int arr_index[HALF_NEIGHBOUR] =
    HALF_NEIGHBOUR_ARR_INDEX(node_x, node_y, m_image_width, m_image_height);
  for (int i = 0; i < HALF_NEIGHBOUR; ++i) {
    Node* arr_node = &m_nodes[arr_index[i]];
    if (arr_index[i] < index && ((cen_node->excess > 0) != (arr_node->excess > 0))) {
      if (cen_node->excess > 0) {
        if (!cen_node->m_is_active) {
          m_activeS1.add(cen_node);
          cen_node->m_is_active = true;
        }
        if (!arr_node->m_is_active) {
          m_activeT1.add(arr_node);
          arr_node->m_is_active = true;
        }
      } else {
        if (!cen_node->m_is_active) {
          m_activeT1.add(cen_node);
          cen_node->m_is_active = true;
        }
        if (!arr_node->m_is_active) {
          m_activeS1.add(arr_node);
          arr_node->m_is_active = true;
        }
      }
      // break;
    }
  }
}

IIBFSGraph::Arc* IIBFSGraph::GetEdge(Node* src_node, Node* dst_node) {
  for (int i = 0; i < src_node->m_out_edges_num; ++i) {
    if (src_node->m_out_edges[i].head == dst_node) {
      return &src_node->m_out_edges[i];
    }
  }
  return NULL;
}

IIBFSGraph::Arc* IIBFSGraph::CreateEdge(Node* src_node, Node* dst_node, double punish_factor) {
  Arc* edge = &src_node->m_out_edges[src_node->m_out_edges_num++];
  Arc* rev_edge = &dst_node->m_out_edges[dst_node->m_out_edges_num++];
  double cap = punish_factor * m_epf(src_node->m_node_colour, dst_node->m_node_colour);
  edge->rCap = cap;
  edge->rev = rev_edge;
  edge->head = dst_node;
  edge->isRevResidual = cap != 0;
  rev_edge->rCap = cap;
  rev_edge->rev = edge;
  rev_edge->head = src_node;
  rev_edge->isRevResidual = cap != 0;
  return edge;
}

void IIBFSGraph::CreateOutEdges(Node* cen_node) {
  assert(cen_node != NULL);
  if (cen_node->m_is_gotten_all_edges) {
    return;
  }
  m_nlink++;
  int coordinate = cen_node->m_node_idx;
  int y_node = coordinate / m_image_width;
  int x_node = coordinate - y_node * m_image_width;
  int arr_nodes_idx[NEIGHBOUR] =
    NEIGHBOUR_ARR_INDEX(x_node, y_node, m_image_width, m_image_height);
  double punish_factor = 0;
  for (int i = 0; i < NEIGHBOUR; ++i) {
    if (cen_node->m_node_idx == arr_nodes_idx[i]) {
      continue;
    }
    Node* arr_node = &m_nodes[arr_nodes_idx[i]];
    if (!GetEdge(cen_node, arr_node)) {
      if (ABS(arr_nodes_idx[i] - cen_node->m_node_idx) == 1 ||
          ABS(arr_nodes_idx[i] - cen_node->m_node_idx) == m_image_width) {
        punish_factor = 1;
      } else {
        punish_factor = iibgraph_div_sqrt2;
      }
      CreateEdge(cen_node, arr_node, punish_factor);
    }
  }
  cen_node->m_is_gotten_all_edges = true;
}
#endif

template<bool sTree> void IIBFSGraph::augmentTree(Node *x, double bottleneck)
{
  Node *y;
  Arc *a;

  for (; ; x=a->head)
  {
    if (x->excess) break;
    a = x->parent;
    if (sTree) {
      a->rCap += bottleneck;
      a->rev->isRevResidual = 1;
      a->rev->rCap -= bottleneck;
    } else {
      a->rev->rCap += bottleneck;
      a->isRevResidual = 1;
      a->rCap -= bottleneck;
    }

    // saturated?
    if ((sTree ? (a->rev->rCap) : (a->rCap)) == 0)
    {
      if (sTree) a->isRevResidual = 0;
      else a->rev->isRevResidual = 0;
      y=x->parent->head->firstSon;
      if (y == x) {
        x->parent->head->firstSon = x->nextPtr;
      } else {
        for (; y->nextPtr != x; y = y->nextPtr);
        y->nextPtr = x->nextPtr;
      }
      ADD_ORPHAN_FRONT(x);
    }
  }
  x->excess += (sTree ? -bottleneck : bottleneck);
  if (x->excess == 0) {
    ADD_ORPHAN_FRONT(x);
  }
}

void IIBFSGraph::augment(Arc *bridge)
{
  Node *x;
  Arc *a;
  double bottleneck, pushesBefore;

  // bottleneck in S
  bottleneck = bridge->rCap;
  int path = 0;
  for (x=bridge->rev->head; ; x=a->head)
  {
    if (x->excess) break;
    a = x->parent;
    if (bottleneck > a->rev->rCap) {
      bottleneck = a->rev->rCap;
    }
    path++;
  }
  if (bottleneck > x->excess) {
    bottleneck = x->excess;
  }

  // bottleneck in T
  for (x=bridge->head; ; x=a->head)
  {
    if (x->excess) break;
    a = x->parent;
    if (bottleneck > a->rCap) {
      bottleneck = a->rCap;
    }
    path++;
  }
  if (bottleneck > (-x->excess)) {
    bottleneck = (-x->excess);
  }
  // if (path != m_path) {
  //   printf("path = %d\n", path);
  //   m_path = path;
  // }

  // augment connecting arc
  bridge->rev->rCap += bottleneck;
  bridge->isRevResidual = 1;
  bridge->rCap -= bottleneck;
  if (bridge->rCap == 0) {
    bridge->rev->isRevResidual = 0;
  }

  // augment T
  augmentTree<false>(bridge->head, bottleneck);
  adoption<false>();

  // augment S
  augmentTree<true>(bridge->rev->head, bottleneck);
  adoption<true>();

  m_flow += bottleneck;
}

template<bool sTree> void IIBFSGraph::adoption()
{
  Node *x, *y, *z;
  Arc *a, *aEnd;
  int minLabel;

  while (m_orphanFirst != IB_ORPHANS_END)
  {
    m_orphan_count++;
    x = m_orphanFirst;
    m_orphanFirst = x->nextPtr;
    //x->nextOrphan = NULL;
    if (sTree) m_uniqOrphansS++;
    else m_uniqOrphansT++;
    // check for same level connection
    x->parent = NULL;
#ifndef ENABLE_DYNAMIC_EDGE
    a = x->firstArc;
    aEnd = (x+1)->firstArc;
#else
    CreateOutEdges(x);
#endif
    if (x->label != (sTree ? 1 : -1))
    {
      minLabel = x->label - (sTree ? 1 : -1);
#ifndef ENABLE_DYNAMIC_EDGE
      for (; a != aEnd; a++) {
#else
      for (int i = 0; i < x->m_out_edges_num; ++i) {
        a = &x->m_out_edges[i];
#endif
        y = a->head;
        if ((sTree ? a->isRevResidual : a->rCap) != 0 &&
          y->label == minLabel)
        {
          x->parent = a;
          x->nextPtr = y->firstSon;
          y->firstSon = x;
          break;
        }
      }
    }
    if (x->parent != NULL) continue;

    // give up on same level - relabel it!
    // (1) create orphan sons
    for (y=x->firstSon; y != NULL; y=z)
    {
      z=y->nextPtr;
      ADD_ORPHAN_BACK(y);
    }
    x->firstSon = NULL;

    // on the top level there is no need to relabel
    // if (x->label == (sTree ? m_topLevelS : -m_topLevelT)) {
    //   x->label = m_numNodes;
    //   continue;
    // }

    // (2) relabel: find the lowest level parent
    minLabel = (sTree ? m_topLevelS : -m_topLevelT);
#ifndef ENABLE_DYNAMIC_EDGE
    for (a=x->firstArc; a != aEnd; a++) {
#else
    for (int i = 0; i < x->m_out_edges_num; ++i) {
      a = &x->m_out_edges[i];
#endif
      y = a->head;
      if ((sTree ? a->isRevResidual : a->rCap) &&
        // y->label != numNodes ---> holds implicitly
        (sTree ? (y->label > 0) : (y->label < 0)) &&
        (sTree ? (y->label < minLabel) : (y->label > minLabel)))
      {
        minLabel = y->label;
        x->parent = a;
        if (minLabel == x->label) break;
      }
    }

    // (3) relabel onto new parent
    if (x->parent != NULL) {
      x->label = minLabel + (sTree ? 1 : -1);
      x->nextPtr = x->parent->head->firstSon;
      x->parent->head->firstSon = x;
      // add to active list of the next growth phase
      if (sTree) {
        if (x->label == m_topLevelS) {
          // if (!x->m_is_active) {
            m_activeS1.add(x);
            x->m_is_active = true;
          // }
        }
      } else {
        if (x->label == -m_topLevelT) {
          // if (!x->m_is_active) {
            m_activeT1.add(x);
            x->m_is_active = true;
          // }
        }
      }
    } else {
      for (int i = 0; i < x->m_out_edges_num; ++i) {
        a = &x->m_out_edges[i];
        y = a->head;
        if (y->label != m_numNodes) {
          if (sTree) {
            if (y->label == m_topLevelS) {
              if (!y->m_is_active) {
                m_activeS1.add(y);
                y->m_is_active = true;
              }
            }
          } else {
            if (y->label == -m_topLevelT) {
              if (!y->m_is_active) {
                m_activeT1.add(y);
                y->m_is_active = true;
              }
            }
          }
        }
      }
      x->label = m_numNodes;
    }
  }
}

template<bool dirS> void IIBFSGraph::growth() {
  Node *x, *y;
  Arc *a, *aEnd;

  for (Node **active=m_active0.list; active != (m_active0.list + m_active0.len); active++) {
    // get active node
    x = (*active);

    // node no longer at level
    if (x->label != (dirS ? (m_topLevelS-1): -(m_topLevelT-1))) {
      assert(x->label == dirS ? m_topLevelS: -m_topLevelT);
      continue;
    }
    assert(!x->m_is_active);

    // grow or augment
#ifndef ENABLE_DYNAMIC_EDGE
    aEnd = (x+1)->firstArc;
    for (a=x->firstArc; a != aEnd; a++) {
#else
    CreateOutEdges(x);
    for (int i = 0; i < x->m_out_edges_num; ++i) {
      a = &x->m_out_edges[i];
#endif
      if ((dirS ? a->rCap : a->isRevResidual) == 0) continue;
      y = a->head;
      if (y->label == m_numNodes)
      {
        // grow node
        y->label = x->label + (dirS ? 1 : -1);
        y->parent = a->rev;
        y->nextPtr = x->firstSon;
        x->firstSon = y;
        if (dirS) {
          // if(!y->m_is_active) {
            m_activeS1.add(y); 
            y->m_is_active = true;
          // }
        }
        else {
          // if (!y->m_is_active) {
            m_activeT1.add(y); 
            y->m_is_active = true;
          // }
        }
      }
      else if (dirS ? (y->label < 0) : (y->label > 0))
      {
        // augment
        augment(dirS ? a : (a->rev));
        if (x->label != (dirS ? (m_topLevelS-1) : -(m_topLevelT-1))) {
          assert(x->label == dirS ? m_topLevelS: -m_topLevelT);
          break;
        }
        if (dirS ? (a->rCap) : (a->isRevResidual)) {
#ifndef ENABLE_DYNAMIC_EDGE
          a--;
#else
          i--;
#endif
        }
      }
    }
  }
  m_active0.clear();
}

double IIBFSGraph::computeMaxFlow() {
  // init
  m_orphanFirst = IB_ORPHANS_END;
  m_topLevelS = m_topLevelT = 1;
  bool dirS = true;
  ActiveList::swapLists(&m_active0, &m_activeS1);

  //
  // IBFS
  //
  while (true) {
    // BFS level
    if (dirS) m_topLevelS++;
    else m_topLevelT++;
    if (dirS) growth<true>();
    else growth<false>();
    // switch to next level
    if (m_activeS1.len == 0 || m_activeT1.len == 0) {
      break;
    }
    if (m_uniqOrphansT == m_uniqOrphansS && dirS || m_uniqOrphansT < m_uniqOrphansS) {
      // grow T
      ActiveList::swapLists(&m_active0, &m_activeT1);
      dirS=false;
    } else {
      // grow S
      ActiveList::swapLists(&m_active0, &m_activeS1);
      dirS=true;
    }
  }
  // printf("path = %d, orphan count = %d, flow = %f\n", m_path, m_orphan_count, m_flow);
  // printf("r = %f\n", static_cast<double>(m_nlink)/(m_image_width * m_image_height));
  return m_flow;
}


