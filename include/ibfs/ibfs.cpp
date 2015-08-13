#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <assert.h>
#include "ibfs.h"

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

IBFSGraph::IBFSGraph()
{
  m_numNodes = 0;
  m_uniqOrphansS = m_uniqOrphansT = 0;
  m_arcs = m_arcEnd = NULL;
  m_nodes = m_nodeEnd = NULL;
  m_topLevelS = m_topLevelT = 0;
  m_flow = 0;
  m_orphanFirst = m_orphanLast = NULL;
  m_memArcs = NULL;
  m_tmpArcs = NULL;
  m_tmpEdges = m_tmpEdgeLast = NULL;
}

IBFSGraph::~IBFSGraph()
{
  if (m_nodes) free(m_nodes);
  if (m_memArcs) free(m_memArcs);
  m_active0.free();
  m_activeS1.free();
  m_activeT1.free();
}

void IBFSGraph::initGraph()
{
  initGraphFast();
}

void IBFSGraph::initSize(int numNodes, int numEdges)
{
  // allocate m_nodes
  m_numNodes = numNodes;
  m_nodes = new Node[numNodes+1];
  memset(m_nodes, 0, sizeof(Node)*(numNodes+1));
  m_nodeEnd = m_nodes+numNodes;
  m_active0.init(numNodes);
  m_activeS1.init(numNodes);
  m_activeT1.init(numNodes);

  // allocate m_arcs
  size_t arcMemsize = sizeof(TmpArc) * (numEdges * 2) + sizeof(TmpEdge) * numEdges;
  if (arcMemsize < sizeof(Arc) * (numEdges * 2)) {
    arcMemsize = sizeof(Arc) * (numEdges * 2);
  }
  m_memArcs = new char[arcMemsize];
  memset(m_memArcs, 0, sizeof(char) * arcMemsize);
  m_tmpEdges = (TmpEdge*)(m_memArcs);
  m_tmpEdgeLast = m_tmpEdges; // will advance as edges are added
  m_tmpArcs = (TmpArc*)(m_memArcs + arcMemsize - sizeof(TmpArc) * (numEdges * 2));
  m_arcs = (Arc*)m_memArcs;
  m_arcEnd = m_arcs + numEdges * 2;

  // init members
  m_flow = 0;
}

void IBFSGraph::initGraphFast()
{
  Node *x;
  Arc *a;
  TmpArc *ta, *taEnd;
  TmpEdge *te;

  // m_tmpEdges:      edges read
  // node.label:      out degree

  // calculate start arc offsets every node
  m_nodes->firstArc = (Arc*)(m_tmpArcs);
  for (x=m_nodes; x != m_nodeEnd; x++) {
    (x+1)->firstArc = (Arc*)(((TmpArc*)(x->firstArc)) + x->label);
    x->label = ((TmpArc*)(x->firstArc))-m_tmpArcs;
  }
  m_nodeEnd->label = m_arcEnd-m_arcs;

  // m_tmpEdges:        edges read
  // node.label:         index into m_arcs array of first out arc
  // node.firstArc-m_tmpArcs:   index into m_arcs array of next out arc to be allocated
  //              (initially the first out arc)

  // copy to temp m_arcs memory
  for (te=m_tmpEdges; te != m_tmpEdgeLast; te++) {
    ta = (TmpArc*)(te->tail->firstArc);
    ta->cap = te->cap;
    ta->rev = (TmpArc*)(te->head->firstArc);

    ta = (TmpArc*)(te->head->firstArc);
    ta->cap = te->revCap;
    ta->rev = (TmpArc*)(te->tail->firstArc);

    te->tail->firstArc = (Arc*)(((TmpArc*)(te->tail->firstArc))+1);
    te->head->firstArc = (Arc*)(((TmpArc*)(te->head->firstArc))+1);
  }

  // m_tmpEdges:        edges read
  // m_tmpArcs:          m_arcs with reverse pointer but no node id
  // node.label:         index into m_arcs array of first out arc
  // node.firstArc-m_tmpArcs:   index into m_arcs array of last allocated out arc

  // copy to permanent m_arcs array, but saving tail instead of head
  a = m_arcs;
  x = m_nodes;
  taEnd = (m_tmpArcs+(m_arcEnd-m_arcs));
  for (ta=m_tmpArcs; ta != taEnd; ta++) {
    while (x->label <= (ta-m_tmpArcs)) x++;
    a->head = (x-1);
    a->rCap = ta->cap;
    a->rev = m_arcs + (ta->rev-m_tmpArcs);
    a++;
  }

  // m_tmpEdges:        overwritten
  // m_tmpArcs:          overwritten
  // m_arcs:          m_arcs array
  // node.label:         index into m_arcs array of first out arc
  // node.firstArc-m_tmpArcs:   index into m_arcs array of last allocated out arc
  // arc.head = tail of arc

  // swap the head and tail pointers and set isRevResidual
  for (a=m_arcs; a != m_arcEnd; a++) {
    if (a->rev <= a) continue;
    x = a->head;
    a->head = a->rev->head;
    a->rev->head = x;
    a->isRevResidual = (a->rev->rCap != 0);
    a->rev->isRevResidual = (a->rCap != 0);
  }

  // set firstArc pointers in m_nodes array
  for (x=m_nodes; x <= m_nodeEnd; x++) {
    x->firstArc = (m_arcs + x->label);
    if (x->excess == 0) {
      x->label = m_numNodes;
      continue;
    }
    if (x->excess > 0) {
      x->label = 1;
      m_activeS1.add(x);
    } else {
      x->label = -1;
      m_activeT1.add(x);
    }
  }
}

template<bool sTree> void IBFSGraph::augmentTree(Node *x, double bottleneck)
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

void IBFSGraph::augment(Arc *bridge)
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
  // printf("path = %d\n", path);

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

template<bool sTree> void IBFSGraph::adoption()
{
  Node *x, *y, *z;
  Arc *a, *aEnd;
  int minLabel;

  while (m_orphanFirst != IB_ORPHANS_END)
  {
    x = m_orphanFirst;
    m_orphanFirst = x->nextPtr;
    //x->nextOrphan = NULL;
    if (sTree) m_uniqOrphansS++;
    else m_uniqOrphansT++;
    // check for same level connection
    a = x->firstArc;
    x->parent = NULL;
    aEnd = (x+1)->firstArc;
    if (x->label != (sTree ? 1 : -1))
    {
      minLabel = x->label - (sTree ? 1 : -1);
      for (; a != aEnd; a++)
      {
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
    for (a=x->firstArc; a != aEnd; a++) {
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
           m_activeS1.add(x);
        }
      } else {
        if (x->label == -m_topLevelT) {
           m_activeT1.add(x);
        }
      }
    } else {
      x->label = m_numNodes;
    }
  }
}

template<bool dirS> void IBFSGraph::growth() {
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

    // grow or augment
    aEnd = (x+1)->firstArc;
    for (a=x->firstArc; a != aEnd; a++)
    {
      if ((dirS ? a->rCap : a->isRevResidual) == 0) continue;
      y = a->head;
      if (y->label == m_numNodes)
      {
        // grow node
        y->label = x->label + (dirS ? 1 : -1);
        y->parent = a->rev;
        y->nextPtr = x->firstSon;
        x->firstSon = y;
        if (dirS) m_activeS1.add(y);
        else m_activeT1.add(y);
      }
      else if (dirS ? (y->label < 0) : (y->label > 0))
      {
        // augment
        augment(dirS ? a : (a->rev));
        if (x->label != (dirS ? (m_topLevelS-1) : -(m_topLevelT-1))) {
          assert(x->label == dirS ? m_topLevelS: -m_topLevelT);
          break;
        }
        if (dirS ? (a->rCap) : (a->isRevResidual)) a--;
      }
    }
  }
  m_active0.clear();
}

double IBFSGraph::computeMaxFlow() {
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
  printf("flow = %f\n", m_flow);
  return m_flow;
}
