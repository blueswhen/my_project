#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include "ibfs.h"
#include "include/CountTime.h"

#define REMOVE_SIBLING(x, tmp) \
  { (tmp) = (x)->parent->head->firstSon; \
  if ((tmp) == (x)) { \
    (x)->parent->head->firstSon = (x)->nextPtr; \
  } else { \
    for (; (tmp)->nextPtr != (x); (tmp) = (tmp)->nextPtr); \
    (tmp)->nextPtr = (x)->nextPtr; \
  } }

#define ADD_SIBLING(x, parentNode) \
  { (x)->nextPtr = (parentNode)->firstSon; \
  (parentNode)->firstSon = (x); \
  }

#define IB_ORPHANS_END   ( (Node *) 1 )
#define ADD_ORPHAN_BACK(n)							\
if (m_orphanFirst != IB_ORPHANS_END)					\
{													\
	m_orphanLast = (m_orphanLast->nextPtr = (n));	\
}													\
else												\
{													\
	m_orphanLast = (m_orphanFirst = (n));				\
}													\
(n)->nextPtr = IB_ORPHANS_END

#define ADD_ORPHAN_FRONT(n)							\
if (m_orphanFirst == IB_ORPHANS_END)					\
{													\
	(n)->nextPtr = IB_ORPHANS_END;					\
	m_orphanLast = (m_orphanFirst = (n));				\
}													\
else												\
{													\
	(n)->nextPtr = m_orphanFirst;						\
	m_orphanFirst = (n);								\
}

IBFSGraph::IBFSGraph(IBFSInitMode a_initMode)
{
  m_orphanFirst = m_orphanLast = NULL;
  initMode = a_initMode;
  arcIter = NULL;
  numNodes = 0;
  uniqOrphansS = uniqOrphansT = 0;
  arcs = arcEnd = NULL;
  nodes = nodeEnd = NULL;
  topLevelS = topLevelT = 0;
  flow = 0;
  memArcs = NULL;
  tmpArcs = NULL;
  tmpEdges = tmpEdgeLast = NULL;
  ptrs = NULL;
}

IBFSGraph::~IBFSGraph()
{
  if (nodes) free(nodes);
  if (memArcs) free(memArcs);
}

void IBFSGraph::initGraph()
{
  if (initMode == IB_INIT_FAST) {
    initGraphFast();
  }
  topLevelS = topLevelT = 1;
}

void IBFSGraph::initSize(int numNodes, int numEdges)
{
  // compute allocation size
  unsigned long long arcTmpMemsize = (unsigned long long)sizeof(TmpEdge)*(unsigned long long)numEdges;
  unsigned long long arcRealMemsize = (unsigned long long)sizeof(Arc)*(unsigned long long)(numEdges*2);
  unsigned long long nodeMemsize = (unsigned long long)sizeof(Node**)*(unsigned long long)(numNodes*3);
  unsigned long long arcMemsize = 0;
  if (initMode == IB_INIT_FAST) {
    arcMemsize = arcRealMemsize + arcTmpMemsize;
  } else if (initMode == IB_INIT_COMPACT) {
    arcTmpMemsize += (unsigned long long)sizeof(TmpArc)*(unsigned long long)(numEdges*2);
    arcMemsize = arcTmpMemsize;
  }
  if (arcMemsize < (arcRealMemsize + nodeMemsize)) {
    arcMemsize = (arcRealMemsize + nodeMemsize);
  }

  // alocate arcs
  memArcs = new char[arcMemsize];
  memset(memArcs, 0, (unsigned long long)sizeof(char)*arcMemsize);
  if (initMode == IB_INIT_FAST) {
    tmpEdges = (TmpEdge*)(memArcs + arcRealMemsize);
  } else if (initMode == IB_INIT_COMPACT) {
    tmpEdges = (TmpEdge*)(memArcs);
    tmpArcs = (TmpArc*)(memArcs +arcMemsize -(unsigned long long)sizeof(TmpArc)*(unsigned long long)(numEdges*2));
  }
  tmpEdgeLast = tmpEdges; // will advance as edges are added
  arcs = (Arc*)memArcs;
  arcEnd = arcs + numEdges*2;

  // allocate nodes
  this->numNodes = numNodes;
  nodes = new Node[numNodes+1];
  memset(nodes, 0, sizeof(Node)*(numNodes+1));
  nodeEnd = nodes+numNodes;
  active0.init((Node**)(arcEnd));
  activeS1.init((Node**)(arcEnd) + numNodes);
  activeT1.init((Node**)(arcEnd) + (2*numNodes));

  // init members
  flow = 0;
}

void IBFSGraph::initNodes()
{
  Node *x;
  for (x=nodes; x <= nodeEnd; x++) {
    x->firstArc = (arcs + x->label);
    if (x->excess == 0) {
      x->label = 0;
      continue;
    }
    if (x->excess > 0) {
      x->label = 1;
      activeS1.add(x);
    } else {
      x->label = -1;
      activeT1.add(x);
    }
  }
}

void IBFSGraph::initGraphFast()
{
  Node *x;
  TmpEdge *te;
  Arc *a;

  // calculate start arc offsets and labels for every node
  nodes->firstArc = arcs;
  for (x=nodes; x != nodeEnd; x++) {
    (x+1)->firstArc = x->firstArc + x->label;
    x->label = x->firstArc-arcs;
  }
  nodeEnd->label = arcEnd-arcs;

  // copy arcs
  for (te=tmpEdges; te != tmpEdgeLast; te++) {
    a = (nodes+te->tail)->firstArc;
    a->rev = (nodes+te->head)->firstArc;
    a->head = nodes+te->head;
    a->rCap = te->cap;
    a->isRevResidual = (te->revCap != 0);

    a = (nodes+te->head)->firstArc;
    a->rev = (nodes+te->tail)->firstArc;
    a->head = nodes+te->tail;
    a->rCap = te->revCap;
    a->isRevResidual = (te->cap != 0);

    ++((nodes+te->head)->firstArc);
    ++((nodes+te->tail)->firstArc);
  }

  initNodes();
  m_path = 0;
  m_orphan_count = 0;
  m_time = 0;
  m_time2 = 0;
}

// @ret: minimum orphan level
template<bool sTree> int IBFSGraph::augmentPath(Node *x, double push)
{
  Node *y;
  Arc *a;
  int orphanMinLevel = (sTree ? topLevelS : topLevelT) + 1;

  for (; ; x=a->head)
  {
    if (x->excess) break;
    a = x->parent;
    if (sTree) {
      a->rCap += push;
      a->rev->isRevResidual = 1;
      a->rev->rCap -= push;
    } else {
      a->rev->rCap += push;
      a->isRevResidual = 1;
      a->rCap -= push;
    }

    // saturated?
    if ((sTree ? (a->rev->rCap) : (a->rCap)) == 0)
    {
      if (sTree) a->isRevResidual = 0;
      else a->rev->isRevResidual = 0;
      REMOVE_SIBLING(x,y);
      orphanMinLevel = (sTree ? x->label : -x->label);
      ADD_ORPHAN_BACK(x);
    }
  }
  x->excess += (sTree ? -push : push);
  if (x->excess == 0) {
    orphanMinLevel = (sTree ? x->label : -x->label);
    ADD_ORPHAN_BACK(x);
  }
  flow += push;

  return orphanMinLevel;
}

// @ret: minimum level in which created an orphan
template<bool sTree> int IBFSGraph::augmentExcess(Node *x, double push)
{
  Node *y;
  Arc *a;
  int orphanMinLevel = (sTree ? topLevelS : topLevelT)+1;

  // start of loop
  //----------------
  // x       the current node along the path
  // a      arc incoming into x
  // push     the amount of flow coming into x
  // a->resCap  updated with incoming flow already
  // x->excess  not updated with incoming flow yet
  //
  // end of loop
  //-----------------
  // x       the current node along the path
  // a      arc outgoing from x
  // push     the amount of flow coming out of x
  // a->resCap  updated with outgoing flow already
  // x->excess  updated with incoming flow already
  while (sTree ? (x->excess <= 0) : (x->excess >= 0))
  {
    a = x->parent;

    // update excess and find next flow
    if ((sTree ? (a->rev->rCap) : (a->rCap)) < (sTree ? (push-x->excess) : (x->excess+push))) {
      // some excess remains, node is an orphan
      x->excess += (sTree ? (a->rev->rCap - push) : (push-a->rCap));
      push = (sTree ? a->rev->rCap : a->rCap);
    } else {
      // all excess is pushed out, node may or may not be an orphan
      push += (sTree ? -(x->excess) : x->excess);
      x->excess = 0;
    }
    m_path++;

    // push flow
    // note: push != 0
    if (sTree) {
      a->rCap += push;
      a->rev->isRevResidual = 1;
      a->rev->rCap -= push;
    } else {
      a->rev->rCap += push;
      a->isRevResidual = 1;
      a->rCap -= push;
    }

    // saturated?
    if ((sTree ? (a->rev->rCap) : (a->rCap)) == 0)
    {
      if (sTree) a->isRevResidual = 0;
      else a->rev->isRevResidual = 0;
      REMOVE_SIBLING(x,y);
      orphanMinLevel = (sTree ? x->label : -x->label);
      ADD_ORPHAN_BACK(x);
    }

    // advance
    // a precondition determines that the first node on the path is not in excess buckets
    // so only the next nodes may need to be removed from there
    x = a->head;
    if (sTree ? (x->excess < 0) : (x->excess > 0)) {
      x->m_in_queue = false;
    };
  }

  // update the excess at the root
  if (push <= (sTree ? (x->excess) : -(x->excess))) flow += push;
  else flow += (sTree ? (x->excess) : -(x->excess));
  x->excess += (sTree ? (-push) : push);
  if (sTree ? (x->excess <= 0) : (x->excess >= 0)) {
    orphanMinLevel = (sTree ? x->label : -x->label);
    ADD_ORPHAN_BACK(x);
  }

  return orphanMinLevel;
}

template<bool sTree> void IBFSGraph::augmentExcesses()
{
  Node *x;
  while (!m_excess_queue.empty()) {
    x = m_excess_queue.front();
    m_excess_queue.pop();
    if (!x->m_in_queue) {
      continue;
    } else {
      x->m_in_queue = false;
    }
    augmentExcess<sTree>(x, 0);
    // if we did not create new orphans
    adoption<sTree>();
  }
}

void IBFSGraph::augment(Arc *bridge)
{
  Node *x, *y;
  Arc *a;
  double bottleneck, bottleneckT, bottleneckS;
  bool forceBottleneck;

  // must compute forceBottleneck once, so that it is constant throughout this method
  forceBottleneck = (IB_EXCESSES ? false : true);
  if (!IB_EXCESSES)
  {
    // limit by end nodes excess
    bottleneck = bridge->rCap;
    if (bridge->head->excess != 0 && -(bridge->head->excess) < bottleneck) {
      bottleneck = -(bridge->head->excess);
    }
    if (bridge->rev->head->excess != 0 && bridge->rev->head->excess < bottleneck) {
      bottleneck = bridge->rev->head->excess;
    }
  }
  else
  {
    bottleneck = bottleneckS = bridge->rCap;
    // if (bottleneck != 1) {
      for (x=bridge->rev->head; ; x=a->head)
      {
        if (x->excess) break;
        a = x->parent;
        if (bottleneckS > a->rev->rCap) {
          bottleneckS = a->rev->rCap;
        }
      }
      if (bottleneckS > x->excess) {
        bottleneckS = x->excess;
      }
      if (IB_EXCESSES && x->label != 1) forceBottleneck = true;
      if (x == bridge->rev->head) bottleneck = bottleneckS;
    // }

    // if (bottleneck != 1) {
      bottleneckT = bridge->rCap;
      for (x=bridge->head; ; x=a->head)
      {
        if (x->excess) break;
        a = x->parent;
        if (bottleneckT > a->rCap) {
          bottleneckT = a->rCap;
        }
      }
      if (bottleneckT > (-x->excess)) {
        bottleneckT = (-x->excess);
      }
      if (IB_EXCESSES && x->label != -1) forceBottleneck = true;
      if (x == bridge->head && bottleneck > bottleneckT) bottleneck = bottleneckT;

      if (forceBottleneck) {
        if (bottleneckS < bottleneckT) bottleneck = bottleneckS;
        else bottleneck = bottleneckT;
      }
    // }
  }

  // augment connecting arc
  bridge->rev->rCap += bottleneck;
  bridge->isRevResidual = 1;
  bridge->rCap -= bottleneck;
  if (bridge->rCap == 0) {
    bridge->rev->isRevResidual = 0;
  }
  flow -= bottleneck;
  m_path++;

  // augment T
  x = bridge->head;
  // if (!IB_EXCESSES || bottleneck == 1 || forceBottleneck) {
  //   augmentPath<false>(x, bottleneck);
  //   adoption<false>();
  // } else {
    augmentExcess<false>(x, bottleneck);
    adoption<false>();
    augmentExcesses<false>();
  // }

  // augment S
  x = bridge->rev->head;
  // if (!IB_EXCESSES || bottleneck == 1 || forceBottleneck) {
  //   augmentPath<true>(x, bottleneck);
  //   adoption<true>();
  // } else {
    augmentExcess<true>(x, bottleneck);
    adoption<true>();
    augmentExcesses<true>();
  // }
}

template<bool sTree> void IBFSGraph::adoption()
{
  Node *x, *y, *z;
  register Arc *a;
  Arc *aEnd;
  int minLabel;

  // CountTime ct;
  // ct.ContBegin();
 	while (m_orphanFirst != IB_ORPHANS_END)
	{
    m_orphan_count++;
		x = m_orphanFirst;
		m_orphanFirst = x->nextPtr;
    if (sTree) uniqOrphansS++;
    else uniqOrphansT++;
    // check for same level connection
    //
    if (x->isParentCurr) {
      a = x->parent;
    } else {
      a = x->firstArc;
      x->isParentCurr = 1;
    }
    x->parent = NULL;
    aEnd = (x+1)->firstArc;
    if (x->label != (sTree ? 1 : -1))
    {
      minLabel = x->label - (sTree ? 1 : -1);
      for (; a != aEnd; a++)
      {
        y = a->head;
        if ((sTree ? a->isRevResidual : a->rCap) != 0 && y->label == minLabel)
        {
          x->parent = a;
          ADD_SIBLING(x,y);
          break;
        }
      }
    }
    if (x->parent != NULL) {
      if (IB_EXCESSES && x->excess) {
        if (!x->m_in_queue) {
          x->m_in_queue = true;
          m_excess_queue.push(x);
        }
      }
      continue;
    }

    //
    // on the top level there is no need to relabel
    //
    if (x->label == (sTree ? topLevelS : -topLevelT)) {
      orphanFree<sTree>(x);
      continue;
    }

    //
    // give up on same level - relabel it!
    // (1) create orphan sons
    //
    for (y=x->firstSon; y != NULL; y=z)
    {
      z=y->nextPtr;
      if (IB_EXCESSES && y->excess) {
        y->m_in_queue = false;
      }
      ADD_ORPHAN_BACK(y);
    }
    x->firstSon = NULL;

    //
    // (2) relabel: find the lowest level parent
    //
    minLabel = (sTree ? topLevelS : -topLevelT);
    if (x->label != minLabel) for (a=x->firstArc; a != aEnd; a++)
    {
      y = a->head;
      if ((sTree ? a->isRevResidual : a->rCap) &&
        // y->label != 0 ---> holds implicitly
        (sTree ? (y->label > 0) : (y->label < 0)) &&
        (sTree ? (y->label < minLabel) : (y->label > minLabel)))
      {
        minLabel = y->label;
        x->parent = a;
        if (minLabel == x->label) break;
      }
    }

    //
    // (3) relabel onto new parent
    //
    if (x->parent != NULL) {
      x->label = minLabel + (sTree ? 1 : -1);
      ADD_SIBLING(x, x->parent->head);
      // add to active list of the next growth phase
      if (sTree) {
        if (x->label == topLevelS) activeS1.add(x);
      } else {
        if (x->label == -topLevelT) activeT1.add(x);
      }
      if (IB_EXCESSES && x->excess) {
        if (!x->m_in_queue) {
          x->m_in_queue = true;
          m_excess_queue.push(x);
        }
      }
    } else {
      orphanFree<sTree>(x);
    }
  }
  // ct.ContEnd();
  // m_time += ct.ContResult() * 1000;
}

template<bool dirS> void IBFSGraph::growth()
{
  Node *x, *y;
  Arc *a, *aEnd;

  for (Node **active=active0.list; active != (active0.list + active0.len); active++)
  {
    // get active node
    x = (*active);

    // node no longer at level
    if (x->label != (dirS ? (topLevelS-1): -(topLevelT-1))) {
      continue;
    }

    // grow or augment
    aEnd = (x+1)->firstArc;
    for (a=x->firstArc; a != aEnd; a++)
    {
      if ((dirS ? a->rCap : a->isRevResidual) == 0) continue;
      y = a->head;
      if (y->label == 0)
      {
        // grow node x (attach y)
        y->isParentCurr = 0;
        y->label = x->label + (dirS ? 1 : -1);
        y->parent = a->rev;
        ADD_SIBLING(y, x);
        if (dirS) activeS1.add(y);
        else activeT1.add(y);
      }
      else if (dirS ? (y->label < 0) : (y->label > 0))
      {
        // augment
  // CountTime ct;
  // ct.ContBegin();
        augment(dirS ? a : (a->rev));
  // ct.ContEnd();
  // m_time2 += ct.ContResult() * 1000;
        if (x->label != (dirS ? (topLevelS-1) : -(topLevelT-1))) {
          break;
        }
        if (dirS ? (a->rCap) : (a->isRevResidual)) a--;
      }
    }
  }
  active0.clear();
}

double IBFSGraph::computeMaxFlow()
{
  return computeMaxFlow(true, false);
}

double IBFSGraph::computeMaxFlow(bool allowIncrements)
{
  return computeMaxFlow(true, allowIncrements);
}

double IBFSGraph::computeMaxFlow(bool initialDirS, bool allowIncrements)
{
 	m_orphanFirst = IB_ORPHANS_END;
  //
  // IBFS
  //
  bool dirS = initialDirS;
  while (true)
  {
    // BFS level
    if (dirS) {
      ActiveList::swapLists(&active0, &activeS1);
      topLevelS++;
    } else {
      ActiveList::swapLists(&active0, &activeT1);
      topLevelT++;
    }
    if (dirS) growth<true>();
    else growth<false>();

    // switch to next level
    // if (!allowIncrements && (activeS1.len == 0 || activeT1.len == 0)) break;
    if (activeS1.len == 0 && activeT1.len == 0) break;
    if (activeT1.len == 0) dirS=true;
    else if (activeS1.len == 0) dirS=false;
    else if (uniqOrphansT == uniqOrphansS && dirS) dirS=false;
    else if (uniqOrphansT < uniqOrphansS) dirS=false;
    else dirS=true;
  }
  // printf("path = %d, path time = %f, orphan count = %d, orphan time = %f\n", m_path, m_time2, m_orphan_count, m_time);
  return flow;
}
