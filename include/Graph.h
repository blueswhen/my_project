// Copyright 2014-12 sxniu
#ifndef INCLUDE_GRAPH_H_
#define INCLUDE_GRAPH_H_

#include <vector>
#include <queue>
#include <stack>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>
#include <unistd.h>
#include "include/CountTime.h"
#include "include/ImageData.h"

#define TERMINAL reinterpret_cast<Edge*>(1)
#define ORPHAN reinterpret_cast<Edge*>(2)
#define INIFINITE_DIST INT_MAX
#define NEIGHBOUR 8
#define T 50
#define LT 10000

#define EIBFS

namespace user {

template <class CapType>
class Graph {
 public:
  enum NodeState {
    SOURCE,
    SINK,
    UNKNOWN
  };
  Graph(int max_nodes_number, int max_edges_number, ImageData<int>* marked_image);
#ifdef EIBFS
  ~Graph() {
    orphanBuckets.free();
    excessBuckets.free();
  }
#endif
  void AddNode(int node_id, CapType source_capacity, CapType sink_capacity);
  void AddEdge(int src_node_id, int dst_node_id, CapType edge_capacity);
  CapType MaxFlow();
  bool IsBelongToSource(int node_id);

 private:
  class Node;
  class Edge {
   public:
    Edge()
      : m_dst_node(NULL)
      , m_edge_capacity(0)
      , m_rev_edge(NULL) {}
    Node* m_dst_node;
    CapType m_edge_capacity;
    Edge* m_rev_edge;
    // next edge originated from the same node
  };

  class Node {
   public:
    Node() 
      : m_excess(0)
      , m_node_state(UNKNOWN)
      , m_first_child_node(NULL)
      , m_next_child_node(NULL)
      , m_parent_edge(NULL)
      // , m_timestamp(0)
      , m_terminal_dist(0)
      , m_next_active_source(NULL)
      , m_next_active_sink(NULL)
      , m_is_source_active(false)
      , m_is_sink_active(false)
      , m_is_new_source_start(false)
      , m_is_new_sink_start(false)
      , m_out_edges_num(0)
#ifdef EIBFS
      , lastAugTimestamp(0)
#endif
      , m_id(0) {}
    CapType m_excess;
    NodeState m_node_state;
    Node* m_first_child_node;
    Node* m_next_child_node;
    Edge* m_parent_edge;
    Node* m_next_active_source;
    Node* m_next_active_sink;
    bool m_is_source_active;
    bool m_is_sink_active;
    bool m_is_new_source_start;
    bool m_is_new_sink_start;
    // the timestamp of the latest dist calculating 
    // int m_timestamp;
    int m_terminal_dist;
    Edge m_out_edges[NEIGHBOUR];
    int m_out_edges_num;
#ifdef EIBFS
    int lastAugTimestamp;
#endif
    int m_id;
  };

  void AddActiveSourceNodeBack(Node* node) {
    assert(node->m_node_state == SOURCE);
    if (!node->m_is_source_active) {
      assert(!node->m_next_active_source);
      node->m_is_source_active = true;
      if (m_last_at_source_node) {
        m_last_at_source_node->m_next_active_source = node;
        m_last_at_source_node = node;
      } else {
        m_first_at_source_node = node;
        m_last_at_source_node = node;
      }
      node->m_next_active_source = NULL;
      if (m_add_new_source_start) {
        m_add_new_source_start = false;
        node->m_is_new_source_start = true;
      }
      assert(node->m_terminal_dist == m_global_source_dist);
    }
  }
  void AddActiveSourceNodeFront(Node* node) {
    assert(node->m_node_state == SOURCE);
    if (!node->m_is_source_active) {
      node->m_is_source_active = true;
      node->m_next_active_source = m_first_at_source_node;
      m_first_at_source_node = node;
      if (!m_last_at_source_node) {
        m_last_at_source_node = m_first_at_source_node;
        m_last_at_source_node->m_next_active_source = NULL;
      }
    }
  }
  Node* GetActiveSourceNode() {
    if (!m_first_at_source_node) {
      m_global_state = SINK;
      return NULL;
    }
    Node* active_node = NULL;
    while (m_first_at_source_node) {
      // arrive the first of new queue
      bool is_break = false;
      if (m_first_at_source_node->m_is_new_source_start) {
        m_global_source_dist++;
        m_add_new_source_start = true;
        m_first_at_source_node->m_is_new_source_start = false; 
      }
      if (m_first_at_source_node->m_next_active_source &&
          m_first_at_source_node->m_next_active_source->m_is_new_source_start) {
        if (m_global_sink_orphan_num <= m_global_source_orphan_num) {
          m_global_state = SINK;
          is_break = true;
        }
      }
      active_node = m_first_at_source_node;
      m_first_at_source_node = m_first_at_source_node->m_next_active_source;
      if (!m_first_at_source_node) {
        m_last_at_source_node = NULL;
      }
      active_node->m_is_source_active = false;
      active_node->m_next_active_source = NULL;
      if (active_node->m_parent_edge && active_node->m_node_state == SOURCE) {
        break;
      }
      active_node = NULL;
      if (is_break) {
        break;
      }
    }
    return active_node;
  }
  bool IsActiveSourceEmpty() {
    if (m_first_at_source_node == NULL && m_last_at_source_node == NULL) {
      return true;
    }
    return false;
  }
  void AddActiveSinkNodeBack(Node* node) {
    assert(node->m_node_state == SINK);
    if (!node->m_is_sink_active) {
      assert(!node->m_next_active_sink);
      node->m_is_sink_active = true;
      if (m_last_at_sink_node) {
        m_last_at_sink_node->m_next_active_sink = node;
        m_last_at_sink_node = node;
      } else {
        m_first_at_sink_node = node;
        m_last_at_sink_node = node;
      }
      node->m_next_active_sink = NULL;
      if (m_add_new_sink_start) {
        m_add_new_sink_start = false;
        node->m_is_new_sink_start = true;
      }
      assert(node->m_terminal_dist == m_global_sink_dist);
    }
  }
  void AddActiveSinkNodeFront(Node* node) {
    assert(node->m_node_state == SINK);
    if (!node->m_is_sink_active) {
      node->m_is_sink_active = true;
      node->m_next_active_sink = m_first_at_sink_node;
      m_first_at_sink_node = node;
      if (!m_last_at_sink_node) {
        m_last_at_sink_node = m_first_at_sink_node;
        m_last_at_sink_node->m_next_active_sink = NULL;
      }
    }
  }
  Node* GetActiveSinkNode() {
    if (!m_first_at_sink_node) {
      m_global_state = SOURCE;
      return NULL;
    }
    Node* active_node = NULL;
    while (m_first_at_sink_node) {
      // arrive the first of new queue
      bool is_break = false;
      if (m_first_at_sink_node->m_is_new_sink_start) {
        m_global_sink_dist++;
        m_add_new_sink_start = true;
        m_first_at_sink_node->m_is_new_sink_start = false; 
      }
      if (m_first_at_sink_node->m_next_active_sink &&
          m_first_at_sink_node->m_next_active_sink->m_is_new_sink_start) {
        if (m_global_source_orphan_num <= m_global_sink_orphan_num) {
          m_global_state = SOURCE;
          is_break = true;
        }
      }
      active_node = m_first_at_sink_node;
      m_first_at_sink_node = m_first_at_sink_node->m_next_active_sink;
      if (!m_first_at_sink_node) {
        m_last_at_sink_node = NULL;
      }
      active_node->m_is_sink_active = false;
      active_node->m_next_active_sink = NULL;
      if (active_node->m_parent_edge && active_node->m_node_state == SINK) {
        break;
      }
      active_node = NULL;
      if (is_break) {
        break;
      }
    }
    return active_node;
  }
  bool IsActiveSinkEmpty() {
    if (m_first_at_sink_node == NULL && m_last_at_sink_node == NULL) {
      return true;
    }
    return false;
  }
  bool AddActiveNodeBack(Node* at_node) {
    if (at_node->m_node_state == SOURCE) {
      if (at_node->m_terminal_dist == m_global_source_dist) {
        AddActiveSourceNodeBack(at_node);
        return true;
      }
    } else {
      if (at_node->m_terminal_dist == m_global_sink_dist) {
        AddActiveSinkNodeBack(at_node);
        return true;
      }
    }
    return false;
  }
  void AddOrphanNode(Node* orphan_node) {
    assert(orphan_node->m_parent_edge != ORPHAN);
    orphan_node->m_parent_edge = ORPHAN;
    m_orphan_nodes.push(orphan_node);
  }
  Node* GetOrphanNode() {
    Node* orphan_node = NULL;
    if (!m_orphan_nodes.empty()) {
      orphan_node = m_orphan_nodes.front();
      m_orphan_nodes.pop();
    }
    return orphan_node;
  }
  void Augment(Edge* meet_edge);
  void FindNewPath(Node* orphan_node);

#ifdef EIBFS

#define IB_ALLOC_INIT_LEVELS 4096

  class BucketsOneSided
  {
  public:
    inline BucketsOneSided() {
      buckets = NULL;
      maxBucket = 0;
      allocLevels = 0;
    }
    inline void init(int numNodes) {
      allocLevels = numNodes/8;
      if (allocLevels < IB_ALLOC_INIT_LEVELS) {
        if (numNodes < IB_ALLOC_INIT_LEVELS) allocLevels = numNodes;
        else allocLevels = IB_ALLOC_INIT_LEVELS;
      }
      buckets = new Node*[allocLevels+1];
      memset(buckets, 0, sizeof(Node*)*(allocLevels+1));
      maxBucket = 0;
    }
    inline void allocate(int numLevels) {
      if (numLevels > allocLevels) {
        allocLevels <<= 1;
        Node **alloc = new Node*[allocLevels+1];
        memset(alloc, 0, sizeof(Node*)*(allocLevels+1));
        delete []buckets;
        buckets = alloc;
      }
    }
    inline void free() {
      if (buckets != NULL) {
        delete []buckets;
        buckets = NULL;
      }
    }
    inline void add(Node* x) {
      int bucket = x->m_terminal_dist;
      x->m_next_child_node = buckets[bucket];
      buckets[bucket] = x;
      if (bucket > maxBucket) maxBucket = bucket;
    }
    inline Node* popFront(int bucket) {
      Node *x;
      if ((x = buckets[bucket]) == NULL) return NULL;
      buckets[bucket] = x->m_next_child_node;
      return x;
    }

    Node **buckets;
    int maxBucket;
    int allocLevels;
  };

#define IB_PREVPTR_EXCESS(x) (ptrs[(((x)->m_id)<<1) + 1])
#define IB_NEXTPTR_EXCESS(x) (ptrs[((x)->m_id)<<1])
  class ExcessBuckets
  {
  public:
    inline ExcessBuckets() {
      buckets = ptrs = NULL;
      allocLevels = maxBucket = minBucket = -1;
    }
    inline void init(int numNodes) {
      allocLevels = numNodes/8;
      if (allocLevels < IB_ALLOC_INIT_LEVELS) {
        if (numNodes < IB_ALLOC_INIT_LEVELS) allocLevels = numNodes;
        else allocLevels = IB_ALLOC_INIT_LEVELS;
      }
      buckets = new Node*[allocLevels+1];
      memset(buckets, 0, sizeof(Node*)*(allocLevels+1));
      ptrs = new Node*[2*numNodes];
      memset(ptrs, 0, sizeof(Node*)*(2*numNodes));
      reset();
    }
    inline void allocate(int numLevels) {
      if (numLevels > allocLevels) {
        allocLevels <<= 1;
        Node **alloc = new Node*[allocLevels+1];
        memset(alloc, 0, sizeof(Node*)*(allocLevels+1));
        delete [] buckets;
        buckets = alloc;
      }
    }
    inline void free() {
      if (buckets != NULL) {
        delete []buckets;
        buckets = NULL;
      }
      if (ptrs != NULL) {
        delete [] ptrs;
        ptrs = NULL;
      }
    }

    inline void add(Node* x) {
      int bucket = x->m_terminal_dist;
      IB_NEXTPTR_EXCESS(x) = buckets[bucket];
      if (buckets[bucket] != NULL) {
        IB_PREVPTR_EXCESS(buckets[bucket]) = x;
      }
      buckets[bucket] = x;
      if (bucket > maxBucket) maxBucket = bucket;
      if (bucket != 0 && bucket < minBucket) minBucket = bucket;
    }
    inline Node* popFront(int bucket) {
      Node *x = buckets[bucket];
      if (x == NULL) return NULL;
      buckets[bucket] = IB_NEXTPTR_EXCESS(x);
      return x;
    }
    inline void remove(Node *x) {
      int bucket = x->m_terminal_dist;
      assert(buckets[bucket]);
      if (buckets[bucket] == x) {
        buckets[bucket] = IB_NEXTPTR_EXCESS(x);
      } else {
        assert(IB_PREVPTR_EXCESS(x));
        IB_NEXTPTR_EXCESS(IB_PREVPTR_EXCESS(x)) = IB_NEXTPTR_EXCESS(x);
        if (IB_NEXTPTR_EXCESS(x) != NULL) IB_PREVPTR_EXCESS(IB_NEXTPTR_EXCESS(x)) = IB_PREVPTR_EXCESS(x);
      }
    }
    inline void incMaxBucket(int bucket) {
      if (maxBucket < bucket) maxBucket = bucket;
    }
    inline bool empty() {
      return maxBucket < minBucket;
    }
    inline void reset() {
      maxBucket = 0;
      minBucket = -1 ^ (1<<31);
    }

    Node **buckets;
    Node **ptrs;
    int maxBucket;
    int minBucket;
    int allocLevels;
  };
#endif
#ifdef EIBFS
  inline void orphanFree(Node *x) {
    if (x->m_excess) {
      x->m_terminal_dist = (x->m_node_state == SOURCE ? m_global_sink_dist : m_global_source_dist);
      x->m_node_state = x->m_node_state == SOURCE ? SINK : SOURCE;
      x->m_parent_edge = TERMINAL;
      if (x->m_node_state == SOURCE) {
        AddActiveSourceNodeBack(x);
      } else {
        AddActiveSinkNodeBack(x);
      }
      // if (sTree) activeT1.add(x);
      // else activeS1.add(x);
      // x->isParentCurr = 0;
    } else {
      x->m_parent_edge = NULL;
      // x->label = 0;
    }
  }
  int augmentExcess(Node *x, double push);
  void augmentExcesses();
  void adoption(int fromLevel, bool toTop);
#endif

  std::vector<Node> m_nodes;
  Node* m_first_at_source_node;
  Node* m_last_at_source_node;
  Node* m_first_at_sink_node;
  Node* m_last_at_sink_node;
  // point to the last item used in m_edges
  // int m_global_timestamp;
  std::queue<Node*> m_orphan_nodes;
  int m_path;
  int m_count;
  bool m_add_new_source_start;
  bool m_add_new_sink_start;
  int m_global_source_dist;
  int m_global_sink_dist;
  NodeState m_global_state;
  int m_global_source_orphan_num;
  int m_global_sink_orphan_num;
  // ImageData<int>* m_marked_image;
#ifdef EIBFS
  int augTimestamp;
  BucketsOneSided orphanBuckets;
  ExcessBuckets excessBuckets;
#endif
  CapType m_flow;
};

template <class CapType>
Graph<CapType>::Graph(int max_nodes_number, int max_edges_number, ImageData<int>* marked_image)
  : m_nodes(std::vector<Node>(max_nodes_number))
  , m_first_at_source_node(NULL)
  , m_last_at_source_node(NULL)
  , m_first_at_sink_node(NULL)
  , m_last_at_sink_node(NULL)
  // , m_global_timestamp(0)
  , m_path(0)
  , m_count(0)
  , m_add_new_source_start(true)
  , m_add_new_sink_start(true)
  , m_global_source_dist(1)
  , m_global_sink_dist(1)
  , m_global_state(SOURCE)
  , m_global_source_orphan_num(0)
  , m_global_sink_orphan_num(0)
  // , m_marked_image(marked_image)
#ifdef EIBFS
  , augTimestamp(0)
#endif
  , m_flow(0) {
#ifdef EIBFS
    excessBuckets.init(max_nodes_number);
    orphanBuckets.init(max_nodes_number);
#endif
  }

template <class CapType>
void Graph<CapType>::AddNode(int node_id, CapType source_capacity, CapType sink_capacity) {
  assert(node_id >= 0 && node_id < m_nodes.size() && m_nodes[node_id].m_excess == 0);
  m_flow += source_capacity < sink_capacity ? source_capacity : sink_capacity;
  CapType node_capacity = source_capacity - sink_capacity;
  m_nodes[node_id].m_excess = node_capacity;
  m_nodes[node_id].m_id = node_id;
}

template <class CapType>
void Graph<CapType>::AddEdge(int src_node_id, int dst_node_id, CapType edge_capacity) {
  assert(src_node_id >= 0 && src_node_id < m_nodes.size());
  assert(dst_node_id >= 0 && dst_node_id < m_nodes.size());

  Node* src_node = &m_nodes[src_node_id];
  Node* dst_node = &m_nodes[dst_node_id];

  Edge* edge = &src_node->m_out_edges[src_node->m_out_edges_num++];
  Edge* rev_edge = &dst_node->m_out_edges[dst_node->m_out_edges_num++];

  edge->m_dst_node = dst_node;
  edge->m_edge_capacity = edge_capacity;
  edge->m_rev_edge = rev_edge;
  rev_edge->m_dst_node = src_node;
  rev_edge->m_edge_capacity = edge_capacity;
  rev_edge->m_rev_edge = edge;
}

#ifndef EIBFS
template <class CapType>
void Graph<CapType>::Augment(Edge* meet_edge) {
  // augment path
  CapType min_capacity = meet_edge->m_edge_capacity;
  // first_edge[0] for source tree and first_edge[1] for sink tree
  Edge* first_edge[2] = {meet_edge->m_rev_edge, meet_edge};
  // find min capacity from path
  int path = 0;
  for (int i = 0; i < 2; ++i) {
    Node* parent_node = first_edge[i]->m_dst_node;
    for (Edge* parent_edge = parent_node->m_parent_edge; parent_edge != TERMINAL;
         parent_node = parent_edge->m_dst_node, parent_edge = parent_node->m_parent_edge) {
      assert(parent_edge);
      Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
      CapType cap = edge->m_edge_capacity;
      if (cap < min_capacity) {
        min_capacity = cap;
      }
      path++;
    }
    CapType final_node_capacity = parent_node->m_excess > 0 ?
                                  parent_node->m_excess :
                                  -parent_node->m_excess;
    if (final_node_capacity < min_capacity) {
      min_capacity = final_node_capacity;
    }
  }
  // if (m_path != path) {
  //   printf("path = %d\n", path);
  //   m_path = path;
  // }

  first_edge[0]->m_edge_capacity += min_capacity;
  first_edge[1]->m_edge_capacity -= min_capacity;
  for (int i = 0; i < 2; ++i) {
    Node* parent_node = first_edge[i]->m_dst_node;
    int factor = i == 0 ? -1 : 1;
    for (Edge* parent_edge = parent_node->m_parent_edge; parent_edge != TERMINAL;
         parent_node = parent_edge->m_dst_node, parent_edge = parent_node->m_parent_edge) {
      parent_edge->m_edge_capacity += (-factor) * min_capacity;
      parent_edge->m_rev_edge->m_edge_capacity += factor * min_capacity;
      Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
      if (!edge->m_edge_capacity) {
        AddOrphanNode(parent_node);
        Node* node = parent_edge->m_dst_node->m_first_child_node;
        if (node == parent_node) {
          parent_edge->m_dst_node->m_first_child_node = parent_node->m_next_child_node;
        } else {
          for (; node->m_next_child_node != parent_node; node = node->m_next_child_node);
          node->m_next_child_node = parent_node->m_next_child_node;
        }
      }
    }
    parent_node->m_excess += factor * min_capacity;
    if (!parent_node->m_excess) {
      AddOrphanNode(parent_node);
    }
  }
  m_flow += min_capacity;
}
#else
template <class CapType>
void Graph<CapType>::Augment(Edge* bridge) {
  Node *x, *y;
  Edge *a;
  double bottleneck, bottleneckT, bottleneckS;
  int minOrphanLevel;
  bool forceBottleneck = false;

  // must compute forceBottleneck once, so that it is constant throughout this method
  bottleneck = bottleneckS = bridge->m_edge_capacity;
  for (x=bridge->m_rev_edge->m_dst_node; ; x=a->m_dst_node)
  {
    if (x->m_excess) break;
    a = x->m_parent_edge;
    if (bottleneckS > a->m_rev_edge->m_edge_capacity) {
      bottleneckS = a->m_rev_edge->m_edge_capacity;
    }
  }
  if (bottleneckS > x->m_excess) {
    bottleneckS = x->m_excess;
  }
  if (x->m_terminal_dist != 1) forceBottleneck = true;
  if (x == bridge->m_rev_edge->m_dst_node) bottleneck = bottleneckS;

  bottleneckT = bridge->m_edge_capacity;
  for (x=bridge->m_dst_node; ; x=a->m_dst_node)
  {
    if (x->m_excess) break;
    a = x->m_parent_edge;
    if (bottleneckT > a->m_edge_capacity) {
      bottleneckT = a->m_edge_capacity;
    }
  }
  if (bottleneckT > (-x->m_excess)) {
    bottleneckT = (-x->m_excess);
  }
  if (x->m_terminal_dist != 1) forceBottleneck = true;
  if (x == bridge->m_dst_node && bottleneck > bottleneckT) bottleneck = bottleneckT;

  if (forceBottleneck) {
    if (bottleneckS < bottleneckT) bottleneck = bottleneckS;
    else bottleneck = bottleneckT;
  }

  // augment connecting arc
  bridge->m_rev_edge->m_edge_capacity += bottleneck;
  // bridge->isRevResidual = 1;
  bridge->m_edge_capacity -= bottleneck;
  m_flow -= bottleneck;

  // augment T
  x = bridge->m_dst_node;
  assert(x->m_node_state == SINK);
  // if (!IB_EXCESSES || bottleneck == 1 || forceBottleneck) {
  //   minOrphanLevel = augmentPath<false>(x, bottleneck);
  //   adoption<false>(minOrphanLevel, true);
  // } else {
    minOrphanLevel = augmentExcess(x, bottleneck);
    adoption(minOrphanLevel, false);
    augmentExcesses();
  // }

  // augment S
  x = bridge->m_rev_edge->m_dst_node;
  assert(x->m_node_state == SOURCE);
  // if (!IB_EXCESSES || bottleneck == 1 || forceBottleneck) {
  //   minOrphanLevel = augmentPath<true>(x, bottleneck);
  //   adoption<true>(minOrphanLevel, true);
  // } else {
    minOrphanLevel = augmentExcess(x, bottleneck);
    adoption(minOrphanLevel, false);
    augmentExcesses();
  // }
}
#endif

#ifdef EIBFS

#define REMOVE_SIBLING(x, tmp) \
  { (tmp) = (x)->m_parent_edge->m_dst_node->m_first_child_node; \
  if ((tmp) == (x)) { \
    (x)->m_parent_edge->m_dst_node->m_first_child_node = (x)->m_next_child_node; \
  } else { \
    for (; (tmp)->m_next_child_node != (x); (tmp) = (tmp)->m_next_child_node); \
    (tmp)->m_next_child_node = (x)->m_next_child_node; \
  } }

#define ADD_SIBLING(x, parentNode) \
  { (x)->m_next_child_node = (parentNode)->m_first_child_node; \
  (parentNode)->m_first_child_node = (x); \
  }

// @ret: minimum level in which created an orphan
template <class CapType>
int Graph<CapType>::augmentExcess(Node *x, double push)
{
  Node *y;
  Edge *a;
  int orphanMinLevel = (x->m_node_state == SOURCE ? m_global_source_dist : m_global_sink_dist) + 1;
  augTimestamp++;

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
  while (x->m_node_state == SOURCE ? (x->m_excess <= 0) : (x->m_excess >= 0))
  {
    a = x->m_parent_edge;

    // update excess and find next flow
    if ((x->m_node_state == SOURCE ? (a->m_rev_edge->m_edge_capacity) : (a->m_edge_capacity))
        < (x->m_node_state == SOURCE ? (push-x->m_excess) : (x->m_excess+push))) {
      // some excess remains, node is an orphan
      x->m_excess += (x->m_node_state == SOURCE ? (a->m_rev_edge->m_edge_capacity- push) : (push-a->m_edge_capacity));
      push = (x->m_node_state == SOURCE ? a->m_rev_edge->m_edge_capacity : a->m_edge_capacity);
    } else {
      // all excess is pushed out, node may or may not be an orphan
      push += (x->m_node_state == SOURCE ? -(x->m_excess) : x->m_excess);
      x->m_excess = 0;
    }

    // push flow
    // note: push != 0
    if (x->m_node_state == SOURCE) {
      a->m_edge_capacity += push;
      // a->rev->isRevResidual = 1;
      a->m_rev_edge->m_edge_capacity -= push;
    } else {
      a->m_rev_edge->m_edge_capacity += push;
      // a->isRevResidual = 1;
      a->m_edge_capacity -= push;
    }

    // saturated?
    if ((x->m_node_state == SOURCE ? (a->m_rev_edge->m_edge_capacity) : (a->m_edge_capacity)) == 0)
    {
      // if (sTree) a->isRevResidual = 0;
      // else a->rev->isRevResidual = 0;
      REMOVE_SIBLING(x,y);
      // Node* node = x->m_parent_edge->m_dst_node->m_first_child_node;
      // if (node == x) {
      //   x->m_parent_edge->m_dst_node->m_first_child_node = x->m_next_child_node;
      // } else {
      //   for (; node->m_next_child_node != x; node = node->m_next_child_node);
      //   node->m_next_child_node = x->m_next_child_node;
      // }
      orphanMinLevel = x->m_terminal_dist;
      orphanBuckets.add(x);
      if (x->m_excess) excessBuckets.incMaxBucket(x->m_terminal_dist);
    }

    // advance
    // a precondition determines that the first node on the path is not in excess buckets
    // so only the next nodes may need to be removed from there
    x = a->m_dst_node;
    if (x->m_node_state == SOURCE ? (x->m_excess < 0) : (x->m_excess > 0)) excessBuckets.remove(x);
  }

  // update the excess at the root
  if (push <= (x->m_node_state == SOURCE ? (x->m_excess) : -(x->m_excess))) m_flow += push;
  else m_flow += (x->m_node_state == SOURCE ? (x->m_excess) : -(x->m_excess));
  x->m_excess += (x->m_node_state == SOURCE ? (-push) : push);
  if (x->m_node_state == SOURCE ? (x->m_excess <= 0) : (x->m_excess >= 0)) {
    orphanMinLevel = x->m_terminal_dist;
    orphanBuckets.add(x);
    if (x->m_excess) excessBuckets.incMaxBucket(x->m_terminal_dist);
  }

  return orphanMinLevel;
}

template <class CapType>
void Graph<CapType>::augmentExcesses()
{
  Node *x;
  int minOrphanLevel;
  int adoptedUpToLevel = excessBuckets.maxBucket;

  if (!excessBuckets.empty())
  for (; excessBuckets.maxBucket != (excessBuckets.minBucket-1); excessBuckets.maxBucket--)
  while ((x=excessBuckets.popFront(excessBuckets.maxBucket)) != NULL)
  {
    minOrphanLevel = augmentExcess(x, 0);
    // if we did not create new orphans
    if (adoptedUpToLevel < minOrphanLevel) minOrphanLevel = adoptedUpToLevel;
    adoption(minOrphanLevel, false);
    adoptedUpToLevel = excessBuckets.maxBucket;
  }
  excessBuckets.reset();
  if (orphanBuckets.maxBucket != 0) adoption(adoptedUpToLevel+1, true);
  while ((x=excessBuckets.popFront(0)) != NULL) orphanFree(x);
}

template <class CapType>
void Graph<CapType>::adoption(int fromLevel, bool toTop)
{
  Node *x, *y, *z;
  Edge *a;
  Edge *aEnd;
  int minLabel;
  int level;

  for (level = fromLevel;
    level <= orphanBuckets.maxBucket; // && (!IB_EXCESSES || toTop || level <= excessBuckets.maxBucket);
    level++)
  while ((x=orphanBuckets.popFront(level)) != NULL)
  {
    if (x->lastAugTimestamp != augTimestamp) {
      x->lastAugTimestamp = augTimestamp;
      if (x->m_node_state == SOURCE) m_global_source_orphan_num++;
      else m_global_sink_orphan_num++;
    }

    x->m_parent_edge = NULL;
    if (x->m_terminal_dist != 1)
    {
      minLabel = x->m_terminal_dist - 1;
      for (int i = 0; i < x->m_out_edges_num; ++i) {
        a = &x->m_out_edges[i];
        y = a->m_dst_node;
        if ((x->m_node_state == SOURCE ? a->m_rev_edge->m_edge_capacity : a->m_edge_capacity) != 0 &&
            x->m_node_state == y->m_node_state && y->m_terminal_dist == minLabel && y->m_parent_edge)
        {
          x->m_parent_edge = a;
          ADD_SIBLING(x,y);
          // x->m_next_child_node = y->m_first_child_node;
          // y->m_first_child_node = x;
          break;
        }
      }
    }
    if (x->m_parent_edge != NULL) {
      if (x->m_excess) excessBuckets.add(x);
      continue;
    }

    //
    // on the top level there is no need to relabel
    //
    if (x->m_terminal_dist == (x->m_node_state == SOURCE ? m_global_source_dist : m_global_sink_dist)) {
      orphanFree(x);
      continue;
    }

    //
    // give up on same level - relabel it!
    // (1) create orphan sons
    //
    for (y=x->m_first_child_node; y != NULL; y=z)
    {
      z=y->m_next_child_node;
      if (y->m_excess) excessBuckets.remove(y);
      orphanBuckets.add(y);
    }
    x->m_first_child_node = NULL;

    //
    // (2) relabel: find the lowest level parent
    //
    minLabel = (x->m_node_state == SOURCE ? m_global_source_dist : m_global_sink_dist);
    if (x->m_terminal_dist != minLabel) {
      for (int i = 0; i < x->m_out_edges_num; ++i) {
        a = &x->m_out_edges[i];
        y = a->m_dst_node;
        if ((x->m_node_state == SOURCE ? a->m_rev_edge->m_edge_capacity : a->m_edge_capacity) &&
          x->m_node_state == y->m_node_state && y->m_terminal_dist < minLabel &&
          y->m_parent_edge) {
          minLabel = y->m_terminal_dist;
          x->m_parent_edge = a;
          if (minLabel == x->m_terminal_dist) break;
        }
      }
    }

    //
    // (3) relabel onto new parent
    //
    if (x->m_parent_edge != NULL) {
      x->m_terminal_dist = minLabel + 1;
      ADD_SIBLING(x, x->m_parent_edge->m_dst_node);
      // x->m_next_child_node = x->m_parent_edge->m_dst_node->m_first_child_node;
      // x->m_parent_edge->m_dst_node->m_first_child_node = x;
      // add to active list of the next growth phase
      AddActiveNodeBack(x);
      if (x->m_excess) excessBuckets.add(x);
    } else {
      orphanFree(x);
    }
  }
  if (level > orphanBuckets.maxBucket) orphanBuckets.maxBucket=0;
}
#endif

template <class CapType>
void Graph<CapType>::FindNewPath(Node* orphan_node) {
  Edge* connected_edge_min = NULL;
  if (orphan_node->m_terminal_dist == 1) {
    orphan_node->m_parent_edge = NULL;
    return;
  }

  for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
    Edge* connected_edge = &orphan_node->m_out_edges[i];
    if (connected_edge->m_dst_node->m_node_state == orphan_node->m_node_state) {
      CapType capacity = orphan_node->m_node_state == SINK ?
                         connected_edge->m_edge_capacity :
                         connected_edge->m_rev_edge->m_edge_capacity;
      if (!capacity || connected_edge->m_dst_node->m_terminal_dist !=
          orphan_node->m_terminal_dist - 1) {
        continue;
      }
      if (connected_edge->m_dst_node->m_parent_edge) {
        connected_edge_min = connected_edge;
        assert(orphan_node->m_terminal_dist ==
               connected_edge_min->m_dst_node->m_terminal_dist + 1);
        break;
      }
    }
  }
  orphan_node->m_parent_edge = connected_edge_min;
  if (connected_edge_min) {
    Node* cen_node = connected_edge_min->m_dst_node;
    orphan_node->m_next_child_node = cen_node->m_first_child_node;
    cen_node->m_first_child_node = orphan_node;
    // orphan_node->m_timestamp = m_global_timestamp;
    orphan_node->m_terminal_dist = connected_edge_min->m_dst_node->m_terminal_dist + 1;
  }
}

template <class CapType>
CapType Graph<CapType>::MaxFlow() {
  // get active nodes
  for (int i = 0; i < m_nodes.size(); ++i) {
    Node* node = &m_nodes[i];
    if (node->m_excess != 0) {
      node->m_node_state = node->m_excess < 0 ? SINK : SOURCE;
      node->m_parent_edge = TERMINAL;
      node->m_terminal_dist = 1;
      if (node->m_node_state == SOURCE) {
        AddActiveSourceNodeBack(node);
      } else {
        AddActiveSinkNodeBack(node);
      }
    }
  }

  Node* at_node = NULL;
  Edge* meet_edge = NULL;
  int path = 0;
  bool sink_empty = false;
  CountTime ct;
  double time = 0;
  while (true) {
    if (meet_edge == NULL || at_node->m_parent_edge == NULL) {
      meet_edge = NULL;
      if (m_global_state == SOURCE) {
        at_node = GetActiveSourceNode();
      } else {
        at_node = GetActiveSinkNode();
      }
      if (IsActiveSourceEmpty() && IsActiveSinkEmpty() && !at_node) {
        break;
      }
      if (!at_node) {
        continue;
      }
#ifdef EIBFS
    orphanBuckets.allocate((m_global_source_dist > m_global_sink_dist) ? m_global_source_dist : m_global_sink_dist);
    excessBuckets.allocate((m_global_source_dist > m_global_sink_dist) ? m_global_source_dist : m_global_sink_dist);
#endif
    }
    assert(at_node);
    meet_edge = NULL;
    if (at_node->m_node_state == SOURCE) {
      if (at_node->m_terminal_dist != m_global_source_dist - 1) {
        assert(at_node->m_terminal_dist == m_global_source_dist);
        AddActiveSourceNodeBack(at_node);
        continue;
      }
    } else {
      if (at_node->m_terminal_dist != m_global_sink_dist - 1) {
        assert(at_node->m_terminal_dist == m_global_sink_dist);
        AddActiveSinkNodeBack(at_node);
        continue;
      }
    }

    // grow source tree and sink tree
    for (int i = 0; i < at_node->m_out_edges_num; ++i) {
      Edge* connected_edge = &at_node->m_out_edges[i];
      CapType capacity = at_node->m_node_state == SINK ?
                         connected_edge->m_rev_edge->m_edge_capacity :
                         connected_edge->m_edge_capacity;
      if (capacity != 0) {
        Node* dst_node = connected_edge->m_dst_node;
        if (!dst_node->m_parent_edge) {
          dst_node->m_parent_edge = connected_edge->m_rev_edge;
          dst_node->m_next_child_node = at_node->m_first_child_node;
          at_node->m_first_child_node = dst_node;
          // dst_node->m_timestamp = at_node->m_timestamp;
          dst_node->m_terminal_dist = at_node->m_terminal_dist + 1;
          dst_node->m_node_state = at_node->m_node_state;
          if (dst_node->m_node_state == SOURCE) {
            AddActiveSourceNodeBack(dst_node);
          } else {
            AddActiveSinkNodeBack(dst_node);
          }
        } else if (dst_node->m_node_state != UNKNOWN &&
            dst_node->m_node_state != at_node->m_node_state) {
          meet_edge = at_node->m_node_state == SINK ?
                      connected_edge->m_rev_edge : connected_edge;
          break;
        }
        // else if (dst_node->m_timestamp <= at_node->m_timestamp &&
        //     dst_node->m_terminal_dist > at_node->m_terminal_dist) {
        //   dst_node->m_parent_edge = connected_edge->m_rev_edge;
        //   dst_node->m_timestamp = at_node->m_timestamp;
        //   dst_node->m_terminal_dist = at_node->m_terminal_dist + 1;
        // }
      }
    }

    // m_global_timestamp++;

    if (meet_edge) {
      Augment(meet_edge);

#ifndef EIBFS
      // adopt orphan nodes
      while (true) {
        Node* orphan_node = GetOrphanNode();
        if (!orphan_node) {
          break;
        }
        if (orphan_node->m_node_state == SOURCE) {
          m_global_source_orphan_num++;
        } else {
          m_global_sink_orphan_num++;
        }

        FindNewPath(orphan_node);

        if (!orphan_node->m_parent_edge) {
          for (Node* child = orphan_node->m_first_child_node;
               child; child = child->m_next_child_node) {
            AddOrphanNode(child);
          }
          orphan_node->m_first_child_node = NULL;

          int dist_min = (orphan_node->m_node_state == SOURCE) ?
                         m_global_source_dist : m_global_sink_dist;
          if (orphan_node->m_terminal_dist != dist_min) {
            for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
              Edge* connected_edge = &orphan_node->m_out_edges[i];
              Node* dst_node = connected_edge->m_dst_node;
              Edge* parent_edge = dst_node->m_parent_edge;
              CapType capacity = dst_node->m_node_state == SINK ?
                                 connected_edge->m_edge_capacity :
                                 connected_edge->m_rev_edge->m_edge_capacity;
              if (parent_edge && capacity &&
                  dst_node->m_node_state == orphan_node->m_node_state &&
                  dst_node->m_terminal_dist < dist_min) {
                orphan_node->m_parent_edge = connected_edge;
                dist_min = dst_node->m_terminal_dist;
                if (dist_min == orphan_node->m_terminal_dist) {
                  break;
                }
              }
            }
            if (orphan_node->m_parent_edge) {
              Node* dst_node = orphan_node->m_parent_edge->m_dst_node;
              orphan_node->m_next_child_node = dst_node->m_first_child_node;
              dst_node->m_first_child_node = orphan_node;
              assert(orphan_node->m_terminal_dist < dst_node->m_terminal_dist + 1);
              // orphan_node->m_timestamp = dst_node->m_timestamp;
              orphan_node->m_terminal_dist = dst_node->m_terminal_dist + 1;
              AddActiveNodeBack(orphan_node);
            }
          }
        }
      }
#endif
    }
  }
  // printf("path = %d, flow = %f, time = %f, count = %d\n", m_path, m_flow, time, m_count);
  return m_flow;
}

template <class CapType>
bool Graph<CapType>::IsBelongToSource(int node_id) {
  return m_nodes[node_id].m_node_state == SOURCE;
}

} // namespace

#undef TERMINAL
#undef ORPHAN
#undef IB_ALLOC_INIT_LEVELS
#undef IB_PREVPTR_EXCESS
#undef IB_NEXTPTR_EXCESS
#undef REMOVE_SIBLING
#undef ADD_SIBLING
#endif  // INCLUDE_GRAPH_H_
