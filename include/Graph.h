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

// no ibfs
#define IFAL

namespace user {

template <class CapType>
class Graph {
 public:
  enum NodeState {
    SOURCE,
    SINK,
    UNKNOWN
  };
  Graph(int max_nodes_number, int max_edges_number);
  void AddNode(int node_id, CapType source_capacity, CapType sink_capacity);
  void AddEdge(int src_node_id, int dst_node_id, CapType edge_capacity);
  void Init();
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
      , m_parent_edge(NULL)
      , m_timestamp(0)
      , m_terminal_dist(0)
      , m_out_edges_num(0)
      , m_next_active(NULL)
      , m_is_active(false)
#ifdef IFAL
      , m_child_edge(NULL)
      , m_needed_flow(0)
      , m_next_node(NULL)
      , m_in_stack(false)
#endif
      , m_id(0) {}
    CapType m_excess;
    NodeState m_node_state;
    Edge* m_parent_edge;
    // the timestamp of the latest dist calculating 
    int m_timestamp;
    int m_terminal_dist;
    Edge m_out_edges[NEIGHBOUR];
    int m_out_edges_num;
    Node* m_next_active;
    bool m_is_active;
#ifdef IFAL
    Edge* m_child_edge;
    CapType m_needed_flow;
    Node* m_next_node;
    bool m_in_stack;
#endif
    int m_id;
  };

  void AddActiveNodeBack(Node* node) {
    if (!node->m_is_active) {
      node->m_is_active = true;
      if (m_last_at_node) {
        m_last_at_node->m_next_active = node;
        m_last_at_node = node;
      } else {
        m_first_at_node = node;
        m_last_at_node = node;
      }
      node->m_next_active = NULL;
    }
  }
  Node* GetActiveNode() {
    Node* active_node = NULL;
    while (m_first_at_node) {
      active_node = m_first_at_node;
      m_first_at_node = m_first_at_node->m_next_active;
      if (!m_first_at_node) {
        m_last_at_node = NULL;
      }
      active_node->m_is_active = false;
      if (active_node->m_parent_edge) {
        break;
      }
      active_node = NULL;
    }
    return active_node;
  }
  void AddOrphanNode(Node* orphan_node) {
    // assert(orphan_node->m_parent_edge != ORPHAN);
    orphan_node->m_parent_edge = ORPHAN;
    m_orphan_nodes.push(orphan_node);
  }
  Node* GetOrphanNode() {
    Node* orphan_node = NULL;
    if (!m_orphan_nodes.empty()) {
      orphan_node = m_orphan_nodes.front();
      // assert(orphan_node->m_parent_edge == ORPHAN);
      m_orphan_nodes.pop();
    }
    return orphan_node;
  }

#ifdef IFAL
  void AdoptNewPath(Node* node);
  void Same(CapType& dst_val, CapType comp_val);
  void PushFlow(Node* src_node, Node* dst_node, Edge* flow_edge, CapType push_flow);
  void SetResFlow(bool node_type, CapType minus_flow);

  bool Empty(bool node_type);
  void Push(Node* node);
  Node*& Top(bool node_type);
  void Pop(bool node_type);
  void PushEnoughFlowToOneNode(bool node_type);

  void PushEnoughFlowToTwoNodes(Node* source_node, Node* sink_node);
  void PushEnoughFlow(Node* source_node, Node* sink_node, CapType* min_edge_capacity);
#endif
  void Augment(Edge* meet_edge);
  void FindNewPath(Node* orphan_node);
  void FindNewOrphans(Node* orphan_node);
  void Adoption();

  std::vector<Node> m_nodes;
  Node* m_first_at_node;
  Node* m_last_at_node;
  // point to the last item used in m_edges
  int m_global_timestamp;
  std::queue<Node*> m_orphan_nodes;
  int m_path;
  int m_count;
  double m_time;
  double m_time2;
  // ImageData<int>* m_marked_image;
#ifdef IFAL
  CapType m_source_res_flow;
  bool m_is_source_end;
  CapType m_sink_res_flow;
  bool m_is_sink_end;
  Node* m_top_source_node;
  Node* m_top_sink_node;
#endif
  CapType m_flow;
};

template <class CapType>
Graph<CapType>::Graph(int max_nodes_number, int max_edges_number)
  : m_nodes(std::vector<Node>(max_nodes_number))
  , m_first_at_node(NULL)
  , m_last_at_node(NULL)
  , m_global_timestamp(1)
  , m_path(0)
  , m_count(0)
  , m_time(0)
  , m_time2(0)
  // , m_marked_image(marked_image)
#ifdef IFAL
  , m_source_res_flow(0)
  , m_is_source_end(false)
  , m_sink_res_flow(0)
  , m_is_sink_end(false)
  , m_top_source_node(NULL)
  , m_top_sink_node(NULL)
#endif
  , m_flow(0) {}

template <class CapType>
void Graph<CapType>::AddNode(int node_id, CapType source_capacity, CapType sink_capacity) {
  // assert(node_id >= 0 && node_id < m_nodes.size() && m_nodes[node_id].m_excess == 0);
  m_flow += source_capacity < sink_capacity ? source_capacity : sink_capacity;
  CapType node_capacity = source_capacity - sink_capacity;
  m_nodes[node_id].m_excess = node_capacity;
  m_nodes[node_id].m_id = node_id;
  if (node_capacity) {
    m_nodes[node_id].m_node_state = node_capacity < 0 ? SINK : SOURCE;
    m_nodes[node_id].m_parent_edge = TERMINAL;
    m_nodes[node_id].m_terminal_dist = 1;
  }
}

template <class CapType>
void Graph<CapType>::AddEdge(int src_node_id, int dst_node_id, CapType edge_capacity) {
  // assert(src_node_id >= 0 && src_node_id < m_nodes.size());
  // assert(dst_node_id >= 0 && dst_node_id < m_nodes.size());

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

#ifdef IFAL
template <class CapType>
void Graph<CapType>::Same(CapType& dst_val, CapType comp_val) {
  if (ABS(dst_val- comp_val) < EPSILON) {
    dst_val = comp_val;
  }
}

template <class CapType>
void Graph<CapType>::PushFlow(
  Node* src_node, Node* dst_node, Edge* flow_edge, CapType push_flow) {
  // assert(src_node->m_node_state == dst_node->m_node_state);
  CapType node_type = src_node->m_node_state;
  CapType flow = std::min(ABS(src_node->m_excess), ABS(push_flow));
  flow = std::min(flow, flow_edge->m_edge_capacity);

  src_node->m_excess -= node_type == SOURCE ? flow : -flow;
  Same(src_node->m_excess, 0);

  dst_node->m_excess += node_type == SOURCE ? flow : -flow;
  flow_edge->m_edge_capacity -= flow;
  Same(flow_edge->m_edge_capacity, 0);

  flow_edge->m_rev_edge->m_edge_capacity += flow;
  dst_node->m_needed_flow -= flow;
  Same(dst_node->m_needed_flow, 0);
  m_path++;
}

template <class CapType>
void Graph<CapType>::AdoptNewPath(Node* node) {
  if (node->m_parent_edge == TERMINAL) {
    AddOrphanNode(node);
  }
  Adoption();
}

template <class CapType>
void Graph<CapType>::SetResFlow(bool node_type, CapType minus_flow) {
  if (!minus_flow) {
    return;
  }
  CapType& res_flow = node_type == SOURCE ? m_source_res_flow : m_sink_res_flow;
  bool& node_end = node_type == SOURCE ? m_is_source_end : m_is_sink_end;
  res_flow -= ABS(minus_flow);
  if (res_flow <= 0) {
    node_end = true;
  }
}

template <class CapType>
bool Graph<CapType>::Empty(bool node_type) {
  Node*& top_node = node_type == SOURCE ? m_top_source_node : m_top_sink_node;
  if (!top_node) {
    return true;
  }
  return false;
}

template <class CapType>
void Graph<CapType>::Push(Node* node) {
  Node*& top_node = node->m_node_state == SOURCE ? m_top_source_node : m_top_sink_node;
  node->m_next_node = top_node;
  top_node = node;
  node->m_in_stack = true;
}

template <class CapType>
typename Graph<CapType>::Node*&
  Graph<CapType>::Top(bool node_type) {
  return node_type == SOURCE ? m_top_source_node : m_top_sink_node;
}

template <class CapType>
void Graph<CapType>::Pop(bool node_type) {
  Node*& top_node = node_type == SOURCE ? m_top_source_node : m_top_sink_node;
  // assert(top_node);
  Node* tmp = top_node;
  top_node = top_node->m_next_node;
  tmp->m_next_node = NULL;
  tmp->m_in_stack = false;
  tmp->m_child_edge = NULL;
  if (tmp->m_parent_edge == TERMINAL) {
    tmp->m_timestamp = m_global_timestamp;
  } else if (tmp->m_parent_edge && tmp->m_parent_edge != ORPHAN) {
    tmp->m_terminal_dist = tmp->m_parent_edge->m_dst_node->m_terminal_dist + 1;
    tmp->m_timestamp = tmp->m_parent_edge->m_dst_node->m_timestamp;
  }
}

template <class CapType>
void Graph<CapType>::PushEnoughFlowToOneNode(bool node_type) {
  // assert(!Empty(node_type));
  Node* node = Top(node_type);
  bool& node_end = node_type == SOURCE ? m_is_source_end : m_is_sink_end;
  CapType& node_res_flow = node_type == SOURCE ? m_source_res_flow : m_sink_res_flow;

  CapType push_flow = node->m_excess;
  if (node->m_needed_flow < 0) {
    push_flow = node->m_needed_flow;
    node->m_needed_flow = 0;
  }
  while (!node_end && node->m_parent_edge && node->m_needed_flow) {
    if (node->m_parent_edge == ORPHAN || node->m_parent_edge == TERMINAL) {
      AdoptNewPath(node);
      if (!node->m_parent_edge) {
        break;
      }
    }

    // assert(node->m_needed_flow && node->m_parent_edge);
    Node* dst_node = node->m_parent_edge->m_dst_node;
    dst_node->m_child_edge = node->m_parent_edge->m_rev_edge;
    Edge* flow_edge = node_type == SINK ?
      node->m_parent_edge : node->m_parent_edge->m_rev_edge;
    // assert(flow_edge->m_edge_capacity);
    CapType src_nd_flow = std::min(flow_edge->m_edge_capacity, node->m_needed_flow);
    CapType dst_nd_flow = src_nd_flow - ABS(dst_node->m_excess);
    CapType minus_flow;
    if (dst_nd_flow > 0) {
      minus_flow = dst_node->m_excess;
      dst_node->m_needed_flow = dst_nd_flow;
    } else {
      minus_flow = src_nd_flow;
      dst_node->m_needed_flow = -src_nd_flow;
      // assert(node->m_needed_flow);
      // assert(dst_node->m_parent_edge == TERMINAL);
    }
    SetResFlow(node_type, minus_flow);
    Push(dst_node);
    return;
  }
  if (node->m_child_edge) {
    Node* child_node = node->m_child_edge->m_dst_node;
    Edge* flow_edge = node_type == SOURCE ?
      node->m_child_edge : node->m_child_edge->m_rev_edge;
    if (push_flow) {
      PushFlow(node, child_node, flow_edge, push_flow);
    }
    if (!flow_edge->m_edge_capacity && child_node->m_parent_edge) {
      // assert(child_node->m_parent_edge->m_dst_node == node);
      AddOrphanNode(child_node);
    }
  }
  Pop(node_type);
}

// node is current Node
// needed_flow is except the excess of the node what the node need in flow.
// res_flow is the total resident flow
template <class CapType>
void Graph<CapType>::PushEnoughFlowToTwoNodes(Node* source_node, Node* sink_node) {
  m_is_source_end = false;
  m_is_sink_end = false;
  source_node->m_child_edge = NULL;
  sink_node->m_child_edge = NULL;
  source_node->m_needed_flow = std::max(m_source_res_flow - source_node->m_excess, 0.0);
  sink_node->m_needed_flow = std::max(m_sink_res_flow + sink_node->m_excess, 0.0);
  SetResFlow(SOURCE, source_node->m_excess);
  SetResFlow(SINK, sink_node->m_excess);
  if (!m_is_source_end) {
    Push(source_node);
  }
  if (!m_is_sink_end) {
    Push(sink_node);
  }

  bool is_source_once = false;
  bool is_sink_once = false;
  while (!Empty(SOURCE) || !Empty(SINK)) {
    if (!Empty(SOURCE) && (m_source_res_flow > m_sink_res_flow || Empty(SINK))) {
      if (m_source_res_flow < m_sink_res_flow && !is_source_once) {
        is_source_once = true;
        Node* top_node = Top(SOURCE);
        if (top_node->m_needed_flow < 0) {
          top_node->m_needed_flow = top_node->m_needed_flow + m_sink_res_flow - m_source_res_flow;
        } else {
          top_node->m_needed_flow = -top_node->m_excess + m_sink_res_flow - m_source_res_flow;
        }
        // assert(top_node->m_needed_flow <= 0);
        m_is_source_end = true;
      }
      PushEnoughFlowToOneNode(SOURCE);
    } else {
      if (m_source_res_flow > m_sink_res_flow && !is_sink_once) {
        is_sink_once = true;
        Node* top_node = Top(SINK);
        if (top_node->m_needed_flow < 0) {
          top_node->m_needed_flow = top_node->m_needed_flow + m_source_res_flow - m_sink_res_flow;
        } else {
          top_node->m_needed_flow = top_node->m_excess + m_source_res_flow - m_sink_res_flow;
        }
        // assert(top_node->m_needed_flow <= 0);
        m_is_sink_end = true;
      }
      PushEnoughFlowToOneNode(SINK);
    }
  }
}

template <class CapType>
void Graph<CapType>::PushEnoughFlow(
  Node* source_node, Node* sink_node, CapType* min_edge_capacity) {
  m_source_res_flow = *min_edge_capacity;
  m_sink_res_flow = *min_edge_capacity;

  PushEnoughFlowToTwoNodes(source_node, sink_node);

  Same(source_node->m_excess, *min_edge_capacity);
  Same(sink_node->m_excess, -*min_edge_capacity);
  Same(source_node->m_excess, -sink_node->m_excess);
  *min_edge_capacity = std::min(*min_edge_capacity, source_node->m_excess);
  *min_edge_capacity = std::min(*min_edge_capacity, -sink_node->m_excess);
}
#endif

template <class CapType>
void Graph<CapType>::FindNewOrphans(Node* orphan_node) {
  // assert(!orphan_node->m_parent_edge);
  for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
    Edge* connected_edge = &orphan_node->m_out_edges[i];
    Node* dst_node = connected_edge->m_dst_node;
    Edge* parent_edge = dst_node->m_parent_edge;

    if (dst_node->m_node_state == orphan_node->m_node_state) {
      AddActiveNodeBack(dst_node);
      if (parent_edge && parent_edge != TERMINAL && parent_edge != ORPHAN &&
          parent_edge->m_dst_node == orphan_node) {
        AddOrphanNode(dst_node);
      }
    }
  }
}

template <class CapType>
void Graph<CapType>::FindNewPath(Node* orphan_node) {
  bool orphan_in_stack = orphan_node->m_in_stack;
  int dist_min = INIFINITE_DIST;
  Edge* connected_edge_min = NULL;
  CapType cap_max = 0;

  for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
    Edge* connected_edge = &orphan_node->m_out_edges[i];
    if (connected_edge->m_dst_node->m_node_state == orphan_node->m_node_state) {
      CapType capacity = orphan_node->m_node_state == SINK ?
                         connected_edge->m_edge_capacity :
                         connected_edge->m_rev_edge->m_edge_capacity;
      if (!capacity) {
        continue;
      }
      Node* dst_node = connected_edge->m_dst_node;
      Edge* parent_edge = dst_node->m_parent_edge;
      CapType dst_cap = 0;
      if (parent_edge) {
        int dist = 0;
        while (true) {
          if (dst_node->m_timestamp == m_global_timestamp) {
            dist += dst_node->m_terminal_dist;
            break;
          }
          parent_edge = dst_node->m_parent_edge;
          dist++;
          if (parent_edge == TERMINAL) {
            dst_node->m_timestamp = m_global_timestamp;
            dst_node->m_terminal_dist = 1;
            dst_cap = ABS(dst_node->m_excess);
            break;
          }
          if ((orphan_in_stack && dst_node->m_in_stack) ||
              (!orphan_in_stack && dst_node->m_terminal_dist > orphan_node->m_terminal_dist) ||
              parent_edge == ORPHAN) {
            dist = INIFINITE_DIST;
            break;
          }
          // assert(parent_edge);
          dst_node = parent_edge->m_dst_node;
        }
        if (dist < INIFINITE_DIST) {
          if (dist < dist_min || (dist == dist_min && dst_cap > cap_max)) {
            connected_edge_min = connected_edge;
            dist_min = dist;
            cap_max = dst_cap;
            if (dist_min == 1) {
              break;
            }
          }
          for (dst_node = connected_edge->m_dst_node;
               dst_node->m_timestamp != m_global_timestamp;
               dst_node = dst_node->m_parent_edge->m_dst_node) {
            dst_node->m_timestamp = m_global_timestamp;
            dst_node->m_terminal_dist = dist--;
          }
        }
      }
    }
  }
  orphan_node->m_parent_edge = connected_edge_min;
  if (connected_edge_min) {
    orphan_node->m_timestamp = m_global_timestamp;
    orphan_node->m_terminal_dist = connected_edge_min->m_dst_node->m_terminal_dist + 1;
  }
}

template <class CapType>
void Graph<CapType>::Adoption() {
  // adopt orphan nodes
  // CountTime ct;
  // ct.ContBegin();
  while (true) {
    Node* orphan_node = GetOrphanNode();
    if (!orphan_node) {
      break;
    }
    m_count++;

    FindNewPath(orphan_node);

    if (!orphan_node->m_parent_edge) {
      FindNewOrphans(orphan_node);
    }
  }
  m_global_timestamp++;
  // ct.ContEnd();
  // m_time += ct.ContResult() * 1000;
}

template <class CapType>
void Graph<CapType>::Augment(Edge* meet_edge) {
  // augment path
  CapType min_capacity = meet_edge->m_edge_capacity;
  // first_edge[0] for source tree and first_edge[1] for sink tree
  Edge* first_edge[2] = {meet_edge->m_rev_edge, meet_edge};
  // find min capacity from path
#ifdef IFAL
  Node* flow_nodes[2] = {meet_edge->m_rev_edge->m_dst_node, meet_edge->m_dst_node};
#else
  for (int i = 0; i < 2; ++i) {
    Node* parent_node = first_edge[i]->m_dst_node;
    for (Edge* parent_edge = parent_node->m_parent_edge; parent_edge != TERMINAL;
         parent_node = parent_edge->m_dst_node, parent_edge = parent_node->m_parent_edge) {
      // assert(parent_edge);
      Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
      CapType cap = edge->m_edge_capacity;
      if (cap < min_capacity) {
        min_capacity = cap;
      }
    }
    CapType final_node_capacity = parent_node->m_excess > 0 ?
                                  parent_node->m_excess :
                                  -parent_node->m_excess;
    if (final_node_capacity < min_capacity) {
      min_capacity = final_node_capacity;
    }
  }
#endif

#ifdef IFAL
  PushEnoughFlow(flow_nodes[0], flow_nodes[1], &min_capacity);

  first_edge[0]->m_edge_capacity += min_capacity;
  first_edge[1]->m_edge_capacity -= min_capacity;
  CapType& source_excess = flow_nodes[0]->m_excess;
  CapType& sink_excess = flow_nodes[1]->m_excess;
  source_excess -= min_capacity;
  sink_excess += min_capacity;
  m_path++;
  // if (source_excess) {
  //   // assert(flow_nodes[0]->m_parent_edge == TERMINAL);
  // } else {
  //   // assert(flow_nodes[0]->m_parent_edge != TERMINAL);
  // }
  // if (sink_excess) {
  //   // assert(flow_nodes[1]->m_parent_edge == TERMINAL);
  // } else {
  //   // assert(flow_nodes[1]->m_parent_edge != TERMINAL);
  // }
#else
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
      }
      m_path++;
    }
    parent_node->m_excess += factor * min_capacity;
    if (!parent_node->m_excess) {
      AddOrphanNode(parent_node);
    }
  }
#endif
  m_flow += min_capacity;
}

template <class CapType>
void Graph<CapType>::Init() {
  // get active nodes
  for (int i = 0; i < m_nodes.size(); ++i) {
    Node* node = &m_nodes[i];
    if (node->m_excess != 0) {
#if 1
      NodeState cen_stat = node->m_node_state;
      for (int i = 0; i < node->m_out_edges_num; ++i) {
        Edge* connected_edge = &node->m_out_edges[i];
        NodeState arr_stat = connected_edge->m_dst_node->m_node_state;
        if (arr_stat != cen_stat) {
          AddActiveNodeBack(node);
          break;
        }
      }
#else
      AddActiveNodeBack(node);
#endif
    }
  }
}

template <class CapType>
CapType Graph<CapType>::MaxFlow() {
  // CountTime ct;
  Node* at_node = NULL;
  Edge* meet_edge = NULL;
  int path = 0;
  bool sink_empty = false;
  double time = 0;
  while (true) {
    if (meet_edge == NULL || at_node->m_parent_edge == NULL) {
      meet_edge = NULL;
      at_node = GetActiveNode();
      if (!at_node) {
        break;
      }
    }
    meet_edge = NULL;

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
          dst_node->m_timestamp = at_node->m_timestamp;
          dst_node->m_terminal_dist = at_node->m_terminal_dist + 1;
          dst_node->m_node_state = at_node->m_node_state;
          AddActiveNodeBack(dst_node);
        } else if (dst_node->m_node_state != UNKNOWN &&
            dst_node->m_node_state != at_node->m_node_state) {
          meet_edge = at_node->m_node_state == SINK ?
                      connected_edge->m_rev_edge : connected_edge;
          break;
        } else if (dst_node->m_timestamp <= at_node->m_timestamp &&
            dst_node->m_terminal_dist > at_node->m_terminal_dist) {
          dst_node->m_parent_edge = connected_edge->m_rev_edge;
          dst_node->m_timestamp = at_node->m_timestamp;
          dst_node->m_terminal_dist = at_node->m_terminal_dist + 1;
        }
      }
    }

    if (meet_edge) {
  // CountTime ct;
  // ct.ContBegin();
      Augment(meet_edge);
      Adoption();
  // ct.ContEnd();
  // m_time2 += ct.ContResult() * 1000;
    }
  }
  // printf("path = %d, path time = %f, orphan count = %d, time = %f\n", m_path, m_time2, m_count, m_time);
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
#undef IFAL
#endif  // INCLUDE_GRAPH_H_
