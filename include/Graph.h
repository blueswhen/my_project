// Copyright 2014-12 sxniu
#ifndef INCLUDE_GRAPH_H_
#define INCLUDE_GRAPH_H_

#include <vector>
#include <queue>
#include <stack>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <limits.h>

#define TERMINAL reinterpret_cast<Edge*>(1)
#define ORPHAN reinterpret_cast<Edge*>(2)
#define INIFINITE_DIST INT_MAX
#define EPSILON 0.0000000001

#define ABS(value) ((value) > 0 ? (value) : -(value))

// #define ENABLE_BFS
// #define ENABLE_PAR

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
  CapType MaxFlow();
  bool IsBelongToSource(int node_id);

 private:
  class Node;
  class Edge {
   public:
    Edge()
      : m_dst_node(NULL)
      , m_edge_capacity(0)
      , m_rev_edge(NULL)
      , m_next_edge(NULL) {}
    Node* m_dst_node;
    CapType m_edge_capacity;
    Edge* m_rev_edge;
    // next edge originated from the same node
    Edge* m_next_edge;
  };

  class Node {
   public:
    Node() 
      : m_excess(0)
      , m_node_state(UNKNOWN)
      , m_first_out_edge(NULL)
      , m_parent_edge(NULL)
      , m_timestamp(0)
      , m_tree_tag(0)
      , m_terminal_dist(0)
      , m_child_edge(NULL)
      , m_next_active(NULL)
      , m_is_active(false)
      , m_needed_flow(0)
      , m_is_allow_recursion(false)
      , m_id(0) {}
    CapType m_excess;
    NodeState m_node_state;
    Edge* m_first_out_edge;
    Edge* m_parent_edge;
    Edge* m_child_edge;
    Node* m_next_active;
    bool m_is_active;
    // the timestamp of the latest dist calculating 
    int m_timestamp;
    int m_tree_tag;
    int m_terminal_dist;
    CapType m_needed_flow;
    bool m_is_allow_recursion;
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
  void AddActiveNodeMid(Node* node) {
    if (!node->m_is_active) {
      if (!m_mid_at_node) {
        AddActiveNodeFront(node);
      } else if (m_mid_at_node != m_last_at_node) {
        node->m_is_active = true;
        node->m_next_active = m_mid_at_node->m_next_active;
        m_mid_at_node->m_next_active = node;
        m_mid_at_node = node;
      } else {
        AddActiveNodeBack(node);
        m_mid_at_node = m_last_at_node;
      }
    }
  }
  void AddActiveNodeFront(Node* node) {
    if (!node->m_is_active) {
      node->m_is_active = true;
      node->m_next_active = m_first_at_node;
      m_first_at_node = node;
      if (!m_last_at_node) {
        m_last_at_node = m_first_at_node;
        m_last_at_node->m_next_active = NULL;
      }
    }
  }
  Node* GetActiveNode() {
    Node* active_node = NULL;
    while (m_first_at_node) {
      active_node = m_first_at_node;
      if (active_node == m_mid_at_node) {
        m_mid_at_node = NULL;
      }
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
    while (!m_orphan_nodes.empty()) {
      orphan_node = m_orphan_nodes.front();
      m_orphan_nodes.pop();
      if (orphan_node->m_parent_edge == ORPHAN) {
        break;
      }
      orphan_node = NULL;
    }
    return orphan_node;
  }

  void SetChildEdge(Node* dst_node, Edge* child_edge);
  void SetTerminalNode(Node* node);
  void Same(CapType& dst_val, CapType comp_val);
  void PushFlow(Node* src_node, Node* dst_node, Edge* flow_edge, CapType push_flow_capacity = 0);
  void FindNewOrphans(Node* orphan_node);
  void AdoptNewPath(Node** node);
  void SetResFlow(Node* node);

  void PushEnoughFlowToOneNode(std::stack<Node*>* nodes);

  void PushEnoughFlowToTwoNodes(Node* source_node, Node* sink_node);
  void PushEnoughFlow(Node* source_node, Node* sink_node, CapType* min_edge_capacity);

  void Augment(Edge* meet_edge);
  void FindNewPath(Node* orphan_node);

  std::vector<Node> m_nodes;
  std::vector<Edge> m_edges;
  Node* m_first_at_node;
  Node* m_mid_at_node;
  Node* m_last_at_node;
  // point to the last item used in m_edges
  int m_last_edge_index;
  int m_global_timestamp;
  int m_global_tree_tag;
  std::queue<Node*> m_orphan_nodes;
  int m_path;
  CapType m_source_res_flow;
  bool m_is_source_end;
  CapType m_sink_res_flow;
  bool m_is_sink_end;
  CapType m_flow;
};

template <class CapType>
Graph<CapType>::Graph(int max_nodes_number, int max_edges_number)
  : m_nodes(std::vector<Node>(max_nodes_number))
  , m_edges(std::vector<Edge>(max_edges_number))
  , m_first_at_node(NULL)
  , m_mid_at_node(NULL)
  , m_last_at_node(NULL)
  , m_last_edge_index(0)
  , m_global_timestamp(0)
  , m_global_tree_tag(0)
  , m_path(0)
  , m_source_res_flow(0)
  , m_is_source_end(false)
  , m_sink_res_flow(0)
  , m_is_sink_end(false)
  , m_flow(0) {}

template <class CapType>
void Graph<CapType>::AddNode(int node_id, CapType source_capacity, CapType sink_capacity) {
  assert(node_id >= 0 && node_id < m_nodes.size() &&
         m_nodes[node_id].m_excess == 0);
  m_flow += source_capacity < sink_capacity ? source_capacity : sink_capacity;
  CapType node_capacity = source_capacity - sink_capacity;
  m_nodes[node_id].m_excess = node_capacity;
  m_nodes[node_id].m_id = node_id;
}

template <class CapType>
void Graph<CapType>::AddEdge(int src_node_id, int dst_node_id, CapType edge_capacity) {
  assert(src_node_id >= 0 && src_node_id < m_nodes.size());
  assert(dst_node_id >= 0 && dst_node_id < m_nodes.size());
  assert(m_last_edge_index < m_edges.size());

  Edge* edge = &m_edges[m_last_edge_index++];
  Edge* rev_edge = &m_edges[m_last_edge_index++];

  edge->m_dst_node = &m_nodes[dst_node_id];
  edge->m_edge_capacity = edge_capacity;
  edge->m_rev_edge = rev_edge;
  rev_edge->m_dst_node = &m_nodes[src_node_id];
  rev_edge->m_edge_capacity = edge_capacity;
  rev_edge->m_rev_edge = edge;

  Node* src_node = &m_nodes[src_node_id];
  Node* dst_node = &m_nodes[dst_node_id];

  assert(src_node != NULL && dst_node != NULL);
  edge->m_next_edge = src_node->m_first_out_edge;
  src_node->m_first_out_edge = edge;
  rev_edge->m_next_edge = dst_node->m_first_out_edge;
  dst_node->m_first_out_edge = rev_edge;
}

template <class CapType>
void Graph<CapType>::SetChildEdge(Node* dst_node, Edge* child_edge) {
  dst_node->m_child_edge = child_edge;
  dst_node->m_tree_tag = m_global_tree_tag;
}

template <class CapType>
void Graph<CapType>::SetTerminalNode(Node* node) {
  node->m_parent_edge = TERMINAL;
  node->m_terminal_dist = 1;
}

template <class CapType>
void Graph<CapType>::Same(CapType& dst_val, CapType comp_val) {
  if (ABS(dst_val- comp_val) < EPSILON) {
    dst_val = comp_val;
  }
}

template <class CapType>
void Graph<CapType>::PushFlow(Node* src_node, Node* dst_node, Edge* flow_edge,
                              CapType push_flow_capacity) {
  CapType flow = std::min(ABS(src_node->m_excess), flow_edge->m_edge_capacity);
  if (push_flow_capacity) {
    if (src_node->m_node_state == SINK) {
      assert(push_flow_capacity < 0);
    } else {
      assert(push_flow_capacity > 0);
    }
    flow = std::min(flow, ABS(push_flow_capacity));
  }
  assert(src_node->m_node_state == dst_node->m_node_state);
  src_node->m_excess -= src_node->m_node_state == SOURCE ? flow : -flow;
  Same(src_node->m_excess, 0);

  dst_node->m_excess += src_node->m_node_state == SOURCE ? flow : -flow;
  flow_edge->m_edge_capacity -= flow;
  Same(flow_edge->m_edge_capacity, 0);

  flow_edge->m_rev_edge->m_edge_capacity += flow;
  dst_node->m_needed_flow -= dst_node->m_node_state == SOURCE ? flow : -flow;
  Same(dst_node->m_needed_flow, 0);

  src_node->m_is_allow_recursion = false;
}

template <class CapType>
void Graph<CapType>::FindNewOrphans(Node* orphan_node) {
  assert(!orphan_node->m_parent_edge);
  for (Edge* connected_edge = orphan_node->m_first_out_edge; connected_edge != NULL;
       connected_edge = connected_edge->m_next_edge) {
    Node* dst_node = connected_edge->m_dst_node;
    Edge* parent_edge = dst_node->m_parent_edge;
    if (dst_node->m_node_state == orphan_node->m_node_state && parent_edge) {
      CapType capacity = orphan_node->m_node_state == SINK ?
                         connected_edge->m_edge_capacity :
                         connected_edge->m_rev_edge->m_edge_capacity;
      if (capacity) {
        AddActiveNodeBack(dst_node);
      }
      if (parent_edge != TERMINAL && parent_edge != ORPHAN &&
          parent_edge->m_dst_node == orphan_node) {
        AddOrphanNode(dst_node);
      }
    }
  }
}

template <class CapType>
void Graph<CapType>::AdoptNewPath(Node** node) {
  AddOrphanNode(*node);
  bool no_parent = false;
  std::vector<Node*> other_type_orphans;
  while (true) {
    Node* orphan_node = GetOrphanNode();
    if (!orphan_node) {
      break;
    }
    if (orphan_node->m_node_state != (*node)->m_node_state) {
      other_type_orphans.push_back(orphan_node);
      continue;
    }
    FindNewPath(orphan_node);
    if (!orphan_node->m_parent_edge) {
      if (!orphan_node->m_child_edge && orphan_node->m_tree_tag == m_global_tree_tag) {
        no_parent = true;
      } else {
        FindNewOrphans(orphan_node);
      }
    }
  }
  for (int i = 0; i < other_type_orphans.size(); ++i) {
    AddOrphanNode(other_type_orphans[i]);
  }
  if (!no_parent) {
    Node* path_node = *node;
    for (; !path_node->m_parent_edge; path_node = path_node->m_child_edge->m_dst_node) {
      assert(path_node->m_child_edge);
      Node* dst_node = path_node->m_child_edge->m_dst_node;
      Edge* flow_edge = path_node->m_node_state == SOURCE ?
        path_node->m_child_edge : path_node->m_child_edge->m_rev_edge;
      assert(flow_edge->m_edge_capacity);
      PushFlow(path_node, dst_node, flow_edge);
      assert(!path_node->m_excess);
    }
    *node = path_node;
    Edge* flow_edge = path_node->m_node_state == SINK ?
      path_node->m_parent_edge : path_node->m_parent_edge->m_rev_edge;
    assert(flow_edge->m_edge_capacity);
  }
}

#if 0
template <class CapType>
void Graph<CapType>::PushEnoughFlowToNode(Node* node, CapType needed_flow, CapType* res_flow) {
  assert(node->m_excess + needed_flow <= *res_flow);
  if (node->m_parent_edge == TERMINAL) {
    assert(node->m_excess > 0);
    (*res_flow) -= node->m_excess;
  }
  while (true) {
    if (node->m_parent_edge == TERMINAL || node->m_parent_edge == ORPHAN) {
      AdoptNewPath(node);
    }
    if (node->m_parent_edge && needed_flow > 0) {
      Node* dst_node = node->m_parent_edge->m_dst_node;
      Edge* flow_edge = dst_node->m_node_state == SINK ?
        node->m_parent_edge : node->m_parent_edge->m_rev_edge;
      CapType nd_flow;
      CapType delt_flow = std::min(ABS(dst_node->m_excess), flow_edge->m_edge_capacity);
      nd_flow = std::min(flow_edge->m_edge_capacity, needed_flow) - delt_flow;
      assert(nd_flow <= *res_flow);
      CapType push_flow = 0;
      if (nd_flow > 0) {
        PushEnoughFlowToNode(dst_node, nd_flow, res_flow);
        push_flow = dst_node->m_excess;
        assert(push_flow <= needed_flow);
      } else {
        push_flow = std::min(flow_edge->m_edge_capacity, needed_flow);
        (*res_flow) -= push_flow;
        if (ABS(*res_flow) < EPSILON) {
          *res_flow = 0;
        }
      }
      PushFlow(dst_node, node, flow_edge, push_flow);
      needed_flow -= push_flow;
      if (node->m_parent_edge && !flow_edge->m_edge_capacity) {
        assert(needed_flow > 0);
        node->m_parent_edge = ORPHAN;
      } else if (node->m_parent_edge) {
        assert(!needed_flow);
      }
      if (!*res_flow) {
        assert(!needed_flow);
        return;
      }
    } else {
      return;
    }
  }
}
#endif

template <class CapType>
void Graph<CapType>::SetResFlow(Node* node) {
  node->m_is_allow_recursion = true;
  CapType& res_flow = node->m_node_state == SOURCE ? m_source_res_flow : m_sink_res_flow;
  bool& node_end = node->m_node_state == SOURCE ? m_is_source_end : m_is_sink_end;
  if (node->m_parent_edge == TERMINAL) {
    res_flow -= node->m_excess;
    Same(res_flow, 0);
    if (node->m_node_state == SOURCE) {
      res_flow = res_flow > 0 ? res_flow : 0;
    } else {
      res_flow = res_flow < 0 ? res_flow : 0;
    }
    if (!res_flow) {
      node_end = true;
    }
    // enter by once
    // AddOrphanNode(node);
  }
}

template <class CapType>
void Graph<CapType>::PushEnoughFlowToOneNode(std::stack<Node*>* nodes) {
  // if (node_type == SOURCE) {
  //   printf("s node = %d, excess = %f, need = %f, res = %f\n", source_node->m_id, source_node->m_excess, source_node->m_needed_flow, m_source_res_flow);
  // } else {
  //   printf("sk node = %d, excess = %f, need = %f, res = %f\n", sink_node->m_id, sink_node->m_excess, sink_node->m_needed_flow, m_sink_res_flow);
  // }
  bool node_type;
  while (!nodes->empty()) {
    Node* tmp = nodes->top();
    assert(tmp);
    if (!tmp->m_is_allow_recursion) {
      // if (tmp->m_excess) {
      //   SetTerminalNode(tmp);
      // }
      nodes->pop();
    } else {
      node_type = tmp->m_node_state;
      break;
    }
  }
  if (nodes->empty()) {
    return;
  }
  Node*& node = nodes->top();
  bool& node_end = node_type == SOURCE ? m_is_source_end : m_is_sink_end;
  bool& other_node_end = node_type == SINK ? m_is_source_end : m_is_sink_end;
  CapType& node_res_flow = node_type == SOURCE ? m_source_res_flow : m_sink_res_flow;
  if (node_end) {
    if (node->m_child_edge) {
      CapType push_flow = 0;
      Node* child_node = node->m_child_edge->m_dst_node;
      Edge* flow_edge = node_type == SOURCE ? node->m_child_edge : node->m_child_edge->m_rev_edge;
      if (node_type == SOURCE ? m_source_res_flow < -m_sink_res_flow :
          m_source_res_flow > -m_sink_res_flow) {
        push_flow = node->m_excess + (m_sink_res_flow + m_source_res_flow);
        assert(ABS(push_flow) < ABS(node->m_excess));
        node_res_flow = node_type == SOURCE ? -m_sink_res_flow : -m_source_res_flow;
        SetTerminalNode(node);
      } else {
        push_flow = node->m_excess;
      }
      if (push_flow) {
        PushFlow(node, child_node, flow_edge, push_flow);
      }
      if (child_node->m_parent_edge && !flow_edge->m_edge_capacity) {
        AddOrphanNode(child_node);
      }
      // if (!node->m_excess && node->m_parent_edge == TERMINAL) {
      //   AddOrphanNode(node);
      // }
    }
    nodes->pop();
    return;
  }

  assert(node->m_parent_edge);
  if (node->m_parent_edge == ORPHAN || node->m_parent_edge == TERMINAL) {
    AdoptNewPath(&node);
    if (!node->m_parent_edge) {
      node_end = true;
    }
  }
  // find next node which needed_flow is not zero
  while (!node_end && !node->m_needed_flow) {
    Node* path_node = node;
    for (; !path_node->m_needed_flow; path_node = path_node->m_child_edge->m_dst_node) {
      if (!path_node->m_child_edge) {
        node_end = true;
        break;
      }
      Node* dst_node = path_node->m_child_edge->m_dst_node;
      Edge* flow_edge = path_node->m_node_state == SOURCE ?
        path_node->m_child_edge : path_node->m_child_edge->m_rev_edge;
      PushFlow(path_node, dst_node, flow_edge);
      assert(!path_node->m_excess);
      if (!flow_edge->m_edge_capacity) {
        path_node = dst_node;
        break;
      }
    }
    if (!node_end) {
      Edge* flow_edge = path_node->m_node_state == SINK ?
        path_node->m_parent_edge : path_node->m_parent_edge->m_rev_edge;
      assert(!flow_edge->m_edge_capacity);
      node = path_node;
      AdoptNewPath(&node);
      if (!node->m_parent_edge) {
        node_end = true;
      }
    }
  }
  if (node_end) {
      other_node_end = true;
    return;
  }
  assert(node->m_needed_flow && node->m_parent_edge);

  Node* dst_node = node->m_parent_edge->m_dst_node;
  SetChildEdge(dst_node, node->m_parent_edge->m_rev_edge);
  Edge* flow_edge = dst_node->m_node_state == SINK ?
    node->m_parent_edge : node->m_parent_edge->m_rev_edge;
  assert(flow_edge->m_edge_capacity);
  CapType dst_nd_flow;
  CapType delt_flow = std::min(ABS(dst_node->m_excess), flow_edge->m_edge_capacity);
  dst_nd_flow = std::min(flow_edge->m_edge_capacity, ABS(node->m_needed_flow)) - delt_flow;
  assert(dst_nd_flow - (node_type == SOURCE ? m_source_res_flow : -m_sink_res_flow) < EPSILON);
  dst_node->m_needed_flow = node_type == SOURCE ? dst_nd_flow : -dst_nd_flow;
  CapType push_flow = 0;
  if (dst_nd_flow > 0) {
    SetResFlow(dst_node);
    nodes->push(dst_node);
    return;
  } else {
    assert(node->m_needed_flow);
    assert(dst_node->m_parent_edge == TERMINAL);
    push_flow = std::min(flow_edge->m_edge_capacity, ABS(node->m_needed_flow));
    if (node_type == SINK) {
      push_flow = -push_flow;
    }
    node_res_flow -= push_flow;
    Same(node_res_flow, 0);
    if (!node_res_flow) {
      node_end = true;
    }
  }
  assert(push_flow);
  PushFlow(dst_node, node, flow_edge, push_flow);
  if (node->m_parent_edge && !flow_edge->m_edge_capacity) {
    assert(!node_end);
    AddOrphanNode(node);
  }
  // if (!dst_node->m_excess && dst_node->m_parent_edge == TERMINAL) {
  //   assert(!node_end);
  //   AddOrphanNode(dst_node);
  // }
}

// node is current Node
// needed_flow is except the excess of the node what the node need in flow.
// res_flow is the total resident flow
template <class CapType>
void Graph<CapType>::PushEnoughFlowToTwoNodes(Node* source_node, Node* sink_node) {
  std::stack<Node*> source_nodes;
  source_nodes.push(source_node);
  std::stack<Node*> sink_nodes;
  sink_nodes.push(sink_node);
  while (!source_nodes.empty() || !sink_nodes.empty()) {
  // printf("s node = %d, excess = %f, need = %f, res = %f\n", source_node->m_id, source_node->m_excess, source_node->m_needed_flow, m_source_res_flow);
  // printf("sk node = %d, excess = %f, need = %f, res = %f\n", sink_node->m_id, sink_node->m_excess, sink_node->m_needed_flow, m_sink_res_flow);
    if (!source_nodes.empty() && (m_source_res_flow > -m_sink_res_flow || sink_nodes.empty())) {
      PushEnoughFlowToOneNode(&source_nodes);
    } else {
      PushEnoughFlowToOneNode(&sink_nodes);
    }
  }
}

template <class CapType>
void Graph<CapType>::PushEnoughFlow(Node* source_node, Node* sink_node,
                                    CapType* min_edge_capacity) {
  m_global_tree_tag++;
  m_source_res_flow = *min_edge_capacity;
  m_sink_res_flow = -*min_edge_capacity;
  m_is_source_end = false;
  m_is_sink_end = false;
  source_node->m_needed_flow = m_source_res_flow - source_node->m_excess > 0 ?
    m_source_res_flow - source_node->m_excess : 0;
  sink_node->m_needed_flow = m_sink_res_flow - sink_node->m_excess < 0 ?
    m_sink_res_flow - sink_node->m_excess : 0;
  SetChildEdge(source_node, NULL);
  SetChildEdge(sink_node, NULL);
  SetResFlow(source_node);
  SetResFlow(sink_node);

  PushEnoughFlowToTwoNodes(source_node, sink_node);

  Same(source_node->m_excess, *min_edge_capacity);
  Same(sink_node->m_excess, -*min_edge_capacity);
  Same(source_node->m_excess, -sink_node->m_excess);
  if (source_node->m_parent_edge == ORPHAN) {
    assert(source_node->m_excess);
    SetTerminalNode(source_node);
  }
  if (sink_node->m_parent_edge == ORPHAN) {
    assert(sink_node->m_excess);
    SetTerminalNode(sink_node);
  }
  *min_edge_capacity = std::min(*min_edge_capacity, source_node->m_excess);
  *min_edge_capacity = std::min(*min_edge_capacity, -sink_node->m_excess);
  if (source_node->m_parent_edge && source_node->m_parent_edge != TERMINAL &&
      source_node->m_excess > *min_edge_capacity) {
    // assert(source_node->m_parent_edge->m_dst_node->m_parent_edge == TERMINAL);
    SetTerminalNode(source_node);
  }
  if (sink_node->m_parent_edge && sink_node->m_parent_edge != TERMINAL &&
      sink_node->m_excess < -*min_edge_capacity) {
    // assert(sink_node->m_parent_edge->m_dst_node->m_parent_edge == TERMINAL);
    SetTerminalNode(sink_node);
  }
}

template <class CapType>
void Graph<CapType>::Augment(Edge* meet_edge) {
  // augment path
  CapType min_capacity = meet_edge->m_edge_capacity;
  // first_edge[0] for source tree and first_edge[1] for sink tree
  Edge* first_edge[2] = {meet_edge->m_rev_edge, meet_edge};
  // find min capacity from path
  Node* flow_nodes[2];
  for (int i = 0; i < 2; ++i) {
    Node* parent_node = first_edge[i]->m_dst_node;
    for (Edge* parent_edge = parent_node->m_parent_edge; parent_edge != TERMINAL;
         parent_node = parent_edge->m_dst_node, parent_edge = parent_node->m_parent_edge) {
      m_path++;
      assert(parent_edge);
      assert(parent_edge != ORPHAN);
      Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
      CapType cap = edge->m_edge_capacity;
      if (cap < min_capacity) {
        min_capacity = cap;
      }
    }
    flow_nodes[i] = parent_node;
  }
  assert(min_capacity > 0 && flow_nodes[0]->m_node_state == SOURCE &&
         flow_nodes[1]->m_node_state == SINK);
  CapType old_min = min_capacity;
  
  // printf("node0 = %d, excess = %f, node1 = %d, excess = %f, min_capacity = %f\n", flow_nodes[0]->m_id, flow_nodes[0]->m_excess, flow_nodes[1]->m_id, flow_nodes[1]->m_excess, min_capacity);
  PushEnoughFlow(flow_nodes[0], flow_nodes[1], &min_capacity);
  assert(flow_nodes[0]->m_parent_edge != ORPHAN && flow_nodes[1]->m_parent_edge != ORPHAN);

  assert(flow_nodes[0]->m_parent_edge || flow_nodes[1]->m_parent_edge);
  if (flow_nodes[0]->m_parent_edge && flow_nodes[1]->m_parent_edge) {
    assert(min_capacity == old_min);
  } else if (!flow_nodes[0]->m_parent_edge) {
    assert(flow_nodes[1]->m_parent_edge);
    assert(min_capacity == flow_nodes[0]->m_excess);
    assert(min_capacity < old_min);
    if (flow_nodes[1]->m_parent_edge == TERMINAL) {
      assert(flow_nodes[1]->m_excess < -min_capacity);
    } else {
      assert(flow_nodes[1]->m_excess == -min_capacity);
    }
  } else {
    assert(flow_nodes[0]->m_parent_edge);
    assert(min_capacity == -flow_nodes[1]->m_excess);
    assert(min_capacity < old_min);
    assert(flow_nodes[0]->m_excess >= min_capacity);
  }

  first_edge[0]->m_edge_capacity += min_capacity;
  first_edge[1]->m_edge_capacity -= min_capacity;
  for (int i = 0; i < 2; ++i) {
    Node* parent_node = first_edge[i]->m_dst_node;
    Edge* parent_edge = NULL;
    int factor = i == 0 ? -1 : 1;
    for (; parent_node != flow_nodes[i]; parent_node = parent_edge->m_dst_node) {
      parent_edge = parent_node->m_parent_edge;
      assert(parent_edge);
      parent_edge->m_edge_capacity += (-factor) * min_capacity;
      parent_edge->m_rev_edge->m_edge_capacity += factor * min_capacity;
      assert(parent_edge->m_edge_capacity >= 0 && parent_edge->m_rev_edge->m_edge_capacity >= 0);
      Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
      if (!edge->m_edge_capacity) {
        AddOrphanNode(parent_node);
      }
    }
    parent_node->m_excess += factor * min_capacity;
    if (parent_node->m_node_state == SOURCE) {
      assert(parent_node->m_excess >= 0);
    } else {
      assert(parent_node->m_excess <= 0);
    }
    if (!parent_node->m_excess &&
        (!parent_node->m_parent_edge || parent_node->m_parent_edge == TERMINAL)) {
      AddOrphanNode(parent_node);
    }
  }
  m_flow += min_capacity;
}

template <class CapType>
void Graph<CapType>::FindNewPath(Node* orphan_node) {
  int dist_min = INIFINITE_DIST;
  CapType max_node_cap = 0;
  Edge* connected_edge_min = NULL;
  m_global_timestamp++;

  for (Edge* connected_edge = orphan_node->m_first_out_edge; connected_edge != NULL;
       connected_edge = connected_edge->m_next_edge) {
    if (connected_edge->m_dst_node->m_node_state == orphan_node->m_node_state) {
      CapType capacity = orphan_node->m_node_state == SINK ?
                         connected_edge->m_edge_capacity :
                         connected_edge->m_rev_edge->m_edge_capacity;
      if (!capacity) {
        continue;
      }
      Node* dst_node = connected_edge->m_dst_node;
      Edge* parent_edge = dst_node->m_parent_edge;
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
            break;
          }
          if (parent_edge == ORPHAN || !parent_edge) {
            dist = INIFINITE_DIST;
            break;
          }
          assert(parent_edge);
          dst_node = parent_edge->m_dst_node;
        }
        if (dist < INIFINITE_DIST) {
          if (dist < dist_min) {
            connected_edge_min = connected_edge;
            dist_min = dist;
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
CapType Graph<CapType>::MaxFlow() {
  // get active nodes
  for (int i = 0; i < m_nodes.size(); ++i) {
    Node* node = &m_nodes[i];
    if (node->m_excess != 0) {
      node->m_node_state = node->m_excess < 0 ? SINK : SOURCE;
      SetTerminalNode(node);
      node->m_timestamp = 0;
      AddActiveNodeBack(node);
    }
  }

  Node* at_node = NULL;
  Edge* meet_edge = NULL;
  int path = 0;
  while (true) {
#ifndef ENABLE_PAR
    if (meet_edge == NULL || at_node->m_parent_edge == NULL) {
#endif
      at_node = GetActiveNode();
      if (!at_node) {
        break;
      }
#ifndef ENABLE_PAR
    }
#endif
    meet_edge = NULL;
    // printf("at_node = %d\n", at_node->m_id);
    // grow source tree and sink tree
    for (Edge* connected_edge = at_node->m_first_out_edge; connected_edge != NULL;
         connected_edge = connected_edge->m_next_edge) {
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
#ifdef ENABLE_PAR
  int origion_length = meet_edge->m_dst_node->m_terminal_dist +
                       meet_edge->m_rev_edge->m_dst_node->m_terminal_dist;
#endif

      Augment(meet_edge);

      // adopt orphan nodes
      while (true) {
        Node* orphan_node = GetOrphanNode();
        if (!orphan_node) {
          break;
        }

        FindNewPath(orphan_node);

        if (orphan_node->m_parent_edge) {
#ifdef ENABLE_BFS
          if (orphan_node->m_child_edge) {
            Node* child_node = orphan_node->m_child_edge->m_dst_node;
            if (child_node->m_parent_edge == orphan_node->m_child_edge->m_rev_edge &&
                child_node->m_terminal_dist <= orphan_node->m_terminal_dist) {
              AddOrphanNode(child_node);
            }
          }
#endif
        } else {
          FindNewOrphans(orphan_node);
        }
      }
#ifdef ENABLE_PAR
      int new_length = meet_edge->m_dst_node->m_terminal_dist +
                       meet_edge->m_rev_edge->m_dst_node->m_terminal_dist;
      // AddActiveNodeBack(at_node);
      if (new_length > origion_length) {
        AddActiveNodeBack(at_node);
      } else {
        AddActiveNodeFront(at_node);
      }
#endif
    }
  }
  printf("path = %d, flow = %f, FindNewPath count = %d\n", m_path, m_flow, m_global_timestamp);
  return m_flow;
}

template <class CapType>
bool Graph<CapType>::IsBelongToSource(int node_id) {
  return m_nodes[node_id].m_node_state == SOURCE;
}

} // namespace

#undef TERMINAL
#undef ORPHAN
#undef ENABLE_BFS
#undef ENABLE_PAR
#endif  // INCLUDE_GRAPH_H_
