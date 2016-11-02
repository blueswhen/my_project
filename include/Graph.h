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
#define HALF_NEIGHBOUR 4
#define HALF_NEIGHBOUR_ARR_INDEX FOUR_ARR_INDEX
#define NEIGHBOUR_ARR_INDEX EIGHT_ARR_INDEX
#define T 50
#define LT 10000

// no ibfs
#define IFAL
// #define ENABLE_DYNAMIC_EDGE

#ifdef ENABLE_DYNAMIC_EDGE
typedef double (*EPF)(int src_node_colour, int dst_node_colour);
const double graph_div_sqrt2 = 1 / sqrt(2.0f);
#endif

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

namespace user {

template <class CapType>
class Graph {
 public:
  enum NodeState {
    SOURCE,
    SINK,
    UNKNOWN
  };
#ifdef ENABLE_DYNAMIC_EDGE
  Graph(int max_nodes_number, int image_width, int image_height, EPF epf);
  void AddNode(int node_id, CapType source_capacity, CapType sink_capacity, int node_colour);
  void AddActiveNodes(int node_x, int node_y);
#endif
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
      // , m_orphan_time(0)
#endif
#ifdef ENABLE_DYNAMIC_EDGE
      , m_is_gotten_all_edges(false)
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
    // int m_orphan_time;
#endif
#ifdef ENABLE_DYNAMIC_EDGE
    bool m_is_gotten_all_edges;
    int m_node_colour;
#endif
    int m_id;
  };

#ifdef ENABLE_DYNAMIC_EDGE
  Edge* CreateEdge(Node* src_node, Node* dst_node, double punish_factor,
                   int edge_index, int rev_edge_index);
  void CreateOutEdges(Node* cen_node);
#endif

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
    orphan_node->m_in_stack = false;
    // orphan_node->m_orphan_time = 0;
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
  void AdoptNewPath(Node*& node);
  void Same(CapType& dst_val, CapType comp_val);
  void DoPushFlow(Node* src_node, Node* dst_node, Edge* flow_edge, CapType push_flow);
  void PushFlow(Node* src_node, bool add_edge_orphan = true, bool add_tel_orphan = true);
  void SetResFlow(bool node_type, CapType minus_flow);

  bool Empty(bool node_type);
  void Push(Node* node);
  Node*& Top(bool node_type);
  void Pop(bool node_type);
  void PushEnoughFlowToOneNode(bool node_type);

  void PushEnoughFlowToTwoNodes(Node* source_node, Node* sink_node, CapType* bridge_flow);
  CapType PushEnoughFlow(Node* source_node, Node* sink_node, Edge* bridge_edge);
#endif
  void Augment(Edge* meet_edge);
  void FindNewPath(Node* orphan_node);
  void FindNewOrphans(Node* orphan_node);
  Node* Adoption(Node* start_orphan = NULL);

  std::vector<Node> m_nodes;
  Node* m_first_at_node;
  Node* m_last_at_node;
  // point to the last item used in m_edges
  int m_global_timestamp;
  std::queue<Node*> m_orphan_nodes;
  long m_path;
  int m_count;
  double m_time;
  double m_time2;
  // ImageData<int>* m_marked_image;
#ifdef IFAL
  CapType m_source_res_flow;
  CapType m_sink_res_flow;
  Node* m_top_source_candidate_orphan;
  Node* m_top_sink_candidate_orphan;
  Node* m_source_next;
  Node* m_sink_next;
  // int m_global_orphan_time;
#endif
#ifdef ENABLE_DYNAMIC_EDGE
  int m_image_width;
  int m_image_height;
  EPF m_epf;
#endif
  CapType m_flow;
};

#ifdef ENABLE_DYNAMIC_EDGE
template <class CapType>
Graph<CapType>::Graph(int max_nodes_number, int image_width, int image_height, EPF epf)
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
  , m_sink_res_flow(0)
  , m_top_source_candidate_orphan(NULL)
  , m_top_sink_candidate_orphan(NULL)
  , m_source_next(NULL)
  , m_sink_next(NULL)
#endif
  , m_flow(0)
  , m_image_width(image_width)
  , m_image_height(image_height)
  , m_epf(epf) {}

template <class CapType>
void Graph<CapType>::AddNode(int node_id, CapType source_capacity, CapType sink_capacity, int node_colour) {
  AddNode(node_id, source_capacity, sink_capacity);
  m_nodes[node_id].m_node_colour = node_colour;
}

template <class CapType>
void Graph<CapType>::AddActiveNodes(int node_x, int node_y) {
  int index = node_y * m_image_width + node_x;
  Node* cen_node = &m_nodes[index];
  int arr_index[HALF_NEIGHBOUR] =
    HALF_NEIGHBOUR_ARR_INDEX(node_x, node_y, m_image_width, m_image_height);
  for (int i = 0; i < HALF_NEIGHBOUR; ++i) {
    Node* arr_node = &m_nodes[arr_index[i]];
    if (arr_index[i] < index && cen_node->m_node_state != arr_node->m_node_state) {
      AddActiveNodeBack(cen_node);
      AddActiveNodeBack(arr_node);
      // break;
    }
  }
}

template <class CapType>
typename Graph<CapType>::Edge* Graph<CapType>::CreateEdge(
  Node* src_node, Node* dst_node, double punish_factor, int edge_index, int rev_edge_index) {
  Edge* edge = &src_node->m_out_edges[edge_index];
  Edge* rev_edge = &dst_node->m_out_edges[rev_edge_index];
  CapType cap = punish_factor * m_epf(src_node->m_node_colour, dst_node->m_node_colour);
  edge->m_edge_capacity = cap;
  edge->m_rev_edge = rev_edge;
  edge->m_dst_node = dst_node;
  rev_edge->m_edge_capacity = cap;
  rev_edge->m_rev_edge = edge;
  rev_edge->m_dst_node = src_node;
  return edge;
}

template <class CapType>
void Graph<CapType>::CreateOutEdges(Node* cen_node) {
  // assert(cen_node != NULL);
  if (cen_node->m_is_gotten_all_edges) {
    return;
  }
  int coordinate = cen_node->m_id;
  int y_node = coordinate / m_image_width;
  int x_node = coordinate - y_node * m_image_width;
  int arr_nodes_idx[NEIGHBOUR] = 
    NEIGHBOUR_ARR_INDEX(x_node, y_node, m_image_width, m_image_height);
  if (y_node == 0) {
    arr_nodes_idx[1] = coordinate;
    arr_nodes_idx[3] = coordinate;
  } else if (y_node == m_image_height - 1) {
    arr_nodes_idx[5] = coordinate;
    arr_nodes_idx[7] = coordinate;
  }
  if (x_node == 0) {
    arr_nodes_idx[1] = coordinate;
    arr_nodes_idx[7] = coordinate;
  } else if (x_node == m_image_width - 1) {
    arr_nodes_idx[3] = coordinate;
    arr_nodes_idx[5] = coordinate;
  }
  double punish_factor = 0;
  for (int i = 0; i < NEIGHBOUR; ++i) {
    if (cen_node->m_id == arr_nodes_idx[i]) {
      continue;
    }
    Node* arr_node = &m_nodes[arr_nodes_idx[i]];
    if (cen_node->m_out_edges[i].m_dst_node == NULL) {
      if (ABS(arr_nodes_idx[i] - cen_node->m_id) == 1 ||
          ABS(arr_nodes_idx[i] - cen_node->m_id) == m_image_width) {
        punish_factor = 1;
      } else {
        punish_factor = graph_div_sqrt2;
      }
      int rev_idx = i - 4 < 0 ? i + 4 : i - 4;
      // assert(arr_node->m_out_edges[rev_idx].m_dst_node == NULL);
      CreateEdge(cen_node, arr_node, punish_factor, i, rev_idx);
    }
  }
  cen_node->m_is_gotten_all_edges = true;
}
#endif

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
  , m_sink_res_flow(0)
  , m_top_source_candidate_orphan(NULL)
  , m_top_sink_candidate_orphan(NULL)
  // , m_global_orphan_time(1)
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
void Graph<CapType>::DoPushFlow(
  Node* src_node, Node* dst_node, Edge* flow_edge, CapType push_flow) {
  // assert(push_flow > 0);
  // assert(src_node->m_node_state == dst_node->m_node_state);
  CapType node_type = src_node->m_node_state;
  CapType flow = push_flow;
  Same(flow, flow_edge->m_edge_capacity);
  // assert(flow <= flow_edge->m_edge_capacity);

  src_node->m_excess = node_type == SOURCE ? -src_node->m_needed_flow : src_node->m_needed_flow;
  // Same(src_node->m_excess, 0);

  dst_node->m_excess += node_type == SOURCE ? flow : -flow;
  flow_edge->m_edge_capacity -= flow;
  // Same(flow_edge->m_edge_capacity, 0);

  flow_edge->m_rev_edge->m_edge_capacity += flow;
  dst_node->m_needed_flow -= flow;
  Same(dst_node->m_needed_flow, 0);
  // assert(dst_node->m_needed_flow >= 0);
  m_path++;
}

template <class CapType>
void Graph<CapType>::PushFlow(Node* src_node, bool add_edge_orphan, bool add_tel_orphan) {
  // assert(src_node->m_child_edge);
  // assert(src_node->m_child_edge->m_dst_node->m_parent_edge != TERMINAL);
  // assert(src_node->m_child_edge->m_dst_node->m_parent_edge != ORPHAN);
  // assert(!src_node->m_child_edge->m_dst_node->m_parent_edge || src_node->m_child_edge->m_dst_node->m_parent_edge->m_dst_node == src_node);
  bool node_type = src_node->m_node_state;
  CapType push_flow = node_type == SOURCE ?
    src_node->m_excess + src_node->m_needed_flow : -src_node->m_excess + src_node->m_needed_flow;
  Same(push_flow, 0);
  // Same(src_node->m_excess, 0);
  if (push_flow) {
    Node* child_node = src_node->m_child_edge->m_dst_node;
    Edge* flow_edge = node_type == SOURCE ?
      src_node->m_child_edge : src_node->m_child_edge->m_rev_edge;
    // assert(push_flow > 0);
    DoPushFlow(src_node, child_node, flow_edge, push_flow);
    if (add_edge_orphan && !flow_edge->m_edge_capacity) {
      AddOrphanNode(child_node);
    }
    if (add_tel_orphan && src_node->m_parent_edge == TERMINAL && !src_node->m_excess) {
      AddOrphanNode(src_node);
    }
  }
  src_node->m_child_edge = NULL;
}

template <class CapType>
void Graph<CapType>::AdoptNewPath(Node*& node) {
  if (node->m_parent_edge == TERMINAL) {
    AddOrphanNode(node);
  }
  node = Adoption(node);
}

template <class CapType>
void Graph<CapType>::SetResFlow(bool node_type, CapType minus_flow) {
  if (!minus_flow) {
    return;
  }
  CapType& res_flow = node_type == SOURCE ? m_source_res_flow : m_sink_res_flow;
  res_flow -= (node_type == SOURCE ? minus_flow : -minus_flow);
  Same(res_flow, 0);
  if (res_flow <= 0) {
    res_flow = 0;
  }
}

template <class CapType>
bool Graph<CapType>::Empty(bool node_type) {
  Node*& top_node = node_type == SOURCE ? m_top_source_candidate_orphan : m_top_sink_candidate_orphan;
  if (!top_node) {
    return true;
  }
  return false;
}

template <class CapType>
void Graph<CapType>::Push(Node* node) {
  // assert(node->m_orphan_time != m_global_orphan_time);
  Node*& top_node = node->m_node_state == SOURCE ? m_top_source_candidate_orphan : m_top_sink_candidate_orphan;
  node->m_next_node = top_node;
  top_node = node;
  node->m_in_stack = true;
  // node->m_orphan_time = m_global_orphan_time;
}

template <class CapType>
typename Graph<CapType>::Node*&
  Graph<CapType>::Top(bool node_type) {
  return node_type == SOURCE ? m_top_source_candidate_orphan : m_top_sink_candidate_orphan;
}

template <class CapType>
void Graph<CapType>::Pop(bool node_type) {
  Node*& top_node = node_type == SOURCE ? m_top_source_candidate_orphan : m_top_sink_candidate_orphan;
  // assert(top_node);
  Node* tmp = top_node;
  top_node = top_node->m_next_node;
  tmp->m_next_node = NULL;
  // assert(tmp->m_in_stack == false);
  // assert(!tmp->m_orphan_time);
}

template <class CapType>
void Graph<CapType>::PushEnoughFlowToOneNode(bool node_type) {
  Node* node = node_type == SOURCE ? m_source_next : m_sink_next;
  // assert(node_type == SOURCE ? m_source_res_flow : m_sink_res_flow);

  while (node->m_needed_flow > 0 && node->m_parent_edge) {
    if (node->m_parent_edge == ORPHAN || node->m_parent_edge == TERMINAL) {
      AdoptNewPath(node);
      if (!node->m_parent_edge) {
        break;
      }
    }

    m_path++;
    Node* dst_node = node->m_parent_edge->m_dst_node;
    if (dst_node->m_child_edge && dst_node->m_child_edge->m_dst_node != node) {
      PushFlow(dst_node);
      // assert(dst_node->m_parent_edge);
    }
    dst_node->m_child_edge = node->m_parent_edge->m_rev_edge;
    Edge* flow_edge = node_type == SINK ?
      node->m_parent_edge : node->m_parent_edge->m_rev_edge;
    // assert(flow_edge->m_edge_capacity);
    CapType src_nd_flow = std::min(flow_edge->m_edge_capacity, node->m_needed_flow);
    CapType dst_nd_flow = src_nd_flow -
      (node_type == SOURCE ? dst_node->m_excess : -dst_node->m_excess);
    CapType minus_flow = 0;
    // Same(dst_nd_flow, 0);
    if (dst_nd_flow >= 0) {
      minus_flow = dst_node->m_excess;
    } else {
      minus_flow = node_type == SOURCE ? src_nd_flow : -src_nd_flow;
      // assert(node_type == SOURCE ? dst_node->m_excess > 0 : dst_node->m_excess < 0);
      // assert(dst_node->m_parent_edge == TERMINAL);
    }
    dst_node->m_needed_flow = dst_nd_flow;
    Same(flow_edge->m_edge_capacity, node->m_needed_flow);
    if (flow_edge->m_edge_capacity <= node->m_needed_flow) {
      Push(node);
    }
    SetResFlow(node_type, minus_flow);
    Node*& next_node = node_type == SOURCE ? m_source_next : m_sink_next;
    next_node = dst_node;
    return;
  }
  Node*& next_node = node_type == SOURCE ? m_source_next : m_sink_next;
  if (!node->m_parent_edge || Empty(node_type)) {
    // assert(!node->m_child_edge);
    next_node = NULL;
    return;
  }
  // assert(!Empty(node_type));
  Node* child_node = Top(node_type);
  // assert(child_node->m_orphan_time == m_global_orphan_time);
  // assert(child_node->m_parent_edge);
  node = child_node->m_parent_edge->m_dst_node;
  // assert(node->m_child_edge && node->m_child_edge->m_dst_node == child_node);
  Edge* flow_edge = node_type == SOURCE ?
    node->m_child_edge : node->m_child_edge->m_rev_edge;
  // if (node_type == SOURCE) {
  //   CapType flow = node->m_excess + node->m_needed_flow;
  //   // Same(flow, flow_edge->m_edge_capacity);
  //   // assert(flow == flow_edge->m_edge_capacity);
  // } else {
  //   CapType flow = -node->m_excess + node->m_needed_flow;
  //   // Same(flow, flow_edge->m_edge_capacity);
  //   // assert(flow == flow_edge->m_edge_capacity);
  // }
  PushFlow(node);
  // assert(node->m_parent_edge);
  // if (node_type == SOURCE ? node->m_excess > 0 : node->m_excess < 0) {
  //   // assert(node->m_parent_edge == TERMINAL);
  // }
  // assert(!flow_edge->m_edge_capacity);
  next_node = child_node;
  Pop(node_type);
}

// node is current Node
// needed_flow is except the excess of the node what the node need in flow.
// res_flow is the total resident flow
template <class CapType>
void Graph<CapType>::PushEnoughFlowToTwoNodes(Node* source_node, Node* sink_node, CapType* bridge_flow) {
  m_source_res_flow = *bridge_flow;
  m_sink_res_flow = *bridge_flow;
  m_top_source_candidate_orphan = NULL;
  m_top_sink_candidate_orphan = NULL;
  m_source_next = NULL;
  m_sink_next = NULL;
  // m_global_orphan_time++;

  if (source_node->m_child_edge) {
    // assert(source_node->m_child_edge->m_dst_node != sink_node);
    // assert(source_node->m_child_edge->m_dst_node->m_parent_edge->m_dst_node == source_node);
    PushFlow(source_node);
    // assert(source_node->m_parent_edge);
  }
  if (sink_node->m_child_edge) {
    // assert(sink_node->m_child_edge->m_dst_node != source_node);
    // assert(sink_node->m_child_edge->m_dst_node->m_parent_edge->m_dst_node == sink_node);
    PushFlow(sink_node);
    // assert(sink_node->m_parent_edge);
  }

  source_node->m_needed_flow = m_source_res_flow - source_node->m_excess;
  sink_node->m_needed_flow = m_sink_res_flow + sink_node->m_excess;
  SetResFlow(SOURCE, source_node->m_excess);
  SetResFlow(SINK, sink_node->m_excess);
  if (m_source_res_flow) {
    m_source_next = source_node;
  }
  if (m_sink_res_flow) {
    m_sink_next = sink_node;
  }

  while ((m_source_res_flow || m_sink_res_flow) && 
         (m_source_next || m_sink_next)) {
    bool is_continue = false;
    if (m_source_next && m_source_res_flow >= m_sink_res_flow) {
      PushEnoughFlowToOneNode(SOURCE);
      is_continue = true;
    }
    if (m_sink_next && m_source_res_flow < m_sink_res_flow) {
      PushEnoughFlowToOneNode(SINK);
      is_continue = true;
    }
    if (is_continue) {
      continue;
    }
    // assert(!m_source_next ^ !m_sink_next);
    // Same(m_source_res_flow, m_sink_res_flow);
    CapType delt_flow = m_source_res_flow < m_sink_res_flow ?
      m_sink_res_flow - m_source_res_flow : m_source_res_flow - m_sink_res_flow;
    Node* top_node = m_source_next ? m_source_next : m_sink_next;
    // assert(top_node);
    // assert(top_node->m_excess);
    if (top_node->m_needed_flow > 0) {
      top_node->m_needed_flow = -delt_flow;
    } else {
      top_node->m_needed_flow = top_node->m_needed_flow - delt_flow;
    }
    // assert(top_node->m_needed_flow <= 0);
    // assert(top_node);
    while (top_node->m_child_edge) {
      Edge* flow_edge = top_node->m_child_edge;
      Node* tmp = top_node;
      top_node = top_node->m_child_edge->m_dst_node;
      PushFlow(tmp);
      top_node->m_needed_flow = 0;
    }
    // Same(source_node->m_excess, -sink_node->m_excess);
    break;
  }
  // assert(source_node->m_parent_edge || sink_node->m_parent_edge);
  if (!source_node->m_parent_edge) {
    *bridge_flow = source_node->m_excess;
  }
  if (!sink_node->m_parent_edge) {
    *bridge_flow = -sink_node->m_excess;
  }
  // Same(*bridge_flow, 0);
  // assert(*bridge_flow >= 0);
}

template <class CapType>
CapType Graph<CapType>::PushEnoughFlow(
  Node* source_node, Node* sink_node, Edge* bridge_edge) {
  // assert(bridge_edge->m_dst_node == sink_node && bridge_edge->m_rev_edge->m_dst_node == source_node);

  CapType bridge_flow = bridge_edge->m_edge_capacity;

  // printf("begin s = %d, ex = %f, t = %d, ex = %f\n", source_node->m_id, source_node->m_excess, sink_node->m_id, sink_node->m_excess);
  PushEnoughFlowToTwoNodes(source_node, sink_node, &bridge_flow);
  // printf("end\n");
  m_path++;

  if (bridge_flow) {
    bridge_edge->m_edge_capacity -= bridge_flow;
    bridge_edge->m_rev_edge->m_edge_capacity += bridge_flow;
    source_node->m_excess -= bridge_flow;
    sink_node->m_excess += bridge_flow;

    // if (!source_node->m_parent_edge) {
    //   // assert(!source_node->m_excess);
    //   // assert(!sink_node->m_excess || (sink_node->m_parent_edge == TERMINAL && sink_node->m_excess < 0));
    // }
    // if (!sink_node->m_parent_edge) {
    //   // assert(!sink_node->m_excess);
    //   // assert(!source_node->m_excess || (source_node->m_parent_edge == TERMINAL && source_node->m_excess > 0));
    // }
    // if (sink_node->m_parent_edge && source_node->m_parent_edge) {
    //   // assert(!bridge_edge->m_edge_capacity);
    // }
  }
  return bridge_flow;
}
#endif

template <class CapType>
void Graph<CapType>::FindNewOrphans(Node* orphan_node) {
  // assert(!orphan_node->m_parent_edge);
#ifndef ENABLE_DYNAMIC_EDGE
  for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
#else
  for (int i = 0; i < NEIGHBOUR; ++i) {
    if (orphan_node->m_out_edges[i].m_dst_node == NULL) {
      continue;
    }
#endif
    Edge* connected_edge = &orphan_node->m_out_edges[i];
    Node* dst_node = connected_edge->m_dst_node;
    Edge* parent_edge = dst_node->m_parent_edge;

    // AddActiveNodeBack(dst_node);
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
  int dist_min = INIFINITE_DIST;
  Edge* connected_edge_min = NULL;
  CapType cap_max = 0;

#ifndef ENABLE_DYNAMIC_EDGE
  for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
#else
  CreateOutEdges(orphan_node);
  for (int i = 0; i < NEIGHBOUR; ++i) {
    if (orphan_node->m_out_edges[i].m_dst_node == NULL) {
      continue;
    }
#endif
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
          if (parent_edge == ORPHAN) {
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
typename Graph<CapType>::Node* Graph<CapType>::Adoption(Node* start_orphan) {
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
      orphan_node->m_needed_flow = 0;
      if (orphan_node->m_child_edge) {
        if (orphan_node == start_orphan) {
          start_orphan = orphan_node->m_child_edge->m_dst_node;
        }
        PushFlow(orphan_node);
        // assert(!orphan_node->m_excess);
      }
      FindNewOrphans(orphan_node);
    }
  }
  if (start_orphan) {
    while (!Empty(SOURCE)) {
      if (Top(SOURCE)->m_in_stack) {
        break;
      }
      Pop(SOURCE);
    }
    while (!Empty(SINK)) {
      if (Top(SINK)->m_in_stack) {
        break;
      }
      Pop(SINK);
    }
  }
  m_global_timestamp++;
  // ct.ContEnd();
  // m_time += ct.ContResult() * 1000;
  return start_orphan;
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
  min_capacity = PushEnoughFlow(flow_nodes[0], flow_nodes[1], meet_edge);
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
  // for (int j = 0; j < m_nodes.size(); ++j) {
  //   int node_y = j / m_image_width;
  //   int node_x = j - node_y * m_image_width;
  //   Node* cen_node = &m_nodes[j];
  //   int arr_index[NEIGHBOUR] =
  //     NEIGHBOUR_ARR_INDEX(node_x, node_y, m_image_width, m_image_height);
  //   for (int i = 0; i < NEIGHBOUR; ++i) {
  //     Node* arr_node = &m_nodes[arr_index[i]];
  //     if (arr_index[i] != j && cen_node->m_node_state != arr_node->m_node_state) {
  //       AddActiveNodeBack(cen_node);
  //       // AddActiveNodeBack(arr_node);
  //       break;
  //     }
  //   }
  // }
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
#ifndef ENABLE_DYNAMIC_EDGE
    for (int i = 0; i < at_node->m_out_edges_num; ++i) {
#else
    CreateOutEdges(at_node);
    for (int i = 0; i < NEIGHBOUR; ++i) {
      if (at_node->m_out_edges[i].m_dst_node == NULL) {
        continue;
      }
#endif
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
        }
        // else if (dst_node->m_timestamp <= at_node->m_timestamp &&
        //     dst_node->m_terminal_dist > at_node->m_terminal_dist) {
        //   Node* old_parent = dst_node->m_parent_edge->m_dst_node;
        //   if (old_parent->m_child_edge) {
        //     PushFlow(old_parent, false);
        //   }
        //   dst_node->m_parent_edge = connected_edge->m_rev_edge;
        //   Node* new_parent = dst_node->m_parent_edge->m_dst_node;
        //   if (new_parent->m_child_edge) {
        //     PushFlow(new_parent);
        //   }
        //   new_parent->m_child_edge = connected_edge;
        //   dst_node->m_timestamp = at_node->m_timestamp;
        //   dst_node->m_terminal_dist = at_node->m_terminal_dist + 1;
        // }
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
  printf("path = %ld, orphan count = %d, time = %f\n", m_path, m_count, m_time2);
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
#undef ENABLE_DYNAMIC_EDGE
#undef HALF_NEIGHBOUR
#undef HALF_NEIGHBOUR_ARR_INDEX
#undef NEIGHBOUR_ARR_INDEX
#undef FOUR_ARR_INDEX
#undef EIGHT_ARR_INDEX 
#endif  // INCLUDE_GRAPH_H_
