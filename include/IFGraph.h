// Copyright 2014-12 sxniu
#ifndef INCLUDE_IFGRAPH_H_
#define INCLUDE_IFGRAPH_H_

#include <vector>
#include <list>
#include <queue>
#include <deque>
#include <algorithm>
#include <set>
#include <unordered_map>
#include <stack>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#include "include/ImageData.h"
#include "include/CountTime.h"
#include "include/utils.h"

#define TERMINAL reinterpret_cast<Edge*>(1)
#define ORPHAN reinterpret_cast<Edge*>(2)
#define INIFINITE_DIST INT_MAX

#define HALF_NEIGHBOUR 4
#define NEIGHBOUR 8
#define HALF_NEIGHBOUR_ARR_INDEX FOUR_ARR_INDEX
#define NEIGHBOUR_ARR_INDEX EIGHT_ARR_INDEX
#define R_MAX 1 

// #define ENABLE_BFS
// #define ENABLE_PAR

const double ifgraph_div_sqrt2 = 1 / sqrt(2.0f);

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

template <class CapType, class EdgePunishFun>
class IFGraph {
 public:
  struct Node;
  enum NodeState {
    SOURCE,
    SINK,
  };
  struct Edge {
    Node* m_dst_node;
    CapType m_edge_capacity;
    Edge* m_rev_edge;
  };

  struct Node {
    int m_node_idx;
    int m_node_colour;
    CapType m_excess;
    NodeState m_node_state;
    Edge* m_parent_edge;
    Edge* m_child_edge;
    // the timestamp of the latest dist calculating 
    int m_timestamp;
    int m_terminal_dist;
    int m_depth;
    bool m_is_active;
    bool m_is_gotten_all_edges;
    Edge m_out_edges[NEIGHBOUR];
    int m_out_edges_num;
    Node* m_next_active;

    CapType m_needed_flow;
    Node* m_next_node;
  };

  IFGraph(int max_nodes_number, int image_width,
         int image_height, EdgePunishFun epf, ImageData<int>* marked_image);
  void AddNode(int node_id, CapType source_capacity, CapType sink_capacity, int node_colour);
  void AddActiveNodes(int node_x, int node_y);
  void MaxFlow();
  bool IsBelongToSource(int node_id);

 private:
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
  void AddOrphanNode(Node* orphan_node) {
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

  void FindNewOrphans(Node* orphan_node);
  void AdoptNewPath(Node* node);
  void SetTerminalNode(Node* node);
  void Same(CapType& dst_val, CapType comp_val);
  void PushFlow(Node* src_node, Node* dst_node, Edge* flow_edge, CapType push_flow_capacity);
  void SetResFlow(Node* node);

  bool Empty(bool node_type);
  void Push(Node* node);
  Node*& Top(bool node_type);
  void Pop(bool node_type);
  void PushEnoughFlowToOneNode(bool node_type);

  void PushEnoughFlowToTwoNodes(Node* source_node, Node* sink_node);
  void PushEnoughFlow(Node* source_node, Node* sink_node, CapType* min_edge_capacity);

  void Augment(Edge* meet_edge);
  void FindNewPath(Node* orphan_node, int r_max = 0);
  void CreateOutEdges(Node* cen_node);
  Edge* GetEdge(Node* src_node, Node* dst_node);
  Edge* CreateEdge(Node* src_node, Node* dst_node, double punish_factor);

  std::vector<Node> m_nodes;
  CapType m_flow;
  Node* m_first_at_node;
  Node* m_mid_at_node;
  Node* m_last_at_node;
  std::queue<Node*> m_orphan_nodes;
  ImageData<int>* m_marked_image;
  int m_image_width;
  int m_image_height;
  int m_global_timestamp;
  int m_path;
  CapType m_source_res_flow;
  bool m_is_source_end;
  CapType m_sink_res_flow;
  bool m_is_sink_end;
  Node* m_top_source_node;
  Node* m_top_sink_node;
  Node* m_source_first_node;
  Node* m_sink_first_node;
  EdgePunishFun m_epf;
};

template <class CapType, class EdgePunishFun>
IFGraph<CapType, EdgePunishFun>::IFGraph(int max_nodes_number, int image_width, int image_height,
                                       EdgePunishFun epf, ImageData<int>* marked_image)
  : m_flow(0)
  , m_first_at_node(NULL)
  , m_mid_at_node(m_first_at_node)
  , m_last_at_node(m_first_at_node)
  , m_marked_image(marked_image)
  , m_image_width(image_width)
  , m_image_height(image_height)
  , m_global_timestamp(0)
  , m_path(0)
  , m_source_res_flow(0)
  , m_is_source_end(false)
  , m_sink_res_flow(0)
  , m_is_sink_end(false)
  , m_top_source_node(NULL)
  , m_top_sink_node(NULL)
  , m_source_first_node(NULL)
  , m_sink_first_node(NULL)
  , m_epf(epf) {
    m_nodes.reserve(max_nodes_number);
  }

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::AddNode(int node_id, CapType source_capacity,
                              CapType sink_capacity, int node_colour) {
  m_flow += source_capacity < sink_capacity ? source_capacity : sink_capacity;
  CapType node_capacity = source_capacity - sink_capacity;
  Node* node = &m_nodes[node_id];
  node->m_node_idx = node_id;
  node->m_node_colour = node_colour;
  node->m_excess = node_capacity;
  node->m_node_state = node_capacity > 0 ? SOURCE : SINK;
  node->m_parent_edge = TERMINAL;
  node->m_child_edge = NULL;
  node->m_timestamp = 0;
  node->m_terminal_dist = 1;
  node->m_depth = 1;
  node->m_is_active = false;
  node->m_is_gotten_all_edges = false;
  node->m_out_edges_num = 0;
  node->m_next_active = NULL;
  node->m_needed_flow = 0;
  node->m_next_node = NULL;
  // SET_PIXEL(m_marked_image, node_id, node_capacity > 0 ? WHITE : BLACK);
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::AddActiveNodes(int node_x, int node_y) {
  int index = node_y * m_image_width + node_x;
  Node* cen_node = &m_nodes[index];
  int arr_index[HALF_NEIGHBOUR] =
    HALF_NEIGHBOUR_ARR_INDEX(node_x, node_y, m_image_width, m_image_height);
  for (int i = 0; i < HALF_NEIGHBOUR; ++i) {
    Node* arr_node = &m_nodes[arr_index[i]];
    if (cen_node->m_node_state != arr_node->m_node_state) {
      AddActiveNodeBack(cen_node);
      // AddActiveNodeBack(arr_node);
      break;
    }
  }
}

template <class CapType, class EdgePunishFun>
typename IFGraph<CapType, EdgePunishFun>::Edge* IFGraph<CapType, EdgePunishFun>::GetEdge(
  Node* src_node, Node* dst_node) {
  for (int i = 0; i < src_node->m_out_edges_num; ++i) {
    if (src_node->m_out_edges[i].m_dst_node == dst_node) {
      return &src_node->m_out_edges[i];
    }
  }
  return NULL;
}

template <class CapType, class EdgePunishFun>
typename IFGraph<CapType, EdgePunishFun>::Edge* IFGraph<CapType, EdgePunishFun>::CreateEdge(
  Node* src_node, Node* dst_node, double punish_factor) {
  Edge* edge = &src_node->m_out_edges[src_node->m_out_edges_num++];
  Edge* rev_edge = &dst_node->m_out_edges[dst_node->m_out_edges_num++];
  CapType cap = punish_factor * m_epf(src_node->m_node_colour, dst_node->m_node_colour);
  edge->m_edge_capacity = cap;
  edge->m_rev_edge = rev_edge;
  edge->m_dst_node = dst_node;
  rev_edge->m_edge_capacity = cap;
  rev_edge->m_rev_edge = edge;
  rev_edge->m_dst_node = src_node;
  return edge;
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::CreateOutEdges(Node* cen_node) {
  assert(cen_node != NULL);
  if (cen_node->m_is_gotten_all_edges) {
    return;
  }
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
      if (abs(arr_nodes_idx[i] - cen_node->m_node_idx) == 1 ||
          abs(arr_nodes_idx[i] - cen_node->m_node_idx) == m_image_width) {
        punish_factor = 1;
      } else {
        punish_factor = ifgraph_div_sqrt2;
      }
      CreateEdge(cen_node, arr_node, punish_factor);
    }
  }
  cen_node->m_is_gotten_all_edges = true;
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::FindNewPath(Node* orphan_node, int r_max) {
  m_global_timestamp++;
  int dist_min = INIFINITE_DIST;
  CapType max_node_cap = 0;
  Edge* connected_edge_min = NULL;

  CreateOutEdges(orphan_node);
  if (!r_max || orphan_node->m_depth <= r_max) {
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
        if (parent_edge) {
          int dist = 0;
          // CapType node_cap = 0;
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
              // node_cap = dst_node->m_excess > 0 ?
              //   dst_node->m_excess : -dst_node->m_excess;
              break;
            }
            if (parent_edge == ORPHAN || !parent_edge) {
              dist = INIFINITE_DIST;
              break;
            }
            dst_node = parent_edge->m_dst_node;
          }
          if (dist < INIFINITE_DIST && (!r_max || dist <= r_max)) {
            if (dist < dist_min) {
              connected_edge_min = connected_edge;
              dist_min = dist;
              // max_node_cap = node_cap;
              // if (dist_min == 1) {
              //   break;
              // }
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
  }
  orphan_node->m_parent_edge = connected_edge_min;
  if (connected_edge_min) {
    orphan_node->m_timestamp = m_global_timestamp;
    orphan_node->m_terminal_dist = connected_edge_min->m_dst_node->m_terminal_dist + 1;
  }
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::SetTerminalNode(Node* node) {
  node->m_parent_edge = TERMINAL;
  node->m_terminal_dist = 1;
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::Same(CapType& dst_val, CapType comp_val) {
  if (ABS(dst_val- comp_val) < EPSILON) {
    dst_val = comp_val;
  }
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::PushFlow(
  Node* src_node, Node* dst_node, Edge* flow_edge, CapType push_flow_capacity) {
  CapType flow = std::min(ABS(src_node->m_excess), flow_edge->m_edge_capacity);
  flow = std::min(flow, ABS(push_flow_capacity));

  assert(src_node->m_node_state == dst_node->m_node_state);
  src_node->m_excess -= src_node->m_node_state == SOURCE ? flow : -flow;
  Same(src_node->m_excess, 0);

  dst_node->m_excess += src_node->m_node_state == SOURCE ? flow : -flow;
  flow_edge->m_edge_capacity -= flow;
  Same(flow_edge->m_edge_capacity, 0);

  flow_edge->m_rev_edge->m_edge_capacity += flow;
  dst_node->m_needed_flow -= dst_node->m_node_state == SOURCE ? flow : -flow;
  Same(dst_node->m_needed_flow, 0);
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::FindNewOrphans(Node* orphan_node) {
  assert(!orphan_node->m_parent_edge);
  for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
    Edge* connected_edge = &orphan_node->m_out_edges[i];
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

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::AdoptNewPath(Node* node) {
  AddOrphanNode(node);
  while (true) {
    Node* orphan_node = GetOrphanNode();
    if (!orphan_node) {
      break;
    }
    FindNewPath(orphan_node, R_MAX);
    if (!orphan_node->m_parent_edge) {
      if (orphan_node == m_source_first_node || orphan_node == m_sink_first_node) {
        if (orphan_node->m_node_state == SOURCE) {
          m_is_source_end = true;
        } else {
          m_is_sink_end = true;
        }
      } else {
        FindNewOrphans(orphan_node);
      }
    }
  }
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::SetResFlow(Node* node) {
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
  }
}

template <class CapType, class EdgePunishFun>
bool IFGraph<CapType, EdgePunishFun>::Empty(bool node_type) {
  Node*& top_node = node_type == SOURCE ? m_top_source_node : m_top_sink_node;
  if (!top_node) {
    return true;
  }
  return false;
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::Push(Node* node) {
  Node*& top_node = node->m_node_state == SOURCE ? m_top_source_node : m_top_sink_node;
  node->m_next_node = top_node;
  top_node = node;
}

template <class CapType, class EdgePunishFun>
typename IFGraph<CapType, EdgePunishFun>::Node*&
  IFGraph<CapType, EdgePunishFun>::Top(bool node_type) {
  return node_type == SOURCE ? m_top_source_node : m_top_sink_node;
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::Pop(bool node_type) {
  Node*& top_node = node_type == SOURCE ? m_top_source_node : m_top_sink_node;
  assert(top_node);
  Node* tmp = top_node;
  top_node = top_node->m_next_node;
  tmp->m_next_node = NULL;
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::PushEnoughFlowToOneNode(bool node_type) {
  assert(!Empty(node_type));
  Node*& node = Top(node_type);
  bool& node_end = node_type == SOURCE ? m_is_source_end : m_is_sink_end;
  bool& other_node_end = node_type == SINK ? m_is_source_end : m_is_sink_end;
  CapType& node_res_flow = node_type == SOURCE ? m_source_res_flow : m_sink_res_flow;
  CapType& other_res_flow = node_type == SINK ? m_source_res_flow : m_sink_res_flow;

  Node* src_node = node;
  CapType push_flow = node->m_excess;
  while (!node_end && node->m_parent_edge && node->m_needed_flow) {
    if (node->m_parent_edge == ORPHAN || node->m_parent_edge == TERMINAL) {
      AdoptNewPath(node);
    }
    if (node_end || !node->m_parent_edge || !node->m_needed_flow) {
      break;
    }
    // if (node->m_child_edge) {
    //   Node* child_node = node->m_child_edge->m_dst_node;
    //   if (child_node->m_parent_edge == node->m_child_edge->m_rev_edge &&
    //       child_node->m_terminal_dist <= node->m_terminal_dist) {
    //     AddOrphanNode(child_node);
    //     break;
    //   }
    // }

    assert(node->m_needed_flow && node->m_parent_edge);
    Node* dst_node = node->m_parent_edge->m_dst_node;
    dst_node->m_child_edge = node->m_parent_edge->m_rev_edge;
    Edge* flow_edge = node_type == SINK ?
      node->m_parent_edge : node->m_parent_edge->m_rev_edge;
    assert(flow_edge->m_edge_capacity);
    CapType dst_nd_flow;
    CapType delt_flow = std::min(ABS(dst_node->m_excess), flow_edge->m_edge_capacity);
    dst_nd_flow = std::min(flow_edge->m_edge_capacity, ABS(node->m_needed_flow)) - delt_flow;
    assert(dst_nd_flow - ABS(node_res_flow) < EPSILON);
    dst_node->m_needed_flow = node_type == SOURCE ? dst_nd_flow : -dst_nd_flow;
    dst_node->m_depth = node->m_depth + 1;
    if (dst_nd_flow > 0) {
      SetResFlow(dst_node);
      Push(dst_node);
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
      src_node = dst_node;
    }
    break;
  }
  if (src_node->m_child_edge) {
    Node* child_node = src_node->m_child_edge->m_dst_node;
    Edge* flow_edge = node_type == SOURCE ?
      src_node->m_child_edge : src_node->m_child_edge->m_rev_edge;
    // if (src_node == node && ABS(node_res_flow) < ABS(other_res_flow) && other_node_end) {
    //   push_flow = node->m_excess + (m_sink_res_flow + m_source_res_flow);
    //   assert(ABS(push_flow) < ABS(node->m_excess));
    //   node_res_flow = -other_res_flow;
    //   SetTerminalNode(node);
    // }
    if (push_flow) {
      PushFlow(src_node, child_node, flow_edge, push_flow);
    }
    if (child_node->m_parent_edge && !flow_edge->m_edge_capacity) {
      AddOrphanNode(child_node);
    }
  }
  if (src_node == node) {
    Pop(node_type);
    if (Empty(node_type)) {
      other_node_end = true;
    }
  }
}

// node is current Node
// needed_flow is except the excess of the node what the node need in flow.
// res_flow is the total resident flow
template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::PushEnoughFlowToTwoNodes(Node* source_node, Node* sink_node) {
  Push(source_node);
  Push(sink_node);
  m_source_first_node = source_node;
  m_sink_first_node = sink_node;
  while (!Empty(SOURCE) || !Empty(SINK)) {
    // if (!source_nodes.empty()) {
    //   printf("s node = %d, excess = %f, need = %f, res = %f\n", source_nodes.top()->m_id, source_nodes.top()->m_excess, source_nodes.top()->m_needed_flow, m_source_res_flow);
    // }
    // if (!sink_nodes.empty()) {
    //   printf("sk node = %d, excess = %f, need = %f, res = %f\n", sink_nodes.top()->m_id, sink_nodes.top()->m_excess, sink_nodes.top()->m_needed_flow, m_sink_res_flow);
    // }
    if (!Empty(SOURCE) && (m_source_res_flow > -m_sink_res_flow || Empty(SINK))) {
      PushEnoughFlowToOneNode(SOURCE);
    } else {
      PushEnoughFlowToOneNode(SINK);
    }
  }
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::PushEnoughFlow(
  Node* source_node, Node* sink_node, CapType* min_edge_capacity) {
  m_source_res_flow = *min_edge_capacity;
  m_sink_res_flow = -*min_edge_capacity;
  m_is_source_end = false;
  m_is_sink_end = false;
  source_node->m_needed_flow = m_source_res_flow - source_node->m_excess > 0 ?
    m_source_res_flow - source_node->m_excess : 0;
  sink_node->m_needed_flow = m_sink_res_flow - sink_node->m_excess < 0 ?
    m_sink_res_flow - sink_node->m_excess : 0;
  source_node->m_child_edge = NULL;
  sink_node->m_child_edge = NULL;
  SetResFlow(source_node);
  SetResFlow(sink_node);
  source_node->m_depth = 1;
  sink_node->m_depth = 1;

  PushEnoughFlowToTwoNodes(source_node, sink_node);

  Same(source_node->m_excess, *min_edge_capacity);
  Same(sink_node->m_excess, -*min_edge_capacity);
  Same(source_node->m_excess, -sink_node->m_excess);
  if (!source_node->m_parent_edge) {
    SetTerminalNode(source_node);
  }
  if (!sink_node->m_parent_edge) {
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

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::Augment(Edge* meet_edge) {
  Edge* first_edge[2] = {meet_edge->m_rev_edge, meet_edge};
	CapType min_capacity = meet_edge -> m_edge_capacity;
  // first_edge[0] for source tree and first_edge[1] for sink tree
  // find min capacity from path
  Node* flow_nodes[2];
  for (int i = 0; i < 2; ++i) {
    Node* parent_node = first_edge[i]->m_dst_node;
#ifdef ENABLE_BFS
    parent_node->m_child_edge = NULL;
#endif
    for (Edge* parent_edge = parent_node->m_parent_edge; parent_edge != TERMINAL;
         parent_node = parent_edge->m_dst_node, parent_edge = parent_node->m_parent_edge) {
      assert(parent_edge);
      m_path++;
#ifdef ENABLE_BFS
      parent_edge->m_dst_node->m_child_edge = parent_edge->m_rev_edge;
#endif
      Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
      CapType cap = edge->m_edge_capacity;
      if (cap < min_capacity) {
        min_capacity = cap;
      }
    }
    flow_nodes[i] = parent_node;
  }

  PushEnoughFlow(flow_nodes[0], flow_nodes[1], &min_capacity);

  first_edge[0]->m_edge_capacity += min_capacity;
  first_edge[1]->m_edge_capacity -= min_capacity;
  for (int i = 0; i < 2; ++i) {
    Node* parent_node = first_edge[i]->m_dst_node;
    int factor = 2 * i - 1;
    for (Edge* parent_edge = parent_node->m_parent_edge; parent_node != flow_nodes[i];
         parent_node = parent_edge->m_dst_node, parent_edge = parent_node->m_parent_edge) {
      parent_edge->m_edge_capacity += (-factor) * min_capacity;
      parent_edge->m_rev_edge->m_edge_capacity += factor * min_capacity;
      Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
      if (!edge->m_edge_capacity) {
        AddOrphanNode(parent_node);
      }
    }
    parent_node->m_excess += factor * min_capacity;
    if (!parent_node->m_excess) {
      AddOrphanNode(parent_node);
    }
  }
  m_flow += min_capacity;
}

template <class CapType, class EdgePunishFun>
void IFGraph<CapType, EdgePunishFun>::MaxFlow() {
  m_mid_at_node = m_last_at_node;
  Node* at_node = NULL;
  Edge* meet_edge = NULL;
  int tree_edges = 0;
  while (true) {
#ifndef ENABLE_PAR
    if (meet_edge == NULL || at_node->m_parent_edge == NULL) {
#endif
      at_node = GetActiveNode();
      if (at_node == NULL) {
        break;
      }
      CreateOutEdges(at_node);
#ifndef ENABLE_PAR
    }
#endif
    meet_edge = NULL;

    // grow source tree and sink tree

    int min_dist = INIFINITE_DIST;
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
          tree_edges++;
          // SET_PIXEL(m_marked_image, dst_node->m_node_idx, dst_node->m_node_state == SOURCE ? COLOUR_DARK_RED : GRAY);
        } else if (dst_node->m_node_state != at_node->m_node_state) {
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
      // augment path
      Augment(meet_edge);

      // adopt orphan nodes
      while (true) {
        Node* orphan_node = GetOrphanNode();
        if (!orphan_node) {
          break;
        }

        FindNewPath(orphan_node);

        if (orphan_node->m_parent_edge) {
          // SET_PIXEL(m_marked_image, orphan_node->m_node_idx, orphan_node->m_node_state == SOURCE ? RED : BLUE);
          tree_edges++;
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
  // printf("augment path = %d, tree_edges = %d, m_flow = %f\n",
  //        m_path, tree_edges, m_flow);
}

template <class CapType, class EdgePunishFun>
bool IFGraph<CapType, EdgePunishFun>::IsBelongToSource(int node_id) {
  return m_nodes[node_id].m_node_state == SOURCE;
}

#undef TERMINAL
#undef ORPHAN
#undef EIGHT_ARR_INDEX
#undef ENABLE_BFS
#undef ENABLE_PAR
#undef R_MAX 
#endif  // INCLUDE_IFGRAPH_H_
