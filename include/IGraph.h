// Copyright 2014-12 sxniu
#ifndef INCLUDE_IGRAPH_H_
#define INCLUDE_IGRAPH_H_

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

#define TERMINAL reinterpret_cast<Edge*>(1)
#define ORPHAN reinterpret_cast<Edge*>(2)
#define INIFINITE_DIST INT_MAX

#define HALF_NEIGHBOUR 4
#define NEIGHBOUR 8
#define HALF_NEIGHBOUR_ARR_INDEX FOUR_ARR_INDEX
#define NEIGHBOUR_ARR_INDEX EIGHT_ARR_INDEX

// #define NEW_AL

const double igraph_div_sqrt2 = 1 / sqrt(2.0f);

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

#define REMOVE_SIBLING(x) \
{ \
  if ((x)->m_parent_edge && (x)->m_parent_edge != TERMINAL && (x)->m_parent_edge != ORPHAN) { \
    Node* tmp = (x)->m_parent_edge->m_dst_node->m_first_child_node; \
    if (tmp == (x)) { \
      (x)->m_parent_edge->m_dst_node->m_first_child_node = (x)->m_next_child_node; \
    } else { \
      for (; tmp->m_next_child_node != (x); tmp = tmp->m_next_child_node); \
      tmp->m_next_child_node = (x)->m_next_child_node; \
    } \
  } \
}

#define ADD_SIBLING(x) \
{ \
  (x)->m_next_child_node = (x)->m_parent_edge->m_dst_node->m_first_child_node; \
  (x)->m_parent_edge->m_dst_node->m_first_child_node = (x); \
}

template <class CapType, class EdgePunishFun>
class IGraph {
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
    Edge ()
      : m_dst_node(NULL)
      , m_edge_capacity(0)
      , m_rev_edge(NULL) {}
  };

  struct Node {
    int m_node_idx;
    int m_node_colour;
    CapType m_excess;
    NodeState m_node_state;
    Edge* m_parent_edge;
    Node* m_first_child_node;
    Node* m_next_child_node;
    Node* m_next_active_source;
    Node* m_next_active_sink;
    bool m_is_source_active;
    bool m_is_sink_active;
    bool m_is_new_source_start;
    bool m_is_new_sink_start;
    // the timestamp of the latest dist calculating 
    int m_timestamp;
    int m_terminal_dist;
    bool m_is_gotten_all_edges;
    Edge m_out_edges[NEIGHBOUR];
    int m_out_edges_num;
    bool m_in_actived;
#ifdef NEW_AL
    Edge* m_child_edge;
    CapType m_needed_flow;
    Node* m_next_node;
    bool m_free;
#endif
    Node()
      : m_out_edges({Edge(),Edge(),Edge(),Edge(),Edge(),Edge(),Edge(),Edge()}) {}
  };

  IGraph(int max_nodes_number, int image_width,
         int image_height, EdgePunishFun epf, ImageData<int>* marked_image);
  ~IGraph() {}
  void AddNode(int node_id, CapType source_capacity, CapType sink_capacity, int node_colour);
  void AddActiveNodes(int node_x, int node_y);
  void MaxFlow();
  bool IsBelongToSource(int node_id);

 private:
  void AddActiveSourceNodeBack(Node* node) {
    assert(node->m_node_state == SOURCE);
    if (!node->m_is_source_active) {
      assert(!node->m_next_active_source);
      node->m_is_source_active = true;
      node->m_in_actived = true;
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
      node->m_in_actived = true;
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
  bool AddActiveNodeBack(Node* node) {
    if (node->m_node_state == SOURCE) {
      if (node->m_terminal_dist == m_global_source_dist) {
        AddActiveSourceNodeBack(node);
        return true;
      }
    } else {
      if (node->m_terminal_dist == m_global_sink_dist) {
        AddActiveSinkNodeBack(node);
        return true;
      }
    }
    return false;
  }
  Node* GetActiveNode() {
    Node* active_node = NULL;
    if (m_global_state == SOURCE) {
      active_node = GetActiveSourceNode();
    } else {
      active_node = GetActiveSinkNode();
    }
    return active_node;
  }
  void AddOrphanNode(Node* orphan_node, bool is_remove_child = true) {
    assert(orphan_node->m_parent_edge != ORPHAN);
    if (is_remove_child) {
      REMOVE_SIBLING(orphan_node);
    }
    orphan_node->m_parent_edge = ORPHAN;
    m_orphan_nodes.push(orphan_node);
  }
  Node* GetOrphanNode() {
    Node* orphan_node = NULL;
    if (!m_orphan_nodes.empty()) {
      orphan_node = m_orphan_nodes.front();
      m_orphan_nodes.pop();
      assert(orphan_node->m_parent_edge == ORPHAN);
    }
    return orphan_node;
  }

  void FindNewPath(Node* orphan_node);
  void FindNewPath_(Node* orphan_node);
  void FindNewOrphans(Node* orphan_node);
  void Adoption();
  void Augment(Edge* meet_edge);
#ifdef NEW_AL
  void AdoptNewPath(Node* node);
  void SetTerminalNode(Node* node);
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
  void CreateOutEdges(Node* cen_node);
  Edge* CreateEdge(Node* src_node, Node* dst_node, double punish_factor,
                   int edge_index, int rev_edge_index);

  std::vector<Node> m_nodes;
  std::queue<Node*> m_orphan_nodes;
  CapType m_flow;
  // ImageData<int>* m_marked_image;
  int m_image_width;
  int m_image_height;
  // int m_nlink;
  EdgePunishFun m_epf;
  Node* m_first_at_source_node;
  Node* m_last_at_source_node;
  Node* m_first_at_sink_node;
  Node* m_last_at_sink_node;
  bool m_add_new_source_start;
  bool m_add_new_sink_start;
  int m_global_source_dist;
  int m_global_sink_dist;
  NodeState m_global_state;
  int m_global_source_orphan_num;
  int m_global_sink_orphan_num;
  int m_global_timestamp;
#ifdef NEW_AL
  CapType m_source_res_flow;
  bool m_is_source_end;
  CapType m_sink_res_flow;
  bool m_is_sink_end;
  Node* m_top_source_node;
  Node* m_top_sink_node;
  Node* m_source_first_node;
  Node* m_sink_first_node;
#endif
  int m_path;
};

template <class CapType, class EdgePunishFun>
IGraph<CapType, EdgePunishFun>::IGraph(int max_nodes_number, int image_width, int image_height,
                                       EdgePunishFun epf, ImageData<int>* marked_image)
  : m_flow(0)
  // , m_marked_image(marked_image)
  , m_image_width(image_width)
  , m_image_height(image_height)
  // , m_nlink(0)
  , m_first_at_source_node(NULL)
  , m_last_at_source_node(NULL)
  , m_first_at_sink_node(NULL)
  , m_last_at_sink_node(NULL)
  , m_add_new_source_start(true)
  , m_add_new_sink_start(true)
  , m_global_source_dist(1)
  , m_global_sink_dist(1)
  , m_global_state(SOURCE)
  , m_global_source_orphan_num(0)
  , m_global_sink_orphan_num(0)
  , m_global_timestamp(0)
#ifdef NEW_AL
  , m_source_res_flow(0)
  , m_is_source_end(false)
  , m_sink_res_flow(0)
  , m_is_sink_end(false)
  , m_top_source_node(NULL)
  , m_top_sink_node(NULL)
  , m_source_first_node(NULL)
  , m_sink_first_node(NULL)
#endif
  , m_path(0)
  , m_epf(epf) {
    m_nodes.reserve(max_nodes_number);
  }

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::AddNode(int node_id, CapType source_capacity,
                              CapType sink_capacity, int node_colour) {
  m_flow += source_capacity < sink_capacity ? source_capacity : sink_capacity;
  CapType node_capacity = source_capacity - sink_capacity;
  Node* node = &m_nodes[node_id];
  node->m_node_idx = node_id;
  node->m_node_colour = node_colour;
  node->m_excess = node_capacity;
  node->m_node_state = node_capacity > 0 ? SOURCE : SINK;
  node->m_parent_edge = TERMINAL;
  node->m_first_child_node = NULL;
  node->m_next_child_node = NULL;
  node->m_next_active_source = NULL;
  node->m_next_active_sink = NULL;
  node->m_is_source_active = false;
  node->m_is_sink_active = false;
  node->m_is_new_source_start = false;
  node->m_is_new_sink_start = false;
  node->m_timestamp = 0;
  node->m_terminal_dist = 1;
  node->m_is_gotten_all_edges = false;
  node->m_out_edges_num = 0;
  node->m_in_actived = false;
#ifdef NEW_AL
  node->m_child_edge = NULL;
  node->m_needed_flow = 0;
  node->m_next_node = NULL;
  node->m_free = false;
#endif
  // SET_PIXEL(m_marked_image, node_id, node_capacity > 0 ? WHITE : BLACK);
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::AddActiveNodes(int node_x, int node_y) {
  int index = node_y * m_image_width + node_x;
  Node* cen_node = &m_nodes[index];
  int arr_index[HALF_NEIGHBOUR] =
    HALF_NEIGHBOUR_ARR_INDEX(node_x, node_y, m_image_width, m_image_height);
  for (int i = 0; i < HALF_NEIGHBOUR; ++i) {
    Node* arr_node = &m_nodes[arr_index[i]];
    if (arr_index[i] < index && cen_node->m_node_state != arr_node->m_node_state) {
      if (cen_node->m_node_state == SOURCE) {
        AddActiveSourceNodeBack(cen_node);
        AddActiveSinkNodeBack(arr_node);
      } else {
        AddActiveSourceNodeBack(arr_node);
        AddActiveSinkNodeBack(cen_node);
      }
    }
  }
}

template <class CapType, class EdgePunishFun>
typename IGraph<CapType, EdgePunishFun>::Edge* IGraph<CapType, EdgePunishFun>::CreateEdge(
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

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::CreateOutEdges(Node* cen_node) {
  assert(cen_node != NULL);
  if (cen_node->m_is_gotten_all_edges) {
    return;
  }
  // m_nlink++;
  int coordinate = cen_node->m_node_idx;
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
    if (cen_node->m_node_idx == arr_nodes_idx[i]) {
      continue;
    }
    Node* arr_node = &m_nodes[arr_nodes_idx[i]];
    // use it when active nodes are not enough
    if (!arr_node->m_in_actived) {
      AddActiveNodeBack(arr_node);
    }
    if (cen_node->m_out_edges[i].m_dst_node == NULL) {
      if (abs(arr_nodes_idx[i] - cen_node->m_node_idx) == 1 ||
          abs(arr_nodes_idx[i] - cen_node->m_node_idx) == m_image_width) {
        punish_factor = 1;
      } else {
        punish_factor = igraph_div_sqrt2;
      }
      int rev_idx = i - 4 < 0 ? i + 4 : i - 4;
      assert(arr_node->m_out_edges[rev_idx].m_dst_node == NULL);
      CreateEdge(cen_node, arr_node, punish_factor, i, rev_idx);
    }
  }
  cen_node->m_is_gotten_all_edges = true;
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::FindNewPath(Node* orphan_node) {
  assert(orphan_node->m_parent_edge == ORPHAN);
  Edge* connected_edge_min = NULL;
  if (orphan_node->m_terminal_dist == 1) {
    CreateOutEdges(orphan_node);
    orphan_node->m_parent_edge = NULL;
    return;
  }
  assert(orphan_node->m_is_gotten_all_edges);
  for (int i = 0; i < NEIGHBOUR; ++i) {
    if (orphan_node->m_out_edges[i].m_dst_node == NULL) {
      continue;
    }
    Edge* connected_edge = &orphan_node->m_out_edges[i];
    if (connected_edge->m_dst_node->m_node_state == orphan_node->m_node_state) {
      CapType capacity = orphan_node->m_node_state == SINK ?
                         connected_edge->m_edge_capacity :
                         connected_edge->m_rev_edge->m_edge_capacity;
      Node* dst_node = connected_edge->m_dst_node;
      if (!capacity
#ifdef NEW_AL
          && (!orphan_node->m_child_edge || orphan_node->m_child_edge->m_dst_node != dst_node)
#endif
          || dst_node->m_terminal_dist != orphan_node->m_terminal_dist - 1) {
        continue;
      }
      if (dst_node->m_parent_edge) {
        connected_edge_min = connected_edge;
        assert(orphan_node->m_terminal_dist == connected_edge_min->m_dst_node->m_terminal_dist + 1);
        break;
      }
    }
  }
  orphan_node->m_parent_edge = connected_edge_min;
  if (connected_edge_min) {
    ADD_SIBLING(orphan_node);
    orphan_node->m_timestamp = m_global_timestamp;
    orphan_node->m_terminal_dist = connected_edge_min->m_dst_node->m_terminal_dist + 1;
  }
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::FindNewPath_(Node* orphan_node) {
  m_global_timestamp++;
  int dist_min = INIFINITE_DIST;
  Edge* connected_edge_min = NULL;
  assert(orphan_node->m_parent_edge == ORPHAN);

  CreateOutEdges(orphan_node);
  for (int i = 0; i < NEIGHBOUR; ++i) {
    if (orphan_node->m_out_edges[i].m_dst_node == NULL) {
      continue;
    }
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
        while (true) {
          if (dst_node->m_timestamp == m_global_timestamp) {
            dist += dst_node->m_terminal_dist;
            break;
          }
          parent_edge = dst_node->m_parent_edge;
          dist++;
          if (parent_edge == TERMINAL) {
            dst_node->m_timestamp = m_global_timestamp;
            // dst_node->m_terminal_dist = 1;
            break;
          }
          if (parent_edge == ORPHAN) {
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
            // dst_node->m_terminal_dist = dist--;
          }
        }
      }
    }
  }
  orphan_node->m_parent_edge = connected_edge_min;
  if (connected_edge_min) {
    ADD_SIBLING(orphan_node);
    orphan_node->m_timestamp = m_global_timestamp;
    // orphan_node->m_terminal_dist = connected_edge_min->m_dst_node->m_terminal_dist + 1;
  }
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::FindNewOrphans(Node* orphan_node) {
  assert(!orphan_node->m_parent_edge);
#ifdef NEW_AL
  orphan_node->m_free = true;
#endif
  for (Node* child = orphan_node->m_first_child_node;
       child; child = child->m_next_child_node) {
    AddOrphanNode(child, false);
  }
  orphan_node->m_first_child_node = NULL;

  int dist_min = (orphan_node->m_node_state == SOURCE) ?
                  m_global_source_dist : m_global_sink_dist;
  if (orphan_node->m_terminal_dist != dist_min) {
    for (int i = 0; i < NEIGHBOUR; ++i) {
      if (orphan_node->m_out_edges[i].m_dst_node == NULL) {
        continue;
      }
      Edge* connected_edge = &orphan_node->m_out_edges[i];
      Node* dst_node = connected_edge->m_dst_node;
      Edge* parent_edge = dst_node->m_parent_edge;
      CapType capacity = dst_node->m_node_state == SINK ?
                         connected_edge->m_edge_capacity :
                         connected_edge->m_rev_edge->m_edge_capacity;
      if (parent_edge && (capacity
#ifdef NEW_AL
          || (orphan_node->m_child_edge && orphan_node->m_child_edge->m_dst_node == dst_node)
#endif
          ) &&
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
      ADD_SIBLING(orphan_node);
      assert(orphan_node->m_terminal_dist < dst_node->m_terminal_dist + 1);
      orphan_node->m_timestamp = dst_node->m_timestamp;
      orphan_node->m_terminal_dist = dst_node->m_terminal_dist + 1;
      AddActiveNodeBack(orphan_node);
    }
  }
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::Adoption() {
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
      FindNewOrphans(orphan_node);
    }
  }
}

#ifdef NEW_AL
template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::SetTerminalNode(Node* node) {
  REMOVE_SIBLING(node);
  node->m_parent_edge = TERMINAL;
  node->m_terminal_dist = 1;
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::Same(CapType& dst_val, CapType comp_val) {
  if (ABS(dst_val- comp_val) < EPSILON) {
    dst_val = comp_val;
  }
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::PushFlow(
  Node* src_node, Node* dst_node, Edge* flow_edge, CapType push_flow) {
  assert(src_node->m_node_state == dst_node->m_node_state);
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
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::AdoptNewPath(Node* node) {
  if (node->m_parent_edge == TERMINAL) {
    AddOrphanNode(node);
  }
  Adoption();
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::SetResFlow(bool node_type, CapType minus_flow) {
  if (!minus_flow) {
    return;
  }
  CapType& res_flow = node_type == SOURCE ? m_source_res_flow : m_sink_res_flow;
  bool& node_end = node_type == SOURCE ? m_is_source_end : m_is_sink_end;
  res_flow -= ABS(minus_flow);
  // Same(res_flow, 0);
  if (!res_flow) {
    node_end = true;
  }
}

template <class CapType, class EdgePunishFun>
bool IGraph<CapType, EdgePunishFun>::Empty(bool node_type) {
  Node*& top_node = node_type == SOURCE ? m_top_source_node : m_top_sink_node;
  if (!top_node) {
    return true;
  }
  return false;
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::Push(Node* node) {
  Node*& top_node = node->m_node_state == SOURCE ? m_top_source_node : m_top_sink_node;
  node->m_next_node = top_node;
  top_node = node;
}

template <class CapType, class EdgePunishFun>
typename IGraph<CapType, EdgePunishFun>::Node*&
  IGraph<CapType, EdgePunishFun>::Top(bool node_type) {
  return node_type == SOURCE ? m_top_source_node : m_top_sink_node;
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::Pop(bool node_type) {
  Node*& top_node = node_type == SOURCE ? m_top_source_node : m_top_sink_node;
  assert(top_node);
  Node* tmp = top_node;
  top_node = top_node->m_next_node;
  tmp->m_next_node = NULL;
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::PushEnoughFlowToOneNode(bool node_type) {
  assert(!Empty(node_type));
  Node* node = Top(node_type);
  bool& node_end = node_type == SOURCE ? m_is_source_end : m_is_sink_end;
  CapType& node_res_flow = node_type == SOURCE ? m_source_res_flow : m_sink_res_flow;

  Node* src_node = node;
  CapType push_flow = node->m_excess;
  if (node->m_needed_flow < 0) {
    push_flow = node->m_needed_flow == -INT_MAX ? 0 : node->m_needed_flow;
    node->m_needed_flow = 0;
  }
  while (!node_end && !node->m_free && node->m_needed_flow) {
    if (node->m_parent_edge == ORPHAN || node->m_parent_edge == TERMINAL) {
      AdoptNewPath(node);
      if (node_end || node->m_free) {
        break;
      }
    }

    assert(node->m_needed_flow && node->m_parent_edge);
    Node* dst_node = node->m_parent_edge->m_dst_node;
    dst_node->m_child_edge = node->m_parent_edge->m_rev_edge;
    Edge* flow_edge = node_type == SINK ?
      node->m_parent_edge : node->m_parent_edge->m_rev_edge;
    assert(flow_edge->m_edge_capacity);
    CapType src_nd_flow = std::min(flow_edge->m_edge_capacity, node->m_needed_flow);
    CapType dst_nd_flow = src_nd_flow - ABS(dst_node->m_excess);
    CapType minus_flow;
    if (dst_nd_flow > 0) {
      minus_flow = dst_node->m_excess;
      dst_node->m_needed_flow = dst_nd_flow;
    } else {
      minus_flow = src_nd_flow;
      dst_node->m_needed_flow = -src_nd_flow;
      assert(node->m_needed_flow);
      assert(dst_node->m_parent_edge == TERMINAL);
    }
    SetResFlow(node_type, minus_flow);
    Push(dst_node);
    return;
  }
  if (node->m_free) {
    node->m_free = false;
  }
  if (src_node->m_child_edge) {
    Node* child_node = src_node->m_child_edge->m_dst_node;
    Edge* flow_edge = node_type == SOURCE ?
      src_node->m_child_edge : src_node->m_child_edge->m_rev_edge;
    if (push_flow) {
      PushFlow(src_node, child_node, flow_edge, push_flow);
    }
    if (!flow_edge->m_edge_capacity && child_node->m_parent_edge &&
        child_node->m_parent_edge->m_dst_node == src_node) {
      AddOrphanNode(child_node);
    }
    src_node->m_child_edge = NULL;
  }
  Pop(node_type);
}

// node is current Node
// needed_flow is except the excess of the node what the node need in flow.
// res_flow is the total resident flow
template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::PushEnoughFlowToTwoNodes(Node* source_node, Node* sink_node) {
  Push(source_node);
  Push(sink_node);
  m_source_first_node = source_node;
  m_sink_first_node = sink_node;
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
        // Same(top_node->m_needed_flow, 0);
        if (top_node->m_needed_flow == 0) {
          top_node->m_needed_flow = -INT_MAX; 
        }
        assert(top_node->m_needed_flow <= 0);
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
        // Same(top_node->m_needed_flow, 0);
        if (top_node->m_needed_flow == 0) {
          top_node->m_needed_flow = -INT_MAX; 
        }
        assert(top_node->m_needed_flow <= 0);
        m_is_sink_end = true;
      }
      PushEnoughFlowToOneNode(SINK);
    }
  }
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::PushEnoughFlow(
  Node* source_node, Node* sink_node, CapType* min_edge_capacity) {
  m_source_res_flow = *min_edge_capacity;
  m_sink_res_flow = *min_edge_capacity;
  m_is_source_end = false;
  m_is_sink_end = false;
  source_node->m_needed_flow = std::max(m_source_res_flow - source_node->m_excess, 0.0);
  sink_node->m_needed_flow = std::max(m_sink_res_flow + sink_node->m_excess, 0.0);
  source_node->m_child_edge = NULL;
  sink_node->m_child_edge = NULL;
  SetResFlow(SOURCE, source_node->m_excess);
  SetResFlow(SINK, sink_node->m_excess);

  PushEnoughFlowToTwoNodes(source_node, sink_node);

  Same(source_node->m_excess, *min_edge_capacity);
  Same(sink_node->m_excess, -*min_edge_capacity);
  Same(source_node->m_excess, -sink_node->m_excess);
  *min_edge_capacity = std::min(*min_edge_capacity, source_node->m_excess);
  *min_edge_capacity = std::min(*min_edge_capacity, -sink_node->m_excess);
}
#endif

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::Augment(Edge* meet_edge) {
  // augment path
  Edge* first_edge[2] = {meet_edge->m_rev_edge, meet_edge};
	CapType min_capacity = meet_edge -> m_edge_capacity;
  // first_edge[0] for source tree and first_edge[1] for sink tree
  // find min capacity from path
  int path = 0;
#ifdef NEW_AL
  Node* flow_nodes[2] = {meet_edge->m_rev_edge->m_dst_node, meet_edge->m_dst_node};
#else
  for (int i = 0; i < 2; ++i) {
    Node* parent_node = first_edge[i]->m_dst_node;
    for (Edge* parent_edge = parent_node->m_parent_edge; parent_edge != TERMINAL;
         parent_node = parent_edge->m_dst_node, parent_edge = parent_node->m_parent_edge) {
      assert(parent_edge);
      assert(parent_edge != ORPHAN);
      Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
      CapType cap = edge->m_edge_capacity;
      assert(cap > 0);
      if (cap < min_capacity) {
        min_capacity = cap;
      }
      m_path++;
    }
    CapType node_cap = parent_node->m_excess;
    CapType final_node_capacity = i == 0 ? node_cap : -node_cap; 
    if (final_node_capacity < min_capacity) {
      min_capacity = final_node_capacity;
    }
  }
#endif
  // if (m_path != path) {
  //   printf("path = %d\n", path);
  //   m_path = path;
  // }
#ifdef NEW_AL
  PushEnoughFlow(flow_nodes[0], flow_nodes[1], &min_capacity);
 
  first_edge[0]->m_edge_capacity += min_capacity;
  first_edge[1]->m_edge_capacity -= min_capacity;
  CapType& source_excess = flow_nodes[0]->m_excess;
  CapType& sink_excess = flow_nodes[1]->m_excess;
  source_excess -= min_capacity;
  sink_excess += min_capacity;
  if (source_excess) {
    assert(flow_nodes[0]->m_parent_edge == TERMINAL);
  } else {
    if (flow_nodes[0]->m_parent_edge == TERMINAL) {
      AddOrphanNode(flow_nodes[0]);
    }
    assert(flow_nodes[0]->m_parent_edge != TERMINAL);
  }
  if (sink_excess) {
    assert(flow_nodes[1]->m_parent_edge == TERMINAL);
  } else {
    if (flow_nodes[1]->m_parent_edge == TERMINAL) {
      AddOrphanNode(flow_nodes[1]);
    }
    assert(flow_nodes[1]->m_parent_edge != TERMINAL);
  }
#else
  first_edge[0]->m_edge_capacity += min_capacity;
  first_edge[1]->m_edge_capacity -= min_capacity;
  for (int i = 0; i < 2; ++i) {
    Node* parent_node = first_edge[i]->m_dst_node;
    int factor = 2 * i - 1;
    for (Edge* parent_edge = parent_node->m_parent_edge; parent_edge != TERMINAL;
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
#endif
  m_flow += min_capacity;
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::MaxFlow() {
  Node* at_node = NULL;
  Edge* meet_edge = NULL;
  int path = 0;
  while (true) {
    if (meet_edge == NULL || at_node->m_parent_edge == NULL) {
      meet_edge = NULL;
      at_node = GetActiveNode();
      if (IsActiveSourceEmpty() && IsActiveSinkEmpty() && !at_node) {
        break;
      }
      if (!at_node) {
        continue;
      }
    }
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
    CreateOutEdges(at_node);
    for (int i = 0; i < NEIGHBOUR; ++i) {
      if (at_node->m_out_edges[i].m_dst_node == NULL) {
        continue;
      }
      Edge* connected_edge = &at_node->m_out_edges[i];
      CapType capacity = at_node->m_node_state == SINK ?
                         connected_edge->m_rev_edge->m_edge_capacity :
                         connected_edge->m_edge_capacity;
      Node* dst_node = connected_edge->m_dst_node;
      if (capacity != 0) {
        if (!dst_node->m_parent_edge) {
          dst_node->m_parent_edge = connected_edge->m_rev_edge;
          ADD_SIBLING(dst_node);
          dst_node->m_timestamp = at_node->m_timestamp;
          dst_node->m_terminal_dist = at_node->m_terminal_dist + 1;
          dst_node->m_node_state = at_node->m_node_state;
          AddActiveNodeBack(dst_node);
        } else if (dst_node->m_node_state != at_node->m_node_state) {
          meet_edge = at_node->m_node_state == SINK ?
                      connected_edge->m_rev_edge : connected_edge;
          break;
        }
        // else if (dst_node->m_timestamp <= at_node->m_timestamp &&
        //            dst_node->m_terminal_dist > at_node->m_terminal_dist) {
        //   dst_node->m_parent_edge = connected_edge->m_rev_edge;
        //   dst_node->m_timestamp = at_node->m_timestamp;
        //   dst_node->m_terminal_dist = at_node->m_terminal_dist + 1;
        // }
      }
    }

    // m_global_timestamp++;

    if (meet_edge) {
      Augment(meet_edge);
      Adoption();
    }
  }
  // printf("augment path = %d, m_flow = %f\n", m_path, m_flow);
  // // printf("r = %f\n", static_cast<double>(m_nlink)/(m_image_width * m_image_height));
}

template <class CapType, class EdgePunishFun>
bool IGraph<CapType, EdgePunishFun>::IsBelongToSource(int node_id) {
  return m_nodes[node_id].m_node_state == SOURCE;
}

#undef TERMINAL
#undef ORPHAN
#undef EIGHT_ARR_INDEX
#undef REMOVE_SIBLING
#undef ADD_SIBLING
#endif  // INCLUDE_IGRAPH_H_
