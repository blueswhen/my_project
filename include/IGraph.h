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
    CapType m_residue_capacity;
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
    // int m_timestamp;
    int m_terminal_dist;
    bool m_is_gotten_all_edges;
    Edge m_out_edges[NEIGHBOUR];
    int m_out_edges_num;
    bool m_in_actived;
    Node* m_next_orphan;
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
  void AddOrphanNodeBack(Node* orphan_node) {
    assert(orphan_node->m_parent_edge != ORPHAN);
    orphan_node->m_parent_edge = ORPHAN;
    if (m_last_orpn_node) {
      m_last_orpn_node->m_next_orphan = orphan_node;
      m_last_orpn_node = orphan_node;
    } else {
      m_first_orpn_node = orphan_node;
      m_last_orpn_node = orphan_node;
    }
    orphan_node->m_next_orphan = NULL;
  }
  Node* GetOrphanNode() {
    Node* orphan_node = m_first_orpn_node;
    if (m_first_orpn_node) {
      m_first_orpn_node = m_first_orpn_node->m_next_orphan;
      if (!m_first_orpn_node) {
        m_last_orpn_node = NULL;
      }
    }
    return orphan_node;
  }

  void FindNewPath(Node* orphan_node);
  void CreateOutEdges(Node* cen_node);
  Edge* CreateEdge(Node* src_node, Node* dst_node, double punish_factor,
                   int edge_index, int rev_edge_index);

  std::vector<Node> m_nodes;
  CapType m_flow;
  Node* m_first_orpn_node;
  Node* m_last_orpn_node;
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
  // int m_global_timestamp;
  int m_path;
};

template <class CapType, class EdgePunishFun>
IGraph<CapType, EdgePunishFun>::IGraph(int max_nodes_number, int image_width, int image_height,
                                       EdgePunishFun epf, ImageData<int>* marked_image)
  : m_flow(0)
  , m_first_orpn_node(NULL)
  , m_last_orpn_node(m_first_orpn_node)
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
  // , m_global_timestamp(0)
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
  node->m_residue_capacity = node_capacity;
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
  // node->m_timestamp = 0;
  node->m_terminal_dist = 1;
  node->m_is_gotten_all_edges = false;
  node->m_out_edges_num = 0;
  node->m_in_actived = false;
  node->m_next_orphan = NULL;
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
      if (!capacity || connected_edge->m_dst_node->m_terminal_dist !=
          orphan_node->m_terminal_dist - 1) {
        continue;
      }
      if (connected_edge->m_dst_node->m_parent_edge) {
        connected_edge_min = connected_edge;
        assert(orphan_node->m_terminal_dist == connected_edge_min->m_dst_node->m_terminal_dist + 1);
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
      if (capacity != 0) {
        Node* dst_node = connected_edge->m_dst_node;
        if (!dst_node->m_parent_edge) {
          dst_node->m_parent_edge = connected_edge->m_rev_edge;
          dst_node->m_next_child_node = at_node->m_first_child_node;
          at_node->m_first_child_node = dst_node;
          // dst_node->m_timestamp = at_node->m_timestamp;
          dst_node->m_terminal_dist = at_node->m_terminal_dist + 1;
          dst_node->m_node_state = at_node->m_node_state;
          AddActiveNodeBack(dst_node);
          // SET_PIXEL(m_marked_image, dst_node->m_node_idx, dst_node->m_node_state == SOURCE ? COLOUR_DARK_RED : GRAY);
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
      // augment path
      Edge* first_edge[2] = {meet_edge->m_rev_edge, meet_edge};
	    CapType min_capacity = meet_edge -> m_edge_capacity;
      // first_edge[0] for source tree and first_edge[1] for sink tree
      // find min capacity from path
      int path = 0;
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
          path++;
        }
        CapType node_cap = parent_node->m_residue_capacity;
        CapType final_node_capacity = i == 0 ? node_cap : -node_cap; 
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
        int factor = 2 * i - 1;
        for (Edge* parent_edge = parent_node->m_parent_edge; parent_edge != TERMINAL;
             parent_node = parent_edge->m_dst_node, parent_edge = parent_node->m_parent_edge) {
          parent_edge->m_edge_capacity += (-factor) * min_capacity;
          parent_edge->m_rev_edge->m_edge_capacity += factor * min_capacity;
          Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
          if (!edge->m_edge_capacity) {
            AddOrphanNodeBack(parent_node);
            Node* node = parent_edge->m_dst_node->m_first_child_node;
            if (node == parent_node) {
              parent_edge->m_dst_node->m_first_child_node = parent_node->m_next_child_node;
            } else {
              for (; node->m_next_child_node != parent_node; node = node->m_next_child_node);
              node->m_next_child_node = parent_node->m_next_child_node;
            }
          }
        }
        parent_node->m_residue_capacity += factor * min_capacity;
        if (!parent_node->m_residue_capacity) {
          AddOrphanNodeBack(parent_node);
        }
      }
      m_flow += min_capacity;

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
            AddOrphanNodeBack(child);
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
    }
  }
  // printf("augment path = %d, m_flow = %f\n", path, m_flow);
  // printf("r = %f\n", static_cast<double>(m_nlink)/(m_image_width * m_image_height));
}

template <class CapType, class EdgePunishFun>
bool IGraph<CapType, EdgePunishFun>::IsBelongToSource(int node_id) {
  return m_nodes[node_id].m_node_state == SOURCE;
}

#undef TERMINAL
#undef ORPHAN
#undef EIGHT_ARR_INDEX
#endif  // INCLUDE_IGRAPH_H_
