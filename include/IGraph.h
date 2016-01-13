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
  };

  struct Node {
    int m_node_idx;
    int m_node_colour;
    CapType m_residue_capacity;
    NodeState m_node_state;
    Edge* m_parent_edge;
    // the timestamp of the latest dist calculating 
    int m_timestamp;
    int m_terminal_dist;
    bool m_is_active;
    bool m_is_gotten_all_edges;
    Edge m_out_edges[NEIGHBOUR];
    int m_out_edges_num;
    Node* m_next_active;
    Node* m_next_orphan;
  };

  IGraph(int max_nodes_number, int image_width,
         int image_height, EdgePunishFun epf, ImageData<int>* marked_image);
  ~IGraph() {}
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
        m_last_at_node= node;
      } else {
        m_first_at_node = node;
        m_last_at_node = node;
      }
      node->m_next_active = NULL;
    }
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

  void FindNewPath(Node* orphan_node);
  void CreateOutEdges(Node* cen_node);
  Edge* GetEdge(Node* src_node, Node* dst_node);
  Edge* CreateEdge(Node* src_node, Node* dst_node, double punish_factor);

  std::vector<Node> m_nodes;
  CapType m_flow;
  int m_actives_num;
  Node* m_first_at_node;
  Node* m_last_at_node;
  Node* m_first_orpn_node;
  Node* m_last_orpn_node;
  ImageData<int>* m_marked_image;
  int m_image_width;
  int m_image_height;
  int m_nlink;
  EdgePunishFun m_epf;
  int m_global_timestamp;
};

template <class CapType, class EdgePunishFun>
IGraph<CapType, EdgePunishFun>::IGraph(int max_nodes_number, int image_width, int image_height,
                                       EdgePunishFun epf, ImageData<int>* marked_image)
  : m_flow(0)
  , m_actives_num(0)
  , m_first_at_node(NULL)
  , m_last_at_node(m_first_at_node)
  , m_first_orpn_node(NULL)
  , m_last_orpn_node(m_first_orpn_node)
  , m_marked_image(marked_image)
  , m_image_width(image_width)
  , m_image_height(image_height)
  , m_nlink(0)
  , m_global_timestamp(0)
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
  node->m_timestamp = 0;
  node->m_terminal_dist = 1;
  node->m_is_active = false;
  node->m_is_gotten_all_edges = false;
  node->m_out_edges_num = 0;
  node->m_next_active = NULL;
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
      AddActiveNodeBack(cen_node);
      AddActiveNodeBack(arr_node);
      // m_actives_num++;
      // break;
    }
  }
}

template <class CapType, class EdgePunishFun>
typename IGraph<CapType, EdgePunishFun>::Edge* IGraph<CapType, EdgePunishFun>::GetEdge(
  Node* src_node, Node* dst_node) {
  for (int i = 0; i < src_node->m_out_edges_num; ++i) {
    if (src_node->m_out_edges[i].m_dst_node == dst_node) {
      return &src_node->m_out_edges[i];
    }
  }
  return NULL;
}

template <class CapType, class EdgePunishFun>
typename IGraph<CapType, EdgePunishFun>::Edge* IGraph<CapType, EdgePunishFun>::CreateEdge(
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
void IGraph<CapType, EdgePunishFun>::CreateOutEdges(Node* cen_node) {
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
      if (abs(arr_nodes_idx[i] - cen_node->m_node_idx) == 1 ||
          abs(arr_nodes_idx[i] - cen_node->m_node_idx) == m_image_width) {
        punish_factor = 1;
      } else {
        punish_factor = igraph_div_sqrt2;
      }
      CreateEdge(cen_node, arr_node, punish_factor);
    }
  }
  cen_node->m_is_gotten_all_edges = true;
}

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::FindNewPath(Node* orphan_node) {
  int dist_min = INIFINITE_DIST;
  CapType max_node_cap = 0;
  Edge* connected_edge_min = NULL;
  orphan_node->m_parent_edge = ORPHAN;

  CreateOutEdges(orphan_node);
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

template <class CapType, class EdgePunishFun>
void IGraph<CapType, EdgePunishFun>::MaxFlow() {
  Node* at_node = NULL;
  Edge* meet_edge = NULL;
  int path = 0;
  int tree_edges = 0;
  int broken_edges = 0;
  while (true) {
    if (meet_edge == NULL || at_node->m_parent_edge == NULL) {
      at_node = GetActiveNode();
      if (at_node == NULL) {
        break;
      }
      CreateOutEdges(at_node);
    }
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

    m_global_timestamp++;

    if (meet_edge) {
      // augment path
      Edge* first_edge[2] = {meet_edge->m_rev_edge, meet_edge};
	    CapType min_capacity = meet_edge -> m_edge_capacity;
      // first_edge[0] for source tree and first_edge[1] for sink tree
      // find min capacity from path
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
      
      first_edge[0]->m_edge_capacity += min_capacity;
      first_edge[1]->m_edge_capacity -= min_capacity;
      bool edge_saturtation = false;
      bool node_saturtation = false;
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
            broken_edges++;
            edge_saturtation = true;
          }
        }
        parent_node->m_residue_capacity += factor * min_capacity;
        if (!parent_node->m_residue_capacity) {
          AddOrphanNodeBack(parent_node);
          node_saturtation = true;
        }
      }
      m_flow += min_capacity;

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
        } else {
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
                AddOrphanNodeBack(dst_node);
                broken_edges++;
              }
            }
          }
        }
      }
    }
  }
  // printf("augment path = %d, tree_edges = %d, broken_edges = %d, m_flow = %f\n",
  //        path, tree_edges, broken_edges, m_flow);
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
