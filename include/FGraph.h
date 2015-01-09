// Copyright 2014-12 sxniu
#ifndef INCLUDE_FGRAPH_H_
#define INCLUDE_FGRAPH_H_

#include <vector>
#include <list>
#include <queue>
#include <deque>
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

const double div_sqrt2 = 1 / sqrt(2.0f);

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
  std::max(node_y - 1, 0) * image_width + node_x, \
  node_y * image_width + std::min(image_width - 1, node_x + 1), \
  std::min(node_y + 1, image_height - 1) * image_width + node_x, \
  std::max(node_y - 1, 0) * image_width + std::max(0, node_x - 1), \
  std::max(node_y - 1, 0) * image_width + std::min(image_width - 1, node_x + 1), \
  std::min(node_y + 1, image_height - 1) * image_width + std::min(image_width - 1, node_x + 1), \
  std::min(node_y + 1, image_height - 1) * image_width + std::max(0, node_x - 1) \
}

template <class CapType, class EdgePunishFun>
class FGraph {
#define CREATE_EDGE(src_node, dst_node, punish_factor) \
{ \
  Edge* edge = &m_edges[m_last_edge_index++]; \
  Edge* rev_edge = &m_edges[m_last_edge_index++]; \
  CapType cap = punish_factor * m_epf(src_node->m_node_colour, dst_node->m_node_colour); \
  edge->m_edge_capacity = cap; \
  rev_edge->m_edge_capacity = cap; \
  edge->m_rev_edge = rev_edge; \
  edge->m_dst_node = dst_node; \
  rev_edge->m_rev_edge = edge; \
  rev_edge->m_dst_node = src_node; \
  edge->m_next_out_edge = src_node->m_first_out_edge; \
  src_node->m_first_out_edge = edge; \
  rev_edge->m_next_out_edge = dst_node->m_first_out_edge; \
  dst_node->m_first_out_edge = rev_edge; \
  src_node->m_out_edges_label[src_node->m_label_idx++] = dst_node->m_node_idx; \
  dst_node->m_out_edges_label[dst_node->m_label_idx++] = src_node->m_node_idx; \
}

 public:
  enum NodeState {
    SOURCE,
    SINK,
  };
  FGraph(int max_nodes_number, int max_edges_number, int image_width,
         int image_height, EdgePunishFun epf);
  void AddNode(int node_id, CapType source_capacity, CapType sink_capacity, int node_colour);
  void AddActiveNodes(int node_x, int node_y);
  void AddEdge(int src_node_id, int dst_node_id, CapType edge_capacity);
  void MaxFlow();
  bool IsBelongToSource(int node_id);

 private:
  class Node;
  class Edge {
   public:
    Edge()
      : m_dst_node(NULL)
      , m_edge_capacity(0)
      , m_rev_edge(NULL)
      , m_next_out_edge(NULL) {}
    Node* m_dst_node;
    CapType m_edge_capacity;
    Edge* m_rev_edge;
    // next edge originated from the same node
    Edge* m_next_out_edge;
  };

  class Node {
   public:
    Node()
      : m_node_idx(0)
      , m_node_colour(BLACK)
      , m_residue_capacity(0)
      , m_node_state(SINK)
      , m_first_out_edge(NULL)
      , m_parent_edge(NULL)
      , m_timestamp(0)
      , m_terminal_dist(0)
      , m_is_active(false)
      , m_is_gotten_all_edges(false)
      , m_label_idx(0) {}
    int m_node_idx;
    int m_node_colour;
    CapType m_residue_capacity;
    NodeState m_node_state;
    Edge* m_first_out_edge;
    Edge* m_parent_edge;
    // the timestamp of the latest dist calculating 
    int m_timestamp;
    int m_terminal_dist;
    bool m_is_active;
    bool m_is_gotten_all_edges;
    int m_out_edges_label[8];
    int m_label_idx;
  };

  void CreateOutEdges(Node* cen_node);
  bool IsEdgeExist(int src_node_idx, int dst_node_idx);

  std::vector<Node> m_nodes;
  std::vector<Edge> m_edges;
  // point to the last item used in m_edges
  int m_last_edge_index;
  CapType m_flow;
  std::queue<Node*> m_active_nodes;
  int m_image_width;
  int m_image_height;
  EdgePunishFun m_epf;
};

template <class CapType, class EdgePunishFun>
FGraph<CapType, EdgePunishFun>::FGraph(int max_nodes_number, int max_edges_number,
                        int image_width, int image_height,
                        EdgePunishFun epf)
  : m_last_edge_index(0)
  , m_flow(0)
  , m_image_width(image_width)
  , m_image_height(image_height)
  , m_epf(epf) {
    m_nodes.reserve(max_nodes_number);
    m_edges.reserve(max_edges_number);
  }

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::AddNode(int node_id, CapType source_capacity,
                              CapType sink_capacity, int node_colour) {
  m_flow += source_capacity < sink_capacity ? source_capacity : sink_capacity;
  CapType node_capacity = source_capacity - sink_capacity;
  Node* node = &m_nodes[node_id];
  node->m_node_idx = node_id;
  node->m_node_colour = node_colour;
  node->m_residue_capacity = node_capacity;
  node->m_node_state = node_capacity > 0 ? SOURCE : SINK;
  node->m_first_out_edge = NULL;
  node->m_parent_edge = TERMINAL;
  node->m_timestamp = 0;
  node->m_terminal_dist = 1;
  node->m_is_active = false;
  node->m_is_gotten_all_edges = false;
  node->m_label_idx = 0;
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::AddActiveNodes(int node_x, int node_y) {
  int index = node_y * m_image_width + node_x;
  Node* cen_node = &m_nodes[index];
  int arr_index[4] = FOUR_ARR_INDEX(node_x, node_y, m_image_width, m_image_height);
  for (int i = 0; i < 4; ++i) {
    Node* arr_node = &m_nodes[arr_index[i]];
    if (cen_node->m_node_state != arr_node->m_node_state) {
      m_active_nodes.push(cen_node);
      break;
    }
  }
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::AddEdge(int src_node_id, int dst_node_id, CapType edge_capacity) {
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
  edge->m_next_out_edge = src_node->m_first_out_edge;
  src_node->m_first_out_edge = edge;

  rev_edge->m_next_out_edge = dst_node->m_first_out_edge;
  dst_node->m_first_out_edge = rev_edge;

  if (src_node->m_node_state != dst_node->m_node_state &&
      (m_active_nodes.empty() || m_active_nodes.back() != src_node)) {
    m_active_nodes.push(src_node);
  }
}

template <class CapType, class EdgePunishFun>
bool FGraph<CapType, EdgePunishFun>::IsEdgeExist(int src_node_idx, int dst_node_idx) {
  Node* src_node = &m_nodes[src_node_idx];
  for (int i = 0; i < src_node->m_label_idx; ++i) {
    if (src_node->m_out_edges_label[i] == dst_node_idx) {
      return true;
    }
  }
  return false;
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::CreateOutEdges(Node* cen_node) {
  if (cen_node->m_is_gotten_all_edges) {
    return;
  }
  int coordinate = cen_node->m_node_idx;
  int y_node = coordinate / m_image_width;
  int x_node = coordinate - y_node * m_image_width;
  int arr_nodes_idx[8] = EIGHT_ARR_INDEX(x_node, y_node, m_image_width, m_image_height);
  double punish_factor = 1;
  for (int i = 0; i < 8; ++i) {
    if (IsEdgeExist(cen_node->m_node_idx, arr_nodes_idx[i]) ||
        cen_node->m_node_idx == arr_nodes_idx[i]) {
      continue;
    }
    if (i > 3) {
      punish_factor = div_sqrt2;
    }
    Node* dst_node = &m_nodes[arr_nodes_idx[i]];
    CREATE_EDGE(cen_node, dst_node, punish_factor);
  }
  cen_node->m_is_gotten_all_edges = true;
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::MaxFlow() {
  std::queue<Node*> orphan_nodes;

#define ADD_ACTIVE_NODE(node) \
{ \
  if (!node->m_is_active) { \
    node->m_is_active = true; \
    m_active_nodes.push(node); \
  } \
}

#define GET_ACTIVE_NODE() \
({ \
  Node* return_value = NULL; \
  while (!m_active_nodes.empty()) { \
    return_value = m_active_nodes.front(); \
    m_active_nodes.pop(); \
    return_value->m_is_active = false; \
    if (return_value->m_parent_edge) { \
      break; \
    } \
  } \
  return_value; \
})

  Node* at_node = NULL;
  int global_timestamp = 0;
  Edge* meet_edge = NULL;
  while(!m_active_nodes.empty()) {
    if (meet_edge == NULL || at_node->m_parent_edge == NULL) {
      at_node = GET_ACTIVE_NODE();
      CreateOutEdges(at_node);
      if (at_node == NULL) {
        break;
      }
    }
    meet_edge = NULL;
    // grow source tree and sink tree
    for (Edge* connected_edge = at_node->m_first_out_edge; connected_edge != NULL;
         connected_edge = connected_edge->m_next_out_edge) {
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
          ADD_ACTIVE_NODE(dst_node);
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

    global_timestamp++;

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
          Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
          CapType cap = edge->m_edge_capacity;
          if (cap < min_capacity) {
            min_capacity = cap;
          }
        }
        CapType node_cap = parent_node->m_residue_capacity;
        CapType final_node_capacity = i == 0 ? node_cap : -node_cap; 
        if (final_node_capacity < min_capacity) {
          min_capacity = final_node_capacity;
        }
      }
      
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
            parent_node->m_parent_edge = ORPHAN;
            orphan_nodes.push(parent_node);
          }
        }
        parent_node->m_residue_capacity += factor * min_capacity;
        if (!parent_node->m_residue_capacity) {
          parent_node->m_parent_edge = ORPHAN;
          orphan_nodes.push(parent_node);
        }
      }
      m_flow += min_capacity;

      // adopt orphan nodes
      while (!orphan_nodes.empty()) {
        Node* orphan_node = orphan_nodes.front();
        orphan_nodes.pop();
        int dist_min = INIFINITE_DIST;
        Edge* connected_edge_min = NULL;

        CreateOutEdges(orphan_node);
        for (Edge* connected_edge = orphan_node->m_first_out_edge; connected_edge != NULL;
             connected_edge = connected_edge->m_next_out_edge) {
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
                if (dst_node->m_timestamp == global_timestamp) {
                  dist += dst_node->m_terminal_dist;
                  break;
                }
                parent_edge = dst_node->m_parent_edge;
                dist++;
                if (parent_edge == TERMINAL) {
                  dst_node->m_timestamp = global_timestamp;
                  dst_node->m_terminal_dist = 1;
                  break;
                }
                if (parent_edge == ORPHAN) {
                  dist = INIFINITE_DIST;
                  break;
                }
                Node* tmp = dst_node;
                dst_node = parent_edge->m_dst_node;
              }
              if (dist < INIFINITE_DIST) {
                if (dist < dist_min) {
                  connected_edge_min = connected_edge;
                  dist_min = dist;
                }
                for (dst_node = connected_edge->m_dst_node;
                     dst_node->m_timestamp != global_timestamp;
                     dst_node = dst_node->m_parent_edge->m_dst_node) {
                  dst_node->m_timestamp = global_timestamp;
                  dst_node->m_terminal_dist = dist--;
                }
              }
            }
          }
        }

        if (orphan_node->m_parent_edge = connected_edge_min) {
          orphan_node->m_timestamp = global_timestamp;
          orphan_node->m_terminal_dist = dist_min + 1;
        } else {
          for (Edge* connected_edge = orphan_node->m_first_out_edge; connected_edge != NULL;
               connected_edge = connected_edge->m_next_out_edge) {
            Node* dst_node = connected_edge->m_dst_node;
            Edge* parent_edge = dst_node->m_parent_edge;
            if (dst_node->m_node_state == orphan_node->m_node_state && parent_edge) {
              CapType capacity = orphan_node->m_node_state == SINK ?
                                 connected_edge->m_edge_capacity :
                                 connected_edge->m_rev_edge->m_edge_capacity;
              if (capacity) {
                ADD_ACTIVE_NODE(dst_node);
              }
              if (parent_edge != TERMINAL && parent_edge != ORPHAN &&
                  parent_edge->m_dst_node == orphan_node) {
                dst_node->m_parent_edge = ORPHAN;
                orphan_nodes.push(dst_node);
              }
            }
          }
        }
      }
    }
  }
}

template <class CapType, class EdgePunishFun>
bool FGraph<CapType, EdgePunishFun>::IsBelongToSource(int node_id) {
  return m_nodes[node_id].m_node_state == SOURCE;
}

#undef TERMINAL
#undef ORPHAN
#undef ADD_ACTIVE_NODE
#undef GET_ACTIVE_NODE
#endif  // INCLUDE_FGRAPH_H_
