// Copyright 2014-12 sxniu
#ifndef INCLUDE_PRGRAPH_H_
#define INCLUDE_PRGRAPH_H_

#include <vector>
#include <queue>
#include <stack>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#include "include/ImageData.h"

#define TERMINAL reinterpret_cast<Edge*>(1)
#define ORPHAN reinterpret_cast<Edge*>(2)
#define INIFINITE_DIST INT_MAX

template <class CapType>
class PRGraph {
 public:
  enum NodeState {
    SOURCE,
    SINK,
    UNKNOWN
  };
  PRGraph(int max_nodes_number, int max_edges_number);
  void AddNode(int node_id, CapType source_capacity, CapType sink_capacity);
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
      : m_excess(0)
      , m_node_state(UNKNOWN)
      , m_first_out_edge(NULL)
      , m_parent_edge(NULL)
      , m_timestamp(0)
      , m_terminal_dist(0)
      , m_id(0) {}
    CapType m_excess;
    NodeState m_node_state;
    Edge* m_first_out_edge;
    Edge* m_parent_edge;
    // the timestamp of the latest dist calculating 
    int m_timestamp;
    int m_terminal_dist;
    int m_id;
  };

  std::vector<Node> m_nodes;
  std::vector<Edge> m_edges;
  // point to the last item used in m_edges
  int m_last_edge_index;
  CapType m_flow;
};

template <class CapType>
PRGraph<CapType>::PRGraph(int max_nodes_number, int max_edges_number)
  : m_nodes(std::vector<Node>(max_nodes_number))
  , m_edges(std::vector<Edge>(max_edges_number))
  , m_last_edge_index(0)
  , m_flow(0) {}

template <class CapType>
void PRGraph<CapType>::AddNode(int node_id, CapType source_capacity, CapType sink_capacity) {
  assert(node_id >= 0 && node_id < m_nodes.size() &&
         m_nodes[node_id].m_excess == 0);
  m_flow += source_capacity < sink_capacity ? source_capacity : sink_capacity;
  CapType node_capacity = source_capacity - sink_capacity;
  m_nodes[node_id].m_excess = node_capacity;
  m_nodes[node_id].m_id = node_id;
}

template <class CapType>
void PRGraph<CapType>::AddEdge(int src_node_id, int dst_node_id, CapType edge_capacity) {
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
  edge->m_next_out_edge = src_node->m_first_out_edge;
  src_node->m_first_out_edge = edge;
  rev_edge->m_next_out_edge = dst_node->m_first_out_edge;
  dst_node->m_first_out_edge = rev_edge;
}

template <class CapType>
void PRGraph<CapType>::MaxFlow() {
  std::queue<Node*> active_nodes;
  std::queue<Node*> orphan_nodes;

#define GET_ACTIVE_NODE() \
({ \
  Node* return_value = NULL; \
  while (!active_nodes.empty()) { \
    return_value = active_nodes.front(); \
    active_nodes.pop(); \
    if (return_value->m_parent_edge) { \
      break; \
    } \
  } \
  return_value; \
})

  // get active nodes
  for (int i = 0; i < m_nodes.size(); ++i) {
    Node* node = &m_nodes[i];
    if (node->m_excess != 0) {
      node->m_node_state = node->m_excess < 0 ? SINK : SOURCE;
      node->m_parent_edge = TERMINAL;
      node->m_timestamp = 0;
      node->m_terminal_dist = 1;
      active_nodes.push(node);
    }
  }

  Node* at_node = NULL;
  int global_timestamp = 0;
  Edge* meet_edge = NULL;
  int path = 0;
  while(!active_nodes.empty()) {
    if (meet_edge == NULL || at_node->m_parent_edge == NULL) {
      at_node = GET_ACTIVE_NODE();
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
          active_nodes.push(dst_node);
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

    global_timestamp++;

    if (meet_edge) {
      // augment path
      CapType min_capacity = meet_edge->m_edge_capacity;
      // first_edge[0] for source tree and first_edge[1] for sink tree
      Edge* first_edge[2] = {meet_edge->m_rev_edge, meet_edge};
      // find min capacity from path
      for (int i = 0; i < 2; ++i) {
        Node* parent_node = first_edge[i]->m_dst_node;
        int k = 0;
        for (Edge* parent_edge = parent_node->m_parent_edge; parent_edge != TERMINAL;
             parent_node = parent_edge->m_dst_node, parent_edge = parent_node->m_parent_edge) {
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
            parent_node->m_parent_edge = ORPHAN;
            orphan_nodes.push(parent_node);
          }
        }
        parent_node->m_excess += factor * min_capacity;
        if (!parent_node->m_excess) {
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

        for (Edge* connected_edge = orphan_node->m_first_out_edge; connected_edge != NULL;
             connected_edge = connected_edge->m_next_out_edge) {
          CapType capacity = orphan_node->m_node_state == SINK ?
                             connected_edge->m_edge_capacity :
                             connected_edge->m_rev_edge->m_edge_capacity;
          if (capacity) {
            Node* dst_node = connected_edge->m_dst_node;
            Edge* parent_edge = dst_node->m_parent_edge;
            if (dst_node->m_node_state == orphan_node->m_node_state && parent_edge) {
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
                active_nodes.push(dst_node);
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
  printf("path = %d, flow = %f\n", path, m_flow);
}

template <class CapType>
bool PRGraph<CapType>::IsBelongToSource(int node_id) {
  return m_nodes[node_id].m_node_state == SOURCE;
}

#undef TERMINAL
#undef ORPHAN
#endif  // INCLUDE_PRGRAPH_H_
