// Copyright 2014-12 sxniu
#ifndef INCLUDE_GRAPH_H_
#define INCLUDE_GRAPH_H_

#include <vector>
#include <queue>
#include <stack>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>
#include "include/CountTime.h"

#define TERMINAL reinterpret_cast<Edge*>(1)
#define ORPHAN reinterpret_cast<Edge*>(2)
#define INIFINITE_DIST INT_MAX
#define NEIGHBOUR 8

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
      , m_rev_edge(NULL) {}
    Node* m_dst_node;
    CapType m_edge_capacity;
    Edge* m_rev_edge;
    // next edge originated from the same node
  };

  class Node {
   public:
    Node() 
      : m_residue_capacity(0)
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
      , m_id(0) {}
    CapType m_residue_capacity;
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
        // if (m_first_at_source_node->m_next_active_source->m_node_state == SOURCE &&
        //     m_first_at_source_node->m_next_active_source->m_parent_edge) {
        //   assert(m_first_at_source_node->m_next_active_source->m_terminal_dist ==
        //          m_global_source_dist);
        // }
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
        // if (m_first_at_sink_node->m_next_active_sink->m_node_state == SINK &&
        //     m_first_at_sink_node->m_next_active_sink->m_parent_edge) {
        //   assert(m_first_at_sink_node->m_next_active_sink->m_terminal_dist ==
        //          m_global_sink_dist);
        // }
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
  CapType m_flow;
};

template <class CapType>
Graph<CapType>::Graph(int max_nodes_number, int max_edges_number)
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
  , m_flow(0) {}

template <class CapType>
void Graph<CapType>::AddNode(int node_id, CapType source_capacity, CapType sink_capacity) {
  assert(node_id >= 0 && node_id < m_nodes.size() && m_nodes[node_id].m_residue_capacity == 0);
  m_flow += source_capacity < sink_capacity ? source_capacity : sink_capacity;
  CapType node_capacity = source_capacity - sink_capacity;
  m_nodes[node_id].m_residue_capacity = node_capacity;
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
    CapType final_node_capacity = parent_node->m_residue_capacity > 0 ?
                                  parent_node->m_residue_capacity :
                                  -parent_node->m_residue_capacity;
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
    parent_node->m_residue_capacity += factor * min_capacity;
    if (!parent_node->m_residue_capacity) {
      AddOrphanNode(parent_node);
    }
  }
  m_flow += min_capacity;
}

template <class CapType>
void Graph<CapType>::FindNewPath(Node* orphan_node) {
  int dist_min = INIFINITE_DIST;
  Edge* connected_edge_min = NULL;

  for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
    Edge* connected_edge = &orphan_node->m_out_edges[i];
    if (connected_edge->m_dst_node->m_node_state == orphan_node->m_node_state) {
      CapType capacity = orphan_node->m_node_state == SINK ?
                         connected_edge->m_edge_capacity :
                         connected_edge->m_rev_edge->m_edge_capacity;
#if 1
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
#else
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
            // node_cap = dst_node->m_residue_capacity > 0 ?
            //   dst_node->m_residue_capacity : -dst_node->m_residue_capacity;
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
#endif
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
    if (node->m_residue_capacity != 0) {
      node->m_node_state = node->m_residue_capacity < 0 ? SINK : SOURCE;
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
              if (orphan_node->m_node_state == SOURCE) {
                if (orphan_node->m_terminal_dist == m_global_source_dist) {
                  AddActiveSourceNodeBack(orphan_node);
                }
              } else {
                if (orphan_node->m_terminal_dist == m_global_sink_dist) {
                  AddActiveSinkNodeBack(orphan_node);
                }
              }
            }
          }
        }
      }
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
#endif  // INCLUDE_GRAPH_H_
