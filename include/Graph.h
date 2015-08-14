// Copyright 2014-12 sxniu
#ifndef INCLUDE_GRAPH_H_
#define INCLUDE_GRAPH_H_

#include <vector>
#include <queue>
#include <stack>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#define TERMINAL reinterpret_cast<Edge*>(1)
#define ORPHAN reinterpret_cast<Edge*>(2)
#define INIFINITE_DIST INT_MAX

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
      : m_residue_capacity(0)
      , m_node_state(UNKNOWN)
      , m_first_child_edge(NULL)
      , m_parent_edge(NULL)
      , m_timestamp(0)
      , m_terminal_dist(0)
#ifdef ENABLE_BFS
      , m_child_edge(NULL)
#endif
      , m_next_active(NULL)
      , m_is_active(false)
      , m_id(0) {}
    CapType m_residue_capacity;
    NodeState m_node_state;
    Edge* m_first_child_edge;
    Edge* m_parent_edge;
#ifdef ENABLE_BFS
    Edge* m_child_edge;
#endif
    Node* m_next_active;
    bool m_is_active;
    // the timestamp of the latest dist calculating 
    int m_timestamp;
    int m_terminal_dist;
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
  std::vector<Edge> m_edges;
  Node* m_first_at_node;
  Node* m_mid_at_node;
  Node* m_last_at_node;
  // point to the last item used in m_edges
  int m_last_edge_index;
  int m_global_timestamp;
  std::queue<Node*> m_orphan_nodes;
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
  , m_flow(0) {}

template <class CapType>
void Graph<CapType>::AddNode(int node_id, CapType source_capacity, CapType sink_capacity) {
  assert(node_id >= 0 && node_id < m_nodes.size() &&
         m_nodes[node_id].m_residue_capacity == 0);
  m_flow += source_capacity < sink_capacity ? source_capacity : sink_capacity;
  CapType node_capacity = source_capacity - sink_capacity;
  m_nodes[node_id].m_residue_capacity = node_capacity;
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
  edge->m_next_edge = src_node->m_first_child_edge;
  src_node->m_first_child_edge = edge;
  rev_edge->m_next_edge = dst_node->m_first_child_edge;
  dst_node->m_first_child_edge = rev_edge;
}

template <class CapType>
void Graph<CapType>::Augment(Edge* meet_edge) {
  // augment path
  CapType min_capacity = meet_edge->m_edge_capacity;
  // first_edge[0] for source tree and first_edge[1] for sink tree
  Edge* first_edge[2] = {meet_edge->m_rev_edge, meet_edge};
  // find min capacity from path
  for (int i = 0; i < 2; ++i) {
    Node* parent_node = first_edge[i]->m_dst_node;
    int k = 0;
#ifdef ENABLE_BFS
    parent_node->m_child_edge = NULL;
#endif
    for (Edge* parent_edge = parent_node->m_parent_edge; parent_edge != TERMINAL;
         parent_node = parent_edge->m_dst_node, parent_edge = parent_node->m_parent_edge) {
#ifdef ENABLE_BFS
      parent_edge->m_dst_node->m_child_edge = parent_edge->m_rev_edge;
#endif
      Edge* edge = i == 0 ? parent_edge->m_rev_edge : parent_edge;
      CapType cap = edge->m_edge_capacity;
      if (cap < min_capacity) {
        min_capacity = cap;
      }
    }
    CapType final_node_capacity = parent_node->m_residue_capacity > 0 ?
                                  parent_node->m_residue_capacity :
                                  -parent_node->m_residue_capacity;
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
        AddOrphanNode(parent_node);
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
  CapType max_node_cap = 0;
  Edge* connected_edge_min = NULL;

  for (Edge* connected_edge = orphan_node->m_first_child_edge; connected_edge != NULL;
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
    if (node->m_residue_capacity != 0) {
      node->m_node_state = node->m_residue_capacity < 0 ? SINK : SOURCE;
      node->m_parent_edge = TERMINAL;
      node->m_timestamp = 0;
      node->m_terminal_dist = 1;
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
    // grow source tree and sink tree
    for (Edge* connected_edge = at_node->m_first_child_edge; connected_edge != NULL;
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

    m_global_timestamp++;

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
          for (Edge* connected_edge = orphan_node->m_first_child_edge; connected_edge != NULL;
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
  printf("path = %d, flow = %f\n", path, m_flow);
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
