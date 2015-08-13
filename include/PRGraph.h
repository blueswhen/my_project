// Copyright 2014-12 sxniu
#ifndef INCLUDE_PRGRAPH_H_
#define INCLUDE_PRGRAPH_H_

#include <vector>
#include <queue>
#include <stack>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#include "include/CountTime.h"
#include "include/ImageData.h"

#define MAX_NUM INT_MAX
// #define SET_STATE
// #define TIME_CONT

template <class CapType>
class PRGraph {
 public:
  enum NodeState {
    SOURCE,
    SINK,
    UNKNOWN
  };
  PRGraph(int max_nodes_number, int max_edges_number, int width, int height, ImageData<int>* marked_image);
  void AddNode(int node_id, CapType source_capacity, CapType sink_capacity);
  void AddEdge(int src_node_id, int dst_node_id, CapType edge_capacity);
  void Initialization();
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
      , m_terminal_dist(MAX_NUM)
      , m_bfs_level(0)
      , m_is_el(false)
      , m_is_dl(false)
      , m_is_rl(false)
      , m_out_edge_begin(NULL)
      , m_id(0) {}
    CapType m_excess;
    NodeState m_node_state;
    Edge* m_first_out_edge;
    int m_terminal_dist;
    int m_bfs_level;
    bool m_is_el;
    bool m_is_dl;
    bool m_is_rl;
    Edge* m_out_edge_begin;
    int m_id;
  };

  enum List {
    EL,
    DL,
    RL
  };

  Node* GetNodeFromList(std::queue<Node*>* list, List ls, int dist) {
    Node* node = NULL;
    while (!list->empty()) {
      node = list->front();
      list->pop();
      bool* is_in_list = NULL;
      if (ls == EL) {
        is_in_list = &(node->m_is_el);
      } else if (ls == DL) {
        is_in_list = &(node->m_is_dl);
      } else {
        is_in_list = &(node->m_is_rl);
      }
      if (*is_in_list && node->m_terminal_dist == dist) {
        *is_in_list = false;
        break;
      }
      node = NULL;
    }
    return node;
  }
  void AddNodeInList(Node* dst_node, std::queue<Node*>* list, List ls) {
    bool* is_in_list = NULL;
    int* max_size = NULL;
    if (ls == EL) {
      is_in_list = &(dst_node->m_is_el);
      max_size = &m_max_el;
    } else if (ls == DL) {
      is_in_list = &(dst_node->m_is_dl);
      max_size = &m_max_dl;
    } else {
      is_in_list = &(dst_node->m_is_rl);
      max_size = &m_max_rl;
    }
    if (!*is_in_list) {
      list->push(dst_node);
      *is_in_list = true;
      if (dst_node->m_terminal_dist > *max_size) {
        *max_size = dst_node->m_terminal_dist;
      }
    }
  }

  void PushFlow(Node* src_node, Node* dst_node, Edge* out_edge);

  std::vector<Node> m_nodes;
  std::vector<Edge> m_edges;
  // point to the last item used in m_edges
  int m_last_edge_index;
  std::vector<std::queue<Node*> > m_el;
  int m_max_el;
  std::vector<std::queue<Node*> > m_dl;
  int m_max_dl;
  std::vector<std::queue<Node*> > m_rl;
  int m_max_rl;
  int m_image_width;
  int m_image_height;
  ImageData<int>* m_marked_image;
  CapType m_flow;
};

template <class CapType>
PRGraph<CapType>::PRGraph(int max_nodes_number, int max_edges_number, int width, int height,
                          ImageData<int>* marked_image)
  : m_nodes(std::vector<Node>(max_nodes_number))
  , m_edges(std::vector<Edge>(max_edges_number))
  , m_last_edge_index(0)
  , m_el(std::vector<std::queue<Node*> >(std::max(width, height)))
  , m_max_el(0)
  , m_dl(std::vector<std::queue<Node*> >(std::max(width, height)))
  , m_max_dl(0)
  , m_rl(std::vector<std::queue<Node*> >(std::max(width, height)))
  , m_max_rl(0)
  , m_image_width(width)
  , m_image_height(height)
  , m_marked_image(marked_image)
  , m_flow(0) {}

template <class CapType>
void PRGraph<CapType>::AddNode(int node_id, CapType source_capacity, CapType sink_capacity) {
  assert(node_id >= 0 && node_id < m_nodes.size() &&
         m_nodes[node_id].m_excess == 0);
  m_flow += source_capacity < sink_capacity ? source_capacity : sink_capacity;
  CapType node_capacity = source_capacity - sink_capacity;
  m_nodes[node_id].m_node_state = node_capacity > 0 ? SOURCE : SINK;
  m_nodes[node_id].m_excess = node_capacity;
  m_nodes[node_id].m_id = node_id;
  // SET_PIXEL(m_marked_image, node_id, node_capacity > 0 ? WHITE : BLACK);
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
  src_node->m_out_edge_begin = edge;
  rev_edge->m_next_out_edge = dst_node->m_first_out_edge;
  dst_node->m_first_out_edge = rev_edge;
  dst_node->m_out_edge_begin = rev_edge;
}

template <class CapType>
void PRGraph<CapType>::Initialization() {
  std::vector<Node*> tmp_vec;
  tmp_vec.reserve(m_nodes.size());
  int tmp_size = 0;
  for (int i = 0; i < m_nodes.size(); ++i) {
    Node* node = &m_nodes[i];
    if (node->m_node_state == SINK) {
      node->m_terminal_dist = 0;
      node->m_bfs_level = MAX_NUM;
      tmp_vec[tmp_size++] = node;
    }
  }
  std::vector<Node*> backup_vec;
  backup_vec.reserve(m_nodes.size());
  int backup_size = 0;
  while (tmp_size) {
    for (int i = 0; i < tmp_size; ++i) {
      Node* node = tmp_vec[i];
      for (Edge* out_edge = node->m_first_out_edge; out_edge != NULL;
           out_edge = out_edge->m_next_out_edge) {
        Node* dst_node = out_edge->m_dst_node;
        if (dst_node->m_terminal_dist == MAX_NUM) {
          assert(dst_node->m_node_state == SOURCE);
          dst_node->m_terminal_dist = node->m_terminal_dist + 1;
          AddNodeInList(dst_node, &m_el[dst_node->m_terminal_dist], EL);
          backup_vec[backup_size++] = dst_node;
        }
      }
    }
    tmp_vec.swap(backup_vec);
    tmp_size = backup_size;
    backup_size = 0;
  }
}

// algorithm3
template <class CapType>
void PRGraph<CapType>::PushFlow(Node* src_node, Node* dst_node, Edge* out_edge) {
  CapType flow = std::min(src_node->m_excess, out_edge->m_edge_capacity);
  src_node->m_excess -= flow;
  if (dst_node->m_excess < 0) {
    m_flow += std::min(flow, -dst_node->m_excess);
  }
  dst_node->m_excess += flow;
  out_edge->m_edge_capacity -= flow;
  out_edge->m_rev_edge->m_edge_capacity += flow;
#ifdef SET_STATE
  if (dst_node->m_node_state == SINK && dst_node->m_excess >= 0) {
    dst_node->m_node_state = SOURCE;
  }
#endif
}

template <class CapType>
void PRGraph<CapType>::MaxFlow() {
  CountTime ct;
  double t = 0;
  while (!m_el[m_max_el].empty()) {
#ifdef TIME_CONT
    ct.ContBegin();
#endif
    // algorithm2
    for (int i = m_max_el; i > 0; --i) {
      std::queue<Node*>& el = m_el[i];
      while (true) {
        Node* node = GetNodeFromList(&el, EL, i);
        if (!node) {
          break;
        }
        assert(node->m_excess >= 0);
        if (!node->m_excess) {
          continue;
        }
        for (Edge* out_edge = node->m_first_out_edge; out_edge != NULL;
             out_edge = out_edge->m_next_out_edge) {
          Node* dst_node = out_edge->m_dst_node;
          if (node->m_terminal_dist > dst_node->m_terminal_dist && out_edge->m_edge_capacity) {
            assert(node->m_terminal_dist == dst_node->m_terminal_dist + 1);
            assert(node->m_excess > 0);
            PushFlow(node, dst_node, out_edge);
            if (dst_node->m_excess > 0) {
              AddNodeInList(dst_node, &m_el[dst_node->m_terminal_dist], EL);
            }
            if (!node->m_excess) {
              break;
            }
          }
        }
        if (node->m_excess) {
          AddNodeInList(node, &m_dl[node->m_terminal_dist], DL);
        }
      }
    }
    m_max_el = 0;
#ifdef TIME_CONT
    ct.ContEnd();
#endif
#ifdef TIME_CONT
    ct.PrintTime();
#endif

    // algorithm4
#ifdef TIME_CONT
    ct.ContBegin();
#endif
    std::queue<Node*>& el0 = m_el[0];
    std::queue<Node*> bfs_list;
    // bfs_list.swap(el0);
    while (true) {
      Node* node = GetNodeFromList(&el0, EL, 0);
      if (!node) {
        break;
      }
      bfs_list.push(node);
      node->m_bfs_level = 0;
    }
    std::queue<Node*> backup_list;
    while (true) {
      assert(backup_list.empty());
      while (!bfs_list.empty()) {
        Node* node = bfs_list.front();
        bfs_list.pop();
        if (!node->m_excess) {
          continue;
        }
        assert(node->m_terminal_dist == 0);
        for (Edge* out_edge = node->m_first_out_edge; out_edge != NULL;
             out_edge = out_edge->m_next_out_edge) {
          Node* dst_node = out_edge->m_dst_node;
          if (out_edge->m_edge_capacity && dst_node->m_bfs_level > node->m_bfs_level &&
              dst_node->m_terminal_dist == 0) {
            assert(node->m_excess > 0);
            PushFlow(node, dst_node, out_edge);
            dst_node->m_bfs_level = node->m_bfs_level + 1;
            if (dst_node->m_excess > 0) {
              backup_list.push(dst_node);
            }
            if (!node->m_excess) {
              break;
            }
          }
        }
        if (node->m_excess > 0) {
          assert(node->m_terminal_dist == 0);
          AddNodeInList(node, &m_dl[node->m_terminal_dist], DL);
        }
      }
      if (backup_list.empty()) {
        break;
      }
      bfs_list.swap(backup_list);
    }
#ifdef TIME_CONT
    ct.ContEnd();
#endif
#ifdef TIME_CONT
    ct.PrintTime();
#endif

#if 1
    // algorithm5
#ifdef TIME_CONT
    ct.ContBegin();
#endif
    for (int i = 0; i <= m_max_dl; ++i) {
      std::queue<Node*>& dl = m_dl[i];
      while (true) {
        Node* node = GetNodeFromList(&dl, DL, i);
        if (!node) {
          break;
        }
        for (Edge* out_edge = node->m_first_out_edge; out_edge != NULL;
             out_edge = out_edge->m_next_out_edge) {
          Node* dst_node = out_edge->m_dst_node;
          if (dst_node->m_terminal_dist == MAX_NUM || dst_node->m_is_dl) {
            continue;
          }
          bool is_add_dl = true;
          if (dst_node->m_terminal_dist == 0) {
            is_add_dl = false;
          } else {
    // ct.ContBegin();
            for (Edge* out_edge = dst_node->m_out_edge_begin; out_edge != NULL;
                 out_edge = out_edge->m_next_out_edge) {
              Node* dd_node = out_edge->m_dst_node;
              if (dd_node != node && dst_node->m_terminal_dist > dd_node->m_terminal_dist &&
                  out_edge->m_edge_capacity) {
                assert(dst_node->m_terminal_dist == dd_node->m_terminal_dist + 1);
                is_add_dl = false;
                dst_node->m_out_edge_begin = out_edge;
                break;
              }
            }
    // ct.ContEnd();
    // t += ct.ContResult();
          }
          if (is_add_dl) {
            AddNodeInList(dst_node, &m_dl[dst_node->m_terminal_dist], DL);
            dst_node->m_is_rl = false;
          }
          if (!dst_node->m_is_dl) {
            assert(dst_node->m_terminal_dist != MAX_NUM);
            AddNodeInList(dst_node, &m_rl[dst_node->m_terminal_dist], RL);
            // SET_PIXEL(m_marked_image, dst_node->m_id, GREEN);
          }
        }
        assert(!node->m_is_rl);
        node->m_terminal_dist = MAX_NUM;
        // SET_PIXEL(m_marked_image, node->m_id, RED);
      }
    }
    m_max_dl = 0;
#ifdef TIME_CONT
    ct.ContEnd();
#endif
#ifdef TIME_CONT
    ct.PrintTime();
#endif

    // algorithm6
#ifdef TIME_CONT
    ct.ContBegin();
#endif
    for (int i = 0; i <= m_max_rl; ++i) {
      std::queue<Node*>& rl = m_rl[i];
      while (!rl.empty()) {
        Node* node = GetNodeFromList(&rl, RL, i);
        if (!node) {
          break;
        }
        node->m_out_edge_begin = node->m_first_out_edge;
        assert(node->m_terminal_dist != MAX_NUM);
        for (Edge* out_edge = node->m_first_out_edge; out_edge != NULL;
             out_edge = out_edge->m_next_out_edge) {
          Node* dst_node = out_edge->m_dst_node;
          if (dst_node->m_terminal_dist == MAX_NUM && out_edge->m_rev_edge->m_edge_capacity) {
            dst_node->m_terminal_dist = node->m_terminal_dist + 1;
            AddNodeInList(dst_node, &m_rl[dst_node->m_terminal_dist], RL);
            if (dst_node->m_excess > 0) {
              AddNodeInList(dst_node, &m_el[dst_node->m_terminal_dist], EL);
            }
          }
        }
      }
    }
    m_max_rl = 0;
#ifdef TIME_CONT
    ct.ContEnd();
#endif
#ifdef TIME_CONT
    ct.PrintTime();
#endif
#endif
    // break;
  }
  printf("max flow = %f\n", m_flow);
  printf("t = %fms\n", t * 1000);
#ifdef SET_STATE
  std::queue<Node*> bfs;
  for (int i = 0; i < m_nodes.size(); ++i) {
    Node* node = &m_nodes[i];
    if (node->m_excess < 0) {
      bfs.push(node);
    }
  }
  while (!bfs.empty()) {
    Node* node = bfs.front();
    bfs.pop();
    for (Edge* out_edge = node->m_first_out_edge; out_edge != NULL;
         out_edge = out_edge->m_next_out_edge) {
      Node* dst_node = out_edge->m_dst_node;
      if (dst_node->m_node_state == SOURCE && out_edge->m_rev_edge->m_edge_capacity) {
        dst_node->m_node_state = SINK;
        bfs.push(dst_node);
      }
    }
  }
#endif
}

template <class CapType>
bool PRGraph<CapType>::IsBelongToSource(int node_id) {
  return m_nodes[node_id].m_node_state == SOURCE;
}

#endif  // INCLUDE_PRGRAPH_H_
