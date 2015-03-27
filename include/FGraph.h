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

#define INIFINITE_DIST INT_MAX

#define HALF_NEIGHBOUR 4
#define NEIGHBOUR 8
#define HALF_NEIGHBOUR_ARR_INDEX FOUR_ARR_INDEX
#define NEIGHBOUR_ARR_INDEX EIGHT_ARR_INDEX

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
 public:
  FGraph(int max_nodes_number, int image_width,
         int image_height, EdgePunishFun epf, ImageData<int>* marked_image);
  void AddNode(int node_id, CapType source_capacity, CapType sink_capacity, int node_colour);
  void AddActiveNodes(int node_x, int node_y);
  void MaxFlow();
  bool IsBelongToSource(int node_id);

 private:
  struct Node;
  struct Edge {
    Node* m_dst_node;
    CapType m_edge_capacity;
    Edge* m_rev_edge;
  };

  struct Node {
    Node()
      : m_node_idx(-1) {}
    int m_node_idx;
    int m_node_colour;
    CapType m_residue_capacity;

    #define SOURCE 0
    #define SINK 1
    bool m_node_property;

    // add --> |=
    // remove --> &= ~
    #define ACTIVE 0x02
    int m_node_state;

    Edge* m_parent_edge;
    // the timestamp of the latest dist calculating 
    int m_timestamp;
    int m_terminal_dist;
    std::list<Node*> m_s_explore_Nodes;
    std::list<Node*> m_t_explore_Nodes;
    bool m_is_gotten_all_edges;
    Edge m_out_edges[NEIGHBOUR];
    int m_out_edges_num;
    Node* m_root_node;
    int m_node_timestamp;
    int m_dist;
  };

  void AddOrphanNode(Node* orphan_node, bool is_add_queue = true);
  void ChangeRootForTree(Node* old_root_node);
  void FindNewTrees(Node* leaf_node, Node** diff_trees_roots, int* diff_trees_roots_num);
  void AddExploreNodes(Node* explore_node, Edge* parent_edge = NULL);
  void FindNewExploreNodes(Node* center_node);
  void FindArroundAtNodes(std::stack<Node*>* connected_at_nodes);
  bool IsNeedAugment();
  bool Augment();
  bool IsExpandArea();
  void MoveExploreNodes(Node* old_root_node);
  void GrowOrphanTree(Node* first_node, Node* root_node);
  bool FindOtherPath();
  void ExpandStateArea();
  void CreateOutEdges(Node* cen_node);
  Edge* GetEdge(Node* src_node, Node* dst_node);
  Edge* CreateEdge(Node* src_node, Node* dst_node, double punish_factor);

  std::vector<Node> m_nodes;
  CapType m_flow;
  std::queue<Node*> m_active_nodes;
  std::queue<Node*> m_orphan_nodes;
  Node* m_curr_active_node;
  int m_image_width;
  int m_image_height;
  EdgePunishFun m_epf;
  int m_path;
  int m_tree_edges;
  int m_broken_edges;
  int m_orphan_timestamp;
  ImageData<int>* m_marked_image;
};

template <class CapType, class EdgePunishFun>
FGraph<CapType, EdgePunishFun>::FGraph(int max_nodes_number, int image_width, int image_height,
                                       EdgePunishFun epf, ImageData<int>* marked_image)
  : m_nodes(std::vector<Node>(max_nodes_number))
  , m_flow(0)
  , m_curr_active_node(NULL)
  , m_image_width(image_width)
  , m_image_height(image_height)
  , m_epf(epf)
  , m_path(0)
  , m_tree_edges(0)
  , m_broken_edges(0)
  , m_orphan_timestamp(0)
  , m_marked_image(marked_image) {
    // m_nodes.reserve(max_nodes_number);
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
  node->m_node_property = node_capacity > 0 ? SOURCE : SINK;
  node->m_node_state = 0;
  node->m_parent_edge = NULL;
  node->m_timestamp = 0;
  node->m_terminal_dist = 1;
  node->m_is_gotten_all_edges = false;
  node->m_out_edges_num = 0;
  node->m_root_node = NULL;
  node->m_node_timestamp = 0;
  node->m_dist = 0;
  SET_PIXEL(m_marked_image, node_id, node_capacity > 0 ? WHITE : BLACK);
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::AddActiveNodes(int node_x, int node_y) {
  int index = node_y * m_image_width + node_x;
  Node* cen_node = &m_nodes[index];
  int arr_index[HALF_NEIGHBOUR] =
    HALF_NEIGHBOUR_ARR_INDEX(node_x, node_y, m_image_width, m_image_height);
  for (int i = 0; i < HALF_NEIGHBOUR; ++i) {
    Node* arr_node = &m_nodes[arr_index[i]];
    if (arr_node->m_node_idx == -1) {
      continue;
    }
    if (cen_node->m_node_property != arr_node->m_node_property) {
      // only souce node can be add to active nodes
      cen_node->m_node_state |= ACTIVE;
      m_active_nodes.push(cen_node);
      break;
    }
  }
}

template <class CapType, class EdgePunishFun>
typename FGraph<CapType, EdgePunishFun>::Edge* FGraph<CapType, EdgePunishFun>::GetEdge(
  Node* src_node, Node* dst_node) {
  for (int i = 0; i < src_node->m_out_edges_num; ++i) {
    if (src_node->m_out_edges[i].m_dst_node == dst_node) {
      return &src_node->m_out_edges[i];
    }
  }
  return NULL;
}

template <class CapType, class EdgePunishFun>
typename FGraph<CapType, EdgePunishFun>::Edge* FGraph<CapType, EdgePunishFun>::CreateEdge(
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
void FGraph<CapType, EdgePunishFun>::CreateOutEdges(Node* cen_node) {
  assert(cen_node != NULL);
  if (cen_node->m_is_gotten_all_edges) {
    return;
  }
  int coordinate = cen_node->m_node_idx;
  int y_node = coordinate / m_image_width;
  int x_node = coordinate - y_node * m_image_width;
  int arr_nodes_idx[NEIGHBOUR] = 
    NEIGHBOUR_ARR_INDEX(x_node, y_node, m_image_width, m_image_height);
  double punish_factor = 1;
  for (int i = 0; i < NEIGHBOUR; ++i) {
    if (cen_node->m_node_idx == arr_nodes_idx[i]) {
      continue;
    }
    Node* arr_node = &m_nodes[arr_nodes_idx[i]];
    if (!GetEdge(cen_node, arr_node)) {
      if (i > 3) {
        punish_factor = div_sqrt2;
      }
      CreateEdge(cen_node, arr_node, punish_factor);
    }
  }
  cen_node->m_is_gotten_all_edges = true;
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::FindNewTrees(
  Node* leaf_node, Node** diff_trees_roots, int* diff_trees_roots_num) {
  Node* trace_node = leaf_node;
  for (; trace_node->m_parent_edge; trace_node = trace_node->m_parent_edge->m_dst_node) {
  }
  if (trace_node != m_curr_active_node) {
    assert(!trace_node->m_parent_edge);
    bool is_find_new_tree = true;
    for (int i = 0; i < *diff_trees_roots_num; ++i) {
      if (trace_node == diff_trees_roots[i]) {
        is_find_new_tree = false;
        break;
      }
    }
    if (is_find_new_tree) {
      diff_trees_roots[(*diff_trees_roots_num)++] = trace_node;
    }
  }
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::AddExploreNodes(Node* explore_node, Edge* parent_edge) {
  Node* root_node = NULL;
  if (parent_edge) {
    assert(!explore_node->m_parent_edge);
    explore_node->m_parent_edge = parent_edge;
    root_node = parent_edge->m_dst_node->m_root_node;
  } else {
    root_node = explore_node->m_parent_edge ? explore_node->m_root_node : explore_node;
  }
  if (explore_node != root_node) {
    auto& old_explore_nodes = explore_node->m_node_property == SOURCE ?
      explore_node->m_s_explore_Nodes : explore_node->m_t_explore_Nodes;
    if (!old_explore_nodes.empty()) {
      MoveExploreNodes(explore_node);
    }
  }
  explore_node->m_root_node = root_node;
  auto& explore_nodes = explore_node->m_node_property == SOURCE ?
    root_node->m_s_explore_Nodes : root_node->m_t_explore_Nodes;
  explore_nodes.push_back(explore_node);
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::FindNewExploreNodes(Node* center_node) {
  CreateOutEdges(center_node);
  Node* diff_trees_roots[NEIGHBOUR];
  int diff_trees_roots_num = 0;
  Node* cen_root_node = center_node->m_root_node;

  for (int i = 0; i < center_node->m_out_edges_num; ++i) {
    Edge* out_edge = &center_node->m_out_edges[i];
    Node* dst_node = out_edge->m_dst_node;
    if (dst_node->m_parent_edge && dst_node->m_root_node &&
        cen_root_node == dst_node->m_root_node ||
        dst_node == cen_root_node) {
      continue;
    }
    if (dst_node->m_node_property == center_node->m_node_property ||
        center_node == cen_root_node) {
      CapType cap = dst_node->m_node_property == SOURCE ?
                    out_edge->m_rev_edge->m_edge_capacity :
                    out_edge->m_edge_capacity;
      if (cap) {
        if (!dst_node->m_parent_edge) {
          AddExploreNodes(dst_node, out_edge->m_rev_edge);
          SET_PIXEL(m_marked_image, dst_node->m_node_idx,
                    dst_node->m_node_property == SOURCE ? RED : BLUE);
          m_tree_edges++;
        } else {
          FindNewTrees(dst_node, diff_trees_roots, &diff_trees_roots_num);
        }
      }
    }
  }
  for (int i = 0; i < diff_trees_roots_num; ++i) {
    Node* root_node = diff_trees_roots[i];
    if (root_node->m_parent_edge) {
      continue;
    }
    // merge new trees
    assert(root_node != cen_root_node);
    ChangeRootForTree(root_node);
  }
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::AddOrphanNode(
  Node* orphan_node, bool is_add_queue) {
  orphan_node->m_parent_edge = NULL;
  if (is_add_queue) {
    m_orphan_nodes.push(orphan_node);
  }
}

template <class CapType, class EdgePunishFun>
bool FGraph<CapType, EdgePunishFun>::IsNeedAugment() {
  for (int i = 0; i < m_curr_active_node->m_out_edges_num; ++i) {
    Edge* out_edge = &m_curr_active_node->m_out_edges[i];
    Node* dst_node = out_edge->m_dst_node;
    if (dst_node->m_node_property != m_curr_active_node->m_node_property) {
      CapType cap = dst_node->m_node_property == SOURCE ?
        out_edge->m_rev_edge->m_edge_capacity : out_edge->m_edge_capacity;
      if (cap) {
        return true;
      }
    }
  }
  return false;
}

template <class CapType, class EdgePunishFun>
bool FGraph<CapType, EdgePunishFun>::Augment() {
  bool is_path_exist = true;
  auto& at_explore_nodes = m_curr_active_node->m_node_property == SOURCE ?
    m_curr_active_node->m_s_explore_Nodes : m_curr_active_node->m_t_explore_Nodes;
  auto& meet_explore_nodes = m_curr_active_node->m_node_property == SINK ?
    m_curr_active_node->m_s_explore_Nodes : m_curr_active_node->m_t_explore_Nodes;

  // process active nodes at first time
  if (at_explore_nodes.empty() && meet_explore_nodes.empty()) {
    if (!IsNeedAugment()) {
      return true;
    }
    m_curr_active_node->m_parent_edge = NULL;
    AddExploreNodes(m_curr_active_node);
    FindNewExploreNodes(m_curr_active_node);
  }

  // find first unsaturated node
  while (!at_explore_nodes.empty() && !meet_explore_nodes.empty()) {
    bool is_break = true;
    Node* at_node = at_explore_nodes.front();
    if (!at_node->m_parent_edge && at_node != m_curr_active_node ||
        !at_node->m_residue_capacity) {
      at_explore_nodes.pop_front();
      is_break = false;
      if (at_node->m_parent_edge) {
        FindNewExploreNodes(at_node);
      }
    }
    Node* meet_node = meet_explore_nodes.front();
    if (!meet_node->m_parent_edge && meet_node != m_curr_active_node ||
        !meet_node->m_residue_capacity) {
      meet_explore_nodes.pop_front();
      is_break = false;
      if (meet_node->m_parent_edge) {
        FindNewExploreNodes(meet_node);
      }
    }
    if (is_break) {
      break;
    }
  }
  if (at_explore_nodes.empty() || meet_explore_nodes.empty()) {
    return true;
  }

  Node* first_node[2] = {at_explore_nodes.front(), meet_explore_nodes.front()};
  assert(first_node[0]->m_residue_capacity && first_node[1]->m_residue_capacity);
  assert(first_node[0]->m_node_property != first_node[1]->m_node_property);
	CapType min_capacity = INT_MAX;
  // find min capacity from path
  for (int i = 0; i < 2; ++i) {
    for (Node* parent_node = first_node[i]; parent_node != m_curr_active_node;
         parent_node = parent_node->m_parent_edge->m_dst_node) {
      Edge* parent_edge = parent_node->m_parent_edge;
      assert(parent_node->m_node_property == first_node[i]->m_node_property);
      assert(parent_edge);
      m_path++;
      CapType cap = first_node[i]->m_node_property == SOURCE ?
        parent_edge->m_edge_capacity : parent_edge->m_rev_edge->m_edge_capacity;
      if (cap < min_capacity) {
        min_capacity = cap;
      }
    }
    CapType node_cap = first_node[i]->m_residue_capacity;
    CapType final_node_capacity = node_cap > 0 ? node_cap : -node_cap;
    if (final_node_capacity < min_capacity) {
      min_capacity = final_node_capacity;
    }
  }

  for (int i = 0; i < 2; ++i) {
    Edge* parent_edge = NULL;
    for (Node* parent_node = first_node[i]; parent_node != m_curr_active_node;
         parent_node = parent_edge->m_dst_node) {
      int factor = parent_node->m_node_property == SOURCE ? -1 : 1;
      parent_edge = parent_node->m_parent_edge;
      assert(parent_node->m_node_property == first_node[i]->m_node_property);
      assert(parent_edge);
      parent_edge->m_edge_capacity += factor * min_capacity;
      parent_edge->m_rev_edge->m_edge_capacity += (-factor) * min_capacity;
      Edge* edge = factor == -1 ? parent_edge : parent_edge->m_rev_edge;
      if (!edge->m_edge_capacity) {
        AddOrphanNode(parent_node);
        is_path_exist = false;
        m_broken_edges++;
      }
    }
    int factor = first_node[i]->m_node_property == SOURCE ? -1 : 1;
    first_node[i]->m_residue_capacity += factor * min_capacity;
    if (!first_node[i]->m_residue_capacity) {
      if (first_node[i]->m_node_property == SOURCE) {
        assert(m_curr_active_node->m_s_explore_Nodes.front() == first_node[i]);
        m_curr_active_node->m_s_explore_Nodes.pop_front();
        FindNewExploreNodes(first_node[i]);
      } else {
        assert(m_curr_active_node->m_t_explore_Nodes.front() == first_node[i]);
        m_curr_active_node->m_t_explore_Nodes.pop_front();
        FindNewExploreNodes(first_node[i]);
      }
    }
  }
  m_flow += min_capacity;
  return is_path_exist;
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::MoveExploreNodes(Node* old_root_node) {
  if (old_root_node->m_s_explore_Nodes.empty() && old_root_node->m_t_explore_Nodes.empty()) {
    return;
  }
  assert(old_root_node->m_s_explore_Nodes.empty() ^ old_root_node->m_t_explore_Nodes.empty());
  auto& src_explore_nodes = !old_root_node->m_s_explore_Nodes.empty() ?
    old_root_node->m_s_explore_Nodes : old_root_node->m_t_explore_Nodes;
  auto& dst_explore_nodes = !old_root_node->m_s_explore_Nodes.empty() ?
    m_curr_active_node->m_s_explore_Nodes : m_curr_active_node->m_t_explore_Nodes;
  for (auto iter = src_explore_nodes.begin(); iter != src_explore_nodes.end(); ++iter) {
    Node* ex_node = *iter;
    if (ex_node->m_root_node != old_root_node && ex_node->m_root_node != m_curr_active_node) {
      src_explore_nodes.erase(iter);
      --iter;
      continue;
    }
    ex_node->m_root_node = m_curr_active_node;
  }

  assert(!src_explore_nodes.empty());
  dst_explore_nodes.splice(dst_explore_nodes.end(), src_explore_nodes);
  assert(!dst_explore_nodes.empty() && src_explore_nodes.empty());
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::GrowOrphanTree(Node* first_node, Node* root_node) {
  std::queue<Node*> ex_orphans;
  ex_orphans.push(first_node);
  while (!ex_orphans.empty()) {
    Node* ex_orphan_node = ex_orphans.front();
    ex_orphans.pop();
    ex_orphan_node->m_root_node = root_node;
    bool add_ex_nodes = false;
    CreateOutEdges(ex_orphan_node);
    for (int i = 0; i < ex_orphan_node->m_out_edges_num; ++i) {
      Edge* out_edge = &ex_orphan_node->m_out_edges[i];
      Node* dst_node = out_edge->m_dst_node;
      if (!dst_node->m_parent_edge && dst_node->m_node_timestamp == m_orphan_timestamp &&
          (dst_node->m_node_property == ex_orphan_node->m_node_property ||
           ex_orphan_node == root_node)) {
        CapType cap = dst_node->m_node_property == SOURCE ?
          out_edge->m_rev_edge->m_edge_capacity : out_edge->m_edge_capacity;
        if (cap) {
          dst_node->m_parent_edge = out_edge->m_rev_edge;
          SET_PIXEL(m_marked_image, dst_node->m_node_idx,
                    dst_node->m_node_property == SOURCE ? RED : BLUE);
          ex_orphans.push(dst_node);
          m_tree_edges++;
        }
      } else if (!dst_node->m_parent_edge) {
        AddExploreNodes(ex_orphan_node);
      }
    }
  }
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::ChangeRootForTree(Node* old_root_node) {
  assert(!old_root_node->m_parent_edge && !old_root_node->m_node_state);
  AddOrphanNode(old_root_node);
  FindOtherPath();
  MoveExploreNodes(old_root_node);
}

template <class CapType, class EdgePunishFun>
bool FGraph<CapType, EdgePunishFun>::FindOtherPath() {
  m_orphan_timestamp++;
  bool path_found = false;
  while (!m_orphan_nodes.empty()) {
    bool find_path = false;
    Edge* new_parent_edge = NULL;
    Node* orphan_node = m_orphan_nodes.front();
    m_orphan_nodes.pop();
    if (orphan_node->m_parent_edge) {
      continue;
    }
    SET_PIXEL(m_marked_image, orphan_node->m_node_idx, YELLOW);
    orphan_node->m_node_timestamp = m_orphan_timestamp;
    bool find_arr_orphan = false;
    CreateOutEdges(orphan_node);
    for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
      Edge* out_edge = &orphan_node->m_out_edges[i];
      Node* dst_node = out_edge->m_dst_node;
      CapType cap = orphan_node->m_node_property == SOURCE ?
        out_edge->m_edge_capacity : out_edge->m_rev_edge->m_edge_capacity;
      if ((dst_node->m_node_property == orphan_node->m_node_property ||
           dst_node == m_curr_active_node) && cap && !new_parent_edge) {
        Node* trace_node = dst_node;
        // orphan parent must null
        for (; trace_node->m_parent_edge;
             trace_node = trace_node->m_parent_edge->m_dst_node) {
        }
        if (trace_node == m_curr_active_node) {
          SET_PIXEL(m_marked_image, orphan_node->m_node_idx,
                    orphan_node->m_node_property == SOURCE ? RED: BLUE);
          find_path = true;
          path_found = true;
          new_parent_edge = out_edge;
        }
      }
      if (!dst_node->m_parent_edge && dst_node->m_node_timestamp == m_orphan_timestamp) {
        find_arr_orphan = true;
      }
    }
    if (!find_path) {
      for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
        Edge* out_edge = &orphan_node->m_out_edges[i];
        Node* dst_node = out_edge->m_dst_node;
        if (dst_node->m_parent_edge != NULL &&
            dst_node->m_parent_edge->m_dst_node == orphan_node) {
          dst_node->m_node_timestamp = m_orphan_timestamp;
          AddOrphanNode(dst_node);
          m_broken_edges++;
        }
      }
      // orphan_node->m_root_node = NULL;
    } else {
#if 0
      orphan_node->m_parent_edge = new_parent_edge;
      m_tree_edges++;
      if (find_arr_orphan) {
        new_parent_edge->m_dst_node->m_root_node = m_curr_active_node;
        AddExploreNodes(orphan_node);
      }
#else
      if (find_arr_orphan) {
        GrowOrphanTree(new_parent_edge->m_dst_node, m_curr_active_node);
        assert(orphan_node->m_parent_edge);
      } else {
        orphan_node->m_parent_edge = new_parent_edge;
        m_tree_edges++;
      }
#endif
    }
  }
  return path_found;
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::ExpandStateArea() {
  if (m_curr_active_node->m_s_explore_Nodes.empty() &&
      m_curr_active_node->m_t_explore_Nodes.empty()) {
    return;
  }
  assert(m_curr_active_node->m_s_explore_Nodes.empty() ^
         m_curr_active_node->m_t_explore_Nodes.empty());
  auto change_prty = m_curr_active_node->m_s_explore_Nodes.empty() ? SINK : SOURCE;
  std::queue<Node*> expand_nodes;
  expand_nodes.push(m_curr_active_node);
  while (!expand_nodes.empty()) {
    Node* expand_node = expand_nodes.front();
    expand_nodes.pop();
    expand_node->m_node_property = change_prty;
    if (expand_node != m_curr_active_node) {
      AddOrphanNode(expand_node, false);
      m_broken_edges++;
    } else {
      // AddExploreNodes(expand_node);
    }
    SET_PIXEL(m_marked_image, expand_node->m_node_idx,
              change_prty == SOURCE ? COLOUR_DARK_RED: GRAY);
    // bool find_next = false;
    for (int i = 0; i < expand_node->m_out_edges_num; ++i) {
      Edge* out_edge = &expand_node->m_out_edges[i];
      Node* dst_node = out_edge->m_dst_node;
      if (dst_node->m_node_property != change_prty && dst_node->m_parent_edge &&
          dst_node->m_parent_edge->m_dst_node == expand_node) {
        CapType cap = expand_node->m_node_property == SOURCE ?
          out_edge->m_edge_capacity : out_edge->m_rev_edge->m_edge_capacity;
        assert(cap);
        assert(!dst_node->m_residue_capacity);
        expand_nodes.push(dst_node);
        // find_next = true;
      }
    }
    // if (expand_node != m_curr_active_node && !find_next) {
    //   expand_node->m_node_state |= ACTIVE;
    //   m_active_nodes.push(expand_node);
    // }
  }
}

template <class CapType, class EdgePunishFun>
bool FGraph<CapType, EdgePunishFun>::IsExpandArea() {
  if (!(m_curr_active_node->m_node_state & ACTIVE) ||
      m_curr_active_node->m_s_explore_Nodes.empty() ||
      m_curr_active_node->m_t_explore_Nodes.empty()) {
    m_curr_active_node->m_node_state &= ~ACTIVE;
    return true;
  }
  return false;
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::FindArroundAtNodes(std::stack<Node*>* connected_at_nodes) {
  CreateOutEdges(m_curr_active_node);
  for (int i = 0; i < m_curr_active_node->m_out_edges_num; ++i) {
    Node* arr_node = m_curr_active_node->m_out_edges[i].m_dst_node;
    if (arr_node->m_node_state & ACTIVE) {
      connected_at_nodes->push(arr_node);
    }
  }
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::MaxFlow() {
  std::stack<Node*> connected_at_nodes;
  while (!m_active_nodes.empty()) {
    Node* active_node = m_active_nodes.front();
    m_active_nodes.pop();
    if (active_node->m_node_state & ACTIVE) {
      connected_at_nodes.push(active_node);
    }
    while (!connected_at_nodes.empty()) {
      m_curr_active_node = connected_at_nodes.top();
      connected_at_nodes.pop();
      if (!(m_curr_active_node->m_node_state & ACTIVE)) {
        continue;
      }
      FindArroundAtNodes(&connected_at_nodes);
      // printf("active = %d\n", m_curr_active_node->m_node_idx);
      while (true) {
        Augment();
        FindOtherPath();
        if (IsExpandArea()) {
          ExpandStateArea();
          break;
        }
      }
    }
  }
  printf("path = %d, tree_edges = %d, broken_edges = %d, m_flow = %f\n",
          m_path, m_tree_edges, m_broken_edges, m_flow);
}

template <class CapType, class EdgePunishFun>
bool FGraph<CapType, EdgePunishFun>::IsBelongToSource(int node_id) {
  return m_nodes[node_id].m_node_property == SOURCE;
}

#undef SOURCE 
#undef SINK 
#undef ACTIVE
#endif  // INCLUDE_FGRAPH_H_
