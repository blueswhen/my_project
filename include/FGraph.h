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
    #define UNSATURATION 0x00
    #define SATURATION 0x01
    #define ORPHAN 0x02
    #define ACTIVE 0x04
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
  };

  Node* GetActiveNode();
  bool PopActiveNode();
  void FindNewExploreNodes(Node* center_node, bool explore_node_prty,
                           std::list<Node*>* explore_nodes);
  bool Augment();
  void MoveExploreNodes(Node* transit_node, Node* new_active_node);
  void GrowOrphanTree(Node* first_node, Node* root_node);
  void FindOtherPath();
  void ExpandStateArea();
  void CreateOutEdges(Node* cen_node);
  Edge* GetEdge(Node* src_node, Node* dst_node);
  Edge* CreateEdge(Node* src_node, Node* dst_node, double punish_factor);

  std::vector<Node> m_nodes;
  CapType m_flow;
  std::list<Node*> m_active_nodes;
  typename std::list<Node*>::iterator m_active_iter;
  Node* m_curr_active_node;
  std::queue<Node*> m_orphan_nodes;
  int m_image_width;
  int m_image_height;
  EdgePunishFun m_epf;
  ImageData<int>* m_marked_image;
};

template <class CapType, class EdgePunishFun>
FGraph<CapType, EdgePunishFun>::FGraph(int max_nodes_number, int image_width, int image_height,
                                       EdgePunishFun epf, ImageData<int>* marked_image)
  : m_nodes(std::vector<Node>(max_nodes_number))
  , m_flow(0)
  , m_active_iter(m_active_nodes.begin())
  , m_image_width(image_width)
  , m_image_height(image_height)
  , m_epf(epf)
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
  node->m_node_state = UNSATURATION;
  node->m_parent_edge = NULL;
  node->m_timestamp = 0;
  node->m_terminal_dist = 1;
  node->m_is_gotten_all_edges = false;
  node->m_out_edges_num = 0;
  node->m_root_node = NULL;
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
      Node* source_node = cen_node->m_node_property == SOURCE ? cen_node : arr_node;
      if (!(source_node->m_node_state & ACTIVE)) {
        // if (source_node->m_node_idx != 215674) {
        //   continue;
        // }
        source_node->m_node_state |= ACTIVE;
        SET_PIXEL(m_marked_image, source_node->m_node_idx, GREEN);
        m_active_nodes.push_back(source_node);
      }
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
typename FGraph<CapType, EdgePunishFun>::Node*
  FGraph<CapType, EdgePunishFun>::GetActiveNode() {
  if (m_active_nodes.empty()) {
    return NULL;
  }
  if (m_active_iter == m_active_nodes.end()) {
    m_active_iter = m_active_nodes.begin();
  }
  return *(m_active_iter++);
}

template <class CapType, class EdgePunishFun>
bool FGraph<CapType, EdgePunishFun>::PopActiveNode() {
  Node* at_node = *(--m_active_iter);
  if (at_node->m_s_explore_Nodes.empty() || at_node->m_t_explore_Nodes.empty()) {
    at_node->m_node_state &= ~ACTIVE;
    m_active_iter = m_active_nodes.erase(m_active_iter);
    return true;
  }
  ++m_active_iter;
  return false;
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::FindNewExploreNodes(
  Node* center_node, bool explore_node_prty, std::list<Node*>* explore_nodes) {
  CreateOutEdges(center_node);
  for (int i = 0; i < center_node->m_out_edges_num; ++i) {
    Edge* out_edge = &center_node->m_out_edges[i];
    Node* dst_node = out_edge->m_dst_node;
    if (dst_node->m_node_property == explore_node_prty &&
        (dst_node->m_node_state == UNSATURATION || dst_node->m_node_state == ORPHAN) &&
        !dst_node->m_parent_edge) {
      CapType cap = explore_node_prty == SOURCE ?
                    out_edge->m_rev_edge->m_edge_capacity :
                    out_edge->m_edge_capacity;
      if (cap) {
        dst_node->m_root_node = m_curr_active_node;
        dst_node->m_node_state &= ~ORPHAN;
        explore_nodes->push_back(dst_node);
        dst_node->m_parent_edge = out_edge->m_rev_edge;
        SET_PIXEL(m_marked_image, dst_node->m_node_idx,
                  explore_node_prty == SOURCE ? RED : BLUE);
      }
    }
  }
}

template <class CapType, class EdgePunishFun>
bool FGraph<CapType, EdgePunishFun>::Augment() {
  assert(m_curr_active_node->m_node_property == SOURCE);
  bool is_path_exist = true;
  auto& at_explore_nodes = m_curr_active_node->m_s_explore_Nodes;
  auto& meet_explore_nodes = m_curr_active_node->m_t_explore_Nodes;

  // process active nodes at first time
  if (at_explore_nodes.empty() && meet_explore_nodes.empty()) {
    at_explore_nodes.push_back(m_curr_active_node);
    m_curr_active_node->m_root_node = m_curr_active_node;
    FindNewExploreNodes(m_curr_active_node, SINK, &meet_explore_nodes);
  }

  // find first unsaturated node
  while (!at_explore_nodes.empty() && !meet_explore_nodes.empty()) {
    bool is_break = true;
    Node* at_node = at_explore_nodes.front();
    if (at_node->m_node_state & SATURATION || at_node->m_root_node != m_curr_active_node) {
      at_explore_nodes.pop_front();
      is_break = false;
    }
    Node* meet_node = meet_explore_nodes.front();
    if (meet_node->m_node_state & SATURATION || meet_node->m_root_node != m_curr_active_node) {
      meet_explore_nodes.pop_front();
      is_break = false;
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
	CapType min_capacity = INT_MAX;
  // find min capacity from path
  for (int i = 0; i < 2; ++i) {
    for (Node* parent_node = first_node[i]; parent_node != m_curr_active_node;
         parent_node = parent_node->m_parent_edge->m_dst_node) {
      Edge* parent_edge = parent_node->m_parent_edge;
      assert(parent_edge);
      CapType cap = parent_node->m_node_property == SOURCE ?
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
      assert(parent_edge);
      parent_edge->m_edge_capacity += factor * min_capacity;
      parent_edge->m_rev_edge->m_edge_capacity += (-factor) * min_capacity;
      Edge* edge = factor == -1 ? parent_edge : parent_edge->m_rev_edge;
      if (!edge->m_edge_capacity) {
        parent_node->m_parent_edge = NULL;
        parent_node->m_node_state |= ORPHAN;
        m_orphan_nodes.push(parent_node);
        is_path_exist = false;
      }
    }
    int factor = first_node[i]->m_node_property == SOURCE ? -1 : 1;
    first_node[i]->m_residue_capacity += factor * min_capacity;
    if (!first_node[i]->m_residue_capacity) {
  // int xy1[2] = GET_XY(first_node[i]->m_node_idx, m_image_width);
  // int id = first_node[i]->m_parent_edge ? first_node[i]->m_parent_edge->m_dst_node->m_node_idx : 0;
  // int xy2[2] = GET_XY(id, m_image_width);
  // printf("SATURATION x,y = %d,%d, parent x,y = %d,%d\n", xy1[0], xy1[1], xy2[0], xy2[1]);
      first_node[i]->m_node_state |= SATURATION;
      if (first_node[i]->m_node_property == SOURCE) {
        assert(m_curr_active_node->m_s_explore_Nodes.front() == first_node[i]);
        m_curr_active_node->m_s_explore_Nodes.pop_front();
        FindNewExploreNodes(first_node[i], SOURCE, &at_explore_nodes);
      } else {
        assert(m_curr_active_node->m_t_explore_Nodes.front() == first_node[i]);
        m_curr_active_node->m_t_explore_Nodes.pop_front();
        FindNewExploreNodes(first_node[i], SINK, &meet_explore_nodes);
      }
    }
  }
  m_flow += min_capacity;
  return is_path_exist;
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::MoveExploreNodes(
  Node* transit_node, Node* new_active_node) {
  auto& src_explore_nodes = transit_node->m_node_property == SOURCE ?
    m_curr_active_node->m_s_explore_Nodes : m_curr_active_node->m_t_explore_Nodes;
  auto& dst_explore_nodes = transit_node->m_node_property == SOURCE ?
    new_active_node->m_s_explore_Nodes : new_active_node->m_t_explore_Nodes;

  for (auto iter = src_explore_nodes.begin(); iter != src_explore_nodes.end(); ++iter) {
    Node* explore_node = *iter;
    Node* trace_node = explore_node;
    if (explore_node->m_root_node != m_curr_active_node) {
      continue;
    }
    for (; !(trace_node->m_node_state & ACTIVE);
         trace_node = trace_node->m_parent_edge->m_dst_node) {
      if (trace_node == transit_node) {
        break;
      }
      assert(trace_node->m_parent_edge);
    }
    if (trace_node == transit_node) {
      iter = src_explore_nodes.erase(iter);
      --iter;
      explore_node->m_root_node = new_active_node;
      dst_explore_nodes.push_front(explore_node);
    }
  }
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::GrowOrphanTree(Node* first_node, Node* root_node) {
  std::queue<Node*> ex_orphans;
  ex_orphans.push(first_node);
  while (!ex_orphans.empty()) {
    Node* ex_orphan_node = ex_orphans.front();
    ex_orphans.pop();
    for (int i = 0; i < ex_orphan_node->m_out_edges_num; ++i) {
      Edge* out_edge = &ex_orphan_node->m_out_edges[i];
      Node* dst_node = out_edge->m_dst_node;
      if (dst_node->m_node_state & ORPHAN && dst_node->m_node_property ==
          ex_orphan_node->m_node_property) {
        assert(!dst_node->m_parent_edge);
        CapType cap = dst_node->m_node_property == SOURCE ?
          out_edge->m_rev_edge->m_edge_capacity : out_edge->m_edge_capacity;
        if (cap) {
          dst_node->m_node_state &= ~ORPHAN;
          dst_node->m_parent_edge = out_edge->m_rev_edge;
          // move to new explore nodes
          if (dst_node->m_node_state == UNSATURATION) {
            dst_node->m_root_node = root_node;
            if (dst_node->m_node_property == SOURCE) {
              root_node->m_s_explore_Nodes.push_front(dst_node);
            } else {
              root_node->m_t_explore_Nodes.push_front(dst_node);
            }
          }
          SET_PIXEL(m_marked_image, dst_node->m_node_idx, dst_node->m_node_property == SOURCE ? RED : BLUE);
          ex_orphans.push(dst_node);
        }
      }
    }
  }
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::FindOtherPath() {
  while (!m_orphan_nodes.empty()) {
    Node* orphan_node = m_orphan_nodes.front();
    m_orphan_nodes.pop();
    if (!(orphan_node->m_node_state & ORPHAN)) {
      continue;
    }
    // int xy[2] = GET_XY(orphan_node->m_node_idx, m_image_width);
    // printf("orphan_node x,y = %d,%d\n", xy[0], xy[1]);
    SET_PIXEL(m_marked_image, orphan_node->m_node_idx, YELLOW);
    bool find_arr_orphan = false;
    Node* new_active_node = NULL;
    for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
      Edge* out_edge = &orphan_node->m_out_edges[i];
      Node* dst_node = out_edge->m_dst_node;
      CapType cap = orphan_node->m_node_property == SOURCE ?
        out_edge->m_edge_capacity : out_edge->m_rev_edge->m_edge_capacity;
      if (dst_node->m_node_property == orphan_node->m_node_property &&
          cap && !orphan_node->m_parent_edge) {
        Node* trace_node = dst_node;
        // orphan parent must null
        for (; trace_node->m_parent_edge && !(trace_node->m_node_state & ACTIVE);
             trace_node = trace_node->m_parent_edge->m_dst_node) {
        }
        if (trace_node->m_node_state & ACTIVE) {
          // int xy[2] = GET_XY(out_edge->m_dst_node->m_node_idx, m_image_width);
          // printf("new parent node x,y = %d,%d\n", xy[0], xy[1]);
          orphan_node->m_parent_edge = out_edge;
          orphan_node->m_node_state &= ~ORPHAN;
          if (trace_node != m_curr_active_node) {
            new_active_node = trace_node;
          }
        }
      }
      if (dst_node->m_node_state & ORPHAN) {
        find_arr_orphan = true;
      }
    }
    if (orphan_node->m_node_state & ORPHAN) {
      // node is orphan and unsaturation
      if (orphan_node->m_node_state == ORPHAN) {
        orphan_node->m_root_node = NULL;
      }
      for (int i = 0; i < orphan_node->m_out_edges_num; ++i) {
        Edge* out_edge = &orphan_node->m_out_edges[i];
        Node* dst_node = out_edge->m_dst_node;
        if (dst_node->m_parent_edge != NULL &&
            dst_node->m_parent_edge->m_dst_node == orphan_node) {
          dst_node->m_parent_edge = NULL;
          dst_node->m_node_state |= ORPHAN;
          m_orphan_nodes.push(dst_node);
        }
      }
    } else {
      if (find_arr_orphan) {
        Node* root_node = new_active_node == NULL ? m_curr_active_node : new_active_node;
        GrowOrphanTree(orphan_node, root_node);
      }
      if (new_active_node) {
        MoveExploreNodes(orphan_node, new_active_node);
      }
    }
  }
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::ExpandStateArea() {
  auto change_prty = m_curr_active_node->m_s_explore_Nodes.empty() ? SINK : SOURCE;
  m_curr_active_node->m_node_property = change_prty;
  SET_PIXEL(m_marked_image, m_curr_active_node->m_node_idx,
            change_prty == SOURCE ? COLOUR_LIGHT_GRAY: GRAY);
  std::queue<Node*> expand_nodes;
  expand_nodes.push(m_curr_active_node);
  while (!expand_nodes.empty()) {
    Node* expand_node = expand_nodes.front();
    expand_nodes.pop();
    for (int i = 0; i < expand_node->m_out_edges_num; ++i) {
      Edge* out_edge = &expand_node->m_out_edges[i];
      Node* dst_node = out_edge->m_dst_node;
      CapType cap = dst_node->m_node_property == SOURCE ?
        out_edge->m_rev_edge->m_edge_capacity : out_edge->m_edge_capacity;
      if (dst_node->m_node_property != change_prty && cap && dst_node->m_parent_edge &&
          dst_node->m_parent_edge->m_dst_node == expand_node) {
        dst_node->m_node_property = change_prty;
        SET_PIXEL(m_marked_image, dst_node->m_node_idx,
                  change_prty == SOURCE ? COLOUR_LIGHT_GRAY: GRAY);
        expand_nodes.push(dst_node);
      }
    }
  }
}

template <class CapType, class EdgePunishFun>
void FGraph<CapType, EdgePunishFun>::MaxFlow() {
  while(m_curr_active_node = GetActiveNode()) {
    bool is_path_exist = Augment();
    if (!is_path_exist) {
      FindOtherPath();
    }
    if (PopActiveNode()) {
      ExpandStateArea();
    }
  }
}

template <class CapType, class EdgePunishFun>
bool FGraph<CapType, EdgePunishFun>::IsBelongToSource(int node_id) {
  return m_nodes[node_id].m_node_property == SOURCE;
}

#undef SOURCE 
#undef SINK 
#undef UNSATURATION
#undef SATURATION
#undef ACTIVE
#undef ORPHAN
#endif  // INCLUDE_FGRAPH_H_
