#include <assert.h>
#include <algorithm>
#include "include/utils.h"
#include "include/CountTime.h"

// #define ENABLE_BFS
// #define ENABLE_PAR
// #define ENABLE_NEWAL
// #define ENABLE_DYNIMIC_EDGE

#define TERMINAL ( (arc *) 1 )    /* to terminal */
#define ORPHAN   ( (arc *) 2 )    /* orphan */

#define HALF_NEIGHBOUR 4
#define NEIGHBOUR 8
#define HALF_NEIGHBOUR_ARR_INDEX FOUR_ARR_INDEX
#define NEIGHBOUR_ARR_INDEX EIGHT_ARR_INDEX
typedef double (*EPF)(int src_node_colour, int dst_node_colour);

const double bkgraph_div_sqrt2 = 1 / sqrt(2.0f);

#define INFINITE_D ((int)(((unsigned)-1)/2))    /* infinite distance to the terminal */

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

/* graph.h */
/*
    Copyright Vladimir Kolmogorov (vnk@ist.ac.at), Yuri Boykov (yuri@csd.uwo.ca)

    This file is part of MAXFLOW.

    MAXFLOW is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MAXFLOW is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MAXFLOW.  If not, see <http://www.gnu.org/licenses/>.

========================

  version 3.03

  This software library implements the maxflow algorithm
  described in

    "An Experimental Comparison of Min-Cut/Max-Flow Algorithms for Energy Minimization in Vision."
    Yuri Boykov and Vladimir Kolmogorov.
    In IEEE Transactions on Pattern Analysis and Machine Intelligence (PAMI),
    September 2004

  This algorithm was developed by Yuri Boykov and Vladimir Kolmogorov
  at Siemens Corporate Research. To make it available for public use,
  it was later reimplemented by Vladimir Kolmogorov based on open publications.

  If you use this software for research purposes, you should cite
  the aforementioned paper in any resulting publication.

  ----------------------------------------------------------------------

  REUSING TREES:

  Starting with version 3.0, there is a also an option of reusing search
  trees from one maxflow computation to the next, as described in

    "Efficiently Solving Dynamic Markov Random Fields Using Graph Cuts."
    Pushmeet Kohli and Philip H.S. Torr
    International Conference on Computer Vision (ICCV), 2005

  If you use this option, you should cite
  the aforementioned paper in any resulting publication.
*/

/*
  For description, license, example usage see README.TXT.
*/

#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "block.h"
// NOTE: in UNIX you need to use -DNDEBUG preprocessor option to supress assert's!!!



// captype: type of edge capacities (excluding t-links)
// tcaptype: type of t-links (edges between nodes and terminals)
// flowtype: type of total flow
//
// Current instantiations are in instances.inc
template <typename captype, typename tcaptype, typename flowtype> class Graph
{
public:
  typedef enum
  {
    SOURCE  = 0,
    SINK  = 1
  } termtype; // terminals
  typedef int node_id;

  /////////////////////////////////////////////////////////////////////////
  //                     BASIC INTERFACE FUNCTIONS                       //
  //              (should be enough for most applications)               //
  /////////////////////////////////////////////////////////////////////////

  // Constructor.
  // The first argument gives an estimate of the maximum number of nodes that can be added
  // to the graph, and the second argument is an estimate of the maximum number of edges.
  // The last (optional) argument is the pointer to the function which will be called
  // if an error occurs; an error message is passed to this function.
  // If this argument is omitted, exit(1) will be called.
  //
  // IMPORTANT: It is possible to add more nodes to the graph than node_num_max
  // (and node_num_max can be zero). However, if the count is exceeded, then
  // the internal memory is reallocated (increased by 50%) which is expensive.
  // Also, temporarily the amount of allocated memory would be more than twice than needed.
  // Similarly for edges.
  // If you wish to avoid this overhead, you can download version 2.2, where nodes and edges are stored in blocks.
  Graph(int node_num_max, int edge_num_max, void (*err_function)(const char *) = NULL);

  // Destructor
  ~Graph();

  // Adds node(s) to the graph. By default, one node is added (num=1); then first call returns 0, second call returns 1, and so on.
  // If num>1, then several nodes are added, and node_id of the first one is returned.
  // IMPORTANT: see note about the constructor
  node_id add_node(int num = 1);

  // Adds a bidirectional edge between 'i' and 'j' with the weights 'cap' and 'rev_cap'.
  // IMPORTANT: see note about the constructor
  void add_edge(node_id i, node_id j, captype cap, captype rev_cap);

  // Adds new edges 'SOURCE->i' and 'i->SINK' with corresponding weights.
  // Can be called multiple times for each node.
  // Weights can be negative.
  // NOTE: the number of such edges is not counted in edge_num_max.
  //       No internal memory is allocated by this call.
  void add_tweights(node_id i, tcaptype cap_source, tcaptype cap_sink);


  // Computes the maxflow. Can be called several times.
  // FOR DESCRIPTION OF reuse_trees, SEE mark_node().
  // FOR DESCRIPTION OF changed_list, SEE remove_from_changed_list().
  flowtype maxflow(bool reuse_trees = false, Block<node_id>* changed_list = NULL);

  // After the maxflow is computed, this function returns to which
  // segment the node 'i' belongs (Graph<captype,tcaptype,flowtype>::SOURCE or Graph<captype,tcaptype,flowtype>::SINK).
  //
  // Occasionally there may be several minimum cuts. If a node can be assigned
  // to both the source and the sink, then default_segm is returned.
  termtype what_segment(node_id i, termtype default_segm = SOURCE);



  //////////////////////////////////////////////
  //       ADVANCED INTERFACE FUNCTIONS       //
  //      (provide access to the graph)       //
  //////////////////////////////////////////////

private:
  struct node;
  struct arc;

public:
#ifdef ENABLE_DYNIMIC_EDGE
  Graph(int max_nodes_number, int image_width, int image_height, EPF epf);
  void add_tweights(node_id i, tcaptype cap_source, tcaptype cap_sink, int node_colour);
  void AddActiveNodes(int node_x, int node_y);
  arc* GetEdge(node* src_node, node* dst_node);
  arc* CreateEdge(node* src_node, node* dst_node, double punish_factor);
  void CreateOutEdges(node* cen_node);
#endif

  ////////////////////////////
  // 1. Reallocating graph. //
  ////////////////////////////

  // Removes all nodes and edges.
  // After that functions add_node() and add_edge() must be called again.
  //
  // Advantage compared to deleting Graph and allocating it again:
  // no calls to delete/new (which could be quite slow).
  //
  // If the graph structure stays the same, then an alternative
  // is to go through all nodes/edges and set new residual capacities
  // (see functions below).
  void reset();

  ////////////////////////////////////////////////////////////////////////////////
  // 2. Functions for getting pointers to arcs and for reading graph structure. //
  //    NOTE: adding new arcs may invalidate these pointers (if reallocation    //
  //    happens). So it's best not to add arcs while reading graph structure.   //
  ////////////////////////////////////////////////////////////////////////////////

  // The following two functions return arcs in the same order that they
  // were added to the graph. NOTE: for each call add_edge(i,j,cap,cap_rev)
  // the first arc returned will be i->j, and the second j->i.
  // If there are no more arcs, then the function can still be called, but
  // the returned arc_id is undetermined.
  typedef arc* arc_id;
  arc_id get_first_arc();
  arc_id get_next_arc(arc_id a);

  // other functions for reading graph structure
  int get_node_num() { return node_num; }
  int get_arc_num() { return (int)(arc_last - arcs); }
  void get_arc_ends(arc_id a, node_id& i, node_id& j); // returns i,j to that a = i->j

  ///////////////////////////////////////////////////
  // 3. Functions for reading residual capacities. //
  ///////////////////////////////////////////////////

  // returns residual capacity of SOURCE->i minus residual capacity of i->SINK
  tcaptype get_trcap(node_id i);
  // returns residual capacity of arc a
  captype get_rcap(arc* a);

  /////////////////////////////////////////////////////////////////
  // 4. Functions for setting residual capacities.               //
  //    NOTE: If these functions are used, the value of the flow //
  //    returned by maxflow() will not be valid!                 //
  /////////////////////////////////////////////////////////////////

  void set_trcap(node_id i, tcaptype trcap);
  void set_rcap(arc* a, captype rcap);

  ////////////////////////////////////////////////////////////////////
  // 5. Functions related to reusing trees & list of changed nodes. //
  ////////////////////////////////////////////////////////////////////

  // If flag reuse_trees is true while calling maxflow(), then search trees
  // are reused from previous maxflow computation.
  // In this case before calling maxflow() the user must
  // specify which parts of the graph have changed by calling mark_node():
  //   add_tweights(i),set_trcap(i)    => call mark_node(i)
  //   add_edge(i,j),set_rcap(a)       => call mark_node(i); mark_node(j)
  //
  // This option makes sense only if a small part of the graph is changed.
  // The initialization procedure goes only through marked nodes then.
  //
  // mark_node(i) can either be called before or after graph modification.
  // Can be called more than once per node, but calls after the first one
  // do not have any effect.
  //
  // NOTE:
  //   - This option cannot be used in the first call to maxflow().
  //   - It is not necessary to call mark_node() if the change is ``not essential'',
  //     i.e. sign(trcap) is preserved for a node and zero/nonzero status is preserved for an arc.
  //   - To check that you marked all necessary nodes, you can call maxflow(false) after calling maxflow(true).
  //     If everything is correct, the two calls must return the same value of flow. (Useful for debugging).
  void mark_node(node_id i);

  // If changed_list is not NULL while calling maxflow(), then the algorithm
  // keeps a list of nodes which could potentially have changed their segmentation label.
  // Nodes which are not in the list are guaranteed to keep their old segmentation label (SOURCE or SINK).
  // Example usage:
  //
  //    typedef Graph<int,int,int> G;
  //    G* g = new Graph(nodeNum, edgeNum);
  //    Block<G::node_id>* changed_list = new Block<G::node_id>(128);
  //
  //    ... // add nodes and edges
  //
  //    g->maxflow(); // first call should be without arguments
  //    for (int iter=0; iter<10; iter++)
  //    {
  //      ... // change graph, call mark_node() accordingly
  //
  //      g->maxflow(true, changed_list);
  //      G::node_id* ptr;
  //      for (ptr=changed_list->ScanFirst(); ptr; ptr=changed_list->ScanNext())
  //      {
  //        G::node_id i = *ptr; assert(i>=0 && i<nodeNum);
  //        g->remove_from_changed_list(i);
  //        // do something with node i...
  //        if (g->what_segment(i) == G::SOURCE) { ... }
  //      }
  //      changed_list->Reset();
  //    }
  //    delete changed_list;
  //    
  // NOTE:
  //  - If changed_list option is used, then reuse_trees must be used as well.
  //  - In the example above, the user may omit calls g->remove_from_changed_list(i) and changed_list->Reset() in a given iteration.
  //    Then during the next call to maxflow(true, &changed_list) new nodes will be added to changed_list.
  //  - If the next call to maxflow() does not use option reuse_trees, then calling remove_from_changed_list()
  //    is not necessary. ("changed_list->Reset()" or "delete changed_list" should still be called, though).
  void remove_from_changed_list(node_id i)
  {
    assert(i>=0 && i<node_num && nodes[i].is_in_changed_list);
    nodes[i].is_in_changed_list = 0;
  }






/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
  
private:
  // internal variables and functions

  struct node
  {
    arc      *first;    // first outcoming arc

    arc      *parent;  // node's parent
    node    *next;    // pointer to the next active node
                //   (or to itself if it is the last node in the list)
    int      TS;      // timestamp showing when DIST was computed
    int      DIST;    // distance to the terminal
    bool    is_sink;  // flag showing whether the node is in the source or in the sink tree (if parent!=NULL)
    bool    is_marked;  // set by mark_node()
    bool    is_in_changed_list; // set by maxflow if
    bool    m_tag;

    tcaptype  tr_cap;    // if tr_cap > 0 then tr_cap is residual capacity of the arc SOURCE->node
                // otherwise         -tr_cap is residual capacity of the arc node->SINK

#ifdef ENABLE_NEWAL
    arc* m_child_edge;
    tcaptype m_needed_flow;
    node* m_next_node;
#endif
#ifdef ENABLE_DYNIMIC_EDGE
    bool m_is_gotten_all_edges;
    arc m_out_edges[NEIGHBOUR];
    int m_out_edges_num;
    int  m_node_idx;
    int m_node_colour;
#endif
  };

  struct arc
  {
    node    *head;    // node the arc points to
    arc      *next;    // next arc with the same originating node
    arc      *sister;  // reverse arc

    captype    r_cap;    // residual capacity
  };

  struct nodeptr
  {
    node      *ptr;
    nodeptr    *next;
  };
  static const int NODEPTR_BLOCK_SIZE = 128;

  node        *nodes, *node_last, *node_max; // node_last = nodes+node_num, node_max = nodes+node_num_max;
  arc          *arcs, *arc_last, *arc_max; // arc_last = arcs+2*edge_num, arc_max = arcs+2*edge_num_max;

  int          node_num;

  DBlock<nodeptr>    *nodeptr_block;

  void  (*error_function)(const char *);  // this function is called if a error occurs,
                    // with a corresponding error message
                    // (or exit(1) is called if it's NULL)

  flowtype      flow;    // total flow
  flowtype      m_min_flow;

  // reusing trees & list of changed pixels
  int          maxflow_iteration; // counter
  Block<node_id>    *changed_list;

  /////////////////////////////////////////////////////////////////////////

  node        *queue_first[2], *queue_last[2];  // list of active nodes
  nodeptr        *orphan_first, *orphan_last;    // list of pointers to orphans
  int          TIME;                // monotonically increasing global counter
  long path;
  int orphan_count;
  int m_num;
  int m_n;

#ifdef ENABLE_NEWAL
  flowtype m_source_res_flow;
  bool m_is_source_end;
  flowtype m_sink_res_flow;
  bool m_is_sink_end;
  node* m_top_source_node;
  node* m_top_sink_node;
  node* m_source_first_node;
  node* m_sink_first_node;
#endif
#ifdef ENABLE_DYNIMIC_EDGE
  int m_image_width;
  int m_image_height;
  EPF m_epf;
#endif

  /////////////////////////////////////////////////////////////////////////

  void reallocate_nodes(int num); // num is the number of new nodes
  void reallocate_arcs();

  // functions for processing active list
  void set_active(node *i);
  void set_active_before(node *i);
  node *next_active();

  // functions for processing orphans list
  void set_orphan_front(node* i); // add to the beginning of the list
  void set_orphan_rear(node* i);  // add to the end of the list

  void add_to_changed_list(node* i);

  void maxflow_init();             // called if reuse_trees == false
  void maxflow_reuse_trees_init(); // called if reuse_trees == true

#ifdef ENABLE_NEWAL
  void AdoptNewPath(node* dst_node);
  void SetTerminalNode(node* dst_node);
  void Same(flowtype& dst_val, flowtype comp_val);
  void PushFlow(node* src_node, node* dst_node, arc* flow_edge, flowtype push_flow_capacity);
  void SetResFlow(node* node);

  bool Empty(bool node_type);
  void Push(node* dst_node);
  node*& Top(bool node_type);
  void Pop(bool node_type);
  void PushEnoughFlowToOneNode(bool node_type);

  void PushEnoughFlowToTwoNodes(node* source_node, node* sink_node);
  void PushEnoughFlow(node* source_node, node* sink_node, flowtype* min_edge_capacity);
#endif
  void augment(arc *middle_arc);
  void process_source_orphan(node *i, bool condition = false);
  void process_sink_orphan(node *i, bool condition = false);

  void test_consistency(node* current_node=NULL); // debug function
};











///////////////////////////////////////
// Implementation - inline functions //
///////////////////////////////////////



template <typename captype, typename tcaptype, typename flowtype>
  inline typename Graph<captype,tcaptype,flowtype>::node_id Graph<captype,tcaptype,flowtype>::add_node(int num)
{
  assert(num > 0);

  if (node_last + num > node_max) reallocate_nodes(num);

  memset(node_last, 0, num*sizeof(node));

  node_id i = node_num;
  node_num += num;
  node_last += num;
  return i;
}

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::add_tweights(node_id i, tcaptype cap_source, tcaptype cap_sink)
{
  assert(i >= 0 && i < node_num);

  tcaptype delta = nodes[i].tr_cap;
  if (delta > 0) cap_source += delta;
  else           cap_sink   -= delta;
  flowtype f = (cap_source < cap_sink) ? cap_source : cap_sink;
  flow += f;
  nodes[i].tr_cap = cap_source - cap_sink;
}

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::add_edge(node_id _i, node_id _j, captype cap, captype rev_cap)
{
  assert(_i >= 0 && _i < node_num);
  assert(_j >= 0 && _j < node_num);
  assert(_i != _j);
  assert(cap >= 0);
  assert(rev_cap >= 0);

  if (arc_last == arc_max) reallocate_arcs();

  arc *a = arc_last ++;
  arc *a_rev = arc_last ++;

  node* i = nodes + _i;
  node* j = nodes + _j;

  a -> sister = a_rev;
  a_rev -> sister = a;
  a -> next = i -> first;
  i -> first = a;
  a_rev -> next = j -> first;
  j -> first = a_rev;
  a -> head = j;
  a_rev -> head = i;
  a -> r_cap = cap;
  a_rev -> r_cap = rev_cap;
}

template <typename captype, typename tcaptype, typename flowtype>
  inline typename Graph<captype,tcaptype,flowtype>::arc* Graph<captype,tcaptype,flowtype>::get_first_arc()
{
  return arcs;
}

template <typename captype, typename tcaptype, typename flowtype>
  inline typename Graph<captype,tcaptype,flowtype>::arc* Graph<captype,tcaptype,flowtype>::get_next_arc(arc* a)
{
  return a + 1;
}

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::get_arc_ends(arc* a, node_id& i, node_id& j)
{
  assert(a >= arcs && a < arc_last);
  i = (node_id) (a->sister->head - nodes);
  j = (node_id) (a->head - nodes);
}

template <typename captype, typename tcaptype, typename flowtype>
  inline tcaptype Graph<captype,tcaptype,flowtype>::get_trcap(node_id i)
{
  assert(i>=0 && i<node_num);
  return nodes[i].tr_cap;
}

template <typename captype, typename tcaptype, typename flowtype>
  inline captype Graph<captype,tcaptype,flowtype>::get_rcap(arc* a)
{
  assert(a >= arcs && a < arc_last);
  return a->r_cap;
}

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::set_trcap(node_id i, tcaptype trcap)
{
  assert(i>=0 && i<node_num);
  nodes[i].tr_cap = trcap;
}

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::set_rcap(arc* a, captype rcap)
{
  assert(a >= arcs && a < arc_last);
  a->r_cap = rcap;
}


template <typename captype, typename tcaptype, typename flowtype>
  inline typename Graph<captype,tcaptype,flowtype>::termtype Graph<captype,tcaptype,flowtype>::what_segment(node_id i, termtype default_segm)
{
  if (nodes[i].parent)
  {
    return (nodes[i].is_sink) ? SINK : SOURCE;
  }
  else
  {
    return default_segm;
  }
}

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::mark_node(node_id _i)
{
  node* i = nodes + _i;
  if (!i->next)
  {
    /* it's not in the list yet */
    if (queue_last[1]) queue_last[1] -> next = i;
    else               queue_first[1]        = i;
    queue_last[1] = i;
    i -> next = i;
  }
  i->is_marked = 1;
}

/*
  special constants for node->parent. Duplicated in maxflow.cpp, both should match!
*/
template <typename captype, typename tcaptype, typename flowtype>
  Graph<captype, tcaptype, flowtype>::Graph(int node_num_max, int edge_num_max, void (*err_function)(const char *))
  : node_num(0),
    nodeptr_block(NULL),
    error_function(err_function)
{
  if (node_num_max < 16) node_num_max = 16;
  if (edge_num_max < 16) edge_num_max = 16;

  nodes = (node*) malloc(node_num_max*sizeof(node));
  arcs = (arc*) malloc(2*edge_num_max*sizeof(arc));
  if (!nodes || !arcs) { if (error_function) (*error_function)("Not enough memory!"); exit(1); }

  node_last = nodes;
  node_max = nodes + node_num_max;
  arc_last = arcs;
  arc_max = arcs + 2*edge_num_max;

  maxflow_iteration = 0;
  flow = 0;

  queue_first[0] = queue_last[0] = NULL;
  queue_first[1] = queue_last[1] = NULL;
  orphan_first = NULL;
  TIME = 0;
}

#ifdef ENABLE_DYNIMIC_EDGE
template <typename captype, typename tcaptype, typename flowtype>
  Graph<captype, tcaptype, flowtype>::Graph(int node_num_max, int image_width, int image_height, EPF epf)
  : node_num(0),
    nodeptr_block(NULL),
    error_function(NULL),
    m_image_width(image_width),
    m_image_height(image_height),
    m_epf(epf) {
  if (node_num_max < 16) node_num_max = 16;

  nodes = (node*) malloc(node_num_max*sizeof(node));

  node_last = nodes;
  node_max = nodes + node_num_max;

  maxflow_iteration = 0;
  flow = 0;

  queue_first[0] = queue_last[0] = NULL;
  queue_first[1] = queue_last[1] = NULL;
  orphan_first = NULL;
  TIME = 0;
}

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::add_tweights(
  node_id i, tcaptype cap_source, tcaptype cap_sink, int node_colour)
{
  add_tweights(i, cap_source, cap_sink);
  nodes[i].next = NULL;
  nodes[i].is_marked = 0;
  nodes[i].is_in_changed_list = 0;
  nodes[i].TS = TIME;
  nodes[i].parent = TERMINAL;
  nodes[i].DIST = 1;
  nodes[i].is_sink = nodes[i].tr_cap > 0 ? 0 : 1;
  // set_active(&nodes[i]);
  nodes[i].m_node_colour = node_colour;
  nodes[i].m_node_idx = i;
  nodes[i].m_is_gotten_all_edges = false;
  nodes[i].m_out_edges_num = 0;
}

template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype, tcaptype, flowtype>::AddActiveNodes(int node_x, int node_y) {
  int index = node_y * m_image_width + node_x;
  node* cen_node = &nodes[index];
  int arr_index[HALF_NEIGHBOUR] =
    HALF_NEIGHBOUR_ARR_INDEX(node_x, node_y, m_image_width, m_image_height);
  for (int i = 0; i < HALF_NEIGHBOUR; ++i) {
    node* arr_node = &nodes[arr_index[i]];
    if (arr_index[i] < index && cen_node->is_sink != arr_node->is_sink) {
      set_active(cen_node);
      break;
    }
  }
}

template <typename captype, typename tcaptype, typename flowtype>
typename Graph<captype, tcaptype, flowtype>::arc* Graph<captype, tcaptype, flowtype>::GetEdge(
  node* src_node, node* dst_node) {
  for (int i = 0; i < src_node->m_out_edges_num; ++i) {
    if (src_node->m_out_edges[i].head == dst_node) {
      return &src_node->m_out_edges[i];
    }
  }
  // for (Edge* connected_edge = src_node->m_first_out_edge; connected_edge;
  //      connected_edge = connected_edge->m_next_out_edge) {
  //   if (connected_edge->m_dst_node == dst_node) {
  //     return connected_edge;
  //   }
  // }
  return NULL;
}

template <typename captype, typename tcaptype, typename flowtype>
typename Graph<captype, tcaptype, flowtype>::arc* Graph<captype, tcaptype, flowtype>::CreateEdge(
  node* src_node, node* dst_node, double punish_factor) {
  // Edge* edge = new Edge();
  // m_edges.push(edge);
  // Edge* rev_edge = new Edge();
  // m_edges.push(rev_edge);
  // CapType cap = punish_factor * m_epf(src_node->m_node_colour, dst_node->m_node_colour);
  // edge->m_edge_capacity = cap;
  // edge->m_rev_edge = rev_edge;
  // edge->m_dst_node = dst_node;
  // rev_edge->m_edge_capacity = cap;
  // rev_edge->m_rev_edge = edge;
  // rev_edge->m_dst_node = src_node;
  // edge->m_next_out_edge = src_node->m_first_out_edge;
  // src_node->m_first_out_edge = edge;
  // rev_edge->m_next_out_edge = dst_node->m_first_out_edge;
  // dst_node->m_first_out_edge = rev_edge;

  arc* edge = &src_node->m_out_edges[src_node->m_out_edges_num++];
  arc* rev_edge = &dst_node->m_out_edges[dst_node->m_out_edges_num++];
  tcaptype cap = punish_factor * m_epf(src_node->m_node_colour, dst_node->m_node_colour);
  edge->r_cap = cap;
  edge->sister = rev_edge;
  edge->head = dst_node;
  rev_edge->r_cap = cap;
  rev_edge->sister = edge;
  rev_edge->head = src_node;
  return edge;
}

template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype, tcaptype, flowtype>::CreateOutEdges(node* cen_node) {
  assert(cen_node != NULL);
  if (cen_node->m_is_gotten_all_edges) {
    return;
  }
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
    node* arr_node = &nodes[arr_nodes_idx[i]];
    if (!GetEdge(cen_node, arr_node)) {
      if (ABS(arr_nodes_idx[i] - cen_node->m_node_idx) == 1 ||
          ABS(arr_nodes_idx[i] - cen_node->m_node_idx) == m_image_width) {
        punish_factor = 1;
      } else {
        punish_factor = bkgraph_div_sqrt2;
      }
      CreateEdge(cen_node, arr_node, punish_factor);
    }
  }
  cen_node->m_is_gotten_all_edges = true;
}
#endif

template <typename captype, typename tcaptype, typename flowtype>
  Graph<captype,tcaptype,flowtype>::~Graph()
{
  if (nodeptr_block)
  {
    delete nodeptr_block;
    nodeptr_block = NULL;
  }
  free(nodes);
#ifndef ENABLE_DYNIMIC_EDGE
  free(arcs);
#endif
}

template <typename captype, typename tcaptype, typename flowtype>
  void Graph<captype,tcaptype,flowtype>::reset()
{
  node_last = nodes;
  arc_last = arcs;
  node_num = 0;

  if (nodeptr_block)
  {
    delete nodeptr_block;
    nodeptr_block = NULL;
  }

  maxflow_iteration = 0;
  flow = 0;
}

template <typename captype, typename tcaptype, typename flowtype>
  void Graph<captype,tcaptype,flowtype>::reallocate_nodes(int num)
{
  int node_num_max = (int)(node_max - nodes);
  node* nodes_old = nodes;

  node_num_max += node_num_max / 2;
  if (node_num_max < node_num + num) node_num_max = node_num + num;
  nodes = (node*) realloc(nodes_old, node_num_max*sizeof(node));
  if (!nodes) { if (error_function) (*error_function)("Not enough memory!"); exit(1); }

  node_last = nodes + node_num;
  node_max = nodes + node_num_max;

  if (nodes != nodes_old)
  {
    node* i;
    arc* a;
    for (i=nodes; i<node_last; i++)
    {
      if (i->next) i->next = (node*) ((char*)i->next + (((char*) nodes) - ((char*) nodes_old)));
    }
    for (a=arcs; a<arc_last; a++)
    {
      a->head = (node*) ((char*)a->head + (((char*) nodes) - ((char*) nodes_old)));
    }
  }
}

template <typename captype, typename tcaptype, typename flowtype>
  void Graph<captype,tcaptype,flowtype>::reallocate_arcs()
{
  int arc_num_max = (int)(arc_max - arcs);
  int arc_num = (int)(arc_last - arcs);
  arc* arcs_old = arcs;

  arc_num_max += arc_num_max / 2; if (arc_num_max & 1) arc_num_max ++;
  arcs = (arc*) realloc(arcs_old, arc_num_max*sizeof(arc));
  if (!arcs) { if (error_function) (*error_function)("Not enough memory!"); exit(1); }

  arc_last = arcs + arc_num;
  arc_max = arcs + arc_num_max;

  if (arcs != arcs_old)
  {
    node* i;
    arc* a;
    for (i=nodes; i<node_last; i++)
    {
      if (i->first) i->first = (arc*) ((char*)i->first + (((char*) arcs) - ((char*) arcs_old)));
      if (i->parent && i->parent != ORPHAN && i->parent != TERMINAL) i->parent = (arc*) ((char*)i->parent + (((char*) arcs) - ((char*) arcs_old)));
    }
    for (a=arcs; a<arc_last; a++)
    {
      if (a->next) a->next = (arc*) ((char*)a->next + (((char*) arcs) - ((char*) arcs_old)));
      a->sister = (arc*) ((char*)a->sister + (((char*) arcs) - ((char*) arcs_old)));
    }
  }
}

/*
  special constants for node->parent. Duplicated in graph.cpp, both should match!
*/
#define TERMINAL ( (arc *) 1 )    /* to terminal */
#define ORPHAN   ( (arc *) 2 )    /* orphan */


/***********************************************************************/

/*
  Functions for processing active list.
  i->next points to the next node in the list
  (or to i, if i is the last node in the list).
  If i->next is NULL iff i is not in the list.

  There are two queues. Active nodes are added
  to the end of the second queue and read from
  the front of the first queue. If the first queue
  is empty, it is replaced by the second queue
  (and the second queue becomes empty).
*/

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::set_active(node *i)
{
  if (!i->next)
  {
    /* it's not in the list yet */
    if (queue_last[1]) queue_last[1] -> next = i;
    else               queue_first[1]        = i;
    queue_last[1] = i;
    i -> next = i;
  }
}

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::set_active_before(node *i)
{
  if (!i->next)
  {
    /* it's not in the list yet */
    assert(queue_first[0] || queue_first[1]);
    if (queue_first[0]) {
      i->next = queue_first[0];
      queue_first[0] = i;
    } else if (queue_first[1]) {
      i->next = queue_first[1];
      queue_first[1] = i;
    }
  }
}
/*
  Returns the next active node.
  If it is connected to the sink, it stays in the list,
  otherwise it is removed from the list
*/
template <typename captype, typename tcaptype, typename flowtype>
  inline typename Graph<captype,tcaptype,flowtype>::node* Graph<captype,tcaptype,flowtype>::next_active()
{
  node *i;

  while ( 1 )
  {
    if (!(i=queue_first[0]))
    {
      queue_first[0] = i = queue_first[1];
      queue_last[0]  = queue_last[1];
      queue_first[1] = NULL;
      queue_last[1]  = NULL;
      if (!i) return NULL;
    }

    /* remove it from the active list */
    if (i->next == i) queue_first[0] = queue_last[0] = NULL;
    else              queue_first[0] = i -> next;
    i -> next = NULL;

    /* a node in the list is active iff it has a parent */
    if (i->parent) return i;
  }
}

/***********************************************************************/

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::set_orphan_front(node *i)
{
  nodeptr *np;
  i -> parent = ORPHAN;
  np = nodeptr_block -> New();
  np -> ptr = i;
  np -> next = orphan_first;
  orphan_first = np;
}

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::set_orphan_rear(node *i)
{
  nodeptr *np;
  i -> parent = ORPHAN;
  np = nodeptr_block -> New();
  np -> ptr = i;
  if (orphan_last) orphan_last -> next = np;
  else             orphan_first        = np;
  orphan_last = np;
  np -> next = NULL;
}

/***********************************************************************/

template <typename captype, typename tcaptype, typename flowtype>
  inline void Graph<captype,tcaptype,flowtype>::add_to_changed_list(node *i)
{
  if (changed_list && !i->is_in_changed_list)
  {
    node_id* ptr = changed_list->New();
    *ptr = (node_id)(i - nodes);
    i->is_in_changed_list = true;
  }
}

/***********************************************************************/

template <typename captype, typename tcaptype, typename flowtype>
  void Graph<captype,tcaptype,flowtype>::maxflow_init()
{
  node *i;

  queue_first[0] = queue_last[0] = NULL;
  queue_first[1] = queue_last[1] = NULL;
  orphan_first = NULL;
  TIME = 0;
  m_num = 0;
  m_n = 0;

#ifdef ENABLE_NEWAL
  flowtype m_source_res_flow = 0;
  bool m_is_source_end = false;
  flowtype m_sink_res_flow = 0;
  bool m_is_sink_end = false;
  m_top_source_node = NULL;
  m_top_sink_node = NULL;
  m_source_first_node = NULL;
  m_sink_first_node = NULL;
#endif

  for (i=nodes; i<node_last; i++)
  {
    m_num++;
    i -> next = NULL;
    i -> is_marked = 0;
    i -> is_in_changed_list = 0;
    i -> TS = TIME;
    i->m_tag = false;
#ifdef ENABLE_NEWAL
    i -> m_child_edge = NULL;
    i -> m_needed_flow = 0;
    i -> m_next_node = NULL;
#endif
    if (i->tr_cap > 0)
    {
      /* i is connected to the source */
      i -> is_sink = 0;
      i -> parent = TERMINAL;
      set_active(i);
      i -> DIST = 1;
    }
    else if (i->tr_cap < 0)
    {
      /* i is connected to the sink */
      i -> is_sink = 1;
      i -> parent = TERMINAL;
      set_active(i);
      i -> DIST = 1;
    }
    else
    {
      i -> parent = NULL;
    }
  }
}

template <typename captype, typename tcaptype, typename flowtype>
  void Graph<captype,tcaptype,flowtype>::maxflow_reuse_trees_init()
{
  node* i;
  node* j;
  node* queue = queue_first[1];
  arc* a;
  nodeptr* np;

  queue_first[0] = queue_last[0] = NULL;
  queue_first[1] = queue_last[1] = NULL;
  orphan_first = orphan_last = NULL;

  TIME ++;

  while ((i=queue))
  {
    queue = i->next;
    if (queue == i) queue = NULL;
    i->next = NULL;
    i->is_marked = 0;
    set_active(i);

    if (i->tr_cap == 0)
    {
      if (i->parent) set_orphan_rear(i);
      continue;
    }

    if (i->tr_cap > 0)
    {
      if (!i->parent || i->is_sink)
      {
        i->is_sink = 0;
        for (a=i->first; a; a=a->next)
        {
          j = a->head;
          if (!j->is_marked)
          {
            if (j->parent == a->sister) set_orphan_rear(j);
            if (j->parent && j->is_sink && a->r_cap > 0) set_active(j);
          }
        }
        add_to_changed_list(i);
      }
    }
    else
    {
      if (!i->parent || !i->is_sink)
      {
        i->is_sink = 1;
        for (a=i->first; a; a=a->next)
        {
          j = a->head;
          if (!j->is_marked)
          {
            if (j->parent == a->sister) set_orphan_rear(j);
            if (j->parent && !j->is_sink && a->sister->r_cap > 0) set_active(j);
          }
        }
        add_to_changed_list(i);
      }
    }
    i->parent = TERMINAL;
    i -> TS = TIME;
    i -> DIST = 1;
  }

  //test_consistency();

  /* adoption */
  while ((np=orphan_first))
  {
    orphan_first = np -> next;
    i = np -> ptr;
    nodeptr_block -> Delete(np);
    if (!orphan_first) orphan_last = NULL;
    if (i->is_sink) process_sink_orphan(i);
    else            process_source_orphan(i);
  }
  /* adoption end */

  //test_consistency();
}

#ifdef ENABLE_NEWAL
template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype,tcaptype,flowtype>::AdoptNewPath(node* dst_node) {
  set_orphan_front(dst_node);
  /* adoption */
  nodeptr *np, *np_next;
  node* i = NULL;
  while ((np=orphan_first))
  {
    np_next = np -> next;
    np -> next = NULL;

    while ((np=orphan_first))
    {
      orphan_first = np -> next;
      i = np -> ptr;
      nodeptr_block -> Delete(np);
      if (!orphan_first) orphan_last = NULL;
      if (i->parent == ORPHAN) {
        if (i->is_sink) process_sink_orphan(i, true);
        else            process_source_orphan(i, true);
      }
    }

    orphan_first = np_next;
  }
}

template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype,tcaptype,flowtype>::SetTerminalNode(node* dst_node) {
  dst_node->parent = TERMINAL;
  dst_node->DIST = 1;
}

template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype,tcaptype,flowtype>::Same(flowtype& dst_val, flowtype comp_val) {
  if (ABS(dst_val- comp_val) < EPSILON) {
    dst_val = comp_val;
  }
}

template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype,tcaptype,flowtype>::
  PushFlow(node* src_node, node* dst_node, arc* flow_edge, flowtype push_flow_capacity) {
  flowtype flow = std::min(ABS(src_node->tr_cap), flow_edge->r_cap);
  flow = std::min(flow, ABS(push_flow_capacity));

  // assert(src_node->is_sink == dst_node->is_sink);
  src_node->tr_cap -= src_node->is_sink == SOURCE ? flow : -flow;
  Same(src_node->tr_cap, 0);

  dst_node->tr_cap += src_node->is_sink == SOURCE ? flow : -flow;
  flow_edge->r_cap -= flow;
  Same(flow_edge->r_cap, 0);

  flow_edge->sister->r_cap += flow;
  dst_node->m_needed_flow -= dst_node->is_sink == SOURCE ? flow : -flow;
  Same(dst_node->m_needed_flow, 0);
}

template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype,tcaptype,flowtype>::SetResFlow(node* dst_node) {
  flowtype& res_flow = dst_node->is_sink == SOURCE ? m_source_res_flow : m_sink_res_flow;
  bool& node_end = dst_node->is_sink == SOURCE ? m_is_source_end : m_is_sink_end;
  if (dst_node->parent == TERMINAL) {
    res_flow -= dst_node->tr_cap;
    Same(res_flow, 0);
    if (dst_node->is_sink == SOURCE) {
      res_flow = res_flow > 0 ? res_flow : 0;
    } else {
      res_flow = res_flow < 0 ? res_flow : 0;
    }
    if (!res_flow) {
      node_end = true;
    }
  }
}

template <typename captype, typename tcaptype, typename flowtype>
bool Graph<captype,tcaptype,flowtype>::Empty(bool node_type) {
  node*& top_node = node_type == SOURCE ? m_top_source_node : m_top_sink_node;
  if (!top_node) {
    return true;
  }
  return false;
}

template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype,tcaptype,flowtype>::Push(node* dst_node) {
  node*& top_node = dst_node->is_sink == SOURCE ? m_top_source_node : m_top_sink_node;
  dst_node->m_next_node = top_node;
  top_node = dst_node;
}

template <typename captype, typename tcaptype, typename flowtype>
typename Graph<captype,tcaptype,flowtype>::node*& Graph<captype,tcaptype,flowtype>::Top(bool node_type) {
  return node_type == SOURCE ? m_top_source_node : m_top_sink_node;
}

template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype,tcaptype,flowtype>::Pop(bool node_type) {
  node*& top_node = node_type == SOURCE ? m_top_source_node : m_top_sink_node;
  // assert(top_node);
  node* tmp = top_node;
  top_node = top_node->m_next_node;
  tmp->m_next_node = NULL;
}

template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype,tcaptype,flowtype>::PushEnoughFlowToOneNode(bool node_type) {
  // assert(!Empty(node_type));
  node*& node_ = Top(node_type);
  bool& node_end = node_type == SOURCE ? m_is_source_end : m_is_sink_end;
  bool& other_node_end = node_type == SINK ? m_is_source_end : m_is_sink_end;
  flowtype& node_res_flow = node_type == SOURCE ? m_source_res_flow : m_sink_res_flow;
  flowtype& other_res_flow = node_type == SINK ? m_source_res_flow : m_sink_res_flow;

  node* src_node = node_;
  flowtype push_flow = node_->tr_cap;
  while (!node_end && node_->parent && node_->m_needed_flow) {
    if (node_->parent == ORPHAN || node_->parent == TERMINAL) {
      AdoptNewPath(node_);
    }
    if (node_end || !node_->parent || !node_->m_needed_flow) {
      break;
    }

    // assert(node_->m_needed_flow && node_->parent);
    node* dst_node = node_->parent->head;
    dst_node->m_child_edge = node_->parent->sister;
    arc* flow_edge = node_type == SINK ?
      node_->parent : node_->parent->sister;
    // assert(flow_edge->r_cap);
    flowtype dst_nd_flow;
    flowtype delt_flow = std::min(ABS(dst_node->tr_cap), flow_edge->r_cap);
    dst_nd_flow = std::min(flow_edge->r_cap, ABS(node_->m_needed_flow)) - delt_flow;
    // assert(dst_nd_flow - ABS(node_res_flow) < EPSILON);
    dst_node->m_needed_flow = node_type == SOURCE ? dst_nd_flow : -dst_nd_flow;
    if (dst_nd_flow > 0) {
      SetResFlow(dst_node);
      Push(dst_node);
      return;
    } else {
      // assert(node_->m_needed_flow);
      // assert(dst_node->parent == TERMINAL);
      push_flow = std::min(flow_edge->r_cap, ABS(node_->m_needed_flow));
      if (node_type == SINK) {
        push_flow = -push_flow;
      }
      node_res_flow -= push_flow;
      Same(node_res_flow, 0);
      if (!node_res_flow) {
        node_end = true;
      }
      src_node = dst_node;
    }
    break;
  }
  if (src_node->m_child_edge) {
    node* child_node = src_node->m_child_edge->head;
    arc* flow_edge = node_type == SOURCE ?
      src_node->m_child_edge : src_node->m_child_edge->sister;
    // if (src_node == node_ && ABS(node_res_flow) < ABS(other_res_flow) && other_node_end) {
    //   push_flow = node_->tr_cap + (m_sink_res_flow + m_source_res_flow);
    //   // assert(ABS(push_flow) < ABS(node_->tr_cap));
    //   node_res_flow = -other_res_flow;
    //   SetTerminalNode(node_);
    // }
    if (push_flow) {
      PushFlow(src_node, child_node, flow_edge, push_flow);
    }
    if (child_node->parent && !flow_edge->r_cap) {
      set_orphan_front(child_node);
    }
  }
  if (src_node == node_) {
    Pop(node_type);
    if (Empty(node_type)) {
      other_node_end = true;
    }
  }
}

// node is current node
// needed_flow is except the excess of the node what the node need in flow.
// res_flow is the total resident flow
template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype,tcaptype,flowtype>::PushEnoughFlowToTwoNodes(node* source_node, node* sink_node) {
  Push(source_node);
  Push(sink_node);
  m_source_first_node = source_node;
  m_sink_first_node = sink_node;
  while (!Empty(SOURCE) || !Empty(SINK)) {
    if (!Empty(SOURCE) && (m_source_res_flow > -m_sink_res_flow || Empty(SINK))) {
      PushEnoughFlowToOneNode(SOURCE);
    } else {
      PushEnoughFlowToOneNode(SINK);
    }
  }
}

template <typename captype, typename tcaptype, typename flowtype>
void Graph<captype,tcaptype,flowtype>::PushEnoughFlow(
  node* source_node, node* sink_node, flowtype* min_edge_capacity) {
  m_source_res_flow = *min_edge_capacity;
  m_sink_res_flow = -*min_edge_capacity;
  m_is_source_end = false;
  m_is_sink_end = false;
  source_node->m_needed_flow = m_source_res_flow - source_node->tr_cap > 0 ?
    m_source_res_flow - source_node->tr_cap : 0;
  sink_node->m_needed_flow = m_sink_res_flow - sink_node->tr_cap < 0 ?
    m_sink_res_flow - sink_node->tr_cap : 0;
  source_node->m_child_edge = NULL;
  sink_node->m_child_edge = NULL;
  SetResFlow(source_node);
  SetResFlow(sink_node);

  PushEnoughFlowToTwoNodes(source_node, sink_node);

  Same(source_node->tr_cap, *min_edge_capacity);
  Same(sink_node->tr_cap, -*min_edge_capacity);
  Same(source_node->tr_cap, -sink_node->tr_cap);
  if (!source_node->parent) {
    SetTerminalNode(source_node);
  }
  if (!sink_node->parent) {
    SetTerminalNode(sink_node);
  }
  *min_edge_capacity = std::min(*min_edge_capacity, source_node->tr_cap);
  *min_edge_capacity = std::min(*min_edge_capacity, -sink_node->tr_cap);
  if (source_node->parent && source_node->parent != TERMINAL &&
      source_node->tr_cap > *min_edge_capacity) {
    // // assert(source_node->parent->head->parent == TERMINAL);
    SetTerminalNode(source_node);
  }
  if (sink_node->parent && sink_node->parent != TERMINAL &&
      sink_node->tr_cap < -*min_edge_capacity) {
    // // assert(sink_node->parent->head->parent == TERMINAL);
    SetTerminalNode(sink_node);
  }
}
#endif

template <typename captype, typename tcaptype, typename flowtype>
  void Graph<captype,tcaptype,flowtype>::augment(arc *middle_arc)
{
  node *i;
  arc *a;
  tcaptype bottleneck;


  /* 1. Finding bottleneck capacity */
  /* 1a - the source tree */
  bottleneck = middle_arc -> r_cap;
#ifdef ENABLE_NEWAL
  node* flow_nodes[2];
#endif
#ifdef ENABLE_BFS
  middle_arc->sister->head->m_child_edge = NULL;
#endif
  for (i=middle_arc->sister->head; ; i=a->head)
  {
    path++;
    if (!i->m_tag) {
      i->m_tag = true;
      m_n++;
    }
    a = i -> parent;
    if (a == TERMINAL) break;
#ifdef ENABLE_BFS
    a->head->m_child_edge = a->sister;
#endif
    if (bottleneck > a->sister->r_cap) bottleneck = a -> sister -> r_cap;
  }
#ifndef ENABLE_NEWAL
  if (bottleneck > i->tr_cap) bottleneck = i -> tr_cap;
#else
  flow_nodes[0] = i;
#endif
  /* 1b - the sink tree */
#ifdef ENABLE_BFS
  middle_arc->head->m_child_edge = NULL;
#endif
  for (i=middle_arc->head; ; i=a->head)
  {
    path++;
    if (!i->m_tag) {
      i->m_tag = true;
      m_n++;
    }
    a = i -> parent;
    if (a == TERMINAL) break;
#ifdef ENABLE_BFS
    a->head->m_child_edge = a->sister;
#endif
    if (bottleneck > a->r_cap) bottleneck = a -> r_cap;
  }
#ifndef ENABLE_NEWAL
  if (bottleneck > - i->tr_cap) bottleneck = - i -> tr_cap;
#else
  flow_nodes[1] = i;
#endif

#ifdef ENABLE_NEWAL
  PushEnoughFlow(flow_nodes[0], flow_nodes[1], &bottleneck);
#endif

  /* 2. Augmenting */
  /* 2a - the source tree */
  middle_arc -> sister -> r_cap += bottleneck;
  middle_arc -> r_cap -= bottleneck;
  for (i=middle_arc->sister->head; ; i=a->head)
  {
    a = i -> parent;
#ifndef ENABLE_NEWAL
    if (a == TERMINAL) break;
#else
    if (i == flow_nodes[0]) break;
#endif
    a -> r_cap += bottleneck;
    a -> sister -> r_cap -= bottleneck;
    if (!a->sister->r_cap)
    {
      set_orphan_front(i); // add i to the beginning of the adoption list
    }
    path++;
  }
  i -> tr_cap -= bottleneck;
  if (!i->tr_cap)
  {
    set_orphan_front(i); // add i to the beginning of the adoption list
  }
  /* 2b - the sink tree */
  for (i=middle_arc->head; ; i=a->head)
  {
    a = i -> parent;
#ifndef ENABLE_NEWAL
    if (a == TERMINAL) break;
#else
    if (i == flow_nodes[1]) break;
#endif
    a -> sister -> r_cap += bottleneck;
    a -> r_cap -= bottleneck;
    if (!a->r_cap)
    {
      set_orphan_front(i); // add i to the beginning of the adoption list
    }
    path++;
  }
  i -> tr_cap += bottleneck;
  if (!i->tr_cap)
  {
    set_orphan_front(i); // add i to the beginning of the adoption list
  }

  flow += bottleneck;
}

/***********************************************************************/

template <typename captype, typename tcaptype, typename flowtype>
  void Graph<captype,tcaptype,flowtype>::process_source_orphan(node *i, bool condition)
{
  node *j;
  arc *a0, *a0_min = NULL, *a;
  int d, d_min = INFINITE_D;

  /* trying to find a new parent */
#ifdef ENABLE_NEWAL
  if (!condition || i == m_source_first_node || i == m_sink_first_node)
#endif
#ifndef ENABLE_DYNIMIC_EDGE
  for (a0=i->first; a0; a0=a0->next) {
#else
  CreateOutEdges(i);
  for (int k = 0; k < i->m_out_edges_num; ++k) {
    a0 = &i->m_out_edges[k];
#endif
    // if (!a0->head->m_tag) {
    //   a0->head->m_tag = true;
    //   m_n++;
    // }
    if (a0->sister->r_cap)
    {
      j = a0 -> head;
      if (!j->is_sink && (a=j->parent))
      {
        /* checking the origin of j */
        d = 0;
        while ( 1 )
        {
          if (j->TS == TIME)
          {
            d += j -> DIST;
            break;
          }
          a = j -> parent;
          d ++;
          if (a==TERMINAL)
          {
            j -> TS = TIME;
            j -> DIST = 1;
            break;
          }
          if (a==ORPHAN || !a) { d = INFINITE_D; break; }
          j = a -> head;
        }
        if (d<INFINITE_D) /* j originates from the source - done */
        {
          if (d<d_min)
          {
            a0_min = a0;
            d_min = d;
          }
          /* set marks along the path */
          for (j=a0->head; j->TS!=TIME; j=j->parent->head)
          {
            j -> TS = TIME;
            j -> DIST = d --;
          }
        }
      }
    }
  }

  if (i->parent = a0_min)
  {
    assert(a0_min->head->is_sink == 0);
    i -> TS = TIME;
    i -> DIST = d_min + 1;
#ifdef ENABLE_BFS
    if (i->m_child_edge) {
      node* child_node = i->m_child_edge->head;
      if (child_node->parent == i->m_child_edge->sister &&
          child_node->DIST <= i->DIST) {
        set_orphan_rear(child_node);
      }
    }
#endif
  }
  else
  {
#ifdef ENABLE_NEWAL
    if (condition) {
      if (i == m_source_first_node || i == m_sink_first_node) {
        if (i->is_sink == SOURCE) {
          m_is_source_end = true;
        } else {
          m_is_sink_end = true;
        }
        return;
      }
    }
#endif
    /* no parent is found */
    add_to_changed_list(i);

    /* process neighbors */
#ifndef ENABLE_DYNIMIC_EDGE
    for (a0=i->first; a0; a0=a0->next) {
#else
    for (int k = 0; k < i->m_out_edges_num; ++k) {
      a0 = &i->m_out_edges[k];
#endif
    // if (!a0->head->m_tag) {
    //   a0->head->m_tag = true;
    //   m_n++;
    // }
      j = a0 -> head;
      if (!j->is_sink && (a=j->parent))
      {
        if (a0->sister->r_cap) set_active(j);
        if (a!=TERMINAL && a!=ORPHAN && a->head==i)
        {
          set_orphan_rear(j); // add j to the end of the adoption list
        }
      }
    }
  }
}

template <typename captype, typename tcaptype, typename flowtype>
  void Graph<captype,tcaptype,flowtype>::process_sink_orphan(node *i, bool condition)
{
  node *j;
  arc *a0, *a0_min = NULL, *a;
  int d, d_min = INFINITE_D;

  /* trying to find a new parent */
#ifdef ENABLE_NEWAL
  if (!condition || i == m_source_first_node || i == m_sink_first_node)
#endif
#ifndef ENABLE_DYNIMIC_EDGE
  for (a0=i->first; a0; a0=a0->next) {
#else
  CreateOutEdges(i);
  for (int k = 0; k < i->m_out_edges_num; ++k) {
    a0 = &i->m_out_edges[k];
#endif
    // if (!a0->head->m_tag) {
    //   a0->head->m_tag = true;
    //   m_n++;
    // }
    if (a0->r_cap)
    {
      j = a0 -> head;
      if (j->is_sink && (a=j->parent))
      {
        /* checking the origin of j */
        d = 0;
        while ( 1 )
        {
          if (j->TS == TIME)
          {
            d += j -> DIST;
            break;
          }
          a = j -> parent;
          d ++;
          if (a==TERMINAL)
          {
            j -> TS = TIME;
            j -> DIST = 1;
            break;
          }
          if (a==ORPHAN || !a) { d = INFINITE_D; break; }
          j = a -> head;
        }
        if (d<INFINITE_D) /* j originates from the sink - done */
        {
          if (d<d_min)
          {
            a0_min = a0;
            d_min = d;
          }
          /* set marks along the path */
          for (j=a0->head; j->TS!=TIME; j=j->parent->head)
          {
            j -> TS = TIME;
            j -> DIST = d --;
          }
        }
      }
    }
  }

  if (i->parent = a0_min)
  {
    assert(a0_min->head->is_sink == 1);
    i -> TS = TIME;
    i -> DIST = d_min + 1;
#ifdef ENABLE_BFS
    if (i->m_child_edge) {
      node* child_node = i->m_child_edge->head;
      if (child_node->parent == i->m_child_edge->sister &&
          child_node->DIST <= i->DIST) {
        set_orphan_rear(child_node);
      }
    }
#endif
  }
  else
  {
#ifdef ENABLE_NEWAL
    if (condition) {
      if (i == m_source_first_node || i == m_sink_first_node) {
        if (i->is_sink == SOURCE) {
          m_is_source_end = true;
        } else {
          m_is_sink_end = true;
        }
        return;
      }
    }
#endif
    /* no parent is found */
    add_to_changed_list(i);

    /* process neighbors */
#ifndef ENABLE_DYNIMIC_EDGE
    for (a0=i->first; a0; a0=a0->next) {
#else
    for (int k = 0; k < i->m_out_edges_num; ++k) {
      a0 = &i->m_out_edges[k];
#endif
    // if (!a0->head->m_tag) {
    //   a0->head->m_tag = true;
    //   m_n++;
    // }
      j = a0 -> head;
      if (j->is_sink && (a=j->parent))
      {
        if (a0->r_cap) set_active(j);
        if (a!=TERMINAL && a!=ORPHAN && a->head==i)
        {
          set_orphan_rear(j); // add j to the end of the adoption list
        }
      }
    }
  }
}

/***********************************************************************/
template <typename captype, typename tcaptype, typename flowtype>
  flowtype Graph<captype,tcaptype,flowtype>::maxflow(bool reuse_trees, Block<node_id>* _changed_list)
{
  path = 0;
  orphan_count = 0;
  double time = 0;
  node *i, *j, *current_node = NULL;
  arc *a;
  nodeptr *np, *np_next;

  if (!nodeptr_block)
  {
    nodeptr_block = new DBlock<nodeptr>(NODEPTR_BLOCK_SIZE, error_function);
  }

  changed_list = _changed_list;
  if (maxflow_iteration == 0 && reuse_trees) { if (error_function) (*error_function)("reuse_trees cannot be used in the first call to maxflow()!"); exit(1); }
  if (changed_list && !reuse_trees) { if (error_function) (*error_function)("changed_list cannot be used without reuse_trees!"); exit(1); }

  if (reuse_trees) maxflow_reuse_trees_init();
#ifndef ENABLE_DYNIMIC_EDGE
  else             maxflow_init();
#endif

  // main loop
  while ( 1 )
  {
#ifndef ENABLE_PAR
    // test_consistency(current_node);
    if ((i=current_node))
    {
      i -> next = NULL; /* remove active flag */
      if (!i->parent) i = NULL;
    }
#else
    i = NULL;
#endif
    if (!i)
    {
      if (!(i = next_active())) break;
    }

    /* growth */
#ifdef ENABLE_DYNIMIC_EDGE
    CreateOutEdges(i);
#endif
    if (!i->is_sink)
    {
      /* grow source tree */
#ifndef ENABLE_DYNIMIC_EDGE
      for (a=i->first; a; a=a->next) {
#else
      for (int k = 0; k < i->m_out_edges_num; ++k) {
        a = &i->m_out_edges[k];
#endif
        if (a->r_cap)
        {
          j = a -> head;
          if (!j->parent)
          {
            j -> is_sink = 0;
            j -> parent = a -> sister;
            j -> TS = i -> TS;
            j -> DIST = i -> DIST + 1;
            set_active(j);
            add_to_changed_list(j);
          }
          else if (j->is_sink) {
            break;
          }
          else if (j->TS <= i->TS &&
                   j->DIST > i->DIST)
          {
            /* heuristic - trying to make the distance from j to the source shorter */
            j -> parent = a -> sister;
            j -> TS = i -> TS;
            j -> DIST = i -> DIST + 1;
          }
        }
#ifdef ENABLE_DYNIMIC_EDGE
        a = NULL;
#endif
      }
    }
    else
    {
      /* grow sink tree */
#ifndef ENABLE_DYNIMIC_EDGE
      for (a=i->first; a; a=a->next) {
#else
      for (int k = 0; k < i->m_out_edges_num; ++k) {
        a = &i->m_out_edges[k];
#endif
        if (a->sister->r_cap)
        {
          j = a -> head;
          if (!j->parent)
          {
            j -> is_sink = 1;
            j -> parent = a -> sister;
            j -> TS = i -> TS;
            j -> DIST = i -> DIST + 1;
            set_active(j);
            add_to_changed_list(j);
          }
          else if (!j->is_sink) {
            a = a -> sister;
            break;
          }
          else if (j->TS <= i->TS &&
                   j->DIST > i->DIST)
          {
            /* heuristic - trying to make the distance from j to the sink shorter */
            j -> parent = a -> sister;
            j -> TS = i -> TS;
            j -> DIST = i -> DIST + 1;
          }
        }
#ifdef ENABLE_DYNIMIC_EDGE
        a = NULL;
#endif
      }
    }

    TIME ++;

    if (a)
    {
      CountTime ct;
      ct.ContBegin();
#ifdef ENABLE_PAR
      int origion_length = a->head->DIST + a->sister->head->DIST;
#endif
      i -> next = i; /* set active flag */
      current_node = i;

      /* augmentation */
      augment(a);
      /* augmentation end */

      /* adoption */
      while ((np=orphan_first))
      {
        np_next = np -> next;
        np -> next = NULL;

        while ((np=orphan_first))
        {
          orphan_first = np -> next;
          i = np -> ptr;
          nodeptr_block -> Delete(np);
          if (!orphan_first) orphan_last = NULL;
          if (i->parent == ORPHAN) {
            orphan_count++;
            if (i->is_sink) process_sink_orphan(i);
            else            process_source_orphan(i);
          }
        }

        orphan_first = np_next;
      }
      /* adoption end */
#ifdef ENABLE_PAR
      current_node->next = NULL;
      int new_length = a->head->DIST + a->sister->head->DIST;
      // set_active(current_node);
      if (new_length > origion_length) {
        set_active_before(current_node);
      } else {
        set_active(current_node);
      }
#endif
      ct.ContEnd();
      time += ct.ContResult();
    }
    else current_node = NULL;
  }
  // test_consistency();

  if (!reuse_trees || (maxflow_iteration % 64) == 0)
  {
    delete nodeptr_block;
    nodeptr_block = NULL;
  }

  maxflow_iteration ++;
  printf("path = %ld, orphan count = %d, time = %f\n", path, orphan_count, time * 1000);
  // printf("m_flow = %f\n", flow);
  // printf("radio = %f\n", m_n/(double)m_num);
  return flow;
}

/***********************************************************************/


template <typename captype, typename tcaptype, typename flowtype>
  void Graph<captype,tcaptype,flowtype>::test_consistency(node* current_node)
{
  node *i;
  arc *a;
  int r;
  int num1 = 0, num2 = 0;

  // test whether all nodes i with i->next!=NULL are indeed in the queue
  for (i=nodes; i<node_last; i++)
  {
    if (i->next || i==current_node) num1 ++;
  }
  for (r=0; r<3; r++)
  {
    i = (r == 2) ? current_node : queue_first[r];
    if (i)
    for ( ; ; i=i->next)
    {
      num2 ++;
      if (i->next == i)
      {
        if (r<2) assert(i == queue_last[r]);
        else     assert(i == current_node);
        break;
      }
    }
  }
  assert(num1 == num2);

  for (i=nodes; i<node_last; i++)
  {
    // test whether all edges in seach trees are non-saturated
    if (i->parent == NULL) {}
    else if (i->parent == ORPHAN) {}
    else if (i->parent == TERMINAL)
    {
      if (!i->is_sink) assert(i->tr_cap > 0);
      else             assert(i->tr_cap < 0);
    }
    else
    {
      if (!i->is_sink) assert (i->parent->sister->r_cap > 0);
      else             assert (i->parent->r_cap > 0);
    }
    // test whether passive nodes in search trees have neighbors in
    // a different tree through non-saturated edges
    if (i->parent && !i->next)
    {
      if (!i->is_sink)
      {
        assert(i->tr_cap >= 0);
        for (a=i->first; a; a=a->next)
        {
          if (a->r_cap > 0) assert(a->head->parent && !a->head->is_sink);
        }
      }
      else
      {
        assert(i->tr_cap <= 0);
        for (a=i->first; a; a=a->next)
        {
          if (a->sister->r_cap > 0) assert(a->head->parent && a->head->is_sink);
        }
      }
    }
    // test marking invariants
    if (i->parent && i->parent!=ORPHAN && i->parent!=TERMINAL)
    {
      assert(i->TS <= i->parent->head->TS);
      if (i->TS == i->parent->head->TS) assert(i->DIST > i->parent->head->DIST);
    }
  }
}

#undef ENABLE_BFS
#undef ENABLE_PAR
#endif
