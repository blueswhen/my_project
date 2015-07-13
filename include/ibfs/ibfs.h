/*
#########################################################
#                                                       #
#  IBFSGraph -  Software for solving                    #
#               Maximum s-t Flow / Minimum s-t Cut      #
#               using the IBFS algorithm                #
#                                                       #
#  http://www.cs.tau.ac.il/~sagihed/ibfs/               #
#                                                       #
#  Haim Kaplan (haimk@cs.tau.ac.il)                     #
#  Sagi Hed (sagihed@post.tau.ac.il)                    #
#                                                       #
#########################################################

This software implements the IBFS (Incremental Breadth First Search) maximum flow algorithm from
	"Maximum flows by incremental breadth-first search"
	Andrew V. Goldberg, Sagi Hed, Haim Kaplan, Robert E. Tarjan, and Renato F. Werneck.
	In Proceedings of the 19th European conference on Algorithms, ESA'11, pages 457-468.
	ISBN 978-3-642-23718-8
	2011

Copyright Haim Kaplan (haimk@cs.tau.ac.il) and Sagi Hed (sagihed@post.tau.ac.il)

###########
# LICENSE #
###########
This software can be used for research purposes only.
If you use this software for research purposes, you should cite the aforementioned paper
in any resulting publication and appropriately credit it.

If you require another license, please contact the above.

*/


#ifndef _IBFS_H__
#define _IBFS_H__

#include <stdio.h>
#include <string.h>

#define IB_ORPHANS_END   ( (Node *) 1 )

class IBFSGraph
{
public:

	IBFSGraph();
	~IBFSGraph();
	void initSize(int numNodes, int numEdges);
	void addEdge(int nodeIndexFrom, int nodeIndexTo, double capacity, double reverseCapacity);
	void addNode(int nodeIndex, double capacityFromSource, double capacityToSink);
	void initGraph();
	double computeMaxFlow();
	bool isNodeOnSrcSide(int nodeIndex);

private:
	struct Node;
	struct Arc;

	struct Arc
	{
		Node*		head;
		Arc*		rev;
		int			isRevResidual :1;
		double rCap;
	};

	struct Node
	{
		int			lastAugTimestamp:31;
		int			isParentCurr:1;
		Arc			*firstArc;
		Arc			*parent;
		Node		*firstSon;
		Node		*nextPtr;
		int			label;	// label > 0: distance from s, label < 0: -distance from t
		double excess;	 // excess > 0: capacity from s, excess < 0: -capacity to t
	};

	class ActiveList
	{
	public:
		inline ActiveList() {
			list = NULL;
			len = 0;
		}
		inline void init(int numNodes) {
			list = new Node*[numNodes];
			len = 0;
		}
		inline void free() {
			if (list != NULL) {
				delete list;
				list = NULL;
			}
		}
		inline void clear() {
			len = 0;
		}
		inline void add(Node* x) {
			list[len] = x;
			len++;
		}
		inline static void swapLists(ActiveList *a, ActiveList *b) {
			ActiveList tmp = (*a);
			(*a) = (*b);
			(*b) = tmp;
		}
		Node **list;
		int len;
	};

	// members
	Node	*nodes, *nodeEnd;
	Arc		*arcs, *arcEnd;
	int 	numNodes;
	double flow;
	short 	augTimestamp;
	unsigned int uniqOrphansS, uniqOrphansT;
	Node* orphanFirst;
	Node* orphanLast;
	int topLevelS, topLevelT;
	ActiveList active0, activeS1, activeT1;

	void augment(Arc *bridge);
	template<bool sTree> void augmentTree(Node *x, double bottleneck);
	template <bool sTree> void adoption();
	template <bool dirS> void growth();

	//
	// Initialization
	//
	struct TmpEdge
	{
		Node*		head;
		Node*		tail;
		double cap;
		double revCap;
	};
	struct TmpArc
	{
		TmpArc		*rev;
		double cap;
	};
	char	*memArcs;
	TmpEdge	*tmpEdges, *tmpEdgeLast;
	TmpArc	*tmpArcs;
	void initGraphFast();
	void initGraphCompact();
};

inline void IBFSGraph::addNode(int nodeIndex, double capacitySource, double capacitySink)
{
	double f = nodes[nodeIndex].excess;
	if (f > 0) {
		capacitySource += f;
	} else {
		capacitySink -= f;
	}
	if (capacitySource < capacitySink) {
		flow += capacitySource;
	} else {
		flow += capacitySink;
	}
	nodes[nodeIndex].excess = capacitySource - capacitySink;
}

inline void IBFSGraph::addEdge(int nodeIndexFrom, int nodeIndexTo, double capacity, double reverseCapacity)
{
	tmpEdgeLast->tail = nodes + nodeIndexFrom;
	tmpEdgeLast->head = nodes + nodeIndexTo;
	tmpEdgeLast->cap = capacity;
	tmpEdgeLast->revCap = reverseCapacity;
	tmpEdgeLast++;

	// use label as a temporary storage
	// to count the out degree of nodes
	nodes[nodeIndexFrom].label++;
	nodes[nodeIndexTo].label++;

	/*
	Arc *aFwd = arcLast;
	arcLast++;
	Arc *aRev = arcLast;
	arcLast++;

	Node* x = nodes + nodeIndexFrom;
	x->label++;
	Node* y = nodes + nodeIndexTo;
	y->label++;

	aRev->rev = aFwd;
	aFwd->rev = aRev;
	aFwd->rCap = capacity;
	aRev->rCap = reverseCapacity;
	aFwd->head = y;
	aRev->head = x;*/
}

inline bool IBFSGraph::isNodeOnSrcSide(int nodeIndex)
{
	if (nodes[nodeIndex].label == numNodes || nodes[nodeIndex].label == 0) {
		return activeT1.len == 0;
	}
	return (nodes[nodeIndex].label > 0);
}

#endif
