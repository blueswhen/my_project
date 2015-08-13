#ifndef _IBFS_H__
#define _IBFS_H__

#include <stdio.h>
#include <string.h>

#define IBDEBUG(X) fprintf(stdout, X"\n"); fflush(stdout)
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

	// inline int getFlow() {
	// 	return m_flow;
	// }
	// inline int getNumNodes() {
	// 	return m_nodeEnd-m_nodes;
	// }
	// inline int getNumArcs() {
	// 	return m_arcEnd-m_arcs;
	// }

private:
	struct Node;
	struct Arc;

	struct Arc
	{
		Node*		head;
		Arc*		rev;
		bool    isRevResidual;
		double  rCap;
	};

	class Node
	{
	public:
		Arc			*firstArc;
		Arc			*parent;
		Node		*firstSon;
		Node		*nextPtr;
		int			label;	// label > 0: distance from s, label < 0: -distance from t
		double  excess;	 // excess > 0: capacity from s, excess < 0: -capacity to t
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
	Node	*m_nodes, *m_nodeEnd;
	Arc		*m_arcs, *m_arcEnd;
	int 	m_numNodes;
	double m_flow;
	unsigned int m_uniqOrphansS, m_uniqOrphansT;
	Node* m_orphanFirst;
	Node* m_orphanLast;
	int m_topLevelS, m_topLevelT;
	ActiveList m_active0, m_activeS1, m_activeT1;

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
	  double 	cap;
		double  revCap;
	};
	struct TmpArc
	{
		TmpArc		*rev;
		double cap;
	};
	char	*m_memArcs;
	TmpEdge	*m_tmpEdges, *m_tmpEdgeLast;
	TmpArc	*m_tmpArcs;
	void initGraphFast();
	void initGraphCompact();
};

inline void IBFSGraph::addNode(int nodeIndex, double capacitySource, double capacitySink)
{
	// int f = m_nodes[nodeIndex].excess;
	// if (f > 0) {
	// 	capacitySource += f;
	// } else {
	// 	capacitySink -= f;
	// }
	if (capacitySource < capacitySink) {
		m_flow += capacitySource;
	} else {
		m_flow += capacitySink;
	}
	m_nodes[nodeIndex].excess = capacitySource - capacitySink;
}

inline void IBFSGraph::addEdge(int nodeIndexFrom, int nodeIndexTo, double capacity, double reverseCapacity)
{
	m_tmpEdgeLast->tail = m_nodes + nodeIndexFrom;
	m_tmpEdgeLast->head = m_nodes + nodeIndexTo;
	m_tmpEdgeLast->cap = capacity;
	m_tmpEdgeLast->revCap = reverseCapacity;
	m_tmpEdgeLast++;

	// use label as a temporary storage
	// to count the out degree of nodes
	m_nodes[nodeIndexFrom].label++;
	m_nodes[nodeIndexTo].label++;

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
	if (m_nodes[nodeIndex].label == m_numNodes || m_nodes[nodeIndex].label == 0) {
		return m_activeT1.len == 0;
	}
	return (m_nodes[nodeIndex].label > 0);
}

#endif
