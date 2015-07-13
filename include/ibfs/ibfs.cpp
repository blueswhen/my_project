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


#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include "ibfs.h"




//
// Orphan handling
//
#define ADD_ORPHAN_BACK(n)							\
if (orphanFirst != IB_ORPHANS_END)					\
{													\
	orphanLast = (orphanLast->nextPtr = (n));	\
}													\
else												\
{													\
	orphanLast = (orphanFirst = (n));				\
}													\
(n)->nextPtr = IB_ORPHANS_END





#define ADD_ORPHAN_FRONT(n)							\
if (orphanFirst == IB_ORPHANS_END)					\
{													\
	(n)->nextPtr = IB_ORPHANS_END;					\
	orphanLast = (orphanFirst = (n));				\
}													\
else												\
{													\
	(n)->nextPtr = orphanFirst;						\
	orphanFirst = (n);								\
}




IBFSGraph::IBFSGraph()
{
	numNodes = 0;
	uniqOrphansS = uniqOrphansT = 0;
	augTimestamp = 0;
	arcs = arcEnd = NULL;
	nodes = nodeEnd = NULL;
	topLevelS = topLevelT = 0;
	flow = 0;
	orphanFirst = orphanLast = NULL;
	memArcs = NULL;
	tmpArcs = NULL;
	tmpEdges = tmpEdgeLast = NULL;
}

IBFSGraph::~IBFSGraph()
{
	if (nodes) free(nodes);
	if (memArcs) free(memArcs);
	active0.free();
	activeS1.free();
	activeT1.free();
}

void IBFSGraph::initGraph()
{
  initGraphFast();
}

void IBFSGraph::initSize(int numNodes, int numEdges)
{
	// allocate nodes
	this->numNodes = numNodes;
	nodes = new Node[numNodes+1];
	memset(nodes, 0, sizeof(Node)*(numNodes+1));
	nodeEnd = nodes+numNodes;
	active0.init(numNodes);
	activeS1.init(numNodes);
	activeT1.init(numNodes);

	// allocate arcs
	unsigned long long arcMemsize = (unsigned long long)sizeof(TmpArc)*(unsigned long long)(numEdges*2) + (unsigned long long)sizeof(TmpEdge)*(unsigned long long)numEdges;
	if (arcMemsize < (unsigned long long)sizeof(Arc)*(unsigned long long)(numEdges*2)) {
		arcMemsize = (unsigned long long)sizeof(Arc)*(unsigned long long)(numEdges*2);
	}
	memArcs = new char[arcMemsize];
	memset(memArcs, 0, (unsigned long long)sizeof(char)*arcMemsize);
	tmpEdges = (TmpEdge*)(memArcs);
	tmpEdgeLast = tmpEdges; // will advance as edges are added
	tmpArcs = (TmpArc*)(memArcs +arcMemsize - (unsigned long long)sizeof(TmpArc)*(unsigned long long)(numEdges*2));
	arcs = (Arc*)memArcs;
	arcEnd = arcs + numEdges * 2;

	// init members
	flow = 0;

}


void IBFSGraph::initGraphFast()
{
	Node *x;
	Arc *a;
	TmpArc *ta, *taEnd;
	TmpEdge *te;

	// tmpEdges:			edges read
	// node.label:			out degree

	// calculate start arc offsets every node
	nodes->firstArc = (Arc*)(tmpArcs);
	for (x=nodes; x != nodeEnd; x++) {
		(x+1)->firstArc = (Arc*)(((TmpArc*)(x->firstArc)) + x->label);
		x->label = ((TmpArc*)(x->firstArc))-tmpArcs;
	}
	nodeEnd->label = arcEnd-arcs;

	// tmpEdges:				edges read
	// node.label: 				index into arcs array of first out arc
	// node.firstArc-tmpArcs: 	index into arcs array of next out arc to be allocated
	//							(initially the first out arc)

	// copy to temp arcs memory
	for (te=tmpEdges; te != tmpEdgeLast; te++) {
		ta = (TmpArc*)(te->tail->firstArc);
		ta->cap = te->cap;
		ta->rev = (TmpArc*)(te->head->firstArc);

		ta = (TmpArc*)(te->head->firstArc);
		ta->cap = te->revCap;
		ta->rev = (TmpArc*)(te->tail->firstArc);

		te->tail->firstArc = (Arc*)((TmpArc*)(te->tail->firstArc)+1);
		te->head->firstArc = (Arc*)((TmpArc*)(te->head->firstArc)+1);
	}

	// tmpEdges:				edges read
	// tmpArcs:					arcs with reverse pointer but no node id
	// node.label: 				index into arcs array of first out arc
	// node.firstArc-tmpArcs: 	index into arcs array of last allocated out arc

	// copy to permanent arcs array, but saving tail instead of head
	a = arcs;
	x = nodes;
	taEnd = (tmpArcs+(arcEnd-arcs));
	for (ta=tmpArcs; ta != taEnd; ta++) {
		while (x->label <= (ta-tmpArcs)) x++;
		a->head = (x-1);
		a->rCap = ta->cap;
		a->rev = arcs + (ta->rev-tmpArcs);
    // printf("num = %ld\n", ta->rev-tmpArcs);
    // printf("cap = %f\n", a->rev->rCap);
		a++;
	}

	// tmpEdges:				overwritten
	// tmpArcs:					overwritten
	// arcs:					arcs array
	// node.label: 				index into arcs array of first out arc
	// node.firstArc-tmpArcs: 	index into arcs array of last allocated out arc
	// arc.head = tail of arc

	// swap the head and tail pointers and set isRevResidual
	for (a=arcs; a != arcEnd; a++) {
		if (a->rev <= a) continue;
		x = a->head;
		a->head = a->rev->head;
		a->rev->head = x;
		a->isRevResidual = (a->rev->rCap != 0);
		a->rev->isRevResidual = (a->rCap != 0);
	}

	// set firstArc pointers in nodes array
	for (x=nodes; x <= nodeEnd; x++) {
		x->firstArc = (arcs + x->label);
		if (x->excess == 0) {
			x->label = numNodes;
			continue;
		}
		if (x->excess > 0) {
			x->label = 1;
			activeS1.add(x);
		} else {
			x->label = -1;
			activeT1.add(x);
		}
	}
}

template<bool sTree> void IBFSGraph::augmentTree(Node *x, double bottleneck)
{
	Node *y;
	Arc *a;

	for (; ; x=a->head)
	{
		if (x->excess) break;
		a = x->parent;
		if (sTree) {
			a->rCap += bottleneck;
			a->rev->isRevResidual = 1;
			a->rev->rCap -= bottleneck;
		} else {
			a->rev->rCap += bottleneck;
			a->isRevResidual = 1;
			a->rCap -= bottleneck;
		}

		// saturated?
		if ((sTree ? (a->rev->rCap) : (a->rCap)) == 0)
		{
			if (sTree) a->isRevResidual = 0;
			else a->rev->isRevResidual = 0;
			y=x->parent->head->firstSon;
			if (y == x) {
				x->parent->head->firstSon = x->nextPtr;
			} else {
				for (; y->nextPtr != x; y = y->nextPtr);
				y->nextPtr = x->nextPtr;
			}
			ADD_ORPHAN_FRONT(x);
		}
	}
	x->excess += (sTree ? -bottleneck : bottleneck);
	if (x->excess == 0) {
		ADD_ORPHAN_FRONT(x);
	}
}


void IBFSGraph::augment(Arc *bridge)
{
	Node *x;
	Arc *a;
	double bottleneck, pushesBefore;

	// bottleneck in S
	bottleneck = bridge->rCap;
	for (x=bridge->rev->head; ; x=a->head)
	{
		if (x->excess) break;
		a = x->parent;
		if (bottleneck > a->rev->rCap) {
			bottleneck = a->rev->rCap;
		}
	}
	if (bottleneck > x->excess) {
		bottleneck = x->excess;
	}

	// bottleneck in T
	for (x=bridge->head; ; x=a->head)
	{
		if (x->excess) break;
		a = x->parent;
		if (bottleneck > a->rCap) {
			bottleneck = a->rCap;
		}
	}
	if (bottleneck > (-x->excess)) {
		bottleneck = (-x->excess);
	}

	// augment connecting arc
	bridge->rev->rCap += bottleneck;
	bridge->isRevResidual = 1;
	bridge->rCap -= bottleneck;
	if (bridge->rCap == 0) {
		bridge->rev->isRevResidual = 0;
	}

	// augment T
	augTimestamp++;
	augmentTree<false>(bridge->head, bottleneck);
	adoption<false>();

	// augment S
	augTimestamp++;
	augmentTree<true>(bridge->rev->head, bottleneck);
	adoption<true>();

	flow += bottleneck;
}




template<bool sTree> void IBFSGraph::adoption()
{
	Node *x, *y, *z;
	Arc *a, *aEnd;
	int minLabel, numOrphans, numOrphansUniq;

	numOrphans=0;
	numOrphansUniq=0;
	while (orphanFirst != IB_ORPHANS_END)
	{
		x = orphanFirst;
		orphanFirst = x->nextPtr;
		//x->nextOrphan = NULL;
		numOrphans++;
		if (x->lastAugTimestamp != augTimestamp) {
			x->lastAugTimestamp = augTimestamp;
			if (sTree) uniqOrphansS++;
			else uniqOrphansT++;
			numOrphansUniq++;
		}

		// check for same level connection
		if (x->isParentCurr) {
			a = x->parent;
		} else {
			a = x->firstArc;
			x->isParentCurr = 1;
		}
		x->parent = NULL;
		aEnd = (x+1)->firstArc;
		if (x->label != (sTree ? 1 : -1))
		{
			minLabel = x->label - (sTree ? 1 : -1);
			for (; a != aEnd; a++)
			{
				y = a->head;
				if ((sTree ? a->isRevResidual : a->rCap) != 0 &&
					y->label == minLabel)
				{
					x->parent = a;
					x->nextPtr = y->firstSon;
					y->firstSon = x;
					break;
				}
			}
		}
		if (x->parent != NULL) continue;

		// give up on same level - relabel it!
		// (1) create orphan sons
		for (y=x->firstSon; y != NULL; y=z)
		{
			z=y->nextPtr;
			ADD_ORPHAN_BACK(y);
		}
		x->firstSon = NULL;

		// on the top level there is no need to relabel
		if (x->label == (sTree ? topLevelS : -topLevelT)) {
			x->label = numNodes;
			continue;
		}

		// (2) relabel: find the lowest level parent
		minLabel = (sTree ? topLevelS : -topLevelT);
		if (x->label != minLabel) for (a=x->firstArc; a != aEnd; a++)
		{
			y = a->head;
			if ((sTree ? a->isRevResidual : a->rCap) &&
				// y->label != numNodes ---> holds implicitly
				(sTree ? (y->label > 0) : (y->label < 0)) &&
				(sTree ? (y->label < minLabel) : (y->label > minLabel)))
			{
				minLabel = y->label;
				x->parent = a;
				if (minLabel == x->label) break;
			}
		}

		// (3) relabel onto new parent
		if (x->parent != NULL) {
			x->label = minLabel + (sTree ? 1 : -1);
			x->nextPtr = x->parent->head->firstSon;
			x->parent->head->firstSon = x;
			// add to active list of the next growth phase
			if (sTree) {
				if (x->label == topLevelS) activeS1.add(x);
			} else {
				if (x->label == -topLevelT) activeT1.add(x);
			}
		} else {
			x->label = numNodes;
		}
	}
}

template<bool dirS> void IBFSGraph::growth()
{
	Node *x, *y;
	Arc *a, *aEnd;

	for (Node **active=active0.list; active != (active0.list + active0.len); active++)
	{
		// get active node
		x = (*active);

		// node no longer at level
		if (x->label != (dirS ? (topLevelS-1): -(topLevelT-1))) {
			continue;
		}

		// grow or augment
		aEnd = (x+1)->firstArc;
		for (a=x->firstArc; a != aEnd; a++)
		{
			if ((dirS ? a->rCap : a->isRevResidual) == 0) continue;
			y = a->head;
			if (y->label == numNodes)
			{
				// grow node
				y->isParentCurr = 0;
				y->label = x->label + (dirS ? 1 : -1);
				y->parent = a->rev;
				y->nextPtr = x->firstSon;
				x->firstSon = y;
				if (dirS) activeS1.add(y);
				else activeT1.add(y);
			}
			else if (dirS ? (y->label < 0) : (y->label > 0))
			{
				// augment
				augment(dirS ? a : (a->rev));
				// if (x->label != (dirS ? (topLevelS-1) : -(topLevelT-1))) {
				// 	break;
				// }
				if (dirS ? (a->rCap) : (a->isRevResidual)) a--;
			}
		}
	}
	active0.clear();
}

double IBFSGraph::computeMaxFlow()
{
	// init
	orphanFirst = IB_ORPHANS_END;
	topLevelS = topLevelT = 1;
	bool dirS = true;
	ActiveList::swapLists(&active0, &activeS1);

	//
	// IBFS
	//
	while (true)
	{
		// BFS level
		if (dirS) topLevelS++;
		else topLevelT++;
		if (dirS) growth<true>();
		else growth<false>();
		// switch to next level
		if (activeS1.len == 0 || activeT1.len == 0) {
			break;
		}
		if (uniqOrphansT == uniqOrphansS && dirS || uniqOrphansT < uniqOrphansS) {
			// grow T
			ActiveList::swapLists(&active0, &activeT1);
			dirS=false;
		} else {
			// grow S
			ActiveList::swapLists(&active0, &activeS1);
			dirS=true;
		}
	}
	
	return flow;
}
