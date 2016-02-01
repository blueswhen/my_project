#ifndef _IBFS_H__
#define _IBFS_H__

#include <stdio.h>
#include <string.h>

#define IB_EXCESSES 1
#define IB_ALLOC_INIT_LEVELS 4096

class IBFSGraph
{
public:
  enum IBFSInitMode { IB_INIT_FAST, IB_INIT_COMPACT };
  IBFSGraph(IBFSInitMode initMode);
  ~IBFSGraph();
  void initSize(int numNodes, int numEdges);
  void addEdge(int nodeIndexFrom, int nodeIndexTo, double capacity, double reverseCapacity);
  void addNode(int nodeIndex, double capFromSource, double capToSink);
  struct Arc;
  void initGraph();
  double computeMaxFlow();
  double computeMaxFlow(bool allowIncrements);
  int isNodeOnSrcSide(int nodeIndex, int freeNodeValue = 0);


  struct Node;

  struct Arc
  {
    Node*    head;
    Arc*    rev;
    int      isRevResidual :1;
    double  rCap;
  };

  struct Node
  {
    int      id;
    int      lastAugTimestamp:30;
    int      isParentCurr:1;
    int      isIncremental:1;
    Arc      *firstArc;
    Arc      *parent;
    Node    *firstSon;
    Node    *nextPtr;
    int      label;  // label > 0: distance from s, label < 0: -distance from t
    double   excess;   // excess > 0: capacity from s, excess < 0: -capacity to t
  };

private:
  Arc *arcIter;
  void augment(Arc *bridge);
  template<bool sTree> int augmentPath(Node *x, double push);
  template<bool sTree> int augmentExcess(Node *x, double push);
  template<bool sTree> void augmentExcesses();
  template <bool sTree> void adoption(int fromLevel, bool toTop);
  template <bool dirS> void growth();

  double computeMaxFlow(bool trackChanges, bool initialDirS);

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
    inline void init(Node **mem) {
      list = mem;
      len = 0;
    }
    inline void free() {
      if (list != NULL) {
        delete []list;
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
    inline Node* pop() {
      len--;
      return list[len];
    }
    inline Node** getEnd() {
      return list+len;
    }
    inline static void swapLists(ActiveList *a, ActiveList *b) {
      ActiveList tmp = (*a);
      (*a) = (*b);
      (*b) = tmp;
    }
    Node **list;
    int len;
  };


  class BucketsOneSided
  {
  public:
    inline BucketsOneSided() {
      buckets = NULL;
      maxBucket = 0;
      allocLevels = 0;
    }
    inline void init(int numNodes) {
      allocLevels = numNodes/8;
      if (allocLevels < IB_ALLOC_INIT_LEVELS) {
        if (numNodes < IB_ALLOC_INIT_LEVELS) allocLevels = numNodes;
        else allocLevels = IB_ALLOC_INIT_LEVELS;
      }
      buckets = new Node*[allocLevels+1];
      memset(buckets, 0, sizeof(Node*)*(allocLevels+1));
      maxBucket = 0;
    }
    inline void allocate(int numLevels) {
      if (numLevels > allocLevels) {
        allocLevels <<= 1;
        Node **alloc = new Node*[allocLevels+1];
        memset(alloc, 0, sizeof(Node*)*(allocLevels+1));
        delete []buckets;
        buckets = alloc;
      }
    }
    inline void free() {
      if (buckets != NULL) {
        delete []buckets;
        buckets = NULL;
      }
    }
    template <bool sTree> inline void add(Node* x) {
      int bucket = (sTree ? (x->label) : (-x->label));
      x->nextPtr = buckets[bucket];
      buckets[bucket] = x;
      if (bucket > maxBucket) maxBucket = bucket;
    }
    inline Node* popFront(int bucket) {
      Node *x;
      if ((x = buckets[bucket]) == NULL) return NULL;
      buckets[bucket] = x->nextPtr;
      return x;
    }

    Node **buckets;
    int maxBucket;
    int allocLevels;
  };

#define IB_PREVPTR_EXCESS(x) (ptrs[(((x)->id)<<1) + 1])
#define IB_NEXTPTR_EXCESS(x) (ptrs[((x)->id)<<1])

  class ExcessBuckets
  {
  public:
    inline ExcessBuckets() {
      buckets = ptrs = NULL;
      allocLevels = maxBucket = minBucket = -1;
    }
    inline void init(int numNodes) {
      allocLevels = numNodes/8;
      if (allocLevels < IB_ALLOC_INIT_LEVELS) {
        if (numNodes < IB_ALLOC_INIT_LEVELS) allocLevels = numNodes;
        else allocLevels = IB_ALLOC_INIT_LEVELS;
      }
      buckets = new Node*[allocLevels+1];
      memset(buckets, 0, sizeof(Node*)*(allocLevels+1));
      ptrs = new Node*[2 * numNodes];
      memset(ptrs, 0, sizeof(Node*)*(2 * numNodes));
      reset();
    }
    inline void allocate(int numLevels) {
      if (numLevels > allocLevels) {
        allocLevels <<= 1;
        Node **alloc = new Node*[allocLevels+1];
        memset(alloc, 0, sizeof(Node*)*(allocLevels+1));
        //memcpy(alloc, buckets, sizeof(Node*)*(allocLevels+1));
        delete []buckets;
        buckets = alloc;
      }
    }
    inline void free() {
      if (buckets != NULL) {
        delete []buckets;
        buckets = NULL;
      }
      if (ptrs != NULL) {
        delete [] ptrs;
        ptrs = NULL;
      }
    }

    template <bool sTree> inline void add(Node* x) {
      int bucket = (sTree ? (x->label) : (-x->label));
      IB_NEXTPTR_EXCESS(x) = buckets[bucket];
      if (buckets[bucket] != NULL) {
        IB_PREVPTR_EXCESS(buckets[bucket]) = x;
      }
      buckets[bucket] = x;
      if (bucket > maxBucket) maxBucket = bucket;
      if (bucket != 0 && bucket < minBucket) minBucket = bucket;
    }
    inline Node* popFront(int bucket) {
      Node *x = buckets[bucket];
      if (x == NULL) return NULL;
      buckets[bucket] = IB_NEXTPTR_EXCESS(x);
      return x;
    }
    template <bool sTree> inline void remove(Node *x) {
      int bucket = (sTree ? (x->label) : (-x->label));
      if (buckets[bucket] == x) {
        buckets[bucket] = IB_NEXTPTR_EXCESS(x);
      } else {
        IB_NEXTPTR_EXCESS(IB_PREVPTR_EXCESS(x)) = IB_NEXTPTR_EXCESS(x);
        if (IB_NEXTPTR_EXCESS(x) != NULL) IB_PREVPTR_EXCESS(IB_NEXTPTR_EXCESS(x)) = IB_PREVPTR_EXCESS(x);
      }
    }
    inline void incMaxBucket(int bucket) {
      if (maxBucket < bucket) maxBucket = bucket;
    }
    inline bool empty() {
      return maxBucket < minBucket;
    }
    inline void reset() {
      maxBucket = 0;
      minBucket = -1 ^ (1<<31);
    }

    Node **buckets;
    Node **ptrs;
    int maxBucket;
    int minBucket;
    int allocLevels;
  };

  // members
  Node  *nodes, *nodeEnd;
  Arc    *arcs, *arcEnd;
  Node  **ptrs;
  int   numNodes;
  double flow;
  short   augTimestamp;
  int topLevelS, topLevelT;
  ActiveList active0, activeS1, activeT1;
  BucketsOneSided orphanBuckets;
  ExcessBuckets excessBuckets;
  //
  // Orphans
  //
  unsigned int uniqOrphansS, uniqOrphansT;
  template <bool sTree> inline void orphanFree(Node *x) {
    if (IB_EXCESSES && x->excess) {
      x->label = (sTree ? -topLevelT : topLevelS);
      if (sTree) activeT1.add(x);
      else activeS1.add(x);
      x->isParentCurr = 0;
    } else {
      x->label = 0;
    }
  }

  //
  // Initialization
  //
  struct TmpEdge
  {
    int    head;
    int    tail;
    double cap;
    double revCap;
  };
  struct TmpArc
  {
    TmpArc    *rev;
    double cap;
  };
  char  *memArcs;
  TmpEdge  *tmpEdges, *tmpEdgeLast;
  TmpArc  *tmpArcs;
  bool isInitializedGraph() {
    return memArcs != NULL;
  }
  IBFSInitMode initMode;
  int m_orphan_count;
  int m_path;
  void initGraphFast();
  void initNodes();
};

inline void IBFSGraph::addNode(int nodeIndex, double capSource, double capSink)
{
  int f = nodes[nodeIndex].excess;
  if (f > 0) {
    capSource += f;
  } else {
    capSink -= f;
  }
  if (capSource < capSink) {
    flow += capSource;
  } else {
    flow += capSink;
  }
  nodes[nodeIndex].excess = capSource - capSink;
  nodes[nodeIndex].id = nodeIndex;
}

inline void IBFSGraph::addEdge(int nodeIndexFrom, int nodeIndexTo, double capacity, double reverseCapacity)
{
  tmpEdgeLast->tail = nodeIndexFrom;
  tmpEdgeLast->head = nodeIndexTo;
  tmpEdgeLast->cap = capacity;
  tmpEdgeLast->revCap = reverseCapacity;
  tmpEdgeLast++;

  // use label as a temporary storage
  // to count the out degree of nodes
  nodes[nodeIndexFrom].label++;
  nodes[nodeIndexTo].label++;
}

inline int IBFSGraph::isNodeOnSrcSide(int nodeIndex, int freeNodeValue)
{
  if (nodes[nodeIndex].label == 0) {
    return freeNodeValue;
  }
  return (nodes[nodeIndex].label > 0 ? 1 : 0);
}
#endif
