// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_ESTIMATE_PROPAGATOR_H
#define G2O_ESTIMATE_PROPAGATOR_H

#include <map>
#include <set>
#include <limits>
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"

#ifdef _MSC_VER
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif

namespace g2o {

  /**
   * \brief cost for traversing along active edges in the optimizer
   *
   * You may derive an own one, if necessary. The default is to return initialEstimatePossible(from, to) for the edge.
   */
  class  EstimatePropagatorCost {
    public:
      EstimatePropagatorCost (SparseOptimizer* graph);
      virtual double operator()(OptimizableGraph::Edge* edge, const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to_) const;
      virtual const char* name() const { return "spanning tree";}
    protected:
      SparseOptimizer* _graph;
  };

  /**
   * \brief cost for traversing only odometry edges.
   *
   * Initialize your graph along odometry edges. An odometry edge is assumed to connect vertices
   * whose IDs only differs by one.
   */
  class  EstimatePropagatorCostOdometry : public EstimatePropagatorCost {
    public:
      EstimatePropagatorCostOdometry(SparseOptimizer* graph);
      virtual double operator()(OptimizableGraph::Edge* edge, const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* to_) const;
      virtual const char* name() const { return "odometry";}
  };

  /**
   * \brief propagation of an initial guess
   */
  class  EstimatePropagator {
    public:

      /**
       * \brief Applying the action for propagating.
       *
       * You may derive an own one, if necessary. The default is to call initialEstimate(from, to) for the edge.
       */
      struct PropagateAction {
        virtual void operator()(OptimizableGraph::Edge* e, const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) const
        {
          if (! to->fixed())
            e->initialEstimate(from, to);
        }
      };

      typedef EstimatePropagatorCost PropagateCost;

      class AdjacencyMapEntry;

      /**
       * \brief priority queue for AdjacencyMapEntry
       */
      class PriorityQueue : public std::multimap<double, AdjacencyMapEntry*> {
        public:
          void push(AdjacencyMapEntry* entry);
          AdjacencyMapEntry* pop();
      };

      /**
       * \brief data structure for loopuk during Dijkstra
       */
      class AdjacencyMapEntry {
        public:
          friend class EstimatePropagator;
          friend class PriorityQueue;
          AdjacencyMapEntry();
          void reset();
          OptimizableGraph::Vertex* child() const {return _child;}
          const OptimizableGraph::VertexSet& parent() const {return _parent;}
          OptimizableGraph::Edge* edge() const {return _edge;}
          double distance() const {return _distance;}
          int frontierLevel() const { return _frontierLevel;}

        protected:
          OptimizableGraph::Vertex* _child;
          OptimizableGraph::VertexSet _parent;
          OptimizableGraph::Edge* _edge;
          double _distance;
          int _frontierLevel;
        private: // for PriorityQueue
          bool inQueue;
          PriorityQueue::iterator queueIt;
      };

      /**
       * \brief hash function for a vertex
       */
      class VertexIDHashFunction {
        public:
          size_t operator ()(const OptimizableGraph::Vertex* v) const { return v->id();}
      };

      typedef std::tr1::unordered_map<OptimizableGraph::Vertex*, AdjacencyMapEntry, VertexIDHashFunction> AdjacencyMap;

    public:
      EstimatePropagator(OptimizableGraph* g);
      OptimizableGraph::VertexSet& visited() {return _visited; }
      AdjacencyMap& adjacencyMap() {return _adjacencyMap; }
      OptimizableGraph* graph() {return _graph;} 

      /**
       * propagate an initial guess starting from v. The function computes a spanning tree
       * whereas the cost for each edge is determined by calling cost() and the action applied to
       * each vertex is action().
       */
      void propagate(OptimizableGraph::Vertex* v, 
          const EstimatePropagator::PropagateCost& cost, 
          const EstimatePropagator::PropagateAction& action = PropagateAction(),
          double maxDistance=std::numeric_limits<double>::max(), 
          double maxEdgeCost=std::numeric_limits<double>::max());

      /**
       * same as above but starting to propagate from a set of vertices instead of just a single one.
       */
      void propagate(OptimizableGraph::VertexSet& vset, 
          const EstimatePropagator::PropagateCost& cost, 
          const EstimatePropagator::PropagateAction& action = PropagateAction(),
          double maxDistance=std::numeric_limits<double>::max(), 
          double maxEdgeCost=std::numeric_limits<double>::max());

    protected:
      void reset();

      AdjacencyMap _adjacencyMap;
      OptimizableGraph::VertexSet _visited;
      OptimizableGraph* _graph;
  };

}
#endif
