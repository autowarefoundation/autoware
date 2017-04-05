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

#ifndef G2O_AIS_GENERAL_DIJKSTRA_HH
#define G2O_AIS_GENERAL_DIJKSTRA_HH

#include <map>
#include <set>
#include <limits>

#include "g2o/core/hyper_graph.h"

namespace g2o{

  struct  HyperDijkstra{
    struct  CostFunction {
      virtual double operator() (HyperGraph::Edge* e, HyperGraph::Vertex* from, HyperGraph::Vertex* to)=0;
      virtual ~CostFunction() { }
    };

    struct  TreeAction {
      virtual double perform(HyperGraph::Vertex* v, HyperGraph::Vertex* vParent, HyperGraph::Edge* e);
      virtual double perform(HyperGraph::Vertex* v, HyperGraph::Vertex* vParent, HyperGraph::Edge* e, double distance);
    };

    
    struct  AdjacencyMapEntry{
      friend struct HyperDijkstra;
      AdjacencyMapEntry(HyperGraph::Vertex* _child=0, 
          HyperGraph::Vertex* _parent=0, 
          HyperGraph::Edge* _edge=0, 
          double _distance=std::numeric_limits<double>::max());
      HyperGraph::Vertex* child() const {return _child;}
      HyperGraph::Vertex* parent() const {return _parent;}
      HyperGraph::Edge* edge() const {return _edge;}
      double distance() const {return _distance;}
      HyperGraph::VertexSet& children() {return _children;}
      const HyperGraph::VertexSet& children() const {return _children;}
      protected:
      HyperGraph::Vertex* _child;
      HyperGraph::Vertex* _parent;
      HyperGraph::Edge* _edge;
      double _distance;
      HyperGraph::VertexSet _children;
    };

    typedef std::map<HyperGraph::Vertex*, AdjacencyMapEntry> AdjacencyMap;
    HyperDijkstra(HyperGraph* g);
    HyperGraph::VertexSet& visited() {return _visited; }
    AdjacencyMap& adjacencyMap() {return _adjacencyMap; }
    HyperGraph* graph() {return _graph;} 

    void shortestPaths(HyperGraph::Vertex* v, 
           HyperDijkstra::CostFunction* cost, 
           double maxDistance=std::numeric_limits< double >::max(), 
           double comparisonConditioner=1e-3, 
           bool directed=false,
           double maxEdgeCost=std::numeric_limits< double >::max());

    void shortestPaths(HyperGraph::VertexSet& vset, 
           HyperDijkstra::CostFunction* cost, 
           double maxDistance=std::numeric_limits< double >::max(), 
           double comparisonConditioner=1e-3, 
           bool directed=false,
           double maxEdgeCost=std::numeric_limits< double >::max());


    static void computeTree(AdjacencyMap& amap);
    static void visitAdjacencyMap(AdjacencyMap& amap, TreeAction* action, bool useDistance=false);
    static void connectedSubset(HyperGraph::VertexSet& connected, HyperGraph::VertexSet& visited, 
           HyperGraph::VertexSet& startingSet, 
           HyperGraph* g, HyperGraph::Vertex* v,
           HyperDijkstra::CostFunction* cost, double distance, double comparisonConditioner,
           double maxEdgeCost=std::numeric_limits< double >::max() );

  protected:
    void reset();

    AdjacencyMap _adjacencyMap;
    HyperGraph::VertexSet _visited;
    HyperGraph* _graph;
  };

  struct  UniformCostFunction: public HyperDijkstra::CostFunction {
    virtual double operator ()(HyperGraph::Edge* edge, HyperGraph::Vertex* from, HyperGraph::Vertex* to);
  };

}
#endif
