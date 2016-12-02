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

#include "g2o/core/hyper_graph.h"

#include <assert.h>
#include <queue>

namespace g2o {

  HyperGraph::Vertex::Vertex(int id) : _id(id)
  {
  }

  HyperGraph::Vertex::~Vertex()
  {
  }

  HyperGraph::Edge::Edge(int id) : _id(id)
  {
  }

  HyperGraph::Edge::~Edge()
  {
  }

  void HyperGraph::Edge::resize(size_t size)
  {
    _vertices.resize(size, 0);
  }

  void HyperGraph::Edge::setId(int id)
  {
    _id = id;
  }

  HyperGraph::Vertex* HyperGraph::vertex(int id)
  {
    VertexIDMap::iterator it=_vertices.find(id);
    if (it==_vertices.end())
      return 0;
    return it->second;
  }

  const HyperGraph::Vertex* HyperGraph::vertex(int id) const
  {
    VertexIDMap::const_iterator it=_vertices.find(id);
    if (it==_vertices.end())
      return 0;
    return it->second;
  }

  bool HyperGraph::addVertex(Vertex* v)
  {
    Vertex* vn=vertex(v->id());
    if (vn)
      return false;
    _vertices.insert( std::make_pair(v->id(),v) );
    return true;
  }

  /**
   * changes the id of a vertex already in the graph, and updates the bookkeeping
   @ returns false if the vertex is not in the graph;
  */
  bool HyperGraph::changeId(Vertex* v, int newId){
    Vertex* v2 = vertex(v->id());
    if (v != v2)
      return false;
    _vertices.erase(v->id());
    v->setId(newId);
    _vertices.insert(std::make_pair(v->id(), v));
    return true;
  }

  bool HyperGraph::addEdge(Edge* e)
  {
    std::pair<EdgeSet::iterator, bool> result = _edges.insert(e);
    if (! result.second)
      return false;
    for (std::vector<Vertex*>::iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      Vertex* v = *it;
      v->edges().insert(e);
    }
    return true;
  }

  bool HyperGraph::removeVertex(Vertex* v)
  {
    VertexIDMap::iterator it=_vertices.find(v->id());
    if (it==_vertices.end())
      return false;
    assert(it->second==v);
    //remove all edges which are entering or leaving v;
    EdgeSet tmp(v->edges());
    for (EdgeSet::iterator it=tmp.begin(); it!=tmp.end(); ++it){
      if (!removeEdge(*it)){
        assert(0);
      }
    }
    _vertices.erase(it);
    delete v;
    return true;
  }

  bool HyperGraph::removeEdge(Edge* e)
  {
    EdgeSet::iterator it = _edges.find(e);
    if (it == _edges.end())
      return false;
    _edges.erase(it);

    for (std::vector<Vertex*>::iterator vit = e->vertices().begin(); vit != e->vertices().end(); ++vit) {
      Vertex* v = *vit;
      it = v->edges().find(e);
      assert(it!=v->edges().end());
      v->edges().erase(it);
    }

    delete e;
    return true;
  }

  HyperGraph::HyperGraph()
  {
  }

  void HyperGraph::clear()
  {
    for (VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); ++it)
      delete (it->second);
    for (EdgeSet::iterator it=_edges.begin(); it!=_edges.end(); ++it)
      delete (*it);
    _vertices.clear();
    _edges.clear();
  }

  HyperGraph::~HyperGraph()
  {
    clear();
  }

} // end namespace
