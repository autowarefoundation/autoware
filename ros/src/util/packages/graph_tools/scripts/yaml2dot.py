#!/usr/bin/env python
#-*- coding: utf-8 -*-

import sys
import os
import argparse
import subprocess
import yaml
import pygraphviz
from datetime import datetime

def get_rospack():
  cwd = os.getcwd()
  lines = subprocess.check_output(['rospack', 'list']).strip().split("\n")
  pkgs = {}
  for line in lines:
    n = line.split(' ')
    f = n[1] + os.sep + 'interface.yaml'
    if cwd in n[1] and os.path.exists(f):
      pkgs[f] = n[0]  # key:path value:name
  return pkgs

def get_cluster(name, clusters):
  for n in clusters:
    if name in clusters[n]:
      return n
  return ""

def check_topics(topics, pkg, data):
  for t in data:
    if t in topics:
      if pkg not in topics[t]:
        topics[t].append(pkg)
    else:
      topics[t] = [pkg]

def get_topic_info(conf):
  ret = [{}, {}] # publish, subscribe
  for pname in conf:
    pkg = conf[pname]
    if not 'nodes' in pkg:
      continue
    name = pkg['name']
    for node in pkg['nodes']:
      if 'publish' in node:
        check_topics(ret[0], name, node['publish'])
      if 'subscribe' in node:
        check_topics(ret[1], name, node['subscribe'])
  return ret

def is_inner_topic(topic, topics):
  return topic in topics[0] and len(topics[0][topic]) == 1 and \
         topic in topics[1] and len(topics[1][topic]) == 1

def is_one_leaf(topic, topics):
  len0 = len(topics[0][topic]) if topic in topics[0] else 0
  len1 = len(topics[1][topic]) if topic in topics[1] else 0
  return len0 + len1 == 1


if __name__ == "__main__":
  # parse args
  parser = argparse.ArgumentParser(description='Convert YAML into DOT')
  parser.add_argument('input_file', nargs='*', type=str,
    help='input yaml files')
  parser.add_argument('-f', '--format', default='pdf', type=str,
    help='output format')
  ofile = os.path.expanduser(
    datetime.now().strftime('~/.autoware/autoware-graph-%Y%m%d.pdf'))
  parser.add_argument('-o', '--output', default=ofile,
    type=argparse.FileType('w'), help='output file')
  parser.add_argument('-d', '--detail', const=True, default=False, nargs='?',
    help='show edges between nodes')
  args = parser.parse_args()

  # initialize input files
  if len(args.input_file) == 0:
    pkgs = get_rospack()
    fargs = pkgs.keys()
  else:
    pkgs = {}
    fargs = args.input_file

  # initialize variables
  graph0 = pygraphviz.AGraph(directed=True, rankdir='LR', compound=True)
  graph_nr = 0
  node_nr = 0

  # read yaml files
  conf = {}
  for rfile in fargs:
    cname = "cluster%d" % graph_nr
    pkg = pkgs[rfile] if rfile in pkgs else cname
    nodes = yaml.load(file(rfile))
    conf[pkg] = {}
    conf[pkg]['file'] = rfile
    conf[pkg]['name'] = cname
    conf[pkg]['nodes'] = nodes
    graph_nr += 1
  topics = get_topic_info(conf)

  # create graph
  for pname in conf:
    pkg = conf[pname]
    rfile = pkg['file']
    cname = pkg['name']
    graph = graph0.add_subgraph(name=cname)
    if rfile in pkgs:
      graph.graph_attr['label'] = '(' + pkgs[rfile] + ')'
    graph.graph_attr['color'] = 'grey'
    n0 = None
    for node in pkg['nodes']:
      if n0 is None:
        n0 = node['name']
      graph.add_node(node['name'], shape='ellipse')
      node_nr += 1
      if 'publish' in node:
        for pub in node['publish']:
          if is_inner_topic(pub, topics):
            if not graph.has_node(pub):
              graph.add_node(pub, shape='box')
            graph.add_edge(node['name'], pub)
          elif args.detail or not is_one_leaf(pub, topics):
            if not graph0.has_node(pub):
              graph0.add_node(pub, shape='box')
            if args.detail:
              graph0.add_edge(node['name'], pub)
            else:
              if not graph0.has_edge(n0, pub):
                graph0.add_edge(n0, pub, ltail=cname)
      if 'subscribe' in node:
        for sub in node['subscribe']:
          if is_inner_topic(sub, topics):
            if not graph.has_node(sub):
              graph.add_node(sub, shape='box')
            graph.add_edge(sub, node['name'])
          elif args.detail or not is_one_leaf(sub, topics):
            if not graph0.has_node(sub):
              graph0.add_node(sub, shape='box')
            if args.detail:
              graph0.add_edge(sub, node['name'])
            else:
              if not graph0.has_edge(sub, n0):
                graph0.add_edge(sub, n0, lhead=cname)

  # output
  print "# output", args.output
  print "# format", args.format
  graph0.draw(path=args.output, format=args.format, prog='dot')
  #print "## ", reduce(lambda a,b: a + "\n### " + b, sorted(graph.nodes()))
  print "# total file#=%d node#=%d topic#=%d" % \
    (graph_nr, node_nr, len(set(reduce(lambda m,n:m.keys()+n.keys(), topics))))
