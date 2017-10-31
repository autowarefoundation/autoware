#include "astar_util.h"


WaveFrontNode::WaveFrontNode()
{
}

WaveFrontNode::WaveFrontNode(int x, int y, double cost)
  : index_x(x)
  , index_y(y)
  , hc(cost)
{
}

SimpleNode::SimpleNode()
{
}

SimpleNode::SimpleNode(int x, int y, int theta, double gc, double hc)
  : index_x(x)
  , index_y(y)
  , index_theta(theta)
  , cost(gc + hc)
{
}
