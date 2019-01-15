/*
 * Polygon.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <grid_map_core/Polygon.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>

namespace grid_map {

Polygon::Polygon()
    : timestamp_(0)
{
}

Polygon::Polygon(std::vector<Position> vertices)
    : Polygon()
{
  vertices_ = vertices;
}

Polygon::~Polygon() {}

bool Polygon::isInside(const Position& point) const
{
  int cross = 0;
  for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    if ( ((vertices_[i].y() > point.y()) != (vertices_[j].y() > point.y()))
           && (point.x() < (vertices_[j].x() - vertices_[i].x()) * (point.y() - vertices_[i].y()) /
            (vertices_[j].y() - vertices_[i].y()) + vertices_[i].x()) )
    {
      cross++;
    }
  }
  return bool(cross % 2);
}

void Polygon::addVertex(const Position& vertex)
{
  vertices_.push_back(vertex);
}

const Position& Polygon::getVertex(const size_t index) const
{
  return vertices_.at(index);
}

void Polygon::removeVertices()
{
  vertices_.clear();
}

const Position& Polygon::operator [](const size_t index) const
{
  return getVertex(index);
}

const std::vector<Position>& Polygon::getVertices() const
{
  return vertices_;
}

const size_t Polygon::nVertices() const
{
  return vertices_.size();
}

const std::string& Polygon::getFrameId() const
{
  return frameId_;
}

void Polygon::setFrameId(const std::string& frameId)
{
  frameId_ = frameId;
}

uint64_t Polygon::getTimestamp() const
{
  return timestamp_;
}

void Polygon::setTimestamp(const uint64_t timestamp)
{
  timestamp_ = timestamp;
}

void Polygon::resetTimestamp()
{
  timestamp_ = 0.0;
}

const double Polygon::getArea() const
{
  double area = 0.0;
  int j = vertices_.size() - 1;
  for (int i = 0; i < vertices_.size(); i++) {
    area += (vertices_.at(j).x() + vertices_.at(i).x())
        * (vertices_.at(j).y() - vertices_.at(i).y());
    j = i;
  }
  return std::abs(area / 2.0);
}

Position Polygon::getCentroid() const
{
  Position centroid = Position::Zero();
  std::vector<Position> vertices = getVertices();
  vertices.push_back(vertices.at(0));
  double area = 0.0;
  for (int i = 0; i < vertices.size() - 1; i++) {
    const double a = vertices[i].x() * vertices[i+1].y() - vertices[i+1].x() * vertices[i].y();
    area += a;
    centroid.x() += a * (vertices[i].x() + vertices[i+1].x());
    centroid.y() += a * (vertices[i].y() + vertices[i+1].y());
  }
  area *= 0.5;
  centroid /= (6.0 * area);
  return centroid;
}

void Polygon::getBoundingBox(Position& center, Length& length) const
{
  double minX = std::numeric_limits<double>::infinity();
  double maxX = -std::numeric_limits<double>::infinity();
  double minY = std::numeric_limits<double>::infinity();
  double maxY = -std::numeric_limits<double>::infinity();
  for (const auto& vertex : vertices_) {
    if (vertex.x() > maxX) maxX = vertex.x();
    if (vertex.y() > maxY) maxY = vertex.y();
    if (vertex.x() < minX) minX = vertex.x();
    if (vertex.y() < minY) minY = vertex.y();
  }
  center.x() = (minX + maxX) / 2.0;
  center.y() = (minY + maxY) / 2.0;
  length.x() = (maxX - minX);
  length.y() = (maxY - minY);
}

bool Polygon::convertToInequalityConstraints(Eigen::MatrixXd& A, Eigen::VectorXd& b) const
{
  Eigen::MatrixXd V(nVertices(), 2);
  for (unsigned int i = 0; i < nVertices(); ++i)
    V.row(i) = vertices_[i];

  // Create k, a list of indices from V forming the convex hull.
  // TODO: Assuming counter-clockwise ordered convex polygon.
  // MATLAB: k = convhulln(V);
  Eigen::MatrixXi k;
  k.resizeLike(V);
  for (unsigned int i = 0; i < V.rows(); ++i)
    k.row(i) << i, (i+1) % V.rows();
  Eigen::RowVectorXd c = V.colwise().mean();
  V.rowwise() -= c;
  A = Eigen::MatrixXd::Constant(k.rows(), V.cols(), NAN);

  unsigned int rc = 0;
  for (unsigned int ix = 0; ix < k.rows(); ++ix) {
    Eigen::MatrixXd F(2, V.cols());
    F.row(0) << V.row(k(ix, 0));
    F.row(1) << V.row(k(ix, 1));
    Eigen::FullPivLU<Eigen::MatrixXd> luDecomp(F);
    if (luDecomp.rank() == F.rows()) {
      A.row(rc) = F.colPivHouseholderQr().solve(Eigen::VectorXd::Ones(F.rows()));
      ++rc;
    }
  }

  A = A.topRows(rc);
  b = Eigen::VectorXd::Ones(A.rows());
  b = b + A * c.transpose();

  return true;
}

bool Polygon::thickenLine(const double thickness)
{
  if (vertices_.size() != 2) return false;
  const Vector connection(vertices_[1] - vertices_[0]);
  const Vector orthogonal = thickness * Vector(connection.y(), -connection.x()).normalized();
  std::vector<Position> newVertices;
  newVertices.reserve(4);
  newVertices.push_back(vertices_[0] + orthogonal);
  newVertices.push_back(vertices_[0] - orthogonal);
  newVertices.push_back(vertices_[1] - orthogonal);
  newVertices.push_back(vertices_[1] + orthogonal);
  vertices_ = newVertices;
  return true;
}

bool Polygon::offsetInward(const double margin)
{
  // Create a list of indices of the neighbours of each vertex.
  // TODO: Assuming counter-clockwise ordered convex polygon.
  std::vector<Eigen::Array2i> neighbourIndices;
  const unsigned int n = nVertices();
  neighbourIndices.resize(n);
  for (unsigned int i = 0; i < n; ++i) {
    neighbourIndices[i] << (i > 0 ? (i-1)%n : n-1), (i + 1) % n;
  }

  std::vector<Position> copy(vertices_);
  for (unsigned int i = 0; i < neighbourIndices.size(); ++i) {
    Eigen::Vector2d v1 = vertices_[neighbourIndices[i](0)] - vertices_[i];
    Eigen::Vector2d v2 = vertices_[neighbourIndices[i](1)] - vertices_[i];
    v1.normalize();
    v2.normalize();
    const double angle = acos(v1.dot(v2));
    copy[i] += margin / sin(angle) * (v1 + v2);
  }
  vertices_ = copy;
  return true;
}

std::vector<Polygon> Polygon::triangulate(const TriangulationMethods& method) const
{
  // TODO Add more triangulation methods.
  // https://en.wikipedia.org/wiki/Polygon_triangulation
  std::vector<Polygon> polygons;
  if (vertices_.size() < 3)
    return polygons;

  size_t nPolygons = vertices_.size() - 2;
  polygons.reserve(nPolygons);

  if (nPolygons < 1) {
    // Special case.
    polygons.push_back(*this);
  } else {
    // General case.
    for (size_t i = 0; i < nPolygons; ++i) {
      Polygon polygon({vertices_[0], vertices_[i + 1], vertices_[i + 2]});
      polygons.push_back((polygon));
    }
  }

  return polygons;
}

Polygon Polygon::fromCircle(const Position center, const double radius,
                                  const int nVertices)
{
  Eigen::Vector2d centerToVertex(radius, 0.0), centerToVertexTemp;

  Polygon polygon;
  for (int j = 0; j < nVertices; j++) {
    double theta = j * 2 * M_PI / (nVertices - 1);
    Eigen::Rotation2D<double> rot2d(theta);
    centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
    polygon.addVertex(center + centerToVertexTemp);
  }
  return polygon;
}

Polygon Polygon::convexHullOfTwoCircles(const Position center1,
                                   const Position center2, const double radius,
                                   const int nVertices)
{
  if (center1 == center2) return fromCircle(center1, radius, nVertices);
  Eigen::Vector2d centerToVertex, centerToVertexTemp;
  centerToVertex = center2 - center1;
  centerToVertex.normalize();
  centerToVertex *= radius;

  grid_map::Polygon polygon;
  for (int j = 0; j < ceil(nVertices / 2.0); j++) {
    double theta = M_PI_2 + j * M_PI / (ceil(nVertices / 2.0) - 1);
    Eigen::Rotation2D<double> rot2d(theta);
    centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
    polygon.addVertex(center1 + centerToVertexTemp);
  }
  for (int j = 0; j < ceil(nVertices / 2.0); j++) {
    double theta = 3 * M_PI_2 + j * M_PI / (ceil(nVertices / 2.0) - 1);
    Eigen::Rotation2D<double> rot2d(theta);
    centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
    polygon.addVertex(center2 + centerToVertexTemp);
  }
  return polygon;
}

Polygon Polygon::convexHull(Polygon& polygon1, Polygon& polygon2)
{
  std::vector<Position> vertices;
  vertices.reserve(polygon1.nVertices() + polygon2.nVertices());
  vertices.insert(vertices.end(), polygon1.getVertices().begin(), polygon1.getVertices().end());
  vertices.insert(vertices.end(), polygon2.getVertices().begin(), polygon2.getVertices().end());

  std::vector<Position> hull(vertices.size()+1);

  // Sort points lexicographically.
  std::sort(vertices.begin(), vertices.end(), sortVertices);

  int k = 0;
  // Build lower hull
  for (int i = 0; i < vertices.size(); ++i) {
    while (k >= 2
        && computeCrossProduct2D(hull.at(k - 1) - hull.at(k - 2),
                                 vertices.at(i) - hull.at(k - 2)) <= 0)
      k--;
    hull.at(k++) = vertices.at(i);
  }

  // Build upper hull.
  for (int i = vertices.size() - 2, t = k + 1; i >= 0; i--) {
    while (k >= t
        && computeCrossProduct2D(hull.at(k - 1) - hull.at(k - 2),
                                 vertices.at(i) - hull.at(k - 2)) <= 0)
      k--;
    hull.at(k++) = vertices.at(i);
  }
  hull.resize(k - 1);

  Polygon polygon(hull);
  return polygon;
}

bool Polygon::sortVertices(const Eigen::Vector2d& vector1,
                           const Eigen::Vector2d& vector2)
{
  return (vector1.x() < vector2.x()
      || (vector1.x() == vector2.x() && vector1.y() < vector2.y()));
}

double Polygon::computeCrossProduct2D(const Eigen::Vector2d& vector1,
                                      const Eigen::Vector2d& vector2)
{
  return (vector1.x() * vector2.y() - vector1.y() * vector2.x());
}

} /* namespace grid_map */

