// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tracking_object_merger/data_association/solver/successive_shortest_path.hpp"

#include <algorithm>
#include <cassert>
#include <functional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gnn_solver
{
struct ResidualEdge
{
  // Destination node
  const int dst;
  int capacity;
  const double cost;
  int flow;
  // Access to the reverse edge by adjacency_list.at(dst).at(reverse)
  const int reverse;

  // ResidualEdge()
  // : dst(0), capacity(0), cost(0), flow(0), reverse(0) {}

  ResidualEdge(int dst, int capacity, double cost, int flow, int reverse)
  : dst(dst), capacity(capacity), cost(cost), flow(flow), reverse(reverse)
  {
  }
};

void SSP::maximizeLinearAssignment(
  const std::vector<std::vector<double>> & cost, std::unordered_map<int, int> * direct_assignment,
  std::unordered_map<int, int> * reverse_assignment)
{
  // NOTE: Need to set as default arguments
  bool sparse_cost = true;
  // bool sparse_cost = false;

  // Hyperparameters
  // double MAX_COST = 6;
  const double MAX_COST = 10;
  const double INF_DIST = 10000000;
  const double EPS = 1e-5;

  // When there is no agents or no tasks, terminate
  if (cost.size() == 0 || cost.at(0).size() == 0) {
    return;
  }

  // Construct a bipartite graph from the cost matrix
  int n_agents = cost.size();
  int n_tasks = cost.at(0).size();

  int n_dummies;
  if (sparse_cost) {
    n_dummies = n_agents;
  } else {
    n_dummies = 0;
  }

  int source = 0;
  int sink = n_agents + n_tasks + 1;
  int n_nodes = n_agents + n_tasks + n_dummies + 2;

  // // Print cost matrix
  // std::cout << std::endl;
  // for (int agent = 0; agent < n_agents; agent++)
  // {
  //   for (int task = 0; task < n_tasks; task++)
  //   {
  //     std::cout << cost.at(agent).at(task) << " ";
  //   }
  //   std::cout << std::endl;
  // }

  // std::chrono::system_clock::time_point start_time, end_time;
  // start_time = std::chrono::system_clock::now();

  // Adjacency list of residual graph (index: nodes)
  //     - 0: source node
  //     - {1, ...,  n_agents}: agent nodes
  //     - {n_agents+1, ...,  n_agents+n_tasks}: task nodes
  //     - n_agents+n_tasks+1: sink node
  //     - {n_agents+n_tasks+2, ..., n_agents+n_tasks+1+n_agents}:
  //       dummy node (when sparse_cost is true)
  std::vector<std::vector<ResidualEdge>> adjacency_list(n_nodes);

  // Reserve memory
  for (int v = 0; v < n_nodes; ++v) {
    if (v == source) {
      // Source
      adjacency_list.at(v).reserve(n_agents);
    } else if (v <= n_agents) {
      // Agents
      adjacency_list.at(v).reserve(n_tasks + 1 + 1);
    } else if (v <= n_agents + n_tasks) {
      // Tasks
      adjacency_list.at(v).reserve(n_agents + 1);
    } else if (v == sink) {
      // Sink
      adjacency_list.at(v).reserve(n_tasks + n_dummies);
    } else {
      // Dummies
      adjacency_list.at(v).reserve(2);
    }
  }

  // Add edges form source
  for (int agent = 0; agent < n_agents; ++agent) {
    // From source to agent
    adjacency_list.at(source).emplace_back(agent + 1, 1, 0, 0, adjacency_list.at(agent + 1).size());
    // From agent to source
    adjacency_list.at(agent + 1).emplace_back(
      source, 0, 0, 0, adjacency_list.at(source).size() - 1);
  }

  // Add edges from agents
  for (int agent = 0; agent < n_agents; ++agent) {
    for (int task = 0; task < n_tasks; ++task) {
      if (!sparse_cost || cost.at(agent).at(task) > EPS) {
        // From agent to task
        adjacency_list.at(agent + 1).emplace_back(
          task + n_agents + 1, 1, MAX_COST - cost.at(agent).at(task), 0,
          adjacency_list.at(task + n_agents + 1).size());

        // From task to agent
        adjacency_list.at(task + n_agents + 1)
          .emplace_back(
            agent + 1, 0, cost.at(agent).at(task) - MAX_COST, 0,
            adjacency_list.at(agent + 1).size() - 1);
      }
    }
  }

  // Add edges form tasks
  for (int task = 0; task < n_tasks; ++task) {
    // From task to sink
    adjacency_list.at(task + n_agents + 1)
      .emplace_back(sink, 1, 0, 0, adjacency_list.at(sink).size());

    // From sink to task
    adjacency_list.at(sink).emplace_back(
      task + n_agents + 1, 0, 0, 0, adjacency_list.at(task + n_agents + 1).size() - 1);
  }

  // Add edges from dummy
  if (sparse_cost) {
    for (int agent = 0; agent < n_agents; ++agent) {
      // From agent to dummy
      adjacency_list.at(agent + 1).emplace_back(
        agent + n_agents + n_tasks + 2, 1, MAX_COST, 0,
        adjacency_list.at(agent + n_agents + n_tasks + 2).size());

      // From dummy to agent
      adjacency_list.at(agent + n_agents + n_tasks + 2)
        .emplace_back(agent + 1, 0, -MAX_COST, 0, adjacency_list.at(agent + 1).size() - 1);

      // From dummy to sink
      adjacency_list.at(agent + n_agents + n_tasks + 2)
        .emplace_back(sink, 1, 0, 0, adjacency_list.at(sink).size());

      // From sink to dummy
      adjacency_list.at(sink).emplace_back(
        agent + n_agents + n_tasks + 2, 0, 0, 0,
        adjacency_list.at(agent + n_agents + n_tasks + 2).size() - 1);
    }
  }

  // Maximum flow value
  const int max_flow = std::min(n_agents, n_tasks);

  // Feasible potentials
  std::vector<double> potentials(n_nodes, 0);

  // Shortest path lengths
  std::vector<double> distances(n_nodes, INF_DIST);

  // Whether previously visited the node or not
  std::vector<bool> is_visited(n_nodes, false);

  // Parent node (<prev_node, edge_index>)
  std::vector<std::pair<int, int>> prev_values(n_nodes);

  for (int i = 0; i < max_flow; ++i) {
    // Initialize priority queue (<distance, node>)
    std::priority_queue<
      std::pair<double, int>, std::vector<std::pair<double, int>>,
      std::greater<std::pair<double, int>>>
      p_queue;

    // Reset all trajectory states
    if (i > 0) {
      std::fill(distances.begin(), distances.end(), INF_DIST);
      std::fill(is_visited.begin(), is_visited.end(), false);
    }

    // Start trajectory from the source node
    p_queue.push(std::make_pair(0, source));
    distances.at(source) = 0;

    while (!p_queue.empty()) {
      // Get the next element
      std::pair<double, int> cur_elem = p_queue.top();
      // std::cout << "[pop]: (" << cur_elem.first << ", " << cur_elem.second << ")" << std::endl;
      p_queue.pop();

      double cur_node_dist = cur_elem.first;
      int cur_node = cur_elem.second;

      // If already visited node, skip and continue
      if (is_visited.at(cur_node)) {
        continue;
      }
      assert(cur_node_dist == distances.at(cur_node));

      // Mark as visited
      is_visited.at(cur_node) = true;
      // Update potential
      potentials.at(cur_node) += cur_node_dist;

      // When reached to the sink node, terminate.
      if (cur_node == sink) {
        break;
      }

      // Loop over the incident nodes(/edges)
      for (auto it_incident_edge = adjacency_list.at(cur_node).cbegin();
           it_incident_edge != adjacency_list.at(cur_node).cend(); it_incident_edge++) {
        // If the node is not visited and have capacity to increase flow, visit.
        if (!is_visited.at(it_incident_edge->dst) && it_incident_edge->capacity > 0) {
          // Calculate reduced cost
          double reduced_cost =
            it_incident_edge->cost + potentials.at(cur_node) - potentials.at(it_incident_edge->dst);
          assert(reduced_cost >= 0);
          if (distances.at(it_incident_edge->dst) > reduced_cost) {
            distances.at(it_incident_edge->dst) = reduced_cost;
            prev_values.at(it_incident_edge->dst) =
              std::make_pair(cur_node, it_incident_edge - adjacency_list.at(cur_node).cbegin());
            // std::cout << "[push]: (" << reduced_cost << ", " << next_v << ")" << std::endl;
            p_queue.push(std::make_pair(reduced_cost, it_incident_edge->dst));
          }
        }
      }
    }

    // Shortest path length to sink is greater than MAX_COST,
    // which means no non-dummy routes left, terminate
    if (potentials.at(sink) >= MAX_COST) {
      break;
    }

    // Update potentials of unvisited nodes
    for (int v = 0; v < n_nodes; ++v) {
      if (!is_visited.at(v)) {
        potentials.at(v) += distances.at(sink);
      }
    }
    // //Print potentials
    // for (int v = 0; v < n_nodes; ++v)
    // {
    //   std::cout << potentials.at(v) << ", ";
    // }
    // std::cout << std::endl;

    // Increase/decrease flow and capacity along the shortest path from the source to the sink
    int v = sink;
    int prev_v;
    while (v != source) {
      ResidualEdge & e_forward =
        adjacency_list.at(prev_values.at(v).first).at(prev_values.at(v).second);
      assert(e_forward.dst == v);
      ResidualEdge & e_backward = adjacency_list.at(v).at(e_forward.reverse);
      prev_v = e_backward.dst;

      if (e_backward.flow == 0) {
        // Increase flow
        // State A
        assert(e_forward.capacity == 1);
        assert(e_forward.flow == 0);
        assert(e_backward.capacity == 0);
        assert(e_backward.flow == 0);

        e_forward.capacity -= 1;
        e_forward.flow += 1;
        e_backward.capacity += 1;

        // State B
        assert(e_forward.capacity == 0);
        assert(e_forward.flow == 1);
        assert(e_backward.capacity == 1);
        assert(e_backward.flow == 0);
      } else {
        // Decrease flow
        // State B
        assert(e_forward.capacity == 1);
        assert(e_forward.flow == 0);
        assert(e_backward.capacity == 0);
        assert(e_backward.flow == 1);

        e_forward.capacity -= 1;
        e_backward.capacity += 1;
        e_backward.flow -= 1;

        // State A
        assert(e_forward.capacity == 0);
        assert(e_forward.flow == 0);
        assert(e_backward.capacity == 1);
        assert(e_backward.flow == 0);
      }

      v = prev_v;
    }

#ifndef NDEBUG
    // Check if the potentials are feasible potentials
    for (int v = 0; v < n_nodes; ++v) {
      for (auto it_incident_edge = adjacency_list.at(v).cbegin();
           it_incident_edge != adjacency_list.at(v).cend(); ++it_incident_edge) {
        if (it_incident_edge->capacity > 0) {
          double reduced_cost =
            it_incident_edge->cost + potentials.at(v) - potentials.at(it_incident_edge->dst);
          assert(reduced_cost >= 0);
        }
      }
    }
#endif
  }

  // Output
  for (int agent = 0; agent < n_agents; ++agent) {
    for (auto it_incident_edge = adjacency_list.at(agent + 1).cbegin();
         it_incident_edge != adjacency_list.at(agent + 1).cend(); ++it_incident_edge) {
      int task = it_incident_edge->dst - (n_agents + 1);

      // If the flow value is 1 and task is not dummy, assign the task to the agent.
      if (it_incident_edge->flow == 1 && 0 <= task && task < n_tasks) {
        (*direct_assignment)[agent] = task;
        (*reverse_assignment)[task] = agent;
        break;
      }
    }
  }

#ifndef NDEBUG
  // Check if the result is valid assignment
  for (int agent = 0; agent < n_agents; ++agent) {
    if (direct_assignment->find(agent) != direct_assignment->cend()) {
      int task = (*direct_assignment).at(agent);
      assert(direct_assignment->at(agent) == task);
      assert(reverse_assignment->at(task) == agent);
    }
  }
#endif
}
}  // namespace gnn_solver
