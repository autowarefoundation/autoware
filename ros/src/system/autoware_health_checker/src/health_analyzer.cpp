/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Masaya Kataoka
 */

#include <autoware_health_checker/health_analyzer.h>

HealthAnalyzer::HealthAnalyzer(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    system_status_summary_pub_ = nh_.advertise<autoware_system_msgs::SystemStatus>("/system_status/summary",1);
    system_status_sub_ = nh_.subscribe("/system_status",1,&HealthAnalyzer::systemStatusCallback,this);
}

HealthAnalyzer::~HealthAnalyzer()
{
    
}

void HealthAnalyzer::systemStatusCallback(const autoware_system_msgs::SystemStatus::ConstPtr msg)
{
    generateDependGraph(*msg);
    return;
}

void HealthAnalyzer::generateDependGraph(autoware_system_msgs::SystemStatus status)
{
    depend_graph_ = graph_t();
    for(auto itr = status.topic_statistics.begin(); itr != status.topic_statistics.end(); itr++)
    {
        addDepend(*itr);
    }
    writeDot();
    return;
}

void HealthAnalyzer::writeDot()
{
    std::string path = ros::package::getPath("autoware_health_checker")+"/data/node_depends.dot";
    std::ofstream f(path.c_str());
    boost::write_graphviz(f, depend_graph_, boost::make_label_writer(get(&node_property::node_name, depend_graph_)));
    return;
}

void HealthAnalyzer::addDepend(rosgraph_msgs::TopicStatistics statistics)
{
    vertex_t node_sub;
    vertex_t node_pub;
    edge_t topic;
    auto vertex_range = boost::vertices(depend_graph_);
    bool pub_node_found = false;
    bool sub_node_found = false;
    for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first)
    {
        vertex_t v = *first;
        if(depend_graph_[v].node_name == statistics.node_pub)
        {
            pub_node_found = true;
            node_pub = v;
        }
        if(depend_graph_[v].node_name == statistics.node_sub)
        {
            sub_node_found = true;
            node_sub = v;
        }
    }
    // the depend was already found.
    if(pub_node_found && sub_node_found)
    {
        return;
    }
    if(!pub_node_found)
    {
        vertex_t v = boost::add_vertex(depend_graph_);
        depend_graph_[v].node_name = statistics.node_pub;
        node_pub = v;
    }
    if(!sub_node_found)
    {
        vertex_t v = boost::add_vertex(depend_graph_);
        depend_graph_[v].node_name = statistics.node_sub;
        node_sub = v;
    }
    bool inserted = false;
    boost::tie(topic, inserted) = boost::add_edge(node_sub, node_pub, depend_graph_);
    depend_graph_[topic].node_sub = statistics.node_sub;
    depend_graph_[topic].node_pub = statistics.node_pub;
    return;
}