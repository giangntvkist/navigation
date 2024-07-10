#ifndef _VISUALIZATION_H
#define _VISUALIZATION_H

#include "human_tracking/defs.hpp"

void visualizeMaker(visualization_msgs::Marker& node, visualization_msgs::Marker& edge, int color, std::string node_ns, std::string edge_ns) {
    /* Nodes - red*/
    node.header.frame_id = "map";
    node.header.stamp = ros::Time::now();
    node.action = visualization_msgs::Marker::ADD;
    node.type = visualization_msgs::Marker::SPHERE;
    node.id = 0;
    node.ns = node_ns;
    node.scale.x = 0.1;
    node.scale.y = 0.1;
    node.scale.z = 0.1;
    node.pose.orientation.x = 0;
    node.pose.orientation.y = 0;
    node.pose.orientation.z = 0.0;
    node.pose.orientation.w = 1.0;
    if(color == 0) {
        node.color.r = 1.0;
        node.color.g = 0.0;
        node.color.b = 0.0;
    } else if(color == 1) {
        node.color.r = 0.0;
        node.color.g = 1.0;
        node.color.b = 0.0;
    } else {
        node.color.r = 0.0;
        node.color.g = 0.0;
        node.color.b = 1.0;
    }

    node.color.a = 1.0;
    node.lifetime = ros::Duration(0);

    /* Edges - blue */
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.id = 0;
    edge.scale.x = 0.02;
    edge.scale.y = 0.02;
    edge.scale.z = 0.02;
    edge.ns = edge_ns;

    edge.pose.orientation.x = 0;
    edge.pose.orientation.y = 0;
    edge.pose.orientation.z = 0.0;
    edge.pose.orientation.w = 1.0;
    if(color == 0) {
        edge.color.r = 1.0;
        edge.color.g = 0.0;
        edge.color.b = 0.0;
    } else if(color == 1) {
        edge.color.r = 0.0;
        edge.color.g = 1.0;
        edge.color.b = 0.0;
    } else {
        edge.color.r = 0.0;
        edge.color.g = 0.0;
        edge.color.b = 1.0;
    }
    edge.color.a = 1.0;
    edge.lifetime = ros::Duration(0);
}

void trajectory_visualization(vk::StateVector x, visualization_msgs::Marker& node, visualization_msgs::Marker& edge, visualization_msgs::MarkerArray& SetOfMarker) {
    int id = SetOfMarker.markers.size();
    node.id = id;
    node.pose.position.x = x(0);
    node.pose.position.y = x(1);
    node.pose.position.z = 0.0;
    SetOfMarker.markers.push_back(node);
    id++;
    if(id >= 3) {
        visualization_msgs::Marker last_node = *(SetOfMarker.markers.end()-3);
        edge.id = id;
        edge.points.clear();
        geometry_msgs::Point p;
        p.x = last_node.pose.position.x;
        p.y = last_node.pose.position.y;
        p.z = last_node.pose.position.z;
        edge.points.push_back(p);

        p.x = node.pose.position.x;
        p.y = node.pose.position.y;
        p.z = node.pose.position.z;
        edge.points.push_back(p);
        SetOfMarker.markers.push_back(edge);
    }
}

void prediction_visualization(vk::StatePredict& x, visualization_msgs::Marker& node, visualization_msgs::Marker& edge, visualization_msgs::MarkerArray& SetOfMarker) {
    int num_nodes = x.size();
    int num_edges = x.size()-1;
    int id = SetOfMarker.markers.size();
    SetOfMarker.markers.clear();
    for(int i = 0; i < num_nodes; i++) {
        node.id = id;
        node.pose.position.x = x.at(i)(0);
        node.pose.position.y = x.at(i)(1);
        node.pose.position.z = 0.0;
        SetOfMarker.markers.push_back(node);
        id++;
        edge.id = id;
        edge.points.clear();
        geometry_msgs::Point p;
        if(i < num_edges) {
            p.x = x.at(i)(0);
            p.y = x.at(i)(1);
            p.z = 0.0;
            edge.points.push_back(p);

            p.x = x.at(i+1)(0);
            p.y = x.at(i+1)(1);
            p.z = 0.0;
            edge.points.push_back(p);
            SetOfMarker.markers.push_back(edge);
            id++;
        }
    }
}

#endif /* _VISUALIZATION_H */