#include "pgo_toy_example.h"

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include <iostream>
#include <vector>

// prev poses (gray sphere).
visualization_msgs::MarkerArray prev_nodes;
// prev edges (gray line).
visualization_msgs::MarkerArray prev_edges;
// optimized poses (black sphere).
visualization_msgs::MarkerArray opt_nodes;
// optimized edges (black line).
visualization_msgs::MarkerArray opt_edges;
visualization_msgs::MarkerArray texts;
double xinit, yinit, zinit;

void SetNodes(PGOToyExample* toy, visualization_msgs::MarkerArray& nodes,
              std::string ns, int id, double r, double g, double b, double a, bool is_opt_node)
{
  visualization_msgs::Marker node;
  node.header.frame_id = "world";
  node.header.stamp = ros::Time::now();
  node.ns = ns;
  node.id = id;
  node.type = visualization_msgs::Marker::SPHERE;
  node.scale.x = node.scale.y = node.scale.z = 0.2;
  node.color.r = r; node.color.g = g; node.color.b = b; node.color.a = a;

  if(!is_opt_node) {
    Quaternion q = (Quaternion)toy->GetOriginalPoses()[id].rotation();

    node.pose.position.x = toy->GetOriginalPoses()[id].translation()[0];
    node.pose.position.y = toy->GetOriginalPoses()[id].translation()[1];
    node.pose.position.z = toy->GetOriginalPoses()[id].translation()[2];
    node.pose.orientation.x = q.x();
    node.pose.orientation.y = q.y();
    node.pose.orientation.z = q.z();
    node.pose.orientation.w = q.w();

    if(node.pose.orientation.w < 0) {
      node.pose.orientation.x *= -1;
      node.pose.orientation.y *= -1;
      node.pose.orientation.z *= -1;
      node.pose.orientation.w *= -1;
    }
    nodes.markers.push_back(node);
  }
  else {
    g2o::VertexSE3Expmap* vtx = static_cast<g2o::VertexSE3Expmap*>(toy->GetOptimizer()->vertex(id));
    Isometry opt_poses = vtx->estimate();
    Quaternion q = (Quaternion)opt_poses.rotation();

    // First node.
    if(id==0) {
      xinit = opt_poses.translation()[0];
      yinit = opt_poses.translation()[1];
      zinit = opt_poses.translation()[2];

      node.pose.position.x = 0;
      node.pose.position.y = 0;
      node.pose.position.z = 0;
    }

    node.pose.position.x = opt_poses.translation()[0]-xinit;
    node.pose.position.y = opt_poses.translation()[1]-yinit;
    node.pose.position.z = opt_poses.translation()[2]-zinit;

    node.pose.orientation.x = q.x();
    node.pose.orientation.y = q.y();
    node.pose.orientation.z = q.z();
    node.pose.orientation.w = q.w();

    if(node.pose.orientation.w < 0) {
      node.pose.orientation.x *= -1;
      node.pose.orientation.y *= -1;
      node.pose.orientation.z *= -1;
      node.pose.orientation.w *= -1;
    }
    nodes.markers.push_back(node);

    visualization_msgs::Marker opt_node_text;
    opt_node_text.header.frame_id = "world";
    opt_node_text.header.stamp = ros::Time();
    opt_node_text.ns = "opt_node_text";
    opt_node_text.id = id;
    opt_node_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    opt_node_text.action = visualization_msgs::Marker::ADD;
    opt_node_text.scale.z = 0.1;
    opt_node_text.color.r = 0.0; opt_node_text.color.g = 0.0; opt_node_text.color.b = 0.0; opt_node_text.color.a = 1.0;
    opt_node_text.pose.position.x = node.pose.position.x;
    opt_node_text.pose.position.y = node.pose.position.y;
    opt_node_text.pose.position.z = node.pose.position.z+0.2;
    std::string txt = "x" + std::to_string(id);
    opt_node_text.text = txt;
    texts.markers.push_back(opt_node_text);
  }
}

void SetEdges(PGOToyExample* toy, visualization_msgs::MarkerArray& edges,
              std::string ns, int id, int start, int end,
              double r, double g, double b, double a, bool is_opt_edge)
{
  visualization_msgs::Marker edge;
  edge.header.frame_id = "world";
  edge.header.stamp = ros::Time::now();
  edge.ns = ns;
  edge.id = id;
  edge.type = visualization_msgs::Marker::LINE_LIST;
  edge.color.r = r; edge.color.g = g; edge.color.b = b; edge.color.a = a;
  edge.pose.orientation.w = 1.0;
  edge.scale.x = 0.01;

  if(!is_opt_edge) {
    geometry_msgs::Point p1;
    p1.x = toy->GetOriginalPoses()[start].translation()[0];
    p1.y = toy->GetOriginalPoses()[start].translation()[1];
    p1.z = toy->GetOriginalPoses()[start].translation()[2];
    edge.points.push_back(p1);

    geometry_msgs::Point p2;
    p2.x = toy->GetOriginalPoses()[end].translation()[0];
    p2.y = toy->GetOriginalPoses()[end].translation()[1];
    p2.z = toy->GetOriginalPoses()[end].translation()[2];
    edge.points.push_back(p2);

    edges.markers.push_back(edge);

    visualization_msgs::Marker prev_edge_text;
    prev_edge_text.header.frame_id = "world";
    prev_edge_text.header.stamp = ros::Time();
    prev_edge_text.ns = "prev_edge_text";
    prev_edge_text.id = id;
    prev_edge_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    prev_edge_text.action = visualization_msgs::Marker::ADD;
    prev_edge_text.scale.z = 0.1;
    prev_edge_text.color.r = r; prev_edge_text.color.g = g; prev_edge_text.color.b = b; prev_edge_text.color.a = a;
    prev_edge_text.pose.position.x = (p1.x+p2.x)/2.;
    prev_edge_text.pose.position.y = (p1.y+p2.y)/2.;
    prev_edge_text.pose.position.z = (p1.z+p2.z)/2. + 0.1;
    std::string txt = "zhat" + std::to_string(start) + std::to_string(end);
    prev_edge_text.text = txt;
    texts.markers.push_back(prev_edge_text);
  }
  else {
    g2o::VertexSE3Expmap* prev_vtx = static_cast<g2o::VertexSE3Expmap*>(toy->GetOptimizer()->vertex(start));
    g2o::VertexSE3Expmap* curr_vtx = static_cast<g2o::VertexSE3Expmap*>(toy->GetOptimizer()->vertex(end));
    Isometry prev_opt_poses = prev_vtx->estimate();
    Isometry curr_opt_poses = curr_vtx->estimate();

    // First opt_node.
    if(start == 0) {
      xinit = prev_opt_poses.translation()[0];
      yinit = prev_opt_poses.translation()[1];
      zinit = prev_opt_poses.translation()[2];
    }

    geometry_msgs::Point p1;
    p1.x = prev_opt_poses.translation()[0] -xinit;
    p1.y = prev_opt_poses.translation()[1] -yinit;
    p1.z = prev_opt_poses.translation()[2] -zinit;
    edge.points.push_back(p1);

    geometry_msgs::Point p2;
    p2.x = curr_opt_poses.translation()[0] -xinit;
    p2.y = curr_opt_poses.translation()[1] -yinit;
    p2.z = curr_opt_poses.translation()[2] -zinit;
    edge.points.push_back(p2);

    edges.markers.push_back(edge);

    visualization_msgs::Marker opt_edge_text;
    opt_edge_text.header.frame_id = "world";
    opt_edge_text.header.stamp = ros::Time();
    opt_edge_text.ns = "opt_edge_text";
    opt_edge_text.id = id;
    opt_edge_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    opt_edge_text.action = visualization_msgs::Marker::ADD;
    opt_edge_text.scale.z = 0.1;
    opt_edge_text.color.r = 0.0; opt_edge_text.color.g = 0.0; opt_edge_text.color.b = 0.0; opt_edge_text.color.a = 1.0;
    opt_edge_text.pose.position.x = (p1.x+p2.x)/2.;
    opt_edge_text.pose.position.y = (p1.y+p2.y)/2.;
    opt_edge_text.pose.position.z = (p1.z+p2.z)/2. + 0.1;
    std::string txt = "z" + std::to_string(start) + std::to_string(end);
    opt_edge_text.text = txt;
    texts.markers.push_back(opt_edge_text);
  }

}


int main(int argc, char **argv) {
  ros::init(argc,argv,"pgo_toy_example");
  ros::NodeHandle nh, priv_nh("~");

  // set publishers.
  ros::Publisher opt_node_pub = nh.advertise<visualization_msgs::MarkerArray>("opt_nodes",1);
  ros::Publisher opt_edge_pub = nh.advertise<visualization_msgs::MarkerArray>("opt_edges",1);
  ros::Publisher prev_node_pub = nh.advertise<visualization_msgs::MarkerArray>("prev_nodes",1);
  ros::Publisher prev_edge_pub = nh.advertise<visualization_msgs::MarkerArray>("prev_edges",1);
  ros::Publisher text_pub = nh.advertise<visualization_msgs::MarkerArray>("texts",1);

  int _rate, _iter;
  priv_nh.param("loop_rate", _rate, 5);
  priv_nh.param("iteration", _iter, 20);

  // create pgo toy example instance.
  PGOToyExample* toy = new PGOToyExample(true);

  while(ros::ok()) {
    // set loop speed.
    ros::Rate loop_rate(_rate); // [hz]

    toy->Reset();
    int num_poses = toy->GetOriginalPosesSize();
    int count = 0;

    while(count < _iter) { // [Iteration]
      // set prev nodes.
      for(int i=0; i<num_poses; i++) {
        SetNodes(toy, prev_nodes, "prev_nodes", i, 0.0, 0.0, 0.0, 0.25, false);
      }

      // set prev_edges.
      for(int i=1; i<num_poses; i++) {
        SetEdges(toy, prev_edges, "prev_edges", i, i-1, i, 0.0, 0.0, 0.0, 0.25, false);
      }
      SetEdges(toy, opt_edges, "prev_edges", 16, 5, 11, 0.0, 0.0, 0.0, 0.25, false);
      SetEdges(toy, opt_edges, "prev_edges", 17, 3, 14, 0.0, 0.0, 0.0, 0.25, false);

      // set optimized poses.
      for(int i=0; i<num_poses; i++) {
        SetNodes(toy, prev_nodes, "opt_nodes", i, 0.0, 0.0, 0.0, 1.0, true);
      }

      // set optimized edges.
      for(int i=1; i<num_poses; i++) {
        SetEdges(toy, opt_edges, "opt_edges", i, i-1, i, 0.0, 0.0, 0.0, 1, true);
      }
      SetEdges(toy, opt_edges, "opt_edges", 16, 5, 11, 0.0, 0.0, 0.0, 1, true);
      SetEdges(toy, opt_edges, "opt_edges", 17, 3, 14, 0.0, 0.0, 0.0, 1, true);


      visualization_msgs::Marker text;
      text.header.frame_id = "world";
      text.header.stamp = ros::Time();
      text.ns = "text";
      text.id = 0;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.scale.z = 0.1;
      text.color.r = 0.0; text.color.g = 0.0; text.color.b = 0.0; text.color.a = 1.0;
      text.pose.position.y = 0.5;
      text.pose.position.z = 1.0;

      std::string comment = "[PGO toy example using g2o]\n\nPose: SE3Quat\nVertex: VertexSE3Expmap\nEdge: EdgeSE3Expmap\nInformation Matrix: Random\n\nBlack: Current Estimated Poses\nGray: Previous Poses";
      text.text = comment;
      texts.markers.push_back(text);

      visualization_msgs::Marker count_text;
      count_text.header.frame_id = "world";
      count_text.header.stamp = ros::Time();
      count_text.ns = "count_text";
      count_text.id = 0;
      count_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      count_text.action = visualization_msgs::Marker::ADD;
      count_text.scale.z = 0.1;
      count_text.color.r = 0.0; count_text.color.g = 0.0; count_text.color.b = 0.0; count_text.color.a = 1.0;
      count_text.pose.position.y = 0.5;
      count_text.pose.position.z = 2.0;

      std::string counttxt = "Iteration: " + std::to_string(count);
      count_text.text = counttxt;
      texts.markers.push_back(count_text);

      // publish to /texts
      text_pub.publish(texts);

      // publish to /prev_edges
      prev_edge_pub.publish(prev_edges);

      // publish to /opt_nodes
      opt_node_pub.publish(opt_nodes);

      // publish to /opt_edges
      opt_edge_pub.publish(opt_edges);

      // publish to /prev_nodes
      prev_node_pub.publish(prev_nodes);

      loop_rate.sleep();
      toy->GetOptimizer()->optimize(1);
      count += 1;
    }

    std::cout << std::endl;
    prev_edges.markers.clear();
    opt_nodes.markers.clear();
    prev_nodes.markers.clear();
    texts.markers.clear();
  }

  return 0;
}
