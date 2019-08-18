//
// Created by xiaomi on 8/11/19.
//

#include "vertex_pose.h"
#include "sophus/so3.hpp"
#include <iostream>
//parameters: tx, ty, tz, qx, qy, qz, qw, 7 DoF

namespace  bundle_adjustment{
  std::string VertexPose::TypeInfo() const {
      return  "VertexPose";

  }

  void VertexPose::Oplus(const Eigen::VectorXd &update) {

      estimate_.head(3)+=update.head(3);

      Eigen::Quaterniond current_quaternion(estimate_(6),estimate_(3),estimate_(4),estimate_(5));

      Eigen::Quaterniond update_quaternion=Sophus::SO3d::exp(Eigen::Vector3d( update(3),update(4),update(5) )).unit_quaternion();
//      std::cout<<"update_vector(x,y,z)"<<update.x()<<','<<update.y()<<','<<update.z()<<std::endl;
//      std::cout<<"update_quaternion(w,x,y,z)"<<update_quaternion.w()<<","<<update_quaternion.x()<<","<<update_quaternion.y()<<","<<update.z()<<std::endl;

      Eigen::Quaterniond  after_update=current_quaternion*update_quaternion;
      after_update.normalize();

      estimate_.tail(4)<<after_update.x(),
                        after_update.y(),
                        after_update.z(),
                        after_update.w();

  }

}