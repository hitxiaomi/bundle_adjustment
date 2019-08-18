//
// Created by xiaomi on 8/11/19.
//

#ifndef BUNDLE_ADJUSTMENT_VERTEX_POSE_H
#define BUNDLE_ADJUSTMENT_VERTEX_POSE_H
#include "base_vatex.h"
#include <Eigen/Core>

//tx,ty,tz,qx,qy,qz,qw
namespace  bundle_adjustment {
    class VertexPose: public BaseVertex {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexPose():BaseVertex(7,6){}


        virtual  std::string TypeInfo()  const override ;
        virtual  void Oplus(const Eigen::VectorXd & update ) override;




    };
}

#endif //BUNDLE_ADJUSTMENT_VERTEX_POSE_H
