//
// Created by xiaomi on 8/11/19.
//

#ifndef BUNDLE_ADJUSTMENT_EDGE_REPROJECTION_H
#define BUNDLE_ADJUSTMENT_EDGE_REPROJECTION_H


#include <string>
#include <vector>
#include "base_edge.h"

namespace bundle_adjustment {


class EdgeReprojection: public BaseEdge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;



    EdgeReprojection(const Eigen::Matrix3d& intrinsics):BaseEdge(2,2,std::vector<std::string>{"VertexPoint", "VertexPose"} ),intrinsics_(intrinsics) {

    }

    virtual  ~EdgeReprojection();





    virtual std::string TypeInfo()  const  ;

    virtual  void  ComputeResidual() override ;
    virtual void ComputeJacobians() override ;





private:

    Eigen::Matrix3d intrinsics_;




    };

}


#endif //BUNDLE_ADJUSTMENT_EDGE_REPROJECTION_H
