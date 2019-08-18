//
// Created by xiaomi on 8/11/19.
//

#ifndef BUNDLE_ADJUSTMENT_VERTEX_POINT_H
#define BUNDLE_ADJUSTMENT_VERTEX_POINT_H
#include "base_vatex.h"

namespace bundle_adjustment{

class VertexPoint:public BaseVertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPoint():BaseVertex(3,3){}
    virtual  std::string TypeInfo() const override ;
    virtual  void Oplus(const Eigen::VectorXd & update ) override ;


};

}

#endif //BUNDLE_ADJUSTMENT_VERTEX_POINT_H
