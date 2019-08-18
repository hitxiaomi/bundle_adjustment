//
// Created by xiaomi on 8/11/19.
//

#include "vertex_point.h"
namespace bundle_adjustment{
    std::string VertexPoint::TypeInfo() const {
        return "VertexPoint";
    }
    void VertexPoint::Oplus(const Eigen::VectorXd &update) {
        estimate_+=update;
    }


}