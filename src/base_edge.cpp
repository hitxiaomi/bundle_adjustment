//
// Created by xiaomi on 8/9/19.
//

#include "base_vatex.h"
#include "base_edge.h"
#include <memory>


namespace  bundle_adjustment{

    BaseEdge::BaseEdge(int residual_dimension, int num_vertices,const std::vector<std::string>& vertices_types):
            residual_dimension_(residual_dimension), num_vertices_(num_vertices)
    {
//        vertices_types_=vertices_types;
        if(!vertices_types.empty())
            vertices_types_=vertices_types;


        vertices_.resize(num_vertices_, nullptr);
        jacobians_.resize(num_vertices_);
        residual_.resize(residual_dimension_);
        measurement_.resize(residual_dimension_);

        information_.resize(residual_dimension_,residual_dimension_);
        information_.setIdentity();
    }

    void BaseEdge::SetInformation(const Eigen::MatrixXd &information) {
        information_=information;
    }
    void BaseEdge::SetMeasurement(const Eigen::VectorXd &measurement) {
        measurement_=measurement;
    }
    void BaseEdge::SetId(unsigned long id) {id_=id;}
    void BaseEdge::SetVertex(int i, std::shared_ptr<BaseVertex> vertex) {
        vertices_[i]=vertex;
    }
    double BaseEdge::Chi2() {
        return  residual_.transpose()*information_*residual_;
    }




}