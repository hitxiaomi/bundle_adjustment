//
// Created by xiaomi on 8/11/19.
//



#include "vertex_point.h"
#include "vertex_pose.h"
#include "edge_reprojection.h"
//parameters: tx, ty, tz, qx, qy, qz, qw, 7 DoF
//q_wc,t_wc
namespace bundle_adjustment {
    void EdgeReprojection::ComputeResidual() {

       Eigen::Vector3d  point_world= vertices_[0]-> Estimate();
       Eigen::VectorXd  param_v1= vertices_[1]->Estimate();
       Eigen::Vector3d  t_wc=param_v1.head(3);
       Eigen::Quaterniond q_wc(param_v1(6),param_v1(3),param_v1(4),param_v1(5) );

       Eigen::Vector3d point_camera=q_wc.inverse()*(point_world-t_wc);

       Eigen::Vector3d point_pixel_homogeneous=intrinsics_*(point_camera/ point_camera(2));

       residual_=point_pixel_homogeneous.head(2)-measurement_;



    }
    void EdgeReprojection::ComputeJacobians() {

        Eigen::Vector3d  point_world= vertices_[0]-> Estimate();
        Eigen::VectorXd  param_v1= vertices_[1]->Estimate();
        Eigen::Vector3d  t_wc=param_v1.head(3);
        Eigen::Quaterniond q_wc(param_v1(6),param_v1(3),param_v1(4),param_v1(5) );

        Eigen::Vector3d point_camera=q_wc.inverse()*(point_world-t_wc);

        double fx=intrinsics_(0,0),fy=intrinsics_(1,1);
        double x=point_camera(0),y=point_camera(1),z=point_camera(2),z_2=z*z;

        Eigen::Matrix<double ,2,3 > jacobian_uv_pc;
        jacobian_uv_pc<<fx/z,0.,-fx*x/z_2,
                        0,fy/z,-fy*y/z_2;


        Eigen::Matrix3d jacobian_pc_qwc;
        jacobian_pc_qwc<<0, -z,  y,
                         z,  0,  -x,
                        -y,  x,  0;

        Eigen::Matrix3d R_wc=q_wc.toRotationMatrix();
        Eigen::Matrix3d  jacobian_pc_twc;
        jacobian_pc_twc=-R_wc.transpose();

        Eigen::Matrix<double,3,6 > jacobian_pc_twc_qwc;
        jacobian_pc_twc_qwc.block(0,0,3,3)=jacobian_pc_twc;
        jacobian_pc_twc_qwc.block(0,3,3,3)=jacobian_pc_qwc;

        Eigen::Matrix<double,2,6> jacobian_uv_twc_qwc;
        jacobian_uv_twc_qwc=jacobian_uv_pc*jacobian_pc_twc_qwc;

        jacobians_[1].resize(2,6);
        jacobians_[1]=jacobian_uv_twc_qwc;

        jacobians_[0].resize(2,3);
        jacobians_[0]=jacobian_uv_pc*R_wc.transpose();


        //数值jacobian


    }
    std::string EdgeReprojection::TypeInfo() const {
        return "EdgeReprojection";
    }



}