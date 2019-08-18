//
// Created by xiaomi on 8/15/19.
//

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <random>
#include "optimizer.h"
#include "base_vatex.h"
#include "vertex_pose.h"
#include "vertex_point.h"
#include "edge_reprojection.h"
#include <iostream>




struct Frame{
    Frame(const Eigen::Matrix3d&  R, Eigen::Vector3d  t ): Rwc_(R),qwc_(R),twc_(std::move(t)){
//        std::cout<<" Frame constructor is called  "<<std::endl;

    }
    Frame(const Frame& frame ):Rwc_(frame.Rwc_),qwc_(frame.qwc_),twc_(frame.twc_),features_(frame.features_){
//        std::cout<<"Frame copy constructor is called"<<std::endl;

    }



    Eigen::Matrix3d Rwc_;
    Eigen::Quaterniond qwc_;
    Eigen::Vector3d twc_;
    std::map<int,Eigen::Vector2d> features_;


};
void GetSimDataInWorldFrame(const Eigen::Matrix3d& camera_intrinsics,std::vector<Frame>& camera_frames, std::vector<Eigen::Vector3d >&  points_world){
    int  num_features=500;
    int num_pose=20;
    double radius=8;
    for(int n=0; n < num_pose; n++){
        double theta= n / static_cast<double>(num_pose)*2*M_PI/4;

        Eigen::Matrix3d R=Eigen::AngleAxisd(theta,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
       camera_frames.push_back(Frame(R,t));
    }

    // 随机数生成三维特征点
    std::default_random_engine generator;
    std::normal_distribution<double> noise_pdf(0., 1. / 1000.);  // 2pixel / focal

    for (int j = 0; j < num_features; ++j) {
        std::uniform_real_distribution<double> xy_rand(-8, 8.0);
        std::uniform_real_distribution<double> z_rand(2., 16.);

        Eigen::Vector3d Pw(xy_rand(generator), xy_rand(generator), z_rand(generator));
        points_world.push_back(Pw);

        // 在每一帧上的观测量
        for (int i = 0; i < num_pose; ++i) {

            Eigen::Vector3d Pc = camera_frames[i].Rwc_.transpose() * (Pw - camera_frames[i].twc_);

            Pc = Pc / Pc.z();  // 归一化图像平面

            Eigen::Vector3d Pu=camera_intrinsics*Pc;



            Pu(0)+= noise_pdf(generator);
            Pu(1)+= noise_pdf(generator);
            camera_frames[i].features_.insert(std::make_pair( j, Pu.head(2) ) );

        }
    }










}



int main(int argc, char** argv){

    std::vector<Frame> camera_frames;
    std::vector<Eigen::Vector3d> points_world;
    Eigen::Matrix3d camera_intrinsics;
    camera_intrinsics<<515,0,320,
                        0,515,240,
                        0,0,1;
    GetSimDataInWorldFrame(camera_intrinsics,camera_frames,points_world);

    bundle_adjustment::Optimizer optimizer;
    optimizer.SetVerbose(true);

    std::vector<std::shared_ptr<bundle_adjustment::VertexPose> >  pose_vertices;
    int vertex_pose_id=0;
    int size_vertex_pose=camera_frames.size();
    for(int i=0;i<size_vertex_pose;i++){
        std::shared_ptr<bundle_adjustment::VertexPose> vertex_pose(new bundle_adjustment::VertexPose());
        vertex_pose->SetId(vertex_pose_id);
        if(i<1)
            vertex_pose->SetFixed(true);
        Eigen::VectorXd pose(7);

        pose<<camera_frames[i].twc_,camera_frames[i].qwc_.x(),camera_frames[i].qwc_.y(),camera_frames[i].qwc_.z(),camera_frames[i].qwc_.w();

        vertex_pose->SetEstimate(pose);
        optimizer.AddVertex(vertex_pose);
        pose_vertices.push_back(vertex_pose);
        vertex_pose_id++;

    }

    std::vector<std::shared_ptr<bundle_adjustment::VertexPoint> > point_vertices;

    int vertex_landmark_id=vertex_pose_id;
    int size_point_vertices=points_world.size();

    std::default_random_engine generator;
    std::normal_distribution<double > noise_pdf(0,1.);
    std::vector<double >  noises;
    double noise=0;

    for(int i=0;i<size_point_vertices;i++){
        std::shared_ptr<bundle_adjustment::VertexPoint> vertex_point(new bundle_adjustment::VertexPoint());
        vertex_point->SetId(vertex_landmark_id);
        vertex_point->SetEstimate(points_world[i]);
        optimizer.AddVertex(vertex_point);
        noise=noise_pdf(generator);
        noises.push_back(noise);

        for(int j=0;j<size_vertex_pose;j++){
            std::shared_ptr<bundle_adjustment::EdgeReprojection > edge(new bundle_adjustment::EdgeReprojection(camera_intrinsics) );
            edge->SetVertex(0,vertex_point);
            edge->SetVertex(1,pose_vertices[j]);
            Eigen::Vector2d measurement=camera_frames[j].features_.find(i)->second;

            measurement(0)+=noise;
            measurement(1)+=noise;


            edge->SetMeasurement(measurement);
            Eigen::Matrix2d information=Eigen::Matrix2d::Identity();
            edge->SetInformation(information);
            optimizer.AddEdge(edge);
        }


        point_vertices.push_back(vertex_point);
        vertex_landmark_id++;

    }
    optimizer.Optimize(100);

    std::cout << "\nCompare MonoBA results after opt..." << std::endl;

    double points_world_error=0;
    for (size_t k = 0; k < points_world.size(); k++) {
        std::cout << "after opt, point " << k << " : gt " <<  points_world[k].transpose() << " ,noise "
                  << noises[k] << " ,opt " << point_vertices[k]->Estimate().transpose() << std::endl;
        Eigen::Vector3d points_diff=points_world[k]-point_vertices[k]->Estimate();
        points_world_error+=points_diff.norm();

    }


    double pose_error=0;

    std::cout<<"------------ pose translation ----------------"<<std::endl;
    for (size_t i = 0; i < camera_frames.size(); ++i) {
       Eigen::Vector3d pose_diff=pose_vertices[i]->Estimate().head(3)-camera_frames[i].twc_;
       pose_error+=pose_diff.norm();
        std::cout<<"translation after opt: "<< i <<" :"<< pose_vertices[i]->Estimate().head(3).transpose() << " || gt: "<<camera_frames[i].twc_.transpose()<<std::endl;
    }

    std::cout<<"-------------"<<std::endl;
    std::cout<<"points_world_error:"<<points_world_error/points_world.size()<<std::endl;
    std::cout<<"pose_error:"<<pose_error/camera_frames.size()<<std::endl;

//    std::cout<<"end of program"<<std::endl;
    return  0;
}
