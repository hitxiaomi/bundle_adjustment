//
// Created by xiaomi on 8/13/19.
//

#ifndef BUNDLE_ADJUSTMENT_OPTIMIZER_H
#define BUNDLE_ADJUSTMENT_OPTIMIZER_H


#include <map>
#include <set>
#include <vector>
#include <Eigen/Core>
#include <memory>
#include "base_vatex.h"
#include "base_edge.h"

namespace  bundle_adjustment {


    class Optimizer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef unsigned  long ulong;
        typedef std::map<ulong,std::shared_ptr<BaseVertex>> IdVertexMap;
        typedef std::set<std::shared_ptr<BaseEdge>> EdgeSet;
        Optimizer(){

        }


        bool AddVertex(const std::shared_ptr<BaseVertex>& vertex);
        bool AddEdge(const std::shared_ptr<BaseEdge>& edge );
        bool Optimize(int iterations);
        void SetVertexHessianIndex();
        bool IsPoseVertex(const std::shared_ptr<BaseVertex>& vertex);
        bool IsLandmarkVertex(const std::shared_ptr<BaseVertex>& vertex);
        bool MakeNormalEquation();
        double  ComputeLambdaInit() const ;
        void Push();
        void SetLambda(double lambda);
        void SolveNormalEquation();
        Eigen::VectorXd PCGSolver(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, int maxIter = -1);
        double ActiveEdgesChi2() const;
        bool  UpdateVertexState(const Eigen::VectorXd& update);
        bool RestoreHessianDiagonal();
        bool ComputeActiveEdgesResidual();
        bool DiscardTop();
        bool Pop();
        void SetVerbose(bool verbose);



    protected:
        IdVertexMap vertices_;
        IdVertexMap pose_vertices_;
        IdVertexMap landmark_vertices_;

        EdgeSet edges_;
        int hessian_dimension_;
        Eigen::MatrixXd Hessian_;
        Eigen::VectorXd b_;
        std::vector<double > hessian_diagonal_backup_;
        Eigen::VectorXd delta_x_;


        double current_lambda_;
        double ni_;
        int ordering_pose_;
        int ordering_landmark_;
        bool  verbose_;


    };
}


#endif //BUNDLE_ADJUSTMENT_OPTIMIZER_H
