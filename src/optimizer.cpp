//
// Created by xiaomi on 8/13/19.
//

#include "optimizer.h"
#include <iostream>
#include <cmath>
namespace bundle_adjustment{

    bool Optimizer::AddVertex(const std::shared_ptr<BaseVertex> &vertex) {
        ulong  vertex_id=vertex->Id();
        if(vertices_.find(vertex_id)!=vertices_.end())
        {
            std::cerr<<"already have this vertex"<<std::endl;
            return false;

        }
        else{
            vertices_.insert(std::make_pair(vertex_id,vertex));
            return  true;
        }

    }
    bool Optimizer::AddEdge(const std::shared_ptr<BaseEdge> &edge) {

        if(edges_.find(edge)!=edges_.end())
            {
            std::cerr<<"already have this edge"<<std::endl;
            return false;
            }
        else{
            edges_.insert(edge);
            return true;
        }

    }
    bool Optimizer::Optimize(int iterations) {



        int num_iter=0;
        bool stop_optimization=false;
        double stop_threshold=0.0;


        while(num_iter<iterations&&!stop_optimization){

            if(num_iter==0){
                SetVertexHessianIndex();
            }

            ComputeActiveEdgesResidual();
            double current_chi= ActiveEdgesChi2();
            double tem_chi=current_chi;

            if(num_iter==0){
                stop_threshold=1e-6*current_chi;
            }
            if(current_chi<stop_threshold){
                stop_optimization=true;
                break;
            }

            MakeNormalEquation();

            if(num_iter==0){
                current_lambda_=ComputeLambdaInit();
                ni_=2;
            }


            int false_cnt = 0,max_false_cnt=10;
            double rho = 0;
//            bool one_step_success=false;

            do {

                Push();
                SetLambda(current_lambda_);
                SolveNormalEquation();
                if(delta_x_.squaredNorm()<=1e-6 || false_cnt>10){
                    stop_optimization=true;
                    break;
                }
                UpdateVertexState(delta_x_);

                RestoreHessianDiagonal();

                ComputeActiveEdgesResidual();
                tem_chi=ActiveEdgesChi2();

                rho= current_chi - tem_chi;

                double scale = 0;
                scale = delta_x_.transpose() * (current_lambda_ * delta_x_ + b_);
                scale += 1e-3;    // make sure it's non-zero :)
                rho/=scale;

                if(rho>0 && std::isfinite(tem_chi)){
                    double alpha=1.0-pow((2*rho-1),3);
                    alpha=(std::min)(alpha,2.0/3.0); // 2.0/3 upbound
                    double scale_factor=(std::max)(1.0/3.0,alpha); //1.0/3.0  lowwer bound
                    current_lambda_*=scale_factor;
                    ni_=2;
                    current_chi=tem_chi;
                    DiscardTop();
//                    one_step_success=true;
                    //
                }
                else{
                    current_lambda_*=ni_;
                    ni_*=2;
                    Pop();
//                    one_step_success= false;
                }
                false_cnt++;


            }while(rho<0&&false_cnt<max_false_cnt);
            if(verbose_){
                std::cout<<"num_iter:"<<num_iter<<"current_chi:"<<current_chi<<"currrent lambda:"<<current_lambda_<<std::endl;
            }

        num_iter++;

        }


        return  true;

    }
    bool Optimizer::IsPoseVertex(const std::shared_ptr<BaseVertex> &vertex) {
        return vertex->TypeInfo()==std::string("VertexPose");
    }
    bool Optimizer::IsLandmarkVertex(const std::shared_ptr<BaseVertex> &vertex) {
        return vertex->TypeInfo()==std::string("VertexPoint");

    }
    void Optimizer::SetVertexHessianIndex() {

        hessian_dimension_=0;

        ordering_pose_=0;
        ordering_landmark_=0;
//        int num_pose_vertex=0;

        for(const auto& vertex:vertices_) {

            hessian_dimension_+=vertex.second->LocalDimension();

            if(IsPoseVertex(vertex.second)){
                vertex.second->SetHessianIndex(ordering_pose_);
                ordering_pose_+=vertex.second->LocalDimension();
                pose_vertices_.insert(vertex);
//                num_pose_vertex++;

            }
            else if(IsLandmarkVertex(vertex.second)){
                landmark_vertices_.insert(vertex);
                ordering_landmark_+=vertex.second->LocalDimension();
            }

        }

        int landmark_hessian_index=ordering_pose_;

        for(const auto& landmark_vertex : landmark_vertices_) {
            landmark_vertex.second->SetHessianIndex(landmark_hessian_index);
            landmark_hessian_index += landmark_vertex.second->LocalDimension();

        }




    }
    bool Optimizer::MakeNormalEquation() {

        Eigen::MatrixXd H=Eigen::MatrixXd::Zero(hessian_dimension_,hessian_dimension_);
        Eigen::VectorXd b=Eigen::VectorXd::Zero(hessian_dimension_);

        for(const auto& edge:edges_ ){
//            edge->ComputeResidual();
            edge->ComputeJacobians();
            std::vector<std::shared_ptr<BaseVertex> >  edge_vertices=edge->GetAllVertex();
            std::vector<Eigen::MatrixXd > edge_jacobians=edge->Jacobians();
            int size_vertices=edge_vertices.size();

            for(int i=0; i < size_vertices; i++){

                if(edge_vertices[i]->IsFixed()){
                    continue;
                }

                int index_i=edge_vertices[i]->HessianIndex();
                int dim_i=edge_vertices[i]->LocalDimension();
                for(int j=i; j < size_vertices; j++){
                    if(edge_vertices[j]->IsFixed())
                        continue;
                    int index_j=edge_vertices[j]->HessianIndex();
                    int dim_j=edge_vertices[j]->LocalDimension();
                    Eigen::MatrixXd hessian;
                    hessian.resize(dim_i,dim_j);
                    hessian=edge_jacobians[i].transpose()*edge->Information()*edge_jacobians[j];

                    H.block(index_i, index_j, dim_i, dim_j).noalias()+=hessian;
                    if(i!=j)
                        H.block(index_j,index_i,dim_j,dim_i).noalias()+=hessian.transpose();

                }
                b.segment(index_i,dim_i).noalias() -=edge_jacobians[i].transpose()*edge->Information()*edge->Residual();

            }

        }

        Hessian_=H;
        b_=b;
        delta_x_=Eigen::VectorXd::Zero(hessian_dimension_);


    return true;


    }
    double Optimizer::ComputeLambdaInit() const  {

        double max_diagonal=0;
        for(int i=0;i<hessian_dimension_;i++){
            max_diagonal=std::max(max_diagonal, Hessian_(i, i));
        }
        double tau=1e-5;
        return  tau*max_diagonal;
    }
    void Optimizer::Push() {
        for(const auto& vertex:vertices_)
        {
            vertex.second->Push();
        }

    }

    void Optimizer::SetLambda(double lambda) {
        const int size_hessian=Hessian_.cols();
        hessian_diagonal_backup_.resize(size_hessian);
        for(int i=0;i<size_hessian;i++){
            hessian_diagonal_backup_[i]=Hessian_(i, i);
            Hessian_(i, i)+=lambda;
        }

    }
    void Optimizer::SolveNormalEquation() {

        int reserve_size = ordering_pose_;
        int marg_size = ordering_landmark_;

        Eigen::MatrixXd Hpp=Hessian_.block(0,0,reserve_size,reserve_size);
        Eigen::MatrixXd Hpm=Hessian_.block(0,reserve_size,reserve_size,marg_size);
        Eigen::MatrixXd Hmp=Hpm.transpose();
        Eigen::MatrixXd Hmm=Hessian_.block(reserve_size,reserve_size,marg_size,marg_size);

        Eigen::VectorXd bpp=b_.segment(0,reserve_size);
        Eigen::VectorXd bmm=b_.segment(reserve_size,marg_size);

        Eigen::MatrixXd Hmm_inv=Eigen::MatrixXd::Zero(marg_size,marg_size);

        for(const auto & landmark_vertex:landmark_vertices_ ){
            int index=landmark_vertex.second->HessianIndex()-reserve_size;
            int size=landmark_vertex.second->LocalDimension();
            Hmm_inv.block(index,index,size,size)=Hmm.block(index,index,size,size).inverse();

        }


        Eigen::MatrixXd temH=Hpm*Hmm_inv;
        Eigen::MatrixXd H_schur=Hpp-temH*Hmp;
        Eigen::VectorXd b_schur=bpp-temH*bmm;

        // step2: solve Hpp * delta_x = bpp
        Eigen::VectorXd delta_x_pp(Eigen::VectorXd::Zero(reserve_size));
        // PCG Solver


        int n = H_schur.rows() * 2;
        delta_x_pp = PCGSolver(H_schur, b_schur, n);
        delta_x_.head(reserve_size) = delta_x_pp;

//        std::cout << delta_x_pp.transpose() << std::endl;



        Eigen::VectorXd delta_x_ll(marg_size);
        delta_x_ll = Hmm_inv*(bmm-Hmp*delta_x_pp);
        delta_x_.tail(marg_size) = delta_x_ll;



    }


    Eigen::VectorXd Optimizer::PCGSolver(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, int maxIter) {
        assert(A.rows() == A.cols() && "PCG solver ERROR: A is not a square matrix");
        int rows = b.rows();
        int n = maxIter < 0 ? rows : maxIter;
        Eigen::VectorXd x(Eigen::VectorXd::Zero(rows));
        Eigen::MatrixXd M_inv = A.diagonal().asDiagonal().inverse();
        Eigen::VectorXd r0(b);  // initial r = b - A*0 = b
        Eigen::VectorXd z0 = M_inv * r0;
        Eigen::VectorXd p(z0);
        Eigen::VectorXd w = A * p;
        double r0z0 = r0.dot(z0);
        double alpha = r0z0 / p.dot(w);
        Eigen::VectorXd r1 = r0 - alpha * w;
        int i = 0;
        double threshold = 1e-6 * r0.norm();
        while (r1.norm() > threshold && i < n) {
            i++;
            Eigen::VectorXd z1 = M_inv * r1;
            double r1z1 = r1.dot(z1);
            double belta = r1z1 / r0z0;
            z0 = z1;
            r0z0 = r1z1;
            r0 = r1;
            p = belta * p + z1;
            w = A * p;
            alpha = r1z1 / p.dot(w);
            x += alpha * p;
            r1 -= alpha * w;
        }
        return x;
    }
    double Optimizer::ActiveEdgesChi2() const {
        double chi=0.0;
        for(const auto& edge:edges_){
           chi+= edge->Chi2();
        }
        return  chi;

    }

    bool Optimizer::UpdateVertexState(const Eigen::VectorXd &update) {

        for(const auto & vertex:vertices_ ){
            int index=vertex.second->HessianIndex();
            int local_dimension=vertex.second->LocalDimension();
            Eigen::MatrixXd vertex_update=update.segment(index,local_dimension);
            vertex.second->Oplus(vertex_update);
        }
        return  true;

    }

    bool Optimizer::RestoreHessianDiagonal() {

        long size=Hessian_.cols();
        for(int i=0;i<size;i++){
            Hessian_(i,i)=hessian_diagonal_backup_[i];
        }

        return  true;
    }
    bool Optimizer::ComputeActiveEdgesResidual() {
        for(const auto& edge:edges_){
            edge->ComputeResidual();
        }
        return true;
    }
    bool Optimizer::DiscardTop() {
        for(const auto& vertex:vertices_){
            vertex.second->DiscardTop();
        }
        return  true;



    }
    bool Optimizer::Pop() {
        for(const auto& vertex:vertices_){
            vertex.second->Pop();
        }
        return  true;
    }

    void Optimizer::SetVerbose(bool verbose) {
        verbose_=verbose;
    }








}//end of namespace