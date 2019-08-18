//
// Created by xiaomi on 8/9/19.
//

#ifndef BUNDLE_ADJUSTMENT_BASE_EDGE_H
#define BUNDLE_ADJUSTMENT_BASE_EDGE_H
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Core>



namespace bundle_adjustment {

    class  BaseVertex;

    class BaseEdge {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        BaseEdge(int residual_dimension,int num_vertices,const std::vector<std::string>& vertices_types=std::vector<std::string>() );




        void  SetId(unsigned long id);
        unsigned   long  Id() const { return  id_;}

        int  NumVertices() const { return num_vertices_;}


        void SetVertex(int i, std::shared_ptr<BaseVertex> vertex );


        std::shared_ptr< BaseVertex>  GetVertex( int i)   { return  vertices_[i] ;}
        const  std::vector<std::shared_ptr <BaseVertex> >&  GetAllVertex()  const   {  return vertices_;}

        void SetInformation( const Eigen::MatrixXd& information);
        const Eigen::MatrixXd& Information()  const { return  information_;};

        void SetMeasurement(const Eigen::VectorXd& measurement);
        const Eigen::VectorXd&   Measurement() const {  return measurement_;}

        virtual void ComputeJacobians()=0;
        const  std::vector<Eigen::MatrixXd > & Jacobians() const  {  return jacobians_;}

        virtual  void  ComputeResidual()=0;
        const Eigen::VectorXd&   Residual()   const  {return residual_;}

        double Chi2();


        virtual  std::string TypeInfo() const {
            return  "BaseEdge";
        }



    protected:
        unsigned long id_;
        int residual_dimension_;
        int num_vertices_;
        std::vector<std::string> vertices_types_;
        std::vector<  std::shared_ptr<BaseVertex>   >  vertices_;
        Eigen::VectorXd residual_;
        std::vector<Eigen::MatrixXd> jacobians_;
        Eigen::MatrixXd information_;
        Eigen::VectorXd measurement_;

    };

}//end of namespace bundle_adjustment
#endif //BUNDLE_ADJUSTMENT_BASE_EDGE_H
