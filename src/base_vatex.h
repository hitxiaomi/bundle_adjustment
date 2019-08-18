//
// Created by xiaomi on 8/8/19.
//

#ifndef BUNDLE_ADJUSTMENT_BASE_VATEX_H
#define BUNDLE_ADJUSTMENT_BASE_VATEX_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <stack>
#include <assert.h>


namespace  bundle_adjustment {
    class  BaseVertex {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef  std::stack<Eigen::VectorXd > BackUpStackType;

        explicit BaseVertex(int dimension,int local_dimension=-1);



        void SetId(int id){ id_=id;}
        unsigned long  Id() const { return id_; }

        int Dimension() const { return  dimension_; }
        int LocalDimension() const {return  local_dimension_;}

        void SetHessianIndex(unsigned long hessian_index)  { hessian_index_=hessian_index; }
        int  HessianIndex()  const {return  hessian_index_; }

        void SetEstimate(const  Eigen::VectorXd & estimate){ estimate_=estimate;    }
        Eigen::VectorXd  Estimate() const { return estimate_;}
        Eigen::VectorXd& Estimate()  {      return  estimate_;  }
        bool IsFixed() const  { return fixed_;};


        void SetFixed(bool fixed ) { fixed_=fixed;}

        virtual  std::string TypeInfo() const  =0;
        virtual  void Oplus(const Eigen::VectorXd & update ) =0;

        virtual  void Push(){
            back_up_estimate_.push(estimate_);

        }
        virtual  void Pop(){
            assert(!back_up_estimate_.empty());
            estimate_=back_up_estimate_.top();
            back_up_estimate_.pop();
        }
        virtual  void DiscardTop(){
            assert(!back_up_estimate_.empty());
            back_up_estimate_.pop();
        }



    protected:
        Eigen::VectorXd  estimate_;
        BackUpStackType back_up_estimate_;
        unsigned long   id_;
        int dimension_;
        int local_dimension_;
        int   hessian_index_;
        bool fixed_=false;



    };

} //end of namespace bundle_adjustment
#endif //BUNDLE_ADJUSTMENT_BASE_VATEX_H
