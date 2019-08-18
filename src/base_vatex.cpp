//
// Created by xiaomi on 8/8/19.
//

#include "base_vatex.h"
namespace  bundle_adjustment{
    BaseVertex:: BaseVertex(int dimension,int local_dimension){
        dimension_= dimension;
        local_dimension_= (local_dimension==-1? dimension:local_dimension);
        estimate_.resize(dimension_);
    }





}