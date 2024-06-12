//
// Created by xiang on 2021/7/19.
//

#ifndef MAPPING_IIC_H
#define MAPPING_IIC_H

#include "common/eigen_types.h"

namespace sad {

struct IIC {
    IIC() {}
    IIC(double timestamp, Vec3d pos, Quatd ori):    timestamp_(timestamp), 
                                                    pos_(pos), 
                                                    ori_(ori){
        pose_ = SE3(ori_, pos_);
    }

    double timestamp_ = 0.0;
    Vec3d pos_ = Vec3d::Zero(); //position
    Quatd ori_ = Quatd::Identity();   //orientation
    SE3 pose_;
};

}  // namespace sad

#endif  // MAPPING_IIC_H
