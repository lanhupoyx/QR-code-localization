//
// Created by xiang on 2021/7/19.
//

#ifndef MAPPING_MAG_H
#define MAPPING_MAG_H

#include "common/eigen_types.h"

namespace sad {

struct Mag {
    Mag() {}
    Mag(double timestamp, Quatd q): timestamp_(timestamp), q_(q){
        pose_ = SO3(q_);
    }

    double timestamp_ = 0.0;
    Quatd q_;   //四元数
    SO3 pose_;  // 用于后处理的6DoF Pose
};

}  // namespace sad

#endif  // MAPPING_MAG_H
