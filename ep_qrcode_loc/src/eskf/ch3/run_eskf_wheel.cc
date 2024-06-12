//
// Created by xiang on 2021/11/11.
//

#include "ch3/eskf.hpp"
#include "ch3/static_imu_init.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"
#include "utm_convert.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>

DEFINE_string(txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, true, "是否显示图形界面");
DEFINE_bool(with_odom, true, "是否加入轮速计信息");

/**
 * 本程序演示使用RTK+IMU进行组合导航
 */
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    std::string path = "/home/xun/ws/calib_data/data_for_lidar2imu_02_91/";

    // FLAGS_txt_path = path + "to_eskf_00.txt";
    // std::ofstream fout(path + "odom_eskf_00.txt");

    FLAGS_txt_path = path + "to_eskf_01.txt";
    std::ofstream fout(path + "odom_eskf_01.txt");
    std::ofstream fout2(path + "pcd_eskf_01.txt");

    // FLAGS_txt_path = path + "to_eskf_10.txt";
    // std::ofstream fout(path + "odom_eskf_10.txt");

    // FLAGS_txt_path = path + "to_eskf_11.txt";
    // std::ofstream fout(path + "odom_eskf_11.txt");
    
    FLAGS_with_odom = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }

    // 初始化器
    sad::StaticIMUInit imu_init;  // 使用默认配置
    sad::ESKFD eskf;

    sad::TxtIO io(FLAGS_txt_path);
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) {
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };
    auto save_mat4 = [](std::ofstream& fout, const Mat4d& m) {
        fout << m(0,0) << " " << m(0,1) << " " << m(0,2) << " " << m(0,3) << " ";
        fout << m(1,0) << " " << m(1,1) << " " << m(1,2) << " " << m(1,3) << " ";
        fout << m(2,0) << " " << m(2,1) << " " << m(2,2) << " " << m(2,3) << " ";
    };

    auto save_result = [&save_vec3, &save_quat](std::ofstream& fout, const sad::NavStated& save_state) {
        fout << std::setprecision(18) << save_state.timestamp_ * 1000000 << " " << std::setprecision(9);
        save_vec3(fout, save_state.p_);
        save_quat(fout, save_state.R_.unit_quaternion());
        save_vec3(fout, save_state.v_);
        save_vec3(fout, save_state.bg_);
        save_vec3(fout, save_state.ba_);
        fout << std::endl;
    };
    auto save_result_2 = [&save_mat4](std::ofstream& fout, const sad::NavStated& save_state) {
        fout << std::setprecision(18) << save_state.timestamp_ * 1000000 << " " << std::setprecision(9);
        save_mat4(fout, save_state.GetSE3().matrix());
        fout << std::endl;
    };

    bool imu_inited = true, gnss_inited = false, mag_inited = false, iic_inited = false; 

    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    

    /// 设置各类回调函数
    bool first_gnss_set = false;
    bool first_iic_set = false;
    Vec3d origin = Vec3d::Zero();

    io.SetIMUProcessFunc([&](const sad::IMU& imu) {
          /// IMU 处理函数
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }

          /// 需要IMU初始化
          if (!imu_inited) {
              // 读取初始零偏，设置ESKF
              sad::ESKFD::Options options;
              // 噪声由初始化器估计
              options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
              options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
              eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
              imu_inited = true;
              return;
          }

        //   if (!mag_inited) {
        //       /// 等待有效的RTK数据
        //       return;
        //   }
          
          if (!iic_inited) {
              /// 等待有效的RTK数据
              return;
          }

          /// GNSS 也接收到之后，再开始进行预测
          eskf.Predict(imu);

          /// predict就会更新ESKF，所以此时就可以发送数据
          auto state = eskf.GetNominalState();
          if (ui) {
              ui->UpdateNavState(state);
          }

          /// 记录数据以供绘图
          save_result_2(fout, state);

          usleep(1e3);
    })
    .SetGNSSProcessFunc([&](const sad::GNSS& gnss) {
        /// GNSS 处理函数
        if (!imu_inited) {
            return;
        }

        sad::GNSS gnss_convert = gnss;
        if (!sad::ConvertGps2UTM(gnss_convert, antenna_pos, FLAGS_antenna_angle) || !gnss_convert.heading_valid_) {
            return;
        }

        /// 去掉原点
        if (!first_gnss_set) {
            origin = gnss_convert.utm_pose_.translation();
            first_gnss_set = true;
        }
        gnss_convert.utm_pose_.translation() -= origin;

        // 要求RTK heading有效，才能合入ESKF
        eskf.ObserveGps(gnss_convert);

        auto state = eskf.GetNominalState();
        if (ui) {
            ui->UpdateNavState(state);
        }
        save_result_2(fout, state);

        gnss_inited = true;
    })
    .SetOdomProcessFunc([&](const sad::Odom& odom) {
        /// Odom 处理函数，本章Odom只给初始化使用
        imu_init.AddOdom(odom);
        if (FLAGS_with_odom && imu_inited) {
            //eskf.ObserveWheelSpeed(odom);
        }
    })
    .SetMagProcessFunc([&](const sad::Mag& mag) {
        /// Mag 处理函数
        if (!imu_inited) {
            return;
        }
        //eskf.ObserveMagnetometer(mag);
        mag_inited = true;
    })
    .SetIICProcessFunc([&](const sad::IIC& const_iic) {
        /// IIC 处理函数
        if (!imu_inited) { return; }

        sad::IIC iic = const_iic;
        /// 去掉原点
        if (!first_iic_set) {
            origin = iic.pose_.translation();
            first_iic_set = true;
        }
        iic.pose_.translation() -= origin;

        /// 合入ESKF
        eskf.ObserveImuIncremental(iic);

        auto state = eskf.GetNominalState();
        if (ui) {
            ui->UpdateNavState(state);
        }
        save_result_2(fout, state);
        save_result(fout2, state);

        iic_inited = true;
    })
    .Go();

    while (ui && !ui->ShouldQuit()) {
        usleep(1e5);
    }
    if (ui) {
        ui->Quit();
    }
    return 0;
}