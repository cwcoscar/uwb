#ifndef UWB_POSITIONING_H_
#define UWB_POSITIONING_H_
#include "ros/ros.h"
#include "uwb_class.h"
#include <uwb_ins_eskf_msgs/uwbRAW.h>
#include <uwb_ins_eskf_msgs/uwbFIX.h>
#include <uwb_ins_eskf_msgs/uwbTAG.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>

#define Tag_number 8
#define Anchor_number 4
#define threshold_iteration_value_2d 0.00001
#define threshold_iteration_value_3d 0.5
#define threshold_iteration_num 10
#define ini_manual 1
#define ini_ublox 2
#define ini_ublox_uwb 3
#define weight_manual 1
#define weight_time_varied 2
#define NCKUEE_LATITUDE 22.99665875 //  degree
#define NCKUEE_LONGITUDE 120.222584889 //  degree
#define NCKUEE_HEIGHT 98.211 // meter

typedef struct Positioning_config{
    int fix_mode;
    int ini_mode;
    bool ublox_received = false;
    int weight_mode;
    std::string enabled_anchor;
    int position_window = 10;
}config;

typedef struct Positioning_param{
    std::array<bool,8> tag_availabiliy;
    std::array<double,8> to_tag_range;
    std::array<Eigen::VectorXd,8> tag_location;
    Eigen::VectorXd estimated_pos;
    Eigen::VectorXd estimated_range;
    Eigen::MatrixXd dr_matrix;
    Eigen::MatrixXd h_matrix;
    Eigen::MatrixXd invert_matrix;
    bool invert_fail;
    Eigen::MatrixXd weight;
    Eigen::MatrixXd delta_w;
    bool iteration_continue;
    double iteration_value;
    double iteration_num;
    // Record the minimum iteration value, corresponding delta_w and corresponding estimated position
    double tmp_iter_v = INT_MAX;
    Eigen::VectorXd tmp_esti_pos;
    Eigen::MatrixXd tmp_delta_w;
}positioning;

class Uwbpositioning{
    private:
        Uwbanchor (*A_)[Anchor_number];
        Uwbtag (*T_)[Tag_number];
        config positioning_config_;
        std::array<double,8> tag_receive_time_;
        ros::Publisher (*pub_)[Anchor_number];
        tf::TransformBroadcaster br_;
        
    public:
        Uwbpositioning(ros::Publisher (& pub)[Anchor_number], Uwbanchor (& A)[Anchor_number], Uwbtag (& T)[Tag_number], config positioning_config);
        void UwbCalibrationCallback(const uwb_ins_eskf_msgs::uwbRAW& msg);
        void UbloxfixCallback(const sensor_msgs::NavSatFix& msg);
        double Distance(Eigen::VectorXd A, Eigen::VectorXd T);
        void Estimated_range(positioning& variables);
        void R_matrix(positioning& variables);// (measured range - estimated range) from each fixed tags
        void H_matrix(positioning& variables);// (estimate location - fixed tag)/estimated range
        void Pseudo_Invert(positioning& variables); //pseudo-inverse
        void Weight_matrix(positioning& variables, Uwbanchor* A); //generate weight as filtering unavailable range
        void Update_estimate(positioning& variables);
        bool Propagate_sol(Uwbanchor* A, Eigen::VectorXd& result);

        // For 2D
        void Estimated_range_2d(positioning& variables);
        void H_matrix_2d(positioning& variables);// (estimate location - fixed tag)/estimated range
        void Pseudo_Invert_2d(positioning& variables); //pseudo-inverse
        void Update_estimate_2d(positioning& variables);
        bool Propagate_sol_2d(Uwbanchor* A, Eigen::VectorXd& result);
        //

        void show_config();
        Eigen::Vector3d estimate_velocity(Eigen::Vector3d now_enu, Eigen::Vector3d last_enu, double time_interval);
        Eigen::Vector3d estimate_orientation(Eigen::Vector3d now_enu, Eigen::Vector3d last_enu);
        void Publish_uwb(uwb_ins_eskf_msgs::uwbFIX &now_fix, Eigen::VectorXd now_enu, int A_num);
        void send_tf(uwb_ins_eskf_msgs::uwbFIX now_fix, Eigen::Vector3d now_enu, int A_num);
        void Test();

};


#endif