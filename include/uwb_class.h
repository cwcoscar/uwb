#ifndef UWB_H_
#define UWB_H_
#include <Eigen/Dense>
#include <iostream>
#include <uwb_ins_eskf_msgs/uwbFIX.h>

class Uwb{
    private:
        int id_;
        Eigen::VectorXd xyz_;

    public:
        Uwb(int id , Eigen::VectorXd xyz);
        void set_id(int id){id_ = id;};
        void set_location(Eigen::VectorXd xyz){xyz_ = xyz;};
        Eigen::VectorXd get_xyz(){return xyz_;};
        void revise_xyz(Eigen::VectorXd estimated_xyz){xyz_ = estimated_xyz;};
};

class Uwbanchor: public Uwb{
    private:
        Eigen::VectorXd xyz_ublox_;
        std::array<bool,8> tag_availabiliy_;
        std::array<double,8> to_tag_range_ = {0};
        std::array<Eigen::VectorXd,8> tag_location_;
        uwb_ins_eskf_msgs::uwbFIX last_fix_;
        Eigen::Vector3d location_b_; // anchor location in b-frame
        std::vector<Eigen::VectorXd> position_window_;

    public:
        Uwbanchor(int id , Eigen::VectorXd xyz, std::array<Eigen::VectorXd,8> tag_location, Eigen::Vector3d location_b);
        void revise_xyz_ublox(Eigen::VectorXd xyz_ublox){xyz_ublox_ = xyz_ublox;};
        Eigen::VectorXd get_xyz_ublox(){return xyz_ublox_;};
        void update_availability_range(bool tag_availabiliy, double to_tag_range, int id);
        std::array<bool,8> get_tag_availabiliy(){return tag_availabiliy_;};
        std::array<double,8> get_to_tag_range(){return to_tag_range_;};
        std::array<Eigen::VectorXd,8> get_tag_location(){return tag_location_;};
        void revise_tag_availabiliy(int num, bool availabilty){tag_availabiliy_[num] = availabilty;};
        void update_last_fix(uwb_ins_eskf_msgs::uwbFIX current_fix){last_fix_ = current_fix;};
        uwb_ins_eskf_msgs::uwbFIX get_last_fix(){return last_fix_;};
        Eigen::VectorXd get_location_b(){return location_b_;};
        void update_position_window(std::vector<Eigen::VectorXd> current_position_window){position_window_ = current_position_window;};
        std::vector<Eigen::VectorXd> get_position_window(){return position_window_;};
};

class Uwbtag: public Uwb{
    private:
        bool anchor_availabiliy_[4];
        double to_anchor_range_[4] = {0};
    public:
        Uwbtag(int id , Eigen::VectorXd xyz);
        void update_availability_range(bool anchor_availabiliy[4], double to_anchor_range[4]);
};

#endif