#ifndef UWB_H_
#define UWB_H_
#include <Eigen/Dense>

class Uwb{
    private:
        int id_;
        Eigen::VectorXd xyz_;

    public:
        Uwb(int id , Eigen::VectorXd xyz);
        void set_id(int id);
        void set_location(Eigen::VectorXd xyz);
        Eigen::VectorXd get_xyz();
        void display_info();
};

class Uwbanchor: public Uwb{
    private:
        bool (*t_a_)[8];
        double (*t_t_r_)[8];
        std::array<bool,8> tag_availabiliy_;
        std::array<double,8> to_tag_range_ = {0};
        std::array<Eigen::VectorXd,8> tag_location_;

    public:
        Uwbanchor(int id , Eigen::VectorXd xyz, std::array<Eigen::VectorXd,8> tag_location);
        void update_availability_range(bool tag_availabiliy, double to_tag_range, int id);
        // bool get_tag_availabiliy(int index);
        // double get_to_tag_range(int index);
        // Eigen::VectorXd get_tag_location(int index);
        std::array<bool,8> get_tag_availabiliy();
        std::array<double,8> get_to_tag_range();
        std::array<Eigen::VectorXd,8> get_tag_location();
        void display_info();
};

class Uwbtag: public Uwb{
    private:
        bool (*a_a_)[4];
        double (*t_a_r_)[4];
        bool anchor_availabiliy_[4];
        double to_anchor_range_[4] = {0};
    public:
        Uwbtag(int id , Eigen::VectorXd xyz);
        void update_availability_range(bool anchor_availabiliy[4], double to_anchor_range[4]);
        void display_info();
};

#endif