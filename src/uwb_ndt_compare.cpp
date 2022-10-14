#include "ros/ros.h"
#include <Eigen/Dense>

#include <uwb_YCHIOT/uwb_raw.h>
#include <uwb_YCHIOT/uwb_ndt_compare.h>
#include <geometry_msgs/PoseStamped.h>

Eigen::VectorXd Fixed0_position(3);
Eigen::VectorXd Fixed1_position(3);
Eigen::VectorXd Fixed2_position(3);
Eigen::VectorXd Fixed3_position(3);
// Adding the position of fixed tag or anchor
Eigen::VectorXd Anchor0_tf(3);
Eigen::VectorXd Anchor1_tf(3);
Eigen::VectorXd Anchor2_tf(3);
Eigen::VectorXd Anchor3_tf(3);
// Adding the tranlation between tag or anchor and Lidar on the vehicle

class UwbCompareNdt{
    private:
        ros::Publisher pub_;
        uwb_YCHIOT::uwb_raw uwb_data_;
        uwb_YCHIOT::uwb_ndt_compare uwb_ndt_data_;
        geometry_msgs::PoseStamped pose_data_;
        Eigen::VectorXd F0_;
        Eigen::VectorXd F1_;
        Eigen::VectorXd F2_;
        Eigen::VectorXd F3_;
        Eigen::VectorXd tf0_;
        Eigen::VectorXd tf1_;
        Eigen::VectorXd tf2_;
        Eigen::VectorXd tf3_;
        double F0_ndt_distance_ = 0;
        double F1_ndt_distance_ = 0;
        double F2_ndt_distance_ = 0;
        double F3_ndt_distance_ = 0;
    
    public:
        UwbCompareNdt(std::vector<Eigen::VectorXd> F_position, std::vector<Eigen::VectorXd> A_tf, ros::Publisher pub);
        void UwbrawCallback(const uwb_YCHIOT::uwb_raw& msg);
        void NdtposeCallback(const geometry_msgs::PoseStamped& msg);
        double Calculate_distance(Eigen::VectorXd v1, Eigen::VectorXd v2);
};

UwbCompareNdt::UwbCompareNdt(std::vector<Eigen::VectorXd> F_position, std::vector<Eigen::VectorXd> A_tf, ros::Publisher pub){
    pub_ = pub;
    F0_ = F_position[0];
    F1_ = F_position[1];
    F2_ = F_position[2];
    F3_ = F_position[3];
    tf0_ = A_tf[0];
    tf1_ = A_tf[1];
    tf2_ = A_tf[2];
    tf3_ = A_tf[3];
}

void UwbCompareNdt::UwbrawCallback(const uwb_YCHIOT::uwb_raw& msg){
    uwb_data_ = msg;
    Eigen::VectorXd v_ndt(3);
    Eigen::VectorXd v_A0(3);
    Eigen::VectorXd v_A1(3);
    Eigen::VectorXd v_A2(3);
    Eigen::VectorXd v_A3(3);
    Eigen::VectorXd F(3);
    v_ndt << pose_data_.pose.position.x, pose_data_.pose.position.y, pose_data_.pose.position.z;
    v_A0 = v_ndt + tf0_;
    v_A1 = v_ndt + tf1_;
    v_A2 = v_ndt + tf2_;
    v_A3 = v_ndt + tf3_;

    if (uwb_data_.Tag_id == 0){
        F = F0_;
    }
    else if (uwb_data_.Tag_id == 1){
        F = F1_;
    }
    else if (uwb_data_.Tag_id == 2){
        F = F2_;
    }
    else if (uwb_data_.Tag_id == 3){
        F = F3_;
    }

    uwb_ndt_data_.stamp = uwb_data_.stamp;
    uwb_ndt_data_.Tag_id = uwb_data_.Tag_id;
    uwb_ndt_data_.A0 = uwb_data_.A0;
    uwb_ndt_data_.A1 = uwb_data_.A1;
    uwb_ndt_data_.A2 = uwb_data_.A2;
    uwb_ndt_data_.A3 = uwb_data_.A3;
    uwb_ndt_data_.uwb_to_A0 = uwb_data_.distance_to_A0;
    uwb_ndt_data_.uwb_to_A1 = uwb_data_.distance_to_A1;
    uwb_ndt_data_.uwb_to_A2 = uwb_data_.distance_to_A2;
    uwb_ndt_data_.uwb_to_A3 = uwb_data_.distance_to_A3;
    uwb_ndt_data_.ndt_to_A0 = Calculate_distance(v_A0, F);
    uwb_ndt_data_.ndt_to_A1 = Calculate_distance(v_A1, F);
    uwb_ndt_data_.ndt_to_A2 = Calculate_distance(v_A2, F);
    uwb_ndt_data_.ndt_to_A3 = Calculate_distance(v_A3, F);

    std::cout << uwb_ndt_data_ << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    pub_.publish(uwb_ndt_data_);
}

void UwbCompareNdt::NdtposeCallback(const geometry_msgs::PoseStamped& msg){
    pose_data_ = msg;
}

double UwbCompareNdt::Calculate_distance(Eigen::VectorXd v1, Eigen::VectorXd v2){
    Eigen::VectorXd v(3);
    v = v2 - v1;
    return v.norm();
}

int main(int argc, char **argv) {
    Fixed0_position << 1, 1, 1;
    Fixed1_position << 1, 1, 1;
    Fixed2_position << 1, 1, 1;
    Fixed3_position << 1, 1, 1;
    Anchor0_tf << 1, 1, 1;
    Anchor1_tf << 1, 1, 1;
    Anchor2_tf << 1, 1, 1;
    Anchor3_tf << 1, 1, 1;
    std::vector<Eigen::VectorXd> A_position = {Fixed0_position, Fixed1_position, Fixed2_position, Fixed3_position};
    std::vector<Eigen::VectorXd> A_tf = {Anchor0_tf, Anchor1_tf, Anchor2_tf, Anchor3_tf};

    ros::init(argc, argv, "uwb_ndt_compare");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<uwb_YCHIOT::uwb_ndt_compare>("uwb_ndt_compare", 1);

    UwbCompareNdt uwbcomparendt(A_position, A_tf, pub);

    ros::Subscriber sub = n.subscribe("/uwb_raw", 1, &UwbCompareNdt::UwbrawCallback, &uwbcomparendt);
    ros::Subscriber sub1 = n.subscribe("/ndt_pose", 1, &UwbCompareNdt::NdtposeCallback, &uwbcomparendt);

    ros::spin();
}