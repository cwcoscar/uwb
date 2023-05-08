#include "ros/ros.h"
#include <Eigen/Dense>
#include <cmath>

#include <uwb_ins_eskf_msgs/uwbRAW.h>
#include <uwb_ins_eskf_msgs/uwb_ndt_compare.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Quaternion.h"
// #include "tf/transform_datatypes.h"

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
        uwb_ins_eskf_msgs::uwbRAW uwb_data_;
        uwb_ins_eskf_msgs::uwb_ndt_compare uwb_ndt_data_;
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
        void UwbrawCallback(const uwb_ins_eskf_msgs::uwbRAW& msg);
        void NdtposeCallback(const geometry_msgs::PoseStamped& msg);
        double Calculate_distance(Eigen::VectorXd v1, Eigen::VectorXd v2);
        Eigen::VectorXd transform2mapframe(Eigen::VectorXd tf);
        Eigen::MatrixXd quaternion2Rotation(geometry_msgs::Quaternion quat);
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

void UwbCompareNdt::UwbrawCallback(const uwb_ins_eskf_msgs::uwbRAW& msg){
    uwb_data_ = msg;
    Eigen::VectorXd v_ndt(3);
    Eigen::VectorXd v_A0(3);
    Eigen::VectorXd v_A1(3);
    Eigen::VectorXd v_A2(3);
    Eigen::VectorXd v_A3(3);
    Eigen::VectorXd F(3);
    v_ndt << pose_data_.pose.position.x, pose_data_.pose.position.y, pose_data_.pose.position.z;
    v_A0 = v_ndt + transform2mapframe(tf0_);
    v_A1 = v_ndt + transform2mapframe(tf1_);
    v_A2 = v_ndt + transform2mapframe(tf2_);
    v_A3 = v_ndt + transform2mapframe(tf3_);

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

    // std::cout << uwb_ndt_data_ << std::endl;
    std::cout << "NDT A0: " << uwb_ndt_data_.ndt_to_A0 << std::endl;
    std::cout << "UWB A0: " << uwb_ndt_data_.uwb_to_A0 << std::endl;
    std::cout << "UWB - NDT: " << uwb_ndt_data_.uwb_to_A0 - uwb_ndt_data_.ndt_to_A0 << std::endl;
    // std::cout << "transform2mapframe: " << std::endl << transform2mapframe(tf0_) << std::endl;
    std::cout << "NDT A3: " << uwb_ndt_data_.ndt_to_A3 << std::endl;
    std::cout << "UWB A3: " << uwb_ndt_data_.uwb_to_A3 << std::endl;
    std::cout << "UWB - NDT: " << uwb_ndt_data_.uwb_to_A3 - uwb_ndt_data_.ndt_to_A3 << std::endl;
    // std::cout << "transform2mapframe: " << std::endl << transform2mapframe(tf3_) << std::endl;
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

Eigen::VectorXd UwbCompareNdt::transform2mapframe(Eigen::VectorXd tf){
    Eigen::Matrix<double, 3, 3> R = quaternion2Rotation(pose_data_.pose.orientation);
    Eigen::VectorXd transform_tf = R*tf;
    return transform_tf;
}

Eigen::MatrixXd UwbCompareNdt::quaternion2Rotation(geometry_msgs::Quaternion quat){
    Eigen::Matrix<double, 3, 3> R;
    double q0 = quat.w;
    double q1 = quat.x;
    double q2 = quat.y;
    double q3 = quat.z;
    R << (2*(pow(q0,2)+pow(q1,2))-1), (2*(q1*q2-q0*q3)), (2*(q1*q3+q0*q2)),
    (2*(q1*q2+q0*q3)), (2*(pow(q0,2)+pow(q2,2))-1), (2*(q2*q3-q0*q1)),
    (2*(q1*q3-q0*q2)), (2*(q2*q3+q0*q1)), (2*(pow(q0,2)+pow(q3,2))-1);
    return R;
}

int main(int argc, char **argv) {
    Fixed0_position << 65.8844985962, 6.86607885361, -42.2941116333;
    Fixed1_position << 1, 1, 1;
    Fixed2_position << 1, 1, 1;
    Fixed3_position << 1, 1, 1;
    Anchor0_tf << 0.97, -0.4, 1.67; // front right
    Anchor1_tf << 1.33, 0.4, 1.67; // front left
    Anchor2_tf << 0.17, -0.4, 1.67; // rear right
    Anchor3_tf << 0.17, 0.4, 1.67; // rear left
    std::vector<Eigen::VectorXd> A_position = {Fixed0_position, Fixed1_position, Fixed2_position, Fixed3_position};
    std::vector<Eigen::VectorXd> A_tf = {Anchor0_tf, Anchor1_tf, Anchor2_tf, Anchor3_tf};

    ros::init(argc, argv, "uwb_ndt_compare");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<uwb_ins_eskf_msgs::uwb_ndt_compare>("uwb_ndt_compare", 1);

    UwbCompareNdt uwbcomparendt(A_position, A_tf, pub);

    ros::Subscriber sub = n.subscribe("/uwb_raw", 1, &UwbCompareNdt::UwbrawCallback, &uwbcomparendt);
    ros::Subscriber sub1 = n.subscribe("/ndt_pose", 1, &UwbCompareNdt::NdtposeCallback, &uwbcomparendt);

    ros::spin();
}