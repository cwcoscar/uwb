#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>

#include "uwb.h"
#include "mec_transformation.h"
#include <uwb_YCHIOT/uwb_raw.h>
#include <uwb_YCHIOT/uwb_positioning.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#define Tag_number 8
#define Anchor_number 4
#define threshold_iteration_value 0.001
#define threshold_iteration_num 7

static const llh_InitTypeDef ee_building = {
        .Lat = 22.99665875,
        .Lon = 120.222584889,
        .High = 98.211
};

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
}positioning;

class Uwbpositioning{
    private:
        std::array<int,Anchor_number> publish_seq_={0};
        uwb_YCHIOT::uwb_positioning topic_data_;
        ros::Publisher (*pub_)[Anchor_number];
        tf::TransformBroadcaster br_;
        Uwbanchor (*A_)[Anchor_number];
        Uwbtag (*T_)[Tag_number];
        std::array<double,8> tag_receive_time_;
        
    public:
        Uwbpositioning(ros::Publisher (& pub)[Anchor_number], Uwbanchor (& A)[Anchor_number], Uwbtag (& T)[Tag_number]);
        void UwbCalibrationCallback(const uwb_YCHIOT::uwb_raw& msg);
        void UbloxfixCallback(const sensor_msgs::NavSatFix& msg);
        double Distance(Eigen::VectorXd A, Eigen::VectorXd T);
        void Estimated_range(positioning& variables);
        void R_matrix(positioning& variables);// (measured range - estimated range) from each fixed tags
        void H_matrix(positioning& variables);// (estimate location - fixed tag)/estimated range
        void Pseudo_Invert(positioning& variables); //pseudo-inverse
        void Weight_matrix(positioning& variables, Uwbanchor* A); //generate weight as filtering unavailable range
        void Update_estimate(positioning& variables);
        Eigen::VectorXd Propagate_sol(Uwbanchor* A);
        void Test();
};

Uwbpositioning::Uwbpositioning(ros::Publisher (& pub)[Anchor_number], Uwbanchor (& A)[Anchor_number], Uwbtag (& T)[Tag_number]) 
: pub_(&pub), A_(&A), T_(&T){}

void Uwbpositioning::UwbCalibrationCallback(const uwb_YCHIOT::uwb_raw& msg){
    int num_id = msg.Tag_id;
    bool anchor_availabiliy[Anchor_number] = {msg.A0, msg.A1, msg.A2, msg.A3};
    // bool anchor_availabiliy[Anchor_number] = {msg.A0, msg.A1};
    double to_anchor_range[Anchor_number] = {msg.distance_to_A0, msg.distance_to_A1, msg.distance_to_A2, msg.distance_to_A3};
    // double to_anchor_range[Anchor_number] = {msg.distance_to_A0, msg.distance_to_A1};
    // (*T_)[num_id].update_availability_range(anchor_availabiliy,to_anchor_range);
    (*T_+num_id) -> update_availability_range(anchor_availabiliy,to_anchor_range);
    tag_receive_time_[num_id] = (double)msg.stamp.nsec/1000000000 + msg.stamp.sec;
    for (int i = 0; i < Anchor_number; i++){
        // (*A_)[i].update_availability_range(anchor_availabiliy[i], to_anchor_range[i], num_id);
        (*A_+i) -> update_availability_range(anchor_availabiliy[i], to_anchor_range[i], num_id);
    }
}

void Uwbpositioning::UbloxfixCallback(const sensor_msgs::NavSatFix& msg){
    llh_InitTypeDef rover = {
        .Lat = msg.latitude,
        .Lon = msg.longitude,
        .High = msg.altitude
    };
    enu_InitTypeDef enu = lla2enu(ee_building.Lat, ee_building.Lon, ee_building.High, rover.Lat, rover.Lon, rover.High);
    Eigen::VectorXd gnss_pos(3);
    gnss_pos << enu.e, enu.n, enu.u;
    // std::cout << std::fixed << std::setprecision(3);
    // std::cout << "ENU: " << enu.e << ", " << enu.n << ", " << enu.u << std::endl;
    for (int i = 0; i < Anchor_number; i++){
        (*A_+i) -> revise_xyz(gnss_pos);
    }
}

double Uwbpositioning::Distance(Eigen::VectorXd A, Eigen::VectorXd T){
    Eigen::VectorXd m = A - T;
    double distance = m.norm();
    return distance;
}

void Uwbpositioning::Estimated_range(positioning& variables){
    Eigen::VectorXd initial_xyz = variables.estimated_pos;
    Eigen::VectorXd estimated_range(8);
    for (int i =0; i < Tag_number; i++){
        estimated_range(i) = Distance(initial_xyz, variables.tag_location[i]);
    }
    variables.estimated_range = estimated_range;
}

void Uwbpositioning::R_matrix(positioning& variables){
    Eigen::MatrixXd delta_r(Tag_number,1);
    for (int i = 0; i < Tag_number; i++){
        delta_r(i,0) = variables.to_tag_range[i] - variables.estimated_range[i];
    }
    variables.dr_matrix = delta_r;
}

void Uwbpositioning::H_matrix(positioning& variables){
    Eigen::VectorXd initial_xyz = variables.estimated_pos;
    Eigen::MatrixXd h_matrix(Tag_number,3);
    for (int i = 0; i < Tag_number; i++){
        for (int j = 0; j < 3; j++){
            h_matrix(i,j) = (initial_xyz(j) - variables.tag_location[i][j])/variables.estimated_range[i];
        }
    }
    variables.h_matrix = h_matrix;
}

void Uwbpositioning::Pseudo_Invert(positioning& variables){
    Eigen::MatrixXd Inverse;
    Inverse = variables.h_matrix.transpose() * variables.weight * variables.h_matrix;
    double inverse_det = Inverse.determinant();
    if (inverse_det < 0.0000000001){
        variables.invert_fail = true;
        variables.invert_matrix = Eigen::MatrixXd::Zero(3,8);
        std::cout << "\033[31m" << "Determinant of (H'AH) is 0 : Invertion failed !" << "\033[0m" << std::endl;
        std::cout << "\033[33m" << "(H'AH): " << std::endl << Inverse << "\033[0m" << std::endl;
    }
    else {
        variables.invert_matrix  = Inverse.inverse() * variables.h_matrix.transpose() * variables.weight;
    }
}

void Uwbpositioning::Weight_matrix(positioning& variables, Uwbanchor* A){
    Eigen::MatrixXd weight = Eigen::MatrixXd::Identity(Tag_number,Tag_number);
    int availabilty_count = 0;
    // double closest_time = *std::max_element(tag_receive_time_.begin(), tag_receive_time_.end());
    double secs =ros::Time::now().toSec();
    // std::cout << std::fixed << std::setprecision(9);
    // std::cout << "Now time: " << secs << std::endl;

    for (int i = 0; i < Tag_number; i++){
        if (variables.tag_availabiliy[i] == false){
            weight(i, i) = 0;
        }
        else{
            weight(i, i) = pow(10,tag_receive_time_[i] - secs);
            availabilty_count++;
            if((secs - tag_receive_time_[i]) > 1){
                A -> revise_tag_availabiliy(i, false);
            }
        }
    }
    if (availabilty_count < 3){
        std::cout << "\033[31m" << "Only " << availabilty_count << " tags being available : weighting failed !" << "\033[0m" << std::endl;
    }
    variables.weight = weight;
}

void Uwbpositioning::Update_estimate(positioning& variables){
    if (variables.iteration_num > threshold_iteration_num){
        variables.iteration_continue = false;
        std::cout << "\033[31m" << "Iteration times over " << threshold_iteration_num << " : Iterarion failed !" << "\033[0m" << std::endl;
    }
    else if (variables.iteration_value < threshold_iteration_value){
        variables.iteration_continue = false;
    }
    else{
        variables.delta_w = variables.invert_matrix * variables.dr_matrix;
        variables.estimated_pos = variables.estimated_pos + variables.delta_w;
        variables.iteration_value = variables.delta_w.norm();
        variables.iteration_num++;
    }
}

Eigen::VectorXd Uwbpositioning::Propagate_sol(Uwbanchor* A){
    positioning variables;
    variables.estimated_pos = A -> get_xyz();
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\033[32m" << "Initial ENU : (" << variables.estimated_pos[0] << ", " 
    << variables.estimated_pos[1] << ", " << variables.estimated_pos[2] << ")\033[0m" << std::endl;
    variables.tag_availabiliy = A -> get_tag_availabiliy();
    variables.to_tag_range = A -> get_to_tag_range();
    variables.tag_location = A -> get_tag_location();
    variables.invert_fail = false;
    variables.iteration_continue = true;
    variables.iteration_num = 1;
    variables.iteration_value = INT_MAX;
    while (variables.iteration_continue){
        Estimated_range(variables);
        R_matrix(variables);
        H_matrix(variables);
        variables.weight = Eigen::MatrixXd::Zero(Tag_number,Tag_number);
        variables.weight(0, 0) = 1;
        variables.weight(1, 1) = 1;
        variables.weight(3, 3) = 1;
        // Weight_matrix(variables, A);
        Pseudo_Invert(variables);
        if (variables.invert_fail){
            break;
        }
        else{
            Update_estimate(variables);
        }
    }
    // if (variables.invert_fail != true) A -> revise_xyz(variables.estimated_pos);
    return (variables.invert_fail == true && variables.iteration_num == 1) ? A -> get_xyz() : variables.estimated_pos;
}

void Uwbpositioning::Test(){
    tf::Transform transform;
    tf::Quaternion current_q;
    ros::Time now = ros::Time::now();

    std::cout << "Time stamp: " << now << std::endl;
    for (int i =0; i < Anchor_number; i++){
        std::cout << "\033[32m" << "Anchor number " << i << "\033[0m" << std::endl;
        Eigen::VectorXd result = Propagate_sol((*A_+i));
        std::cout << "\033[32m" << "Localization: (" << result[0] << ", " << result[1] << ", " << result[2] << ")" << "\033[0m" << std::endl;
        
        topic_data_.header.seq = publish_seq_[i]++;
        topic_data_.header.stamp = now;
        topic_data_.header.frame_id = "uwb_A" + std::to_string(i);
        topic_data_.x = result[0];
        topic_data_.y = result[1];
        topic_data_.z = result[2];
        topic_data_.roll = 0;
        topic_data_.pitch = 0;
        topic_data_.yaw = 0;
        (*pub_+i) -> publish(topic_data_); 

        current_q.setRPY(topic_data_.roll, topic_data_.pitch, topic_data_.yaw);
        transform.setOrigin(tf::Vector3(topic_data_.x, topic_data_.y, topic_data_.z));
        transform.setRotation(current_q);
        br_.sendTransform(tf::StampedTransform(transform, now, "/map", "uwb_A" + std::to_string(i)));

        if (i < Anchor_number-1){
            std::cout << "\033[32m" << "---" << "\033[0m" << std::endl;
        }
    }
    std::cout << "--------------------" << std::endl;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "uwb_positioning");
    ros::NodeHandle n;

    ros::Publisher pub0 = n.advertise<uwb_YCHIOT::uwb_positioning>("/uwb_position/A0", 1);
    ros::Publisher pub1 = n.advertise<uwb_YCHIOT::uwb_positioning>("/uwb_position/A1", 1);
    ros::Publisher pub2 = n.advertise<uwb_YCHIOT::uwb_positioning>("/uwb_position/A2", 1);
    ros::Publisher pub3 = n.advertise<uwb_YCHIOT::uwb_positioning>("/uwb_position/A3", 1);
    ros::Publisher pub[Anchor_number] = {pub0, pub1, pub2, pub3};
    // ros::Publisher pub[Anchor_number] = {pub0, pub1};

    Eigen::VectorXd xyz(3);
    // xyz << 51, 190, -47;
    xyz << 45, 175, -60;
    Eigen::VectorXd location(3);
    std::array<Eigen::VectorXd,Tag_number> tag_location;
    location << 59.0414, 194.1903, -47.7806;
    tag_location[0] = location;

    location << 43.1544, 194.8983, -47.5321;
    tag_location[1] = location;

    location << 0, 10, 0;
    tag_location[2] = location;

    location << 51.7699, 184.3686, -46.6854;
    tag_location[3] = location;

    location << 5, 0, 0;
    tag_location[4] = location;

    location << 6, 0, 0;
    tag_location[5] = location;

    location << 7, 0, 0;
    tag_location[6] = location;

    location << 8, 0, 0;
    tag_location[7] = location;

    // Establish Anchor 0~3
    Uwbanchor Anchor[Anchor_number] = {
        Uwbanchor(0, xyz, tag_location),
        Uwbanchor(1, xyz, tag_location),
        Uwbanchor(2, xyz, tag_location),
        Uwbanchor(3, xyz, tag_location)
    };
    // Uwbanchor Anchor[Anchor_number] = {
    //     Uwbanchor(0, xyz, tag_location),
    //     Uwbanchor(1, xyz, tag_location)
    // };
    // Establish Tag 0~7
    Uwbtag Tag[Tag_number] = {
        Uwbtag(0, tag_location[0]),
        Uwbtag(1, tag_location[1]),
        Uwbtag(2, tag_location[2]),
        Uwbtag(3, tag_location[3]),
        Uwbtag(4, tag_location[4]),
        Uwbtag(5, tag_location[5]),
        Uwbtag(6, tag_location[6]),
        Uwbtag(7, tag_location[7])
    };

    Uwbpositioning uwbpositioning(pub, Anchor, Tag);

    ros::Subscriber sub = n.subscribe("/uwb_calibration", 1, &Uwbpositioning::UwbCalibrationCallback, &uwbpositioning);
    ros::Subscriber sub1 = n.subscribe("/ublox_f9k/fix", 1, &Uwbpositioning::UbloxfixCallback, &uwbpositioning);



    ros::Rate loop_rate(3);//uwb_calibration update averaging 3.57 Hz  
    while (ros::ok()){
        uwbpositioning.Test();
        ros::spinOnce();
        loop_rate.sleep();
    }
}