#include "coordinate_mat_transformation.h" //For llh enu tf
#include "uwb_positioning.h"

Uwbpositioning::Uwbpositioning(ros::Publisher (& pub)[Anchor_number], Uwbanchor (& A)[Anchor_number], Uwbtag (& T)[Tag_number], config positioning_config) 
: pub_(&pub), A_(&A), T_(&T), positioning_config_(positioning_config){
    uwb::uwbFIX ini_fix;
    ini_fix.header.frame_id = "ini";
    for (int i = 0; i < Anchor_number; i++){
        (*A_+i) -> update_last_fix(ini_fix);
    }
}

void Uwbpositioning::UwbCalibrationCallback(const uwb::uwbRAW& msg){
    int num_id = msg.Tag_id;
    bool anchor_availabiliy[Anchor_number] = {msg.A0, msg.A1, msg.A2, msg.A3};
    double to_anchor_range[Anchor_number] = {msg.distance_to_A0, msg.distance_to_A1, msg.distance_to_A2, msg.distance_to_A3};
    
    // Update the data of UWB received 
    (*T_+num_id) -> update_availability_range(anchor_availabiliy,to_anchor_range);
    tag_receive_time_[num_id] = (double)msg.stamp.nsec/1000000000 + msg.stamp.sec;
    for (int i = 0; i < Anchor_number; i++){
        (*A_+i) -> update_availability_range(anchor_availabiliy[i], to_anchor_range[i], num_id);
    }
    // double secs =ros::Time::now().toSec();
    // std::cout << "t d: " << secs - tag_receive_time_[num_id] << std::endl;
    // std::cout << "t: " << tag_receive_time_[num_id] << std::endl;

    // if (num_id == 3){Test();}
    Test();
}

void Uwbpositioning::UbloxfixCallback(const sensor_msgs::NavSatFix& msg){
    Eigen::Vector3d rover(msg.latitude, msg.longitude, msg.altitude);
    Eigen::Vector3d ref_lla(NCKUEE_LATITUDE, NCKUEE_LONGITUDE, NCKUEE_HEIGHT);

    //Transform the LLH of rover into ENU based on the LLH of EE building
    Eigen::Vector3d gnss_pos = coordinate_mat_transformation::lla2enu(rover, ref_lla);
    // add error
    // gnss_pos(0) = gnss_pos(0) - 10;
    // gnss_pos(1) = gnss_pos(1) - 10;


    //Update the intial position of the rover in the algorithm
    for (int i = 0; i < Anchor_number; i++){
        (*A_+i) -> revise_xyz_ublox(gnss_pos);
        (*A_+i) -> revise_xyz(gnss_pos);
    }
    positioning_config_.ublox_received = true;
}

// Calculate distance between 2 vector
double Uwbpositioning::Distance(Eigen::VectorXd A, Eigen::VectorXd T){
    Eigen::VectorXd m = A - T;
    double distance = m.norm();
    return distance;
}

// Calculate the distance between guess location and each static stations(tags)
void Uwbpositioning::Estimated_range(positioning& variables){
    Eigen::VectorXd initial_xyz = variables.estimated_pos;
    Eigen::VectorXd estimated_range(8);
    for (int i =0; i < Tag_number; i++){
        estimated_range(i) = Distance(initial_xyz, variables.tag_location[i]);
    }
    variables.estimated_range = estimated_range;
}

// Calculate the difference between measurement range and guess range
void Uwbpositioning::R_matrix(positioning& variables){
    Eigen::MatrixXd delta_r(Tag_number,1);
    for (int i = 0; i < Tag_number; i++){
        delta_r(i,0) = variables.to_tag_range[i] - variables.estimated_range[i];
    }
    variables.dr_matrix = delta_r;
}

// Calculate each elements in H matrix. [(x_head - x_tag)/dist_head ...]
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

// Generate weight depending on the time difference between the last data received and now
//            (last data time - now)               -(time difference)
// weight = 10                       => weight = 10
void Uwbpositioning::Weight_matrix(positioning& variables, Uwbanchor* A){
    Eigen::MatrixXd weight = Eigen::MatrixXd::Identity(Tag_number,Tag_number);
    int availabilty_count = 0;
    double secs =ros::Time::now().toSec();
    if (positioning_config_.weight_mode == weight_time_varied){
        for (int i = 0; i < Tag_number; i++){
            if (variables.tag_availabiliy[i] == false){
                weight(i, i) = 0;
            }
            else{
                weight(i, i) = pow(20,tag_receive_time_[i] - secs);
                availabilty_count++;
                // std::cout << "weight:" << std::endl << weight << std::endl;
                // If the data hadn't been updated over 1 second, it will be set as unavailable
                if((secs - tag_receive_time_[i]) > 1){
                    A -> revise_tag_availabiliy(i, false);
                }
            }
        }
    }
    else if (positioning_config_.weight_mode == weight_manual){
        for (int i = 0; i < Tag_number; i++){
            if (variables.tag_availabiliy[i] == false){
                weight(i, i) = 0;
            }
            else{
                weight(i, i) = 1;
                availabilty_count++;
                // If the data hadn't been updated over 1 second, it will be set as unavailable
                if((secs - tag_receive_time_[i]) > 1.5){
                    A -> revise_tag_availabiliy(i, false);
                }
            }
        }
    }
    if (availabilty_count < 3){
        std::cout << "\033[31m" << "Only " << availabilty_count << " tags being available : weighting failed !" << "\033[0m" << std::endl;
        
    }
    variables.weight = weight;
    // std::cout << "weight: " << weight << std::endl;
}

// dr = H * dw
//   T               T
// (H  * W) * dr = (H  * W) * H * dw
//   T         -1    T
// (H  * W * H)   * H  * W * dr = dw
//   T         
// (H  * W * H) => Inverse
//   T         -1    T
// (H  * W * H)   * H  * W => invert_matrix
void Uwbpositioning::Pseudo_Invert(positioning& variables){
    Eigen::MatrixXd Inverse;
    Inverse = variables.h_matrix.transpose() * variables.weight * variables.h_matrix;
    double inverse_det = Inverse.determinant();
    // If determinant is too small, the inverse of the matrix will diverge => invert fail  
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

void Uwbpositioning::Update_estimate(positioning& variables){
    if (variables.iteration_num > threshold_iteration_num){
        variables.iteration_continue = false;
        std::cout << "\033[31m" << "Iteration times over " << threshold_iteration_num << " !" << "\033[0m" << std::endl;
    }
    else if (variables.iteration_value < threshold_iteration_value_3d){
        std::cout << "\033[32m" << "Iteration value < " << threshold_iteration_value_3d << " !" << "\033[0m" << std::endl;
        variables.iteration_continue = false;
    }
    else{
        variables.delta_w = variables.invert_matrix * variables.dr_matrix;
        variables.estimated_pos = variables.estimated_pos + variables.delta_w;
        variables.iteration_value = variables.delta_w.norm();
        variables.iteration_num++;
        std::cout << "\033[33m" << "Iteration value : " << variables.iteration_value << " !" << "\033[0m" << std::endl;
    }
}

Eigen::VectorXd Uwbpositioning::Propagate_sol(Uwbanchor* A){
    positioning variables;
    variables.estimated_pos = A -> get_xyz();
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\033[32m" << "Initial ENU : (" << variables.estimated_pos[0] << ", " 
    << variables.estimated_pos[1] << ", " << variables.estimated_pos[2] << ")\033[0m" << std::endl;
    variables.tag_availabiliy = A -> get_tag_availabiliy();
    variables.to_tag_range = A -> get_to_tag_range();
    variables.tag_location = A -> get_tag_location();
    variables.invert_fail = false;
    variables.iteration_continue = true;
    variables.iteration_num = 0;
    variables.iteration_value = INT_MAX;
    // If the iteration had been continued over the threshold times, the iteration will be interrupted
    while (variables.iteration_continue){
        Estimated_range(variables);
        R_matrix(variables);
        H_matrix(variables);
        Weight_matrix(variables, A);
        Pseudo_Invert(variables);
        // If the invertion of H matrix failed, the iteration will be interrupted 
        if (variables.invert_fail){
            break;
        }
        else{
            // Check 2 iterarion thresholds: times and value
            Update_estimate(variables);
        }
    }
    if (variables.invert_fail == true || variables.iteration_num == 1){
        return A -> get_xyz();
    }
    else{
        if(positioning_config_.ini_mode == ini_ublox_uwb){
            A -> revise_xyz(variables.estimated_pos);
        }
        return variables.estimated_pos;
    }
}

// For 2D solutions

void Uwbpositioning::Estimated_range_2d(positioning& variables){
    Eigen::VectorXd initial_xyz = variables.estimated_pos.segment(0,2);
    Eigen::VectorXd estimated_range(8);
    // std::cout << "initial_xyz: " << initial_xyz << std::endl;
    for (int i =0; i < Tag_number; i++){
        // std::cout << "tag_location " << i << " " << variables.tag_location[i].segment(0,2) << std::endl;
        estimated_range(i) = Distance(initial_xyz, variables.tag_location[i].segment(0,2));
    }
    variables.estimated_range = estimated_range;
    // std::cout << "estimated_range: " << estimated_range << std::endl;
}

void Uwbpositioning::H_matrix_2d(positioning& variables){
    Eigen::VectorXd initial_xyz = variables.estimated_pos.segment(0,2);
    Eigen::MatrixXd h_matrix(Tag_number,2);
    for (int i = 0; i < Tag_number; i++){
        for (int j = 0; j < 2; j++){
            h_matrix(i,j) = (initial_xyz(j) - variables.tag_location[i][j])/variables.estimated_range[i];
        }
    }
    variables.h_matrix = h_matrix;
    // std::cout << "h_matrix: " << h_matrix << std::endl;
}

void Uwbpositioning::Pseudo_Invert_2d(positioning& variables){
    Eigen::MatrixXd Inverse;
    Inverse = variables.h_matrix.transpose() * variables.weight * variables.h_matrix;
    // std::cout << "H'AH: " << Inverse << std::endl;
    double inverse_det = Inverse.determinant();
    // If determinant is too small, the inverse of the matrix will diverge => invert fail  
    if (inverse_det < 0.0000000001){
        variables.invert_fail = true;
        variables.invert_matrix = Eigen::MatrixXd::Zero(2,8);
        std::cout << "\033[31m" << "Determinant of (H'AH) is 0 : Invertion failed !" << "\033[0m" << std::endl;
        // std::cout << "\033[33m" << "(H'AH): " << std::endl << Inverse << "\033[0m" << std::endl;
    }
    else {
        variables.invert_matrix  = Inverse.inverse() * variables.h_matrix.transpose() * variables.weight;
        // std::cout << "(H'AH)-1H'A: " << variables.invert_matrix << std::endl;
    }
}

void Uwbpositioning::Update_estimate_2d(positioning& variables){
    if (variables.iteration_num > threshold_iteration_num){
        variables.iteration_continue = false;
        std::cout << "\033[31m" << "Iteration times over " << threshold_iteration_num << " !" << "\033[0m" << std::endl;
    }
    else if (variables.iteration_value < threshold_iteration_value_2d){
        std::cout << "\033[32m" << "Iteration value < " << threshold_iteration_value_2d << " !" << "\033[0m" << std::endl;
        variables.iteration_continue = false;
    }
    else{
        variables.delta_w = variables.invert_matrix * variables.dr_matrix;
        variables.estimated_pos.segment(0,2) = variables.estimated_pos.segment(0,2) + variables.delta_w;
        variables.iteration_value = variables.delta_w.norm();
        variables.iteration_num++;
        std::cout << "\033[33m" << "Iteration value : " << variables.iteration_value << " !" << "\033[0m" << std::endl;
    }
}

Eigen::VectorXd Uwbpositioning::Propagate_sol_2d(Uwbanchor* A){
    positioning variables;
    variables.estimated_pos = A -> get_xyz();
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\033[32m" << "Initial ENU : (" << variables.estimated_pos[0] << ", " << variables.estimated_pos[1] << ")\033[0m" << std::endl;
    variables.tag_availabiliy = A -> get_tag_availabiliy();
    variables.to_tag_range = A -> get_to_tag_range();
    variables.tag_location = A -> get_tag_location();
    variables.invert_fail = false;
    variables.iteration_continue = true;
    variables.iteration_num = 0;
    variables.iteration_value = INT_MAX;
    // If the iteration had been continued over the threshold times, the iteration will be interrupted
    while (variables.iteration_continue){
        Estimated_range_2d(variables);
        R_matrix(variables);
        H_matrix_2d(variables);
        Weight_matrix(variables, A);
        Pseudo_Invert_2d(variables);
        // If the invertion of H matrix failed, the iteration will be interrupted 
        if (variables.invert_fail){
            break;
        }
        else{
            // Check 2 iterarion thresholds: times and value
            Update_estimate_2d(variables);
        }
    }
    if (variables.invert_fail == true || variables.iteration_num == 1){
        return A -> get_xyz();
    }
    else{
        if(positioning_config_.ini_mode == ini_ublox_uwb){
            A -> revise_xyz(variables.estimated_pos);
        }
        return variables.estimated_pos;
    }
}

const std::vector<std::string> Split(const std::string &str, const char &delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string tok;

    while (std::getline(ss, tok, delimiter)) {
        result.push_back(tok);
    }
    return result;
}

void Uwbpositioning::show_config(){
    std::vector<std::string> enabled_anchor = Split(positioning_config_.enabled_anchor, ' ');
    std::cout << "Time stamp: " << ros::Time::now() << std::endl;
    std::cout << "Fix Mode: " << positioning_config_.fix_mode << std::endl;
    std::cout << "Initial Mode: " << positioning_config_.ini_mode << std::endl;
    std::cout << "Weight Mode: " << positioning_config_.weight_mode << std::endl;
    std::cout << "Enabled anchor: " << enabled_anchor[0] << enabled_anchor[1] << enabled_anchor[2] << enabled_anchor[3] << std::endl;
}

Eigen::Vector3d Uwbpositioning::estimate_velocity(Eigen::Vector3d now_enu, Eigen::Vector3d last_enu, double time_interval){
    return (now_enu - last_enu)/time_interval;;
}

Eigen::Vector3d Uwbpositioning::estimate_orientation(Eigen::Vector3d now_enu, Eigen::Vector3d last_enu){
    Eigen::Vector3d att_enu;
    Eigen::Vector3d diff_enu = now_enu - last_enu;

    // z-axis(up)
    if (atan2(diff_enu(0),diff_enu(1)) > 0){
        att_enu(2) = 360 - coordinate_mat_transformation::rad2Deg(atan2(diff_enu(0),diff_enu(1)));
    }
    else if (atan2(diff_enu(0),diff_enu(1)) < 0){
        att_enu(2) = -coordinate_mat_transformation::rad2Deg(atan2(diff_enu(0),diff_enu(1)));
    }
    else att_enu(2) = coordinate_mat_transformation::rad2Deg(atan2(diff_enu(0),diff_enu(1)));

    // x-axis(east)
    att_enu(0) = asin(diff_enu(2));

    // y-axis(north)
    // unable to calculate from only a vector
    att_enu(1) = 0;

    return att_enu;
}

void Uwbpositioning::Publish_uwb(uwb::uwbFIX &now_fix, Eigen::VectorXd now_enu, int A_num){
    Eigen::Vector3d ref_lla(NCKUEE_LATITUDE, NCKUEE_LONGITUDE, NCKUEE_HEIGHT);
    ros::Time now = ros::Time::now();

    now_fix.header.stamp = now;
    now_fix.header.frame_id = "uwb_A" + std::to_string(A_num);

    Eigen::VectorXd result_lla = coordinate_mat_transformation::enu2Geodetic(now_enu ,ref_lla);
    now_fix.latitude = result_lla[0];
    now_fix.longitude = result_lla[1];
    now_fix.altitude = result_lla[2];

    uwb::uwbFIX last_fix = (*A_+A_num) -> get_last_fix();
    if (last_fix.header.frame_id != "ini"){
        Eigen::Vector3d last_rover(last_fix.latitude, last_fix.longitude, last_fix.altitude);
        Eigen::Vector3d last_enu = coordinate_mat_transformation::lla2enu(last_rover, ref_lla);
        double time_interval = (now - last_fix.header.stamp).toSec();

        Eigen::Vector3d result_v_enu = estimate_velocity(now_enu, last_enu, time_interval);
        now_fix.velocity_e = result_v_enu[0];
        now_fix.velocity_n = result_v_enu[1];
        now_fix.velocity_u = result_v_enu[2];

        Eigen::Vector3d att_enu = estimate_orientation(now_enu, last_enu);
        now_fix.att_e = att_enu(0);
        now_fix.att_n = att_enu(1);
        now_fix.att_u = att_enu(2);
    }
    (*pub_+A_num) -> publish(now_fix);
}

void Uwbpositioning::send_tf(uwb::uwbFIX now_fix, Eigen::Vector3d now_enu, int A_num){
    tf::Transform transform;
    tf::Quaternion current_q;

    current_q.setRPY(now_fix.att_n, now_fix.att_e, -now_fix.att_u);
    transform.setOrigin(tf::Vector3(now_enu(0), now_enu(1), now_enu(2)));
    transform.setRotation(current_q);
    br_.sendTransform(tf::StampedTransform(transform, now_fix.header.stamp, "/map", "uwb_A" + std::to_string(A_num)));
}

void Uwbpositioning::Test(){
    show_config();
    std::vector<std::string> enabled_anchor = Split(positioning_config_.enabled_anchor, ' ');

    if(positioning_config_.ini_mode == ini_manual || positioning_config_.ublox_received == true){
        for (int i = 0; i < Anchor_number; i++){
            if(enabled_anchor[i] == "1"){
                Eigen::VectorXd result;
                // Propagate the positioning solution of each anchor
                if (positioning_config_.fix_mode == 2){
                    result = Propagate_sol_2d((*A_+i));
                }
                else if (positioning_config_.fix_mode == 3){
                    result = Propagate_sol((*A_+i));
                }
                std::cout << "\033[32m" << "Anchor number " << i << "\033[0m" << std::endl;
                std::cout << "\033[32m" << "Localization: (" << result[0] << ", " << result[1] << ", " << result[2] << ")" << "\033[0m" << std::endl;

                // Publish UWB solution and send tf
                uwb::uwbFIX now_fix;
                Publish_uwb(now_fix, result, i);
                send_tf(now_fix, result,i);
                (*A_+i) -> update_last_fix(now_fix);

                // next Anchor
                if (i < Anchor_number-1){
                    std::cout << "\033[32m" << "---" << "\033[0m" << std::endl;
                }
            }
        }
        std::cout << "--------------------" << std::endl;
    }
    else{
        std::cout << "\033[33m" << "Waiting for the first Ublox fix" << "\033[0m" << std::endl;
        std::cout << "--------------------" << std::endl;
    }
}