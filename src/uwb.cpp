#include "uwb.h"
#include <iostream>

Uwb::Uwb(int id , Eigen::VectorXd xyz) : id_(id), xyz_(xyz){}

void Uwb::set_id(int id){
    id_ = id;
}

void Uwb::set_location(Eigen::VectorXd xyz){
    xyz_ = xyz;
}

Eigen::VectorXd Uwb::get_xyz(){
    return xyz_;
}

void Uwb::display_info(){
    std::cout << "id: " << id_ << std::endl;
    std::cout << "location: " << xyz_[0] << " " << xyz_[1] << " " << xyz_[2] << std::endl;
}

Uwbanchor::Uwbanchor(int id , Eigen::VectorXd xyz, std::array<Eigen::VectorXd,8> tag_location) 
: Uwb(id, xyz), tag_location_(tag_location){
    for (int i = 0; i < sizeof(tag_availabiliy_); i++){
        tag_availabiliy_[i] = false;
    }
}

void Uwbanchor::update_availability_range(bool tag_availabiliy, double to_tag_range, int id){
    tag_availabiliy_[id] = tag_availabiliy;
    (tag_availabiliy) ? to_tag_range_[id] = to_tag_range : to_tag_range_[id] = 0;
}

// bool Uwbanchor::get_tag_availabiliy(int index){
//     return tag_availabiliy_[index];
// }

// double Uwbanchor::get_to_tag_range(int index){
//     return to_tag_range_[index];
// }

// Eigen::VectorXd Uwbanchor::get_tag_location(int index){
//     return tag_location_[index];
// }

std::array<bool,8> Uwbanchor::get_tag_availabiliy(){
    return tag_availabiliy_;
}

std::array<double,8> Uwbanchor::get_to_tag_range(){
    return to_tag_range_;
}

std::array<Eigen::VectorXd,8> Uwbanchor::get_tag_location(){
    return tag_location_;
}


void Uwbanchor::display_info(){
    std::cout << "Type: Anchor" << std::endl;
    Uwb::display_info();
    std::cout << "tag_availabiliy" << std::endl;
    for (int i =0; i < 8; i++){
        std::cout << i << " : " << tag_availabiliy_[i] << std::endl;
    }
    std::cout << "to_tag_range :" << std::endl;
    for (int i =0; i < 8; i++){
        std::cout << i << " : " << to_tag_range_[i] << std::endl;
    }
    std::cout << "tag_location :" << std::endl;
    for (int i =0; i < 8; i++){
        std::cout << i << " : " << tag_location_[i][0] << " " << tag_location_[i][1] << " " << tag_location_[i][2] << std::endl;
    }
    // for (int i = 0; i < 8; i++){
    //     tag_availabiliy_[i] == true ? std::cout << "to_tag" << i << "_range : " << to_tag_range_[i] << std::endl : continue;
    // }
}

Uwbtag::Uwbtag(int id , Eigen::VectorXd xyz) 
: Uwb(id, xyz){
    for (int i = 0; i < sizeof(anchor_availabiliy_); i++){
        anchor_availabiliy_[i] = false;
    }
}

void Uwbtag::update_availability_range(bool anchor_availabiliy[4], double to_anchor_range[4]){
    for (int i = 0; i < sizeof(anchor_availabiliy); i++){
        if(anchor_availabiliy[i] == true){
            anchor_availabiliy_[i] = true;
            to_anchor_range_[i] = to_anchor_range[i];
        }
        else {
            anchor_availabiliy_[i] = false;
            to_anchor_range_[i] = 0;
        }
    }
}

void Uwbtag::display_info(){
    std::cout << "Type: Tag" << std::endl;
    Uwb::display_info();
    std::cout << "anchor_availabiliy" << std::endl;
    for (int i =0; i < 4; i++){
        std::cout << i << " : " << anchor_availabiliy_[i] << std::endl;
    }
    std::cout << "to_anchor_range :" << std::endl;
    for (int i =0; i < 4; i++){
        std::cout << i << " : " << to_anchor_range_[i] << std::endl;
    }
}




