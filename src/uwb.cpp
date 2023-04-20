#include "uwb.h"

Uwb::Uwb(int id , Eigen::VectorXd xyz) : id_(id), xyz_(xyz){}

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