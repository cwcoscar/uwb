#include "uwb_positioning.h"

int main(int argc, char **argv) {
    std::string ublox_fix_topic_;
    config positioning_config;

    ros::init(argc, argv, "uwb_positioning");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param("ublox_fix_topic", ublox_fix_topic_, std::string("/ublox_f9k/fix"));

    // Some parameters used in positioning process
    nh.param("fix_mode", positioning_config.fix_mode, 2);
    nh.param("ini_mode", positioning_config.ini_mode, 1);
    nh.param("weight_mode", positioning_config.weight_mode, 1);
    nh.param("enabled_anchor", positioning_config.enabled_anchor, std::string("1000"));
    
    // Publishers of navigation solutions
    ros::Publisher pub0 = n.advertise<uwb_YCHIOT::uwb_fix>("/uwb_position/A0", 1);
    ros::Publisher pub1 = n.advertise<uwb_YCHIOT::uwb_fix>("/uwb_position/A1", 1);
    ros::Publisher pub2 = n.advertise<uwb_YCHIOT::uwb_fix>("/uwb_position/A2", 1);
    ros::Publisher pub3 = n.advertise<uwb_YCHIOT::uwb_fix>("/uwb_position/A3", 1);
    ros::Publisher pub[Anchor_number] = {pub0, pub1, pub2, pub3};

    ros::Publisher pub_tag = n.advertise<uwb_YCHIOT::uwb_tag>("/uwb_tag_location", 1);

    // Initial position of vehicle (baselink)
    // If ublox fix is received, the data will be overrided.
    Eigen::VectorXd xyz(3);
    if (nh.hasParam("ini_x") && nh.hasParam("ini_y") && nh.hasParam("ini_z")){
        nh.getParam("ini_x", xyz[0]);
        nh.getParam("ini_y", xyz[1]);
        nh.getParam("ini_z", xyz[2]);
    }
    else xyz << 45, 175, -60;

    //Initial position of tags (static stations)
    Eigen::VectorXd location(3);
    std::array<Eigen::VectorXd,Tag_number> tag_location;

    if (nh.hasParam("T0_x") && nh.hasParam("T0_y") && nh.hasParam("T0_z")){
        nh.getParam("T0_x", location[0]);
        nh.getParam("T0_y", location[1]);
        nh.getParam("T0_z", location[2]);
        // std::cout << "tag_location 0: " << location.segment(0,2) << std::endl;
    }
    else location << 59.0414, 194.1903, -47.7806;
    tag_location[0] = location;

    if (nh.hasParam("T1_x") && nh.hasParam("T1_y") && nh.hasParam("T1_z")){
        nh.getParam("T1_x", location[0]);
        nh.getParam("T1_y", location[1]);
        nh.getParam("T1_z", location[2]);
        // std::cout << "tag_location 1: " << location.segment(0,2) << std::endl;
    }
    else location << 43.1544, 194.8983, -47.5321;
    tag_location[1] = location;

    if (nh.hasParam("T2_x") && nh.hasParam("T2_y") && nh.hasParam("T2_z")){
        nh.getParam("T2_x", location[0]);
        nh.getParam("T2_y", location[1]);
        nh.getParam("T2_z", location[2]);
        // std::cout << "tag_location 2: " << location.segment(0,2) << std::endl;
    }
    else location << 0, 10, 0;
    tag_location[2] = location;

    if (nh.hasParam("T3_x") && nh.hasParam("T3_y") && nh.hasParam("T3_z")){
        nh.getParam("T3_x", location[0]);
        nh.getParam("T3_y", location[1]);
        nh.getParam("T3_z", location[2]);
        // std::cout << "tag_location 3: " << location.segment(0,2) << std::endl;
    }
    else location << 51.7699, 184.3686, -46.6854;
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

    Uwbpositioning uwbpositioning(pub, Anchor, Tag, positioning_config);

    // Subscribers of UWB ranging data and ublox fix
    ros::Subscriber sub = n.subscribe("/uwb_calibration", 1, &Uwbpositioning::UwbCalibrationCallback, &uwbpositioning);
    ros::Subscriber sub1;
    if(positioning_config.ini_mode != ini_manual){
        sub1 = n.subscribe(ublox_fix_topic_, 1, &Uwbpositioning::UbloxfixCallback, &uwbpositioning);
    }

    // Publish tag location once
    uwb_YCHIOT::uwb_tag topic_data;
    geometry_msgs::Pose tag_locations[Tag_number];
    for (int j = 0 ; j < Tag_number; j++){
        tag_locations[j].position.x = tag_location[j][0];
        tag_locations[j].position.y = tag_location[j][1];
        tag_locations[j].position.z = tag_location[j][2];
        topic_data.tag_location.push_back(tag_locations[j]);
    }
    
    ros::Rate loop_rate(35);
    int amount = 0;

    while (ros::ok()){
        if (amount >= 100){
            pub_tag.publish(topic_data);
            amount = 0;
        }
        else amount++;
        // uwbpositioning.Test();
        ros::spinOnce();
        loop_rate.sleep();
    }
}