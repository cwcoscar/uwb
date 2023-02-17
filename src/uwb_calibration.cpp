#include "ros/ros.h"
#include <uwb_YCHIOT/uwb_raw.h>

class UwbCalibration{
    private:
        ros::Publisher _pub;
        uwb_YCHIOT::uwb_raw _data_calibrated;
        const double T0_slope_ = 0.9993;
        const double T0_offset_ = -0.5814;
        const double T3_slope_ = 0.9999;
        const double T3_offset_ = -0.6753;

        const double _T0_bias = 0.60264459806428794;
        const double _T3_bias = 0.67916991348614453;
    
    public:
        UwbCalibration(ros::Publisher pub);
        void Calibration(uwb_YCHIOT::uwb_raw raw_data);
        void UwbrawCallback(const uwb_YCHIOT::uwb_raw& msg);

};

UwbCalibration::UwbCalibration(ros::Publisher pub){
    _pub = pub;
}

void UwbCalibration::Calibration(uwb_YCHIOT::uwb_raw raw_data){
    _data_calibrated = raw_data;
    if (_data_calibrated.Tag_id == 0){
        if (raw_data.A0){
        _data_calibrated.distance_to_A0 = _data_calibrated.distance_to_A0 * T0_slope_ + T0_offset_;
        }
        if (raw_data.A1){
            _data_calibrated.distance_to_A1 = _data_calibrated.distance_to_A1 * T0_slope_ + T0_offset_;
        }
        if (raw_data.A2){
            _data_calibrated.distance_to_A2 = _data_calibrated.distance_to_A2 * T0_slope_ + T0_offset_;
        }
        if (raw_data.A3){
            _data_calibrated.distance_to_A3 = _data_calibrated.distance_to_A3 * T0_slope_ + T0_offset_;
        } 
    }
    else if (_data_calibrated.Tag_id == 1){
        if (raw_data.A0){
        _data_calibrated.distance_to_A0 = _data_calibrated.distance_to_A0 - _T3_bias;
        }
        if (raw_data.A1){
            _data_calibrated.distance_to_A1 = _data_calibrated.distance_to_A1 - _T3_bias;
        }
        if (raw_data.A2){
            _data_calibrated.distance_to_A2 = _data_calibrated.distance_to_A2 - _T3_bias;
        }
        if (raw_data.A3){
            _data_calibrated.distance_to_A3 = _data_calibrated.distance_to_A3 - _T3_bias;
        } 
    }
    else if (_data_calibrated.Tag_id == 2){
        if (raw_data.A0){
        _data_calibrated.distance_to_A0 = _data_calibrated.distance_to_A0 - _T3_bias;
        }
        if (raw_data.A1){
            _data_calibrated.distance_to_A1 = _data_calibrated.distance_to_A1 - _T3_bias;
        }
        if (raw_data.A2){
            _data_calibrated.distance_to_A2 = _data_calibrated.distance_to_A2 - _T3_bias;
        }
        if (raw_data.A3){
            _data_calibrated.distance_to_A3 = _data_calibrated.distance_to_A3 - _T3_bias;
        } 
    }
    else if (_data_calibrated.Tag_id == 3){
        if (raw_data.A0){
        _data_calibrated.distance_to_A0 = _data_calibrated.distance_to_A0 * T3_slope_ + T3_offset_;
        }
        if (raw_data.A1){
            _data_calibrated.distance_to_A1 = _data_calibrated.distance_to_A1 * T3_slope_ + T3_offset_;
        }
        if (raw_data.A2){
            _data_calibrated.distance_to_A2 = _data_calibrated.distance_to_A2 * T3_slope_ + T3_offset_;
        }
        if (raw_data.A3){
            _data_calibrated.distance_to_A3 = _data_calibrated.distance_to_A3 * T3_slope_ + T3_offset_;
        } 
    }
}


void UwbCalibration::UwbrawCallback(const uwb_YCHIOT::uwb_raw& msg){
    Calibration(msg);
    _pub.publish(_data_calibrated);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "uwb_calibration");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<uwb_YCHIOT::uwb_raw>("uwb_calibration", 1);

    UwbCalibration uwbCalibration(pub);

    ros::Subscriber sub = n.subscribe("uwb_raw", 1, &UwbCalibration::UwbrawCallback, &uwbCalibration);

    ros::spin();
}