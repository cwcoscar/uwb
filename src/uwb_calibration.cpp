#include "ros/ros.h"
#include <uwb_YCHIOT/uwb_raw.h>

#define SQUARE(a) (a*a)

class UwbCalibration{
    private:
        ros::Publisher _pub;
        uwb_YCHIOT::uwb_raw _data_calibrated;
        const double T0_p1_ = -0.000097660810875505;
        const double T0_p2_ = 0.00616639731263843;
        const double T0_p3_ = 0.446374032;
        const double T1_p1_ = -0.00000983464925135247;
        const double T1_p2_ = 0.000322596878032999;
        const double T1_p3_ = 0.552711055620107;
        const double T2_p1_ = -0.0000502392442241953;
        const double T2_p2_ = 0.00231082675773457;
        const double T2_p3_ = 0.50699279142121;
        const double T3_p1_ = -0.0000910185297181136;
        const double T3_p2_ = 0.00611243530949391;
        const double T3_p3_ = 0.497332682169962;
    
    public:
        UwbCalibration(ros::Publisher pub);
        void Calibration(uwb_YCHIOT::uwb_raw raw_data);
        void UwbrawCallback(const uwb_YCHIOT::uwb_raw& msg);
        double Uwb_error_fit_curve_T0(double measurement);
        double Uwb_error_fit_curve_T1(double measurement);
        double Uwb_error_fit_curve_T2(double measurement);
        double Uwb_error_fit_curve_T3(double measurement);

};

UwbCalibration::UwbCalibration(ros::Publisher pub){
    _pub = pub;
}

inline double UwbCalibration::Uwb_error_fit_curve_T0(double measurement){
    return T0_p1_ * SQUARE(measurement) + T0_p2_ * measurement + T0_p3_;
}

inline double UwbCalibration::Uwb_error_fit_curve_T1(double measurement){
    return T1_p1_ * SQUARE(measurement) + T1_p2_ * measurement + T1_p3_;
}

inline double UwbCalibration::Uwb_error_fit_curve_T2(double measurement){
    return T2_p1_ * SQUARE(measurement) + T2_p2_ * measurement + T2_p3_;
}

inline double UwbCalibration::Uwb_error_fit_curve_T3(double measurement){
    return T3_p1_ * SQUARE(measurement) + T3_p2_ * measurement + T3_p3_;
}

void UwbCalibration::Calibration(uwb_YCHIOT::uwb_raw raw_data){
    _data_calibrated = raw_data;
    if (_data_calibrated.Tag_id == 0){
        if (raw_data.A0){
        _data_calibrated.distance_to_A0 = _data_calibrated.distance_to_A0 - Uwb_error_fit_curve_T0(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A1){
            _data_calibrated.distance_to_A1 = _data_calibrated.distance_to_A1 - Uwb_error_fit_curve_T0(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A2){
            _data_calibrated.distance_to_A2 = _data_calibrated.distance_to_A2 - Uwb_error_fit_curve_T0(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A3){
            _data_calibrated.distance_to_A3 = _data_calibrated.distance_to_A3 - Uwb_error_fit_curve_T0(_data_calibrated.distance_to_A0);
        } 
    }
    else if (_data_calibrated.Tag_id == 1){
        if (raw_data.A0){
        _data_calibrated.distance_to_A0 = _data_calibrated.distance_to_A0- Uwb_error_fit_curve_T1(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A1){
            _data_calibrated.distance_to_A1 = _data_calibrated.distance_to_A1 - Uwb_error_fit_curve_T1(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A2){
            _data_calibrated.distance_to_A2 = _data_calibrated.distance_to_A2 - Uwb_error_fit_curve_T1(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A3){
            _data_calibrated.distance_to_A3 = _data_calibrated.distance_to_A3 - Uwb_error_fit_curve_T1(_data_calibrated.distance_to_A0);
        } 
    }
    else if (_data_calibrated.Tag_id == 2){
        if (raw_data.A0){
        _data_calibrated.distance_to_A0 = _data_calibrated.distance_to_A0 - Uwb_error_fit_curve_T2(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A1){
            _data_calibrated.distance_to_A1 = _data_calibrated.distance_to_A1 - Uwb_error_fit_curve_T2(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A2){
            _data_calibrated.distance_to_A2 = _data_calibrated.distance_to_A2 - Uwb_error_fit_curve_T2(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A3){
            _data_calibrated.distance_to_A3 = _data_calibrated.distance_to_A3 - Uwb_error_fit_curve_T2(_data_calibrated.distance_to_A0);
        } 
    }
    else if (_data_calibrated.Tag_id == 3){
        if (raw_data.A0){
        _data_calibrated.distance_to_A0 = _data_calibrated.distance_to_A0 - Uwb_error_fit_curve_T3(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A1){
            _data_calibrated.distance_to_A1 = _data_calibrated.distance_to_A1 - Uwb_error_fit_curve_T3(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A2){
            _data_calibrated.distance_to_A2 = _data_calibrated.distance_to_A2 - Uwb_error_fit_curve_T3(_data_calibrated.distance_to_A0);
        }
        if (raw_data.A3){
            _data_calibrated.distance_to_A3 = _data_calibrated.distance_to_A3 - Uwb_error_fit_curve_T3(_data_calibrated.distance_to_A0);
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