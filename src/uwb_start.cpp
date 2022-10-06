#include <CppLinuxSerial/SerialPort.hpp>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <uwb_YCHIOT/uwb_raw.h>
#include "ros/ros.h"
#include <signal.h>

pthread_mutex_t mutex;
uwb_YCHIOT::uwb_raw uwb_data;

struct arg_struct {
    mn::CppLinuxSerial::SerialPort arg1;
    ros::Publisher arg2;
};

using namespace mn::CppLinuxSerial;

void Siginthandler(int sig){
    std::cout << "ROS shutting down!" << std::endl;
    ros::shutdown();
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

void Available_anchor(const std::string& message, uwb_YCHIOT::uwb_raw& data){
    int a_c = std::stoi(message);
    a_c % 2 == 1 ? data.A0 = true : data.A0 = false;
    a_c % 4 > 1 ? data.A1 = true : data.A1 = false;
    a_c % 8 > 3 ? data.A2 = true : data.A2 = false;
    a_c > 7 ? data.A3 = true : data.A3 = false;
}

double Convert_distance(const std::string& message){
    double result = std::stoi(message, nullptr, 16);
    return result/1000;
}

uwb_YCHIOT::uwb_raw Fill_in_topic(const std::vector<std::string> data) {
    uwb_YCHIOT::uwb_raw result;
    struct timeval t_now;
    gettimeofday(&t_now,NULL);

    result.stamp.sec = t_now.tv_sec;
    result.stamp.nsec = t_now.tv_usec*1000;
    result.MID = data[0];
    Available_anchor(data[1], result);
    result.distance_to_A0 = Convert_distance(data[2]);
    result.distance_to_A1 = Convert_distance(data[3]);
    result.distance_to_A2 = Convert_distance(data[4]);
    result.distance_to_A3 = Convert_distance(data[5]);
    result.Tag_id = (int)data[9][1] - '0';
    return result;
}

static void *Reader_handler(void *arguments)
{
    std::cout << "\033[32m" << "ENTER UWB Reader thread.\n" << "\033[0m" << std::endl;
    struct arg_struct *args = (struct arg_struct *)arguments;

    while (ros::ok()) {
        std::string readData;
        std::string message_raw = "";
        
		pthread_mutex_lock(&mutex);
        while(readData != "\n"){
            args->arg1.Read(readData);
            message_raw = message_raw + readData;
        }
		pthread_mutex_unlock(&mutex);
        std::vector<std::string> message_data = Split(message_raw, ' ');

        // for(int i = 0; i < message_data.size(); i++){
        //     std::cout << "\033[33m" << message_data[i] << "\033[0m" << std::endl;
        // }

        uwb_data = Fill_in_topic(message_data);
		std::cout << "\033[33m" << uwb_data << "\033[0m" << std::endl;

        args->arg2.publish(uwb_data);
    }
    std::cout << "EXIT UWB Reader thread.\n" << std::endl;
    return nullptr;
}

int main(int argc, char **argv) {
    signal(SIGINT, Siginthandler);

    pthread_t thread_reader;

    ros::init(argc, argv, "uwb_start");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<uwb_YCHIOT::uwb_raw>("uwb_raw", 1);

	// Create serial port object and open serial port
    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(-1); // Block when reading until any data is received
	serialPort.Open();
    std::cout << "\033[32m" << "Open serial port" << "\033[0m" << std::endl;

    struct arg_struct args;
    args.arg1 = serialPort;
    args.arg2 = pub;

    if (pthread_create(&thread_reader, NULL, Reader_handler, &args)) {
        perror("could not create thread for Reader_handler");
        return -1;
    }
    if (pthread_detach(thread_reader) != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    sleep(1); // 1s
	while(ros::ok());

	// Close the serial port
	serialPort.Close();
    return 0;
}