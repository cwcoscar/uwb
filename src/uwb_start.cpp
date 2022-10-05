#include <CppLinuxSerial/SerialPort.hpp>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <uwb/uwb_raw.h>
#include "ros/ros.h"

pthread_mutex_t mutex;
uwb::uwb_raw uwb_data;

using namespace mn::CppLinuxSerial;

const std::vector<std::string> split(const std::string &str, const char &delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string tok;

    while (std::getline(ss, tok, delimiter)) {
        result.push_back(tok);
    }
    return result;
}

void available_anchor(const std::string& message, uwb::uwb_raw& data){
    int a_c = std::stoi(message);
    a_c % 2 == 1 ? data.A0 = true : data.A0 = false;
    a_c % 4 > 1 ? data.A1 = true : data.A1 = false;
    a_c % 8 > 3 ? data.A2 = true : data.A2 = false;
    a_c > 7 ? data.A3 = true : data.A3 = false;
}

double convert_distance(const std::string& message){
    double result = std::stoi(message, nullptr, 16);
    return result/1000;
}

uwb::uwb_raw fill_in_topic(const std::vector<std::string> data) {
    uwb::uwb_raw result;
    struct timeval t_now;
    gettimeofday(&t_now,NULL);

    result.stamp.sec = t_now.tv_sec;
    result.stamp.nsec = t_now.tv_usec*1000;
    result.MID = data[0];
    available_anchor(data[1], result);
    result.distance_to_A0 = convert_distance(data[2]);
    result.distance_to_A1 = convert_distance(data[3]);
    result.distance_to_A2 = convert_distance(data[4]);
    result.distance_to_A3 = convert_distance(data[5]);
    result.Tag_id = (int)data[9][1] - '0';
    return result;
}

static void *reader_handler(void *args)
{
    std::cout << "\033[32m" << "ENTER UWB Reader thread.\n" << "\033[0m" << std::endl;
    SerialPort *serialPort = (SerialPort *) args;

    while (1) {
        std::string readData;
        std::string message_raw = "";
        
		pthread_mutex_lock(&mutex);
        while(readData != "\n"){
            serialPort->Read(readData);
            message_raw = message_raw + readData;
        }
		pthread_mutex_unlock(&mutex);
        std::vector<std::string> message_data = split(message_raw, ' ');

        // for(int i = 0; i < message_data.size(); i++){
        //     std::cout << "\033[33m" << message_data[i] << "\033[0m" << std::endl;
        // }

        uwb_data = fill_in_topic(message_data);
		std::cout << "\033[33m" << uwb_data << "\033[0m" << std::endl;
    }
    std::cout << "EXIT UWB Reader thread.\n" << std::endl;
    return nullptr;
}

int main(int argc, char **argv) {
    pthread_t thread_reader;

	// Create serial port object and open serial port
    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(-1); // Block when reading until any data is received
	serialPort.Open();
    std::cout << "\033[32m" << "Open serial port" << "\033[0m" << std::endl;

    if (pthread_create(&thread_reader, NULL, reader_handler, &serialPort)) {
        perror("could not create thread for reader_handler");
        return -1;
    }
    if (pthread_detach(thread_reader) != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    sleep(1); // 1s
	while(1);

	// Close the serial port
	serialPort.Close();
    return 0;
}