#include <extended_kalman_filter/measurement_package.hpp>
#include <extended_kalman_filter/track_filter.hpp>
#include <iostream>
#include <fstream>
#include <vector>



int main(){
    std::vector<localization::extended_kalman_filter::MeasurementPackage> measure_pack_list;

    std::string filename = "obj_pose-laser-radar-synthetic-input.txt";

    std::ifstream ifs;
    ifs.open(filename.c_str(),std::ifstream::in);

    if (!ifs.is_open()) {
    std::cout << "Cannot open input file: " <<filename<< std::endl;
    }

    std::string line;
    int i =0;

    while(getline(ifs,line)){
        localization::extended_kalman_filter::MeasurementPackage packet;
        std::istringstream iss(line);
        std::string sensor_type;
        iss >> sensor_type;
        int64_t timestamp;

        if(sensor_type.compare("L") == 0){
            packet.sensor_type = localization::extended_kalman_filter::SensorType::LASER;
            packet.raw_measurements_ = Eigen::VectorXd(2);
            float x,y;
            iss>>x;
            iss>>y;
            packet.raw_measurements_<<x,y;
            iss>>timestamp;
            packet.timestamp_ = timestamp;
            measure_pack_list.push_back(packet);
        }
        else if(sensor_type.compare("R") == 0){
            packet.sensor_type = localization::extended_kalman_filter::SensorType::RADAR;
            packet.raw_measurements_ = Eigen::VectorXd(3);
            float rho,phi,rho_dot;
            iss>>rho;
            iss>>phi;
            iss>>rho_dot;
            packet.raw_measurements_<<rho,phi,rho_dot;
            iss>>timestamp;
            packet.timestamp_ = timestamp;
            measure_pack_list.push_back(packet);

        }
        ++i;
    }

    size_t N = measure_pack_list.size();
    localization::extended_kalman_filter::Tracker t;

    for(size_t k = 0;k<N;++k){
        t.measurement_update(measure_pack_list[k]);
    }

    if(ifs.is_open()){
        ifs.close();
    }
    
    return 0;
}