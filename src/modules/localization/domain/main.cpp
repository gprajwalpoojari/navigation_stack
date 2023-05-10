#include "kalman_filter/Kalman_Filter.h"
#include "kalman_filter/Dynamics.h"
#include "kalman_filter/TrackFilter.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;


int main(){


    vector<MeasurementPackage> measure_pack_list;

    string filename = "kalman_filter/obj_pose-laser-radar-synthetic-input.txt";

    ifstream ifs;
    ifs.open(filename.c_str(),ifstream::in);

    if (!ifs.is_open()) {
    cout << "Cannot open input file: " <<filename<< endl;
    }

    string line;
    int i =0;

    while(getline(ifs,line) && i<=7){
        MeasurementPackage packet;
        istringstream iss(line);
        string sensor_type;
        iss >> sensor_type;
        int64_t timestamp;

        if(sensor_type.compare("L") == 0){
            packet.sensor_type = MeasurementPackage::LASER;
            packet.raw_measurements_ = VectorXd(2);
            float x,y;
            iss>>x;
            iss>>y;
            packet.raw_measurements_<<x,y;
            iss>>timestamp;
            packet.timestamp_ = timestamp;
            measure_pack_list.push_back(packet);
        }
        else if(sensor_type.compare("R") == 0){
            continue;
        }
        ++i;
    }

    size_t N = measure_pack_list.size();
    Tracker t;

    for(size_t k = 0;k<N;++k){
        t.measurement_update(measure_pack_list[k]);
    }

    if(ifs.is_open()){
        ifs.close();
    }





    return 0;
}