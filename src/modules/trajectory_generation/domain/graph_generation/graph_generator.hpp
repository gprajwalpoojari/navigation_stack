#ifndef TRAJECTORY_GENERATION__GRAPH_GENERATION__GRAPH_GENERATOR_HPP
#define TRAJECTORY_GENERATION__GRAPH_GENERATION__GRAPH_GENERATOR_HPP

#include <core_datastructures/posture.hpp>
#include <vector>
#include <unordered_map>
#include <i_trajectory_generator.hpp>
#include <memory>

namespace trajectory_generation::graph_generation {
    /**
     * @brief GraphGenerator Class for generating cubic splines given two postures
     * 
     */
    class GraphGenerator {
        public:
            /**
             * @brief Construct a new Cubic Spline Generator object
             * 
             * @param[in] trajectory_generator std::shared_ptr<trajectory_generation::trajectory_generation::ITrajectoryGenerator>
             *                              A shared pointer to spline_generator object
             * @param[in] road_center std::vector<core_datastructures::Posture> road center line information
             * @param[in] long_sampling_dist    double The sampling distance along the road center line
             * @param[in] lane_width    double The lane width in meters
             */
            GraphGenerator(std::shared_ptr<trajectory_generation::ITrajectoryGenerator> trajectory_generator, 
                            const std::vector<core_datastructures::Posture>& road_center, 
                            double long_sampling_dist=2, double lane_width=3.7);


            /**
             * @brief Get the s coordinate given index @p h
             * 
             * @param h The index for which the frenet coordinate s will be calculated
             * @return * double 
             */
            double get_s(int h);

            /**
             * @brief Get the l coordinate given index @p i
             * 
             * @param i The index for which the frenet coordinate l will be calculated
             * @return double 
             */
            double get_l(int i);

            /**
             * @brief Convert the Frenet coordinates into cartesian coordinates
             * 
             * @param h     The index of s coordinate
             * @param i     The index of l coordinate
             * @return const core_datastructures::Posture 
             */
            const core_datastructures::Posture to_cartesian(int h, int i);

            /**
             * @brief Searches the graph by generating a state lattice
             * 
             */
            void search_graph();

            /**
             * @brief Downsamples the Road center line
             * 
             */
            void downsample();
            
        private:
            std::vector<core_datastructures::Posture> road_center;
            std::shared_ptr<trajectory_generation::ITrajectoryGenerator> trajectory_generator;
    
            double a_s; // Sampling distance along the load length
            double a_l; // Half of lane width
            double b_l;  // Sampling distance along lane width
            int h_max;  //  Maximum number of longitudinal samples
            int i_max;      // Maximum number of lateral samples
            

    };
}


#endif  /*TRAJECTORY_GENERATION__GRAPH_GENERATION__GRAPH_GENERATOR_HPP*/