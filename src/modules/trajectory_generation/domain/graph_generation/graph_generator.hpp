#ifndef TRAJECTORY_GENERATION__GRAPH_GENERATION__GRAPH_GENERATOR_HPP
#define TRAJECTORY_GENERATION__GRAPH_GENERATION__GRAPH_GENERATOR_HPP

#include <core_datastructures/posture.hpp>
#include <vector>
#include <unordered_map>
#include <i_spline_generator.hpp>
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
             * @param[in] spline_generator std::shared_ptr<trajectory_generation::spline_generation::ISplineGenerator>
             *                              A shared pointer to spline_generator object
             * @param[in] road_center std::vector<core_datastructures::Posture> road center line information
             * @param[in] long_sampling_dist    double The sampling distance along the road center line
             * @param[in] lane_width    double The lane width in meters
             */
            GraphGenerator(std::shared_ptr<trajectory_generation::spline_generation::ISplineGenerator> spline_generator, 
                            const std::vector<core_datastructures::Posture>& road_center, 
                            double long_sampling_dist=2, double lane_width=3.7);

            double get_s(int h);
            double get_l(int i);
            const core_datastructures::Posture to_cartesian(int h, int i);
            void search_graph();
            void downsample();

            std::vector<std::vector<core_datastructures::Posture>> get_spline_lattice() {
                return splines;
            }
            
        private:
            std::vector<core_datastructures::Posture> road_center;
            std::shared_ptr<trajectory_generation::spline_generation::ISplineGenerator> spline_generator;
            // std::unordered_map<core_datastructures::Posture, std::vector<core_datastructures::Posture>, 
            //                     core_datastructures::PostureHash, core_datastructures::PostureEqual> adjacency_list;

            double a_s; // Sampling distance along the load length
            double a_l; // Half of lane width
            double b_l;  // Sampling distance along lane width
            int h_max;  //  Maximum number of longitudinal samples
            int i_max;      // Maximum number of lateral samples

            std::vector<std::vector<core_datastructures::Posture>> splines;
            

    };
}


#endif  /*TRAJECTORY_GENERATION__GRAPH_GENERATION__GRAPH_GENERATOR_HPP*/