//
// Created by collab on 06.06.24.
//

#ifndef LODUHOVA_BP_HELPER_FUNCTION_H
#define LODUHOVA_BP_HELPER_FUNCTION_H

#include <cstdio>
#include "k4abt.h"
#include "k4a/k4a.h"
#include <vector>
#include "kinect_control/kinect_control.hpp"

void count_dist(k4a_float3_t lhPalm, k4a_float3_t lhElbow, k4a_float3_t lShoulder, k4a_float3_t chest,
                k4a_float3_t rhPalm, k4a_float3_t rhElbow, k4a_float3_t rShoulder,
                std::vector<double> *dist_palm_elbowL, std::vector<double> *dist_elbow_shoulderL,
                std::vector<double> *dist_palm_shoulderL, std::vector<double> *dist_chest_elbowL,
                std::vector<double> *dist_palm_elbowR, std::vector<double> *dist_elbow_shoulderR,
                std::vector<double> *dist_palm_shoulderR, std::vector<double> *dist_chest_elbowR,
                std::vector<double> *dist_rPalm_chest);

#endif //LODUHOVA_BP_HELPER_FUNCTION_H
