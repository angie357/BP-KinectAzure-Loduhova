//
// Created by collab on 06.06.24.
//

#include "kinect_control/helper_function.h"

void count_dist(k4a_float3_t lhPalm, k4a_float3_t lhElbow, k4a_float3_t lShoulder, k4a_float3_t chest,
                k4a_float3_t rhPalm, k4a_float3_t rhElbow, k4a_float3_t rShoulder,
                std::vector<double> *dist_palm_elbowL, std::vector<double> *dist_elbow_shoulderL,
                std::vector<double> *dist_palm_shoulderL, std::vector<double> *dist_chest_elbowL,
                std::vector<double> *dist_palm_elbowR, std::vector<double> *dist_elbow_shoulderR,
                std::vector<double> *dist_palm_shoulderR, std::vector<double> *dist_chest_elbowR,
                std::vector<double> *dist_rPalm_chest) {

    double dist_Palm_Elbow, dist_Elbow_Shoulder, dist_Palm_Shoulder, dist_chest_Elbow, dist_rPalm_Chest;
    double dX_palmElbow, dY_palmElbow, dZ_palmElbow, dX_rPalm_Chest, dY_rPalm_Chest, dZ_rPalm_Chest;
    double dX_elbowShoulder, dY_elbowShoulder, dZ_elbowShoulder;
    double dX_palmShoulder, dY_palmShoulder, dZ_palmShoulder;
    double dX_chestElbow, dY_chestElbow, dZ_chestElbow;

    // LEFT SIDE -----------------------------------------------------------------------------------------------
    dX_palmElbow = std::abs(lhPalm.v[0] / 1000 - lhElbow.v[0] / 1000);
    dY_palmElbow = std::abs(lhPalm.v[1] / 1000 - lhElbow.v[1] / 1000);
    dZ_palmElbow = std::abs(lhPalm.v[2] / 1000 - lhElbow.v[2] / 1000);

    dX_elbowShoulder = std::abs(lhElbow.v[0] / 1000 - lShoulder.v[0] / 1000);
    dY_elbowShoulder = std::abs(lhElbow.v[1] / 1000 - lShoulder.v[1] / 1000);
    dZ_elbowShoulder = std::abs(lhElbow.v[2] / 1000 - lShoulder.v[2] / 1000);

    dX_palmShoulder = std::abs(lhPalm.v[0] / 1000 - lShoulder.v[0] / 1000);
    dY_palmShoulder = std::abs(lhPalm.v[1] / 1000 - lShoulder.v[1] / 1000);
    dZ_palmShoulder = std::abs(lhPalm.v[2] / 1000 - lShoulder.v[2] / 1000);

    dX_chestElbow = std::abs(chest.v[0] / 1000 - lhElbow.v[0] / 1000);
    dY_chestElbow = std::abs(chest.v[1] / 1000 - lhElbow.v[1] / 1000);
    dZ_chestElbow = std::abs(chest.v[2] / 1000 - lhElbow.v[2] / 1000);

    dist_Palm_Elbow = sqrt(dX_palmElbow * dX_palmElbow + dY_palmElbow * dY_palmElbow + dZ_palmElbow * dZ_palmElbow);
    dist_Elbow_Shoulder = sqrt(dX_elbowShoulder * dX_elbowShoulder + dY_elbowShoulder * dY_elbowShoulder +
                               dZ_elbowShoulder * dZ_elbowShoulder);
    dist_Palm_Shoulder = sqrt(
            dX_palmShoulder * dX_palmShoulder + dY_palmShoulder * dY_palmShoulder + dZ_palmShoulder * dZ_palmShoulder);
    dist_chest_Elbow = sqrt(
            dX_chestElbow * dX_chestElbow + dY_chestElbow * dY_chestElbow + dZ_chestElbow * dZ_chestElbow);

    dist_palm_elbowL->push_back(dist_Palm_Elbow);
    dist_elbow_shoulderL->push_back(dist_Elbow_Shoulder);
    dist_palm_shoulderL->push_back(dist_Palm_Shoulder);
    dist_chest_elbowL->push_back(dist_chest_Elbow);

    // RIGHT SIDE -----------------------------------------------------------------------------------------------
    dX_palmElbow = std::abs(rhPalm.v[0] / 1000 - rhElbow.v[0] / 1000);
    dY_palmElbow = std::abs(rhPalm.v[1] / 1000 - rhElbow.v[1] / 1000);
    dZ_palmElbow = std::abs(rhPalm.v[2] / 1000 - rhElbow.v[2] / 1000);

    dX_elbowShoulder = std::abs(rhElbow.v[0] / 1000 - rShoulder.v[0] / 1000);
    dY_elbowShoulder = std::abs(rhElbow.v[1] / 1000 - rShoulder.v[1] / 1000);
    dZ_elbowShoulder = std::abs(rhElbow.v[2] / 1000 - rShoulder.v[2] / 1000);

    dX_palmShoulder = std::abs(rhPalm.v[0] / 1000 - rShoulder.v[0] / 1000);
    dY_palmShoulder = std::abs(rhPalm.v[1] / 1000 - rShoulder.v[1] / 1000);
    dZ_palmShoulder = std::abs(rhPalm.v[2] / 1000 - rShoulder.v[2] / 1000);

    dX_chestElbow = std::abs(chest.v[0] / 1000 - rhElbow.v[0] / 1000);
    dY_chestElbow = std::abs(chest.v[1] / 1000 - rhElbow.v[1] / 1000);
    dZ_chestElbow = std::abs(chest.v[2] / 1000 - rhElbow.v[2] / 1000);

    dist_Palm_Elbow = sqrt(dX_palmElbow * dX_palmElbow + dY_palmElbow * dY_palmElbow + dZ_palmElbow * dZ_palmElbow);
    dist_Elbow_Shoulder = sqrt(dX_elbowShoulder * dX_elbowShoulder + dY_elbowShoulder * dY_elbowShoulder +
                               dZ_elbowShoulder * dZ_elbowShoulder);
    dist_Palm_Shoulder = sqrt(
            dX_palmShoulder * dX_palmShoulder + dY_palmShoulder * dY_palmShoulder + dZ_palmShoulder * dZ_palmShoulder);
    dist_chest_Elbow = sqrt(
            dX_chestElbow * dX_chestElbow + dY_chestElbow * dY_chestElbow + dZ_chestElbow * dZ_chestElbow);

    dist_palm_elbowR->push_back(dist_Palm_Elbow);
    dist_elbow_shoulderR->push_back(dist_Elbow_Shoulder);
    dist_palm_shoulderR->push_back(dist_Palm_Shoulder);
    dist_chest_elbowR->push_back(dist_chest_Elbow);

    // RIGHT PALM TO CHEST ---------------------------------------------------------------------------------------
    dX_rPalm_Chest = std::abs(rhPalm.v[0] / 1000 - chest.v[0] / 1000);
    dY_rPalm_Chest = std::abs(rhPalm.v[1] / 1000 - chest.v[1] / 1000);
    dZ_rPalm_Chest = std::abs(rhPalm.v[2] / 1000 - chest.v[2] / 1000);

    dist_rPalm_Chest = sqrt(dX_rPalm_Chest * dX_rPalm_Chest + dY_rPalm_Chest * dY_rPalm_Chest + dZ_rPalm_Chest * dZ_rPalm_Chest);
    dist_rPalm_chest->push_back(dist_rPalm_Chest);
}