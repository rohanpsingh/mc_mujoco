#ifndef MJ_UTILS_H
#define MJ_UTILS_H

#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <vector>

#include "mj_sim_impl.h"

namespace mc_mujoco
{

/** Merge multiple mujoco models into one
 *
 * For each merged model a prefix is added to every named entities in the model
 *
 * Warnings are displayed when some global parameters conflict, in such cases, the value from the first model where the
 * parameter appeared will prevail
 *
 * \param mujocoObjects Objects added to the simulation but not in mc_rtc
 *
 * \param mcrtcObjects Objects added to the simulation and to mc_rtc
 *
 * \param mjObjects MuJoCo ids for objects not in mc_rtc will be filled with parameters in the merged model
 *
 * \param mjRobots MuJoCo ids for objects in mc_rtc will be filled with parameters in the merged model
 *
 * \returns The path to the generated model
 */
std::string merge_mujoco_models(const std::map<std::string, std::string> & mujocoObjects,
                                const std::map<std::string, std::string> & mcrtcObjects,
                                std::vector<MjObject> & mjObjects,
                                std::vector<MjRobot> & mjRobots);

/*! Load XML model and initialize */
bool mujoco_init(MjSimImpl * mj_sim,
                 const std::map<std::string, std::string> & mujocoObjects,
                 const std::map<std::string, std::string> & mcrtcObjects);

/*! Create GLFW window */
void mujoco_create_window(MjSimImpl * mj_sim);

/** Returns a sensor id from name, -1 if the type does not match or the sensor does not exist */
int mujoco_get_sensor_id(const mjModel & m, const std::string & name, mjtSensor type);

/** Reads a MuJoCo sensor into the provided data pointer, the data must have the correct size for the sensor type */
void mujoco_get_sensordata(const mjModel & model, const mjData & data, int sensor_id, double * sensor_reading);

/*! Cleanup. */
void mujoco_cleanup(MjSimImpl * mj_sim);

} // namespace mc_mujoco

#endif // MJ_UTILS_H_
