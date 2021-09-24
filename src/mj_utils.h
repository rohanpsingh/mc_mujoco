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
 * \param robots Prefix applied to each model
 *
 * \param xmlFiles Individual models
 *
 * \returns The path to the generated model
 */
std::string merge_mujoco_models(const std::vector<std::string> & robots,
                                const std::vector<std::string> & xmlFiles,
                                std::vector<MjRobot> & mjRobots);

/*! Load XML model and initialize */
bool mujoco_init(MjSimImpl * mj_sim,
                 const std::vector<std::string> & robots,
                 const std::vector<std::string> & xmlFiles);

/*! Create GLFW window */
void mujoco_create_window(MjSimImpl * mj_sim);

/*! Sets initial qpos and qvel in mjData */
bool mujoco_set_const(mjModel * m, mjData * d, const std::vector<double> & qpos, const std::vector<double> & qvel);

/*! Generic function to read sensor meansurements.
 * Returns false if sensor name is not present in model.
 */
bool mujoco_get_sensordata(const mjModel & m,
                           const mjData & d,
                           std::vector<double> & read,
                           const std::string & sensor_name);

/*! Cleanup. */
void mujoco_cleanup(MjSimImpl * mj_sim);

} // namespace mc_mujoco

#endif // MJ_UTILS_H_
