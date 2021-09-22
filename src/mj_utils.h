#ifndef MJ_UTILS_H
#define MJ_UTILS_H

#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <vector>

#include "mj_sim_impl.h"

namespace mc_mujoco
{

/*! Load XML model and initialize */
bool mujoco_init(MjSimImpl * mj_sim, const char * modelfile);

/*! Create GLFW window */
void mujoco_create_window(MjSimImpl * mj_sim);

/*! Sets initial qpos and qvel in mjData */
bool mujoco_set_const(mjModel * m, mjData * d, const std::vector<double> & qpos, const std::vector<double> & qvel);

/*! Get true position of robot root joint. */
void mujoco_get_root_pos(const mjData & d, Eigen::Vector3d & pos);

/*! Get true orientation of robot root joint. */
void mujoco_get_root_orient(const mjData & d, Eigen::Quaterniond & quat);

/*! Get true linear velocity of robot root joint. */
void mujoco_get_root_lin_vel(const mjData & d, Eigen::Vector3d & linvel);

/*! Get true angular velocity of robot root joint. */
void mujoco_get_root_ang_vel(const mjData & d, Eigen::Vector3d & angvel);

/*! Get true linear acceleration of robot root joint. */
void mujoco_get_root_lin_acc(const mjData & d, Eigen::Vector3d & linacc);

/*! Get true angular acceleration of robot root joint. */
void mujoco_get_root_ang_acc(const mjData & d, Eigen::Vector3d & angacc);

/*! Get positions of all joints except the freejoint. */
void mujoco_get_joint_pos(const mjModel & m, const mjData & d, std::vector<double> & qpos);

/*! Get velocities of all joints except the freejoint. */
void mujoco_get_joint_vel(const mjModel & m, const mjData & d, std::vector<double> & qvel);

/*! Get joint torques (in joint space).
 * See definition of qfrc_actuator at mujoco.org.
 */
void mujoco_get_joint_qfrc(const mjModel & m, const mjData & d, std::vector<double> & qfrc);

/*! Generic function to read sensor meansurements.
 * Returns false if sensor name is not present in model.
 */
bool mujoco_get_sensordata(const mjModel & m,
                           const mjData & d,
                           std::vector<double> & read,
                           const std::string & sensor_name);

/*! Get names and ids of all joints except the freejoint. */
void mujoco_get_joints(const mjModel & m, std::vector<std::string> & names, std::vector<int> & ids);

/*! Get names and ids of all actuated joints (NOT the names of actuators). */
void mujoco_get_motors(const mjModel & m, std::vector<std::string> & names, std::vector<int> & ids);

/*! Sets the control data after scaling down by gear ratio.
 * Returns false if there is a vector size mismatch.
 */
bool mujoco_set_ctrl(const mjModel & m, mjData & d, const std::vector<double> & ctrl);

/*! Cleanup. */
void mujoco_cleanup(MjSimImpl * mj_sim);

} // namespace mc_mujoco

#endif // MJ_UTILS_H_
