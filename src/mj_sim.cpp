#include "mj_sim.h"
#include "mj_utils.h"

#include <type_traits>
#include <chrono>
#include <cassert>


struct MjSimImpl
{
private:
  mc_control::MCGlobalController & controller;

  size_t iterCount_ = 0;
  size_t frameskip_ = 1;

  Eigen::Vector3d root_pos;
  Eigen::Quaterniond root_orient;
  Eigen::Vector3d root_linvel;
  Eigen::Vector3d root_angvel;
  Eigen::Vector3d root_linacc;
  Eigen::Vector3d root_angacc;
  std::vector<double> encoders;
  std::vector<double> alphas;
  std::vector<double> torques;
  std::map<std::string, sva::ForceVecd> wrenches;

  std::vector<double> kp = {};
  std::vector<double> kd = {};

  std::vector<std::string> mj_act_names;
  std::vector<std::string> mj_jnt_names;


  /* PD control */
  double PD(double idx,
	    double q_ref, double q,
	    double qdot_ref, double qdot)
  {
    double p_error = q_ref-q;
    double v_error = qdot_ref-qdot;
    double ret = (kp[idx]*p_error + kd[idx]*v_error);
    return ret;
  }

  /* Load PD gains from file (taken from RobotHardware/robot.cpp) */
  bool loadGain(const std::string & path_to_pd, const std::vector<std::string> & joints)
  {
    std::ifstream strm(path_to_pd.c_str());
    if (!strm.is_open()) {
	std::cerr << path_to_pd << " not found" << std::endl;
	return false;
    }

    int num_joints = joints.size();
    std::vector<double> default_pgain(num_joints, 0);
    std::vector<double> default_dgain(num_joints, 0);
    for (int i = 0; i < num_joints; i++) {
	std::string str;
	bool getlinep;
	while ((getlinep = !!(std::getline(strm, str)))) {
	    if (str.empty())   {
		continue;
	    }
	    if (str[0] == '#') {
		continue;
	    }
	    double tmp;
	    std::istringstream sstrm(str);
	    sstrm >> tmp;
	    default_pgain[i] = tmp;
	    if(sstrm.eof()) break;

	    sstrm >> tmp;
	    default_dgain[i] = tmp;
	    if(sstrm.eof()) break;
	    break;
	}
	if(!getlinep) {
	    if (i < num_joints) {
		std::cerr << "[mc_mujoco] loadGain error: size of gains reading from file ("
			  << path_to_pd
			  << ") does not match size of joints" << std::endl;
	    }
	    break;
	}
    }

    strm.close();
    // Print loaded gain
    std::cerr << "[mc_mujoco] loadGain" << std::endl;
    for (unsigned int i=0; i<num_joints; i++) {
	std::cerr << "[mc_mujoco]   " << joints[i]
		  << ", pgain = " << default_pgain[i]
		  << ", dgain = " << default_dgain[i]
		  << std::endl;
	// push to kp and kd
	kp.push_back(default_pgain[i]);
	kd.push_back(default_dgain[i]);
    }
    return true;
  }


public:
  MjSimImpl(mc_control::MCGlobalController & controller,
	    const MjConfiguration & config)
  : controller(controller)
  {
    // initial mujoco here and load XML model 
    bool initialized = mujoco_init(config.xmlPath.c_str());
    if(!initialized)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Initialized failed.");
    }

    // read PD gains from file
    std::string path_to_pd = config.pdGains.c_str();
    std::vector<std::string> rjo = controller.ref_joint_order();
    loadGain(path_to_pd, rjo);

    mujoco_create_window();
    mc_rtc::log::info("[mc_mujoco] Initialized successful.");
  }

  void cleanup()
  {
    mujoco_cleanup();
  }

  void startSimulation()
  {
    // get names of all joints
    mujoco_get_joint_names(mj_jnt_names);
    // get names of actuated joints
    mujoco_get_motor_names(mj_act_names);

    // init attitude from mc-rtc
    const auto q0 = controller.robot().module().default_attitude();
    std::vector<double> mj_qpos_init{q0[4], q0[5], q0[6], q0[0], q0[1], q0[2], q0[3]};
    std::vector<double> mj_qvel_init{0, 0, 0, 0, 0, 0};

    // init qpos and qvel from mc-rtc
    {
      auto & mbc = controller.robot().mbc();
      const auto & rjo = controller.ref_joint_order();
      for(const auto & jn : rjo)
      {
	if(controller.robot().hasJoint(jn))
	{
	  for(auto & qj : mbc.q[controller.robot().jointIndexByName(jn)])
	  {
	    encoders.push_back(qj);
	    if(std::find(mj_jnt_names.begin(), mj_jnt_names.end(), jn)!=mj_jnt_names.end())
	    {
	      mj_qpos_init.push_back(qj);
	      mj_qvel_init.push_back(0);
	    }
	  }
	}
	else
	{
	  // FIXME This assumes that a joint that is in ref_joint_order but missing from the robot is of size 1 (very
	  // likely to be true)
	  encoders.push_back(0);
	}
      }
    }
    alphas.resize(encoders.size());
    std::fill(alphas.begin(), alphas.end(), 0);

    // sanity check
    if(encoders.size()!=mj_jnt_names.size())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Num encoders size mismatch.");
    }
    // zero force/torque wrenches
    for (const auto & fs : controller.robot().module().forceSensors())
    {
      wrenches[fs.name()] = sva::ForceVecd(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    }

    // set initial qpos, qvel in mujoco
    if (!mujoco_set_const(mj_qpos_init, mj_qvel_init))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Set inital state failed.");
    }

    // get sim timestep and set the frameskip parameter
    double simTimestep = mujoco_get_timestep();
    frameskip_ = std::round(controller.timestep() / simTimestep);
    mc_rtc::log::info("[mc_mujoco] MC-RTC timestep: {}. MJ timestep: {}", controller.timestep(), simTimestep);
    mc_rtc::log::info("[mc_mujoco] Hence, Frameskip: {}", frameskip_);

    controller.setEncoderValues(encoders);
    controller.init(encoders);
    controller.running = true;
  }

  void updateData()
  {
    auto & robot = controller.controller().robots().robot(0);

    /* True state of floating base */
    if(robot.hasBodySensor("FloatingBase"))
    {
      // set root position
      mujoco_get_root_pos(root_pos);
      controller.setSensorPositions({{"FloatingBase", root_pos}});
      // set root orientation
      mujoco_get_root_orient(root_orient);
      controller.setSensorOrientations({{"FloatingBase", root_orient}});
      // set linear velocity
      mujoco_get_root_lin_vel(root_linvel);
      controller.setSensorLinearVelocities({{"FloatingBase", root_linvel}});
      // set angular velocity
      mujoco_get_root_ang_vel(root_angvel);
      controller.setSensorAngularVelocities({{"FloatingBase", root_angvel}});
      // set linear acceleration
      mujoco_get_root_lin_acc(root_linacc);
      controller.setSensorLinearAccelerations({{"FloatingBase", root_linacc}});
      // set angular acceleration
      mujoco_get_root_ang_acc(root_angacc);
      controller.setSensorAngularAccelerations({{"FloatingBase", root_angacc}});
    }
    controller.setSensorPosition(root_pos);
    controller.setSensorOrientation(root_orient);
    controller.setSensorLinearVelocity(root_linvel);
    controller.setSensorAngularVelocity(root_angvel);
    controller.setSensorLinearAcceleration(root_linacc);
    controller.setSensorAngularAcceleration(root_linacc);

    /* State from sensor data*/
    // set angular velocity
    std::vector<double> _gyro;
    mujoco_get_sensordata(_gyro, "root_gyro");
    if (!_gyro.size()==3) mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] gyro read failed.");
    controller.setSensorAngularVelocities({{"Accelerometer", Eigen::Vector3d(_gyro.data())}});

    // set linear acceleration
    std::vector<double> _accel;
    mujoco_get_sensordata(_accel, "root_accel");
    if (!_accel.size()==3) mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] accel read failed.");
    controller.setSensorLinearAccelerations({{"Accelerometer", Eigen::Vector3d(_accel.data())}});

    // set encoders
    encoders.resize(robot.refJointOrder().size());
    mujoco_get_joint_pos(encoders);
    controller.setEncoderValues(encoders);

    // set velocities
    alphas.resize(robot.refJointOrder().size());
    mujoco_get_joint_vel(alphas);
    controller.setEncoderVelocities(alphas);

    // set foot force
    std::vector<double> _rf_fsensor, _rf_tsensor;
    std::vector<double> _lf_fsensor, _lf_tsensor;
    std::vector<double> _rh_fsensor, _rh_tsensor;
    std::vector<double> _lh_fsensor, _lh_tsensor;
    mujoco_get_sensordata(_rf_fsensor, "rf_fsensor");
    mujoco_get_sensordata(_lf_fsensor, "lf_fsensor");
    mujoco_get_sensordata(_rf_tsensor, "rf_tsensor");
    mujoco_get_sensordata(_lf_tsensor, "lf_tsensor");
    mujoco_get_sensordata(_rh_fsensor, "rh_fsensor");
    mujoco_get_sensordata(_lh_fsensor, "lh_fsensor");
    mujoco_get_sensordata(_rh_tsensor, "rh_tsensor");
    mujoco_get_sensordata(_lh_tsensor, "lh_tsensor");
    wrenches["RightFootForceSensor"].force()  = -1*Eigen::Vector3d(_rf_fsensor.data());
    wrenches["RightFootForceSensor"].couple() = -1*Eigen::Vector3d(_rf_tsensor.data());
    wrenches["LeftFootForceSensor"].force()   = -1*Eigen::Vector3d(_lf_fsensor.data());
    wrenches["LeftFootForceSensor"].couple()  = -1*Eigen::Vector3d(_lf_tsensor.data());
    wrenches["RightHandForceSensor"].force()  = -1*Eigen::Vector3d(_rh_fsensor.data());
    wrenches["RightHandForceSensor"].couple() = -1*Eigen::Vector3d(_rh_tsensor.data());
    wrenches["LeftHandForceSensor"].force()   = -1*Eigen::Vector3d(_lh_fsensor.data());
    wrenches["LeftHandForceSensor"].couple()  = -1*Eigen::Vector3d(_lh_tsensor.data());
    controller.setWrenches(wrenches);
  }

  bool controlStep()
  {
    // skip control.run() from frameskip_ number of iters.
    if (iterCount_++ % frameskip_ != 0)
    {
      return false;
    }

    std::vector<double> ctrl;
    // step controller and get QP result
    if (controller.run())
    {
      double t = 0; //unused argument
      const mc_solver::QPResultMsg & res = controller.send(t);
      const auto & rjo = controller.robot().refJointOrder();
      for (size_t i = 0; i < rjo.size(); ++i)
      {
	const auto & jn = rjo[i];
	if(res.robots_state[0].q.count(jn))
	{
	  auto id = std::find(mj_act_names.begin(), mj_act_names.end(), jn);
	  if (id!=mj_act_names.end())
	  {
	    unsigned int index_ = std::distance(mj_act_names.begin(), id);
	    // convert PD targets to torque using PD control
	    double tau = PD(index_,
			    res.robots_state[0].q.at(jn)[0], encoders[i],
			    0, alphas[i]);
	    ctrl.push_back(tau);
	  }
	}
      }
    }

    // send control signal to mujoco
    bool done = true;
    if (controller.running)
    {
      if (mujoco_set_ctrl(ctrl))
      {
	done = false;
      }
    }
    return done;
  }

  void simStep()
  {
    // take one step in simulation
    // model.opt.timestep will be used here
    mujoco_step();
  }

  bool render()
  {
    // render in mujoco window
    return mujoco_render();
  }

  void stopSimulation()
  {
  }
};


MjSim::MjSim(mc_control::MCGlobalController & controller, const MjConfiguration & config)
: impl(new MjSimImpl(controller, config))
{
}

MjSim::~MjSim()
{
  impl->cleanup();
}

void MjSim::startSimulation()
{
  impl->startSimulation();
}

bool MjSim::controlStep()
{
  impl->controlStep();
}

void MjSim::simStep()
{
  impl->simStep();
}

void MjSim::stopSimulation()
{
  impl->stopSimulation();
}

void MjSim::updateData()
{
  impl->updateData();
}

bool MjSim::render()
{
  impl->render();
}
