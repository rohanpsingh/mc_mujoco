#include "mujoco.h"
#include "glfw3.h"

#include "mj_utils.h"

/*******************************************************************************
 * Global library state
 ******************************************************************************/

static bool glfw_initialized = false;
static bool mujoco_initialized = false;

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
GLFWwindow* window;                 // GLFWwindow
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


/*******************************************************************************
 * Callbacks for GLFWwindow
 ******************************************************************************/

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  // backspace: reset simulation
  if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
  {
      mj_resetData(m, d);
      mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
  // update button state
  button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if( !button_left && !button_middle && !button_right )
      return;

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
		    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if( button_right )
      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if( button_left )
      action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
      action = mjMOUSE_ZOOM;

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

/*******************************************************************************
 * Mujoco utility functions
 ******************************************************************************/

bool mujoco_init(const char *file_input)
{
  // Initialize MuJoCo
  if (!mujoco_initialized) {
      // Activate MuJoCo
      const char* key_buf = getenv("MUJOCO_KEY_PATH");
      mj_activate(key_buf);

      // Load the model;
      const char* modelfile = file_input;
      char error[1000] = "Could not load XML model";
      m = mj_loadXML(modelfile, 0, error, 1000); 
      if (!m) {
	  std::cerr << error << std::endl;
	  return false;
      }

      // make data
      d = mj_makeData(m);
      mujoco_initialized = true;
  }
  // Initialize GLFW
  if (!glfw_initialized) {
      if (!glfwInit()) {
	  return false;
      }
      glfw_initialized = true;
  }

  return mujoco_initialized&&glfw_initialized;
}

void mujoco_create_window()
{
  // create window, make OpenGL context current, request v-sync
  window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);
}

bool mujoco_set_const(const std::vector<double> & qpos, const std::vector<double> & qvel)
{
  if (qpos.size()!=m->nq || qvel.size()!=m->nv)
  {
    std::cerr << "qpos size: " << qpos.size() << ". Should be: " << m->nq << std::endl;
    std::cerr << "qvel size: " << qvel.size() << ". Should be: " << m->nv << std::endl;
    return false;
  }

  mj_setConst(m, d);
  const double* qpos_init = &qpos[0];
  const double* qvel_init = &qvel[0];
  mju_copy(d->qpos, qpos_init, m->nq);
  mju_copy(d->qvel, qvel_init, m->nv);
  d->time = 0.0;
  mj_forward(m, d);
  return true;
}

void mujoco_step()
{
  mj_step(m, d);
}

bool mujoco_render() 
{
  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  // update scene and render
  opt.geomgroup[0] = false;
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
  return !glfwWindowShouldClose(window);
}

double mujoco_get_timestep()
{
  return m->opt.timestep;
}

void mujoco_get_root_pos(Eigen::Vector3d & pos)
{
  pos.setZero();
  pos[0] = d->qpos[0];
  pos[1] = d->qpos[1];
  pos[2] = d->qpos[2];
}

void mujoco_get_root_orient(Eigen::Quaterniond & quat)
{
  quat.coeffs().setZero();
  quat.w() = d->qpos[3];
  quat.x() = d->qpos[4];
  quat.y() = d->qpos[5];
  quat.z() = d->qpos[6];
  quat = quat.inverse();  // mc-rtc convention
}

void mujoco_get_root_lin_vel(Eigen::Vector3d & linvel)
{
  linvel.setZero();
  linvel[0] = d->qvel[0];
  linvel[1] = d->qvel[1];
  linvel[2] = d->qvel[2];
}

void mujoco_get_root_ang_vel(Eigen::Vector3d & angvel)
{
  angvel.setZero();
  angvel[0] = d->qvel[3];
  angvel[1] = d->qvel[4];
  angvel[2] = d->qvel[5];
}

void mujoco_get_root_lin_acc(Eigen::Vector3d & linacc)
{
  linacc.setZero();
  linacc[0] = d->qacc[0];
  linacc[1] = d->qacc[1];
  linacc[2] = d->qacc[2];
}

void mujoco_get_root_ang_acc(Eigen::Vector3d & angacc)
{
  angacc.setZero();
  angacc[0] = d->qacc[3];
  angacc[1] = d->qacc[4];
  angacc[2] = d->qacc[5];
}

void mujoco_get_joint_pos(std::vector<double> & qpos)
{
  //NOTE: a jointpos sensor will return the same data as d->qpos.
  unsigned int index_ = 0;
  for (unsigned int i = 0; i < m->njnt; ++i)
  {
    if (m->jnt_type[i]!=mjJNT_FREE)
    {
      qpos[index_] = d->qpos[m->jnt_qposadr[i]];
      index_++;
    }
  }
}

void mujoco_get_joint_vel(std::vector<double> & qvel)
{
  //NOTE: a jointvel sensor will return the same data as d->qvel.
  unsigned int index_ = 0;
  for (unsigned int i = 0; i < m->njnt; ++i)
  {
    if (m->jnt_type[i]!=mjJNT_FREE)
    {
      qvel[index_] = d->qvel[m->jnt_dofadr[i]];
      index_++;
    }
  }
}

bool mujoco_get_sensordata(std::vector<double> & read, const std::string & sensor_name)
{
  bool success = false;
  for (unsigned int i = 0; i < m->nsensor; ++i)
  {
    if (mj_id2name(m, mjOBJ_SENSOR, i)==sensor_name)
    {
      success = true;
      for (unsigned int j = 0; j < m->sensor_dim[i]; ++j)
      {
	read.push_back(d->sensordata[m->sensor_adr[i]+j]);
      }
    }
  }
  return success;
}

void mujoco_get_joint_names(std::vector<std::string> & names)
{
  names.clear();
  for (size_t i = 0; i < m->njnt; ++i)
  {
    if (m->jnt_type[i]!=mjJNT_FREE)
    {
      names.push_back(mj_id2name(m, mjOBJ_JOINT, i));
    }
  }
}

void mujoco_get_motor_names(std::vector<std::string> & names)
{
  names.clear();
  for (size_t i = 0; i < m->nu; ++i)
  {
    unsigned int jnt_id = m->actuator_trnid[2*i];
    names.push_back(mj_id2name(m, mjOBJ_JOINT, jnt_id));
  }
}

bool mujoco_set_ctrl(const std::vector<double> & ctrl)
{
  mju_zero(d->ctrl, m->nu);
  if (ctrl.size()!=m->nu)
  {
    std::cerr << "Invalid size of control signal(" << ctrl.size() << ")." << std::endl;
    return false;
  }
  // TODO: Check if mapping is correct.
  unsigned int index = 0;

  for (const auto i : ctrl)
  {
    double ratio = m->actuator_gear[6*index];
    d->ctrl[index] = i/ratio;
    index++;
  }
  return true;
}

void mujoco_cleanup()
{
  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data, deactivate
  mj_deleteData(d);
  mj_deleteModel(m);
  mj_deactivate();

  mujoco_initialized = false;
  glfw_initialized = false;
}
