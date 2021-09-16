#include "glfw3.h"
#include "mujoco.h"

#include "mj_utils.h"

#include "config.h"

#include "imgui.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "implot.h"

#include "ImGuizmo.h"

#include "Robot_Regular_ttf.h"

#include "MujocoClient.h"

#include "widgets/details/InteractiveMarker.h"

namespace mc_mujoco
{

/*******************************************************************************
 * Global library state
 ******************************************************************************/

static bool glfw_initialized = false;
static bool mujoco_initialized = false;

// MuJoCo data structures
mjModel * m = NULL; // MuJoCo model
mjData * d = NULL; // MuJoCo data
GLFWwindow * window; // GLFWwindow
mjvCamera cam; // abstract camera
mjvOption opt; // visualization options
mjvScene scn; // abstract scene
mjrContext con; // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// mc_rtc client
std::unique_ptr<MujocoClient> client;

/*******************************************************************************
 * Callbacks for GLFWwindow
 ******************************************************************************/

// keyboard callback
void keyboard(GLFWwindow * window, int key, int scancode, int act, int mods)
{
  if(ImGui::GetIO().WantCaptureKeyboard)
  {
    return;
  }
  // C: show contacts
  if(act == GLFW_PRESS)
  {
    if(key == GLFW_KEY_C)
    {
      opt.flags[mjVIS_CONTACTPOINT] = !opt.flags[mjVIS_CONTACTPOINT];
    }
    if(key == GLFW_KEY_F)
    {
      opt.flags[mjVIS_CONTACTFORCE] = !opt.flags[mjVIS_CONTACTFORCE];
    }
    if(key >= GLFW_KEY_0 && key <= GLFW_KEY_9)
    {
      int group = key - GLFW_KEY_0;
      opt.geomgroup[group] = !opt.geomgroup[group];
    }
  }
}

// mouse button callback
void mouse_button(GLFWwindow * window, int button, int act, int mods)
{
  if(ImGui::GetIO().WantCaptureMouse)
  {
    return;
  }
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow * window, double xpos, double ypos)
{
  if(ImGui::GetIO().WantCaptureMouse)
  {
    return;
  }
  // no buttons down: nothing to do
  if(!button_left && !button_middle && !button_right) return;

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift =
      (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if(button_right)
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if(button_left)
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
    action = mjMOUSE_ZOOM;

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow * window, double xoffset, double yoffset)
{
  if(ImGui::GetIO().WantCaptureMouse)
  {
    return;
  }
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

/*******************************************************************************
 * Mujoco utility functions
 ******************************************************************************/

bool mujoco_init(const char * file_input)
{
  // Initialize MuJoCo
  if(!mujoco_initialized)
  {
    // Activate MuJoCo
    const char * key_buf_ptr = getenv("MUJOCO_KEY_PATH");
    std::string key_buf = [&]() -> std::string {
      if(key_buf_ptr)
      {
        return key_buf_ptr;
      }
      return mc_mujoco::MUJOCO_KEY_PATH;
    }();
    mj_activate(key_buf.c_str());

    // Load the model;
    const char * modelfile = file_input;
    char error[1000] = "Could not load XML model";
    m = mj_loadXML(modelfile, 0, error, 1000);
    if(!m)
    {
      std::cerr << error << std::endl;
      return false;
    }

    // make data
    d = mj_makeData(m);
    mujoco_initialized = true;
  }
  // Initialize GLFW
  if(!glfw_initialized)
  {
    if(!glfwInit())
    {
      return false;
    }
    glfw_initialized = true;
  }

  return mujoco_initialized && glfw_initialized;
}

void mujoco_create_window()
{
  // create window, make OpenGL context current, request v-sync
  window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  cam.lookat[0] = 0.0f;
  cam.lookat[1] = 0.0f;
  cam.lookat[2] = 0.75f;
  cam.distance = 6.0f;
  cam.azimuth = -150.0f;
  cam.elevation = -20.0f;
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);
  // set the geom group to false by default
  opt.geomgroup[0] = false;

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  /** Initialize Dear Imgui */

  // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
  // GL ES 2.0 + GLSL 100
  const char * glsl_version = "#version 100";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
  // GL 3.2 + GLSL 150
  const char * glsl_version = "#version 150";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Required on Mac
#else
  // GL 3.0 + GLSL 130
  const char * glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
  // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO & io = ImGui::GetIO();
  ImFontConfig fontConfig;
  fontConfig.FontDataOwnedByAtlas = false;
  io.FontDefault = io.Fonts->AddFontFromMemoryTTF(Roboto_Regular_ttf, Roboto_Regular_ttf_len, 18.0f, &fontConfig);

  ImGui::StyleColorsLight();
  auto & style = ImGui::GetStyle();
  style.FrameRounding = 6.0f;
  auto & bgColor = style.Colors[ImGuiCol_WindowBg];
  bgColor.w = 0.5f;
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  client = std::make_unique<MujocoClient>();
}

bool mujoco_set_const(const std::vector<double> & qpos, const std::vector<double> & qvel)
{
  if(qpos.size() != m->nq || qvel.size() != m->nv)
  {
    std::cerr << "qpos size: " << qpos.size() << ". Should be: " << m->nq << std::endl;
    std::cerr << "qvel size: " << qvel.size() << ". Should be: " << m->nv << std::endl;
    return false;
  }

  mj_setConst(m, d);
  const double * qpos_init = &qpos[0];
  const double * qvel_init = &qvel[0];
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
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();

  // update mc_rtc GUI client
  client->update();

  // Render ImGui
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();
  ImGuiIO & io = ImGui::GetIO();
  ImGuizmo::AllowAxisFlip(false);
  ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
  client->draw2D(window);
  client->draw3D();
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

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
  quat = quat.inverse(); // mc-rtc convention
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
  // NOTE: a jointpos sensor will return the same data as d->qpos.
  unsigned int index_ = 0;
  for(unsigned int i = 0; i < m->njnt; ++i)
  {
    if(m->jnt_type[i] != mjJNT_FREE)
    {
      qpos[index_] = d->qpos[m->jnt_qposadr[i]];
      index_++;
    }
  }
}

void mujoco_get_joint_vel(std::vector<double> & qvel)
{
  // NOTE: a jointvel sensor will return the same data as d->qvel.
  unsigned int index_ = 0;
  for(unsigned int i = 0; i < m->njnt; ++i)
  {
    if(m->jnt_type[i] != mjJNT_FREE)
    {
      qvel[index_] = d->qvel[m->jnt_dofadr[i]];
      index_++;
    }
  }
}

void mujoco_get_joint_qfrc(std::vector<double> & qfrc)
{
  qfrc.clear();
  for (unsigned int i = 0; i < m->nv; ++i)
  {
    if (m->jnt_type[m->dof_jntid[i]] != mjJNT_FREE)
    {
      qfrc.push_back(d->qfrc_actuator[i]);
    }
  }
}

bool mujoco_get_sensordata(std::vector<double> & read, const std::string & sensor_name)
{
  read.clear();
  for(unsigned int i = 0; i < m->nsensor; ++i)
  {
    if(mj_id2name(m, mjOBJ_SENSOR, i) == sensor_name)
    {
      for(unsigned int j = 0; j < m->sensor_dim[i]; ++j)
      {
        read.push_back(d->sensordata[m->sensor_adr[i] + j]);
      }
    }
  }
  return (read.size() ? true : false);
}

void mujoco_get_joints(std::vector<std::string> & names, std::vector<int> & ids)
{
  names.clear();
  ids.clear();
  for(size_t i = 0; i < m->njnt; ++i)
  {
    if(m->jnt_type[i] != mjJNT_FREE)
    {
      names.push_back(mj_id2name(m, mjOBJ_JOINT, i));
      ids.push_back(i);
    }
  }
}

void mujoco_get_motors(std::vector<std::string> & names, std::vector<int> & ids)
{
  names.clear();
  ids.clear();
  for(size_t i = 0; i < m->nu; ++i)
  {
    unsigned int jnt_id = m->actuator_trnid[2 * i];
    names.push_back(mj_id2name(m, mjOBJ_JOINT, jnt_id));
    ids.push_back(jnt_id);
  }
}

bool mujoco_set_ctrl(const std::vector<double> & ctrl)
{
  mju_zero(d->ctrl, m->nu);
  if(ctrl.size() != m->nu)
  {
    std::cerr << "Invalid size of control signal(" << ctrl.size() << ", expected " << m->nu << ")." << std::endl;
    return false;
  }
  // TODO: Check if mapping is correct.
  unsigned int index = 0;

  for(const auto i : ctrl)
  {
    double ratio = m->actuator_gear[6 * index];
    d->ctrl[index] = i / ratio;
    index++;
  }
  return true;
}

void mujoco_cleanup()
{
  // Close the window
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data, deactivate
  mj_deleteData(d);
  mj_deleteModel(m);
  mj_deactivate();

  mujoco_initialized = false;
  glfw_initialized = false;
  // FIXME Segfault?
  // glfwTerminate();
}

} // namespace mc_mujoco
