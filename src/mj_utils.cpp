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

namespace mc_mujoco
{

/*******************************************************************************
 * Global library state
 ******************************************************************************/

static bool glfw_initialized = false;
static bool mujoco_initialized = false;

/*******************************************************************************
 * Callbacks for GLFWwindow
 ******************************************************************************/

// keyboard callback
void keyboard(GLFWwindow * window, int key, int scancode, int act, int mods)
{
  auto mj_sim = static_cast<MjSimImpl *>(glfwGetWindowUserPointer(window));
  auto & opt = mj_sim->options;
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
  auto mj_sim = static_cast<MjSimImpl *>(glfwGetWindowUserPointer(window));
  auto & button_left = mj_sim->button_left;
  auto & button_middle = mj_sim->button_middle;
  auto & button_right = mj_sim->button_right;
  auto & lastx = mj_sim->lastx;
  auto & lasty = mj_sim->lasty;
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
  auto mj_sim = static_cast<MjSimImpl *>(glfwGetWindowUserPointer(window));
  auto & button_left = mj_sim->button_left;
  auto & button_middle = mj_sim->button_middle;
  auto & button_right = mj_sim->button_right;
  auto & lastx = mj_sim->lastx;
  auto & lasty = mj_sim->lasty;
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
  mjv_moveCamera(mj_sim->model, action, dx / height, dy / height, &mj_sim->scene, &mj_sim->camera);
}

// scroll callback
void scroll(GLFWwindow * window, double xoffset, double yoffset)
{
  if(ImGui::GetIO().WantCaptureMouse)
  {
    return;
  }
  auto mj_sim = static_cast<MjSimImpl *>(glfwGetWindowUserPointer(window));
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(mj_sim->model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &mj_sim->scene, &mj_sim->camera);
}

/*******************************************************************************
 * Mujoco utility functions
 ******************************************************************************/

bool mujoco_init(MjSimImpl * mj_sim, const std::vector<std::string> & robots, const std::vector<std::string> & xmlFiles)
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
    mujoco_initialized = true;
  }

  // Load the model;
  std::string model = merge_mujoco_models(robots, xmlFiles, mj_sim->robots);
  char error[1000] = "Could not load XML model";
  mj_sim->model = mj_loadXML(model.c_str(), 0, error, 1000);
  if(!mj_sim->model)
  {
    std::cerr << error << std::endl;
    return false;
  }

  // make data
  mj_sim->data = mj_makeData(mj_sim->model);

  return true;
}

void mujoco_create_window(MjSimImpl * mj_sim)
{
  // Initialize GLFW
  if(!glfw_initialized)
  {
    if(!glfwInit())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] GLFW initialization failed");
    }
    glfw_initialized = true;
  }

  // create window, make OpenGL context current, request v-sync
  mj_sim->window = glfwCreateWindow(1600, 900, "mc_mujoco", NULL, NULL);
  if(!mj_sim->window)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] GLFW window creation failed");
  }
  glfwMakeContextCurrent(mj_sim->window);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(mj_sim->window, static_cast<void *>(mj_sim));

  // initialize visualization data structures
  mj_sim->camera.lookat[0] = 0.0f;
  mj_sim->camera.lookat[1] = 0.0f;
  mj_sim->camera.lookat[2] = 0.75f;
  mj_sim->camera.distance = 6.0f;
  mj_sim->camera.azimuth = -150.0f;
  mj_sim->camera.elevation = -20.0f;
  mjv_defaultOption(&mj_sim->options);
  mj_sim->options.geomgroup[0] = mj_sim->config.visualize_collisions;
  mj_sim->options.geomgroup[1] = mj_sim->config.visualize_visual;
  mjv_defaultScene(&mj_sim->scene);
  mjr_defaultContext(&mj_sim->context);

  // create scene and context
  mjv_makeScene(mj_sim->model, &mj_sim->scene, 2000);
  mjr_makeContext(mj_sim->model, &mj_sim->context, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(mj_sim->window, keyboard);
  glfwSetCursorPosCallback(mj_sim->window, mouse_move);
  glfwSetMouseButtonCallback(mj_sim->window, mouse_button);
  glfwSetScrollCallback(mj_sim->window, scroll);

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
  ImVector<ImWchar> ranges;
  ImFontGlyphRangesBuilder builder;
  builder.AddText(u8"μ");
  builder.AddRanges(io.Fonts->GetGlyphRangesDefault());
  builder.BuildRanges(&ranges);
  io.FontDefault =
      io.Fonts->AddFontFromMemoryTTF(Roboto_Regular_ttf, Roboto_Regular_ttf_len, 18.0f, &fontConfig, ranges.Data);
  io.Fonts->Build();

  ImGui::StyleColorsLight();
  auto & style = ImGui::GetStyle();
  style.FrameRounding = 6.0f;
  auto & bgColor = style.Colors[ImGuiCol_WindowBg];
  bgColor.w = 0.5f;
  ImGui_ImplGlfw_InitForOpenGL(mj_sim->window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);
}

bool mujoco_set_const(mjModel * m, mjData * d, const std::vector<double> & qpos, const std::vector<double> & qvel)
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

void mujoco_cleanup(MjSimImpl * mj_sim)
{
  if(mj_sim->config.with_visualization)
  {
    // Close the window
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(mj_sim->window);
  }

  // free visualization storage
  mjv_freeScene(&mj_sim->scene);
  mjr_freeContext(&mj_sim->context);

  // free MuJoCo model and data, deactivate
  mj_deleteData(mj_sim->data);
  mj_deleteModel(mj_sim->model);

  // FIXME glfwTerminate will segfault so we never de-init glfw
  // Ref: http://www.mujoco.org/forum/index.php?threads/segmentation-fault-for-record-on-ubuntu-16-04.3516/#post-4205
  // glfw_initialized = false;
  // glfwTerminate();
}

int mujoco_get_sensor_id(const mjModel & m, const std::string & name, mjtSensor type)
{
  auto id = mj_name2id(&m, mjOBJ_SENSOR, name.c_str());
  return (id != -1 && m.sensor_type[id] == type) ? id : -1;
}

void mujoco_get_sensordata(const mjModel & model, const mjData & data, int sensor_id, double * sensor_reading)
{
  if(sensor_id == -1)
  {
    return;
  }
  std::memcpy(sensor_reading, &data.sensordata[model.sensor_adr[sensor_id]],
              model.sensor_dim[sensor_id] * sizeof(double));
}

} // namespace mc_mujoco
