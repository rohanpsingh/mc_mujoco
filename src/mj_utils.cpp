#include "glfw3.h"
#include "mujoco.h"

#ifndef USE_UI_ADAPTER
#  include "uitools.h"
#else
#  include "our_glfw_adapter.h"
#endif

#include "mj_utils.h"

#include "config.h"

#include "imgui.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "implot.h"

#include "ImGuizmo.h"

#include "Robot_Regular_ttf.h"

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

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

// set window layout
void uiLayout(mjuiState * state)
{
  auto mj_sim = static_cast<MjSimImpl *>(state->userdata);

  mjrRect * rect = state->rect;
  // set number of rectangles
  state->nrect = 1;
  // rect 0: entire framebuffer
  rect[0].left = 0;
  rect[0].bottom = 0;
#ifdef USE_UI_ADAPTER
  std::tie(rect[0].width, rect[0].height) = mj_sim->platform_ui_adapter->GetFramebufferSize();
#else
  glfwGetFramebufferSize(mj_sim->window, &rect[0].width, &rect[0].height);
#endif
}

// handle UI event
void uiEvent(mjuiState * state)
{
  auto mj_sim = static_cast<MjSimImpl *>(state->userdata);

  if(state->type == mjEVENT_KEY && ImGui::GetIO().WantCaptureKeyboard)
  {
    return;
  }
  if(state->type != mjEVENT_KEY && ImGui::GetIO().WantCaptureMouse)
  {
    return;
  }
  if(state->type == mjEVENT_KEY && state->key != 0)
  {
    // C: show contact points
    if(state->key == GLFW_KEY_C)
    {
      mj_sim->options.flags[mjVIS_CONTACTPOINT] = !mj_sim->options.flags[mjVIS_CONTACTPOINT];
    }
    // F: show contact forces
    if(state->key == GLFW_KEY_F)
    {
      if(!mj_sim->options.flags[mjVIS_CONTACTFORCE])
      {
        mj_sim->options.flags[mjVIS_CONTACTFORCE] = 1;
        mj_sim->options.flags[mjVIS_CONTACTSPLIT] = 0;
      }
      else
      {
        if(!mj_sim->options.flags[mjVIS_CONTACTSPLIT])
        {
          mj_sim->options.flags[mjVIS_CONTACTSPLIT] = 1;
        }
        else
        {
          mj_sim->options.flags[mjVIS_CONTACTFORCE] = 0;
          mj_sim->options.flags[mjVIS_CONTACTSPLIT] = 0;
        }
      }
    }
    // 0-mjNGROUP: Toggle visiblity of geom groups
    if(state->key >= GLFW_KEY_0 && state->key < (GLFW_KEY_0 + mjNGROUP))
    {
      int group = state->key - GLFW_KEY_0;
      mj_sim->options.geomgroup[group] = !mj_sim->options.geomgroup[group];
    }
    // Ctrl+S save the visualization state
    if(state->key == GLFW_KEY_S && state->control)
    {
      mj_sim->saveGUISettings();
    }
    // SPACE: play/pause the simulation
    if(state->key == GLFW_KEY_SPACE)
    {
      mj_sim->config.step_by_step = !mj_sim->config.step_by_step;
    }
    // RIGHT: advance simulation by one control step
    if(state->key == GLFW_KEY_RIGHT)
    {
      if(mj_sim->config.step_by_step)
      {
        mj_sim->rem_steps = 1;
      }
    }
    // E: visualize frames
    if(state->key == GLFW_KEY_E)
    {
      mj_sim->options.frame += 1;
      if(mj_sim->options.frame == mjNFRAME)
      {
        mj_sim->options.frame = 0;
      }
    }
    // T: make transparent
    if(state->key == GLFW_KEY_T)
    {
      mj_sim->options.flags[mjVIS_TRANSPARENT] = !mj_sim->options.flags[mjVIS_TRANSPARENT];
    }
    // V: render convex hull
    if(state->key == GLFW_KEY_V)
    {
      mj_sim->options.flags[mjVIS_CONVEXHULL] = !mj_sim->options.flags[mjVIS_CONVEXHULL];
    }
    // TAB: switch cameras
    if(state->key == GLFW_KEY_TAB)
    {
      mj_sim->camera.fixedcamid += 1;
      mj_sim->camera.type = mjCAMERA_FIXED;
      if(mj_sim->camera.fixedcamid == mj_sim->model->ncam)
      {
        mj_sim->camera.fixedcamid = -1;
        mj_sim->camera.type = mjCAMERA_FREE;
      }
    }
    return;
  }

  // 3D scroll
  if(state->type == mjEVENT_SCROLL && state->mouserect == 0 && mj_sim->model)
  {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(mj_sim->model, mjMOUSE_ZOOM, 0, -0.05 * state->sy, &mj_sim->scene, &mj_sim->camera);
    return;
  }

  // 3D press
  if(state->type == mjEVENT_PRESS && state->mouserect == 0 && mj_sim->model)
  {
    // set perturbation
    int newperturb = 0;
    if(state->control && mj_sim->pert.select > 0)
    {
      // right: translate;  left: rotate
      if(state->right)
        newperturb = mjPERT_TRANSLATE;
      else if(state->left)
        newperturb = mjPERT_ROTATE;

      // perturbation onset: reset reference
      if(newperturb && !mj_sim->pert.active)
        mjv_initPerturb(mj_sim->model, mj_sim->data, &mj_sim->scene, &mj_sim->pert);
    }
    mj_sim->pert.active = newperturb;

    // handle double-click
    if(state->doubleclick)
    {
      // determine selection mode
      int selmode;
      if(state->button == mjBUTTON_LEFT)
        selmode = 1;
      else if(state->control)
        selmode = 3;
      else
        selmode = 2;

      // find geom and 3D click point, get corresponding body
      mjrRect r = state->rect[0];
      mjtNum selpnt[3];
      int selgeom;
      int selskin;
#if mjVERSION_HEADER < 300
      int selbody =
          mjv_select(mj_sim->model, mj_sim->data, &mj_sim->options, (mjtNum)r.width / (mjtNum)r.height,
                     (mjtNum)(state->x - r.left) / (mjtNum)r.width, (mjtNum)(state->y - r.bottom) / (mjtNum)r.height,
                     &mj_sim->scene, selpnt, &selgeom, &selskin);
#else
      int selflex;
      int selbody =
          mjv_select(mj_sim->model, mj_sim->data, &mj_sim->options, (mjtNum)r.width / (mjtNum)r.height,
                     (mjtNum)(state->x - r.left) / (mjtNum)r.width, (mjtNum)(state->y - r.bottom) / (mjtNum)r.height,
                     &mj_sim->scene, selpnt, &selgeom, &selflex, &selskin);

      selbody >= 0 ? mj_sim->pert.flexselect = selflex : mj_sim->pert.flexselect = -1;
#endif

      // set lookat point, start tracking is requested
      if(selmode == 2 || selmode == 3)
      {
        // copy selpnt if anything clicked
        if(selbody >= 0) mju_copy3(mj_sim->camera.lookat, selpnt);

        // switch to tracking camera if dynamic body clicked
        if(selmode == 3 && selbody > 0)
        {
          // mujoco camera
          mj_sim->camera.type = mjCAMERA_TRACKING;
          mj_sim->camera.trackbodyid = selbody;
          mj_sim->camera.fixedcamid = -1;
        }
      }

      // set body selection
      else
      {
        if(selbody >= 0)
        {

          // record selection
          mj_sim->pert.select = selbody;
          mj_sim->pert.skinselect = selskin;

          // compute localpos
          mjtNum tmp[3];
          mju_sub3(tmp, selpnt, mj_sim->data->xpos + 3 * mj_sim->pert.select);
          mju_mulMatTVec(mj_sim->pert.localpos, mj_sim->data->xmat + 9 * mj_sim->pert.select, tmp, 3, 3);
        }
        else
        {
          mj_sim->pert.select = 0;
          mj_sim->pert.skinselect = -1;
        }
      }

      // stop perturbation on select
      mj_sim->pert.active = 0;
    }
    return;
  }

  // 3D release
  if(state->type == mjEVENT_RELEASE && state->dragrect == 0 && mj_sim->model)
  {
    // stop perturbation
    mj_sim->pert.active = 0;
    return;
  }

  // 3D move
  if(state->type == mjEVENT_MOVE && state->dragrect == 0 && mj_sim->model)
  {
    // determine action based on mouse button
    mjtMouse action;
    if(state->right)
      action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if(state->left)
      action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
      action = mjMOUSE_ZOOM;

    // move perturb or camera
    mjrRect r = state->rect[0];
    if(mj_sim->pert.active)
      mjv_movePerturb(mj_sim->model, mj_sim->data, action, state->dx / r.height, -state->dy / r.height, &mj_sim->scene,
                      &mj_sim->pert);
    else
      mjv_moveCamera(mj_sim->model, action, state->dx / r.height, -state->dy / r.height, &mj_sim->scene,
                     &mj_sim->camera);
    return;
  }
}

void uiRender(mjuiState * state)
{
  auto mj_sim = static_cast<MjSimImpl *>(state->userdata);
  mj_sim->render();
}

/*******************************************************************************
 * Mujoco utility functions
 ******************************************************************************/

bool mujoco_init(MjSimImpl * mj_sim,
                 const std::map<std::string, std::string> & mujocoObjects,
                 const std::map<std::string, std::string> & mcrtcObjects)
{
#if mjVERSION_HEADER <= 200
  // Initialize MuJoCo
  if(!mujoco_initialized)
  {
    // Activate MuJoCo
    const char * key_buf_ptr = getenv("MUJOCO_KEY_PATH");
    std::string key_buf = [&]() -> std::string
    {
      if(key_buf_ptr)
      {
        return key_buf_ptr;
      }
      return mc_mujoco::MUJOCO_KEY_PATH;
    }();
    mj_activate(key_buf.c_str());
    mujoco_initialized = true;
  }
#endif

  // Load the model;
  std::string model = merge_mujoco_models(mujocoObjects, mcrtcObjects, mj_sim->objects, mj_sim->robots);
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
#ifdef USE_UI_ADAPTER
  mj_sim->platform_ui_adapter.reset(new mujoco::GlfwAdapter());
  mj_sim->platform_ui_adapter->SetWindowTitle("mc_mujoco");
#else
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
#endif

  // initialize visualization data structures
  auto config = [&]() -> mc_rtc::Configuration
  {
    auto path = fmt::format("{}/mc_mujoco.yaml", USER_FOLDER);
    if(bfs::exists(path))
    {
      return {path};
    }
    return {};
  }();
  auto camera = config("camera", mc_rtc::Configuration{});
  mjv_defaultCamera(&mj_sim->camera);
  int ctype = static_cast<int>(camera("type", 0));
  int cid = static_cast<int>(camera("fixedcamid", -1));
  int bid = static_cast<int>(camera("trackbodyid", -1));
  if(ctype == mjCAMERA_FIXED && cid < mj_sim->model->ncam)
  {
    mj_sim->camera.type = ctype;
    mj_sim->camera.fixedcamid = cid;
  }
  if(ctype == mjCAMERA_TRACKING && bid < mj_sim->model->nbody)
  {
    mj_sim->camera.type = ctype;
    mj_sim->camera.trackbodyid = bid;
  }
  auto lookat = camera("lookat", std::array<double, 3>{0.0, 0.0, 0.75});
  mj_sim->camera.lookat[0] = static_cast<float>(lookat[0]);
  mj_sim->camera.lookat[1] = static_cast<float>(lookat[1]);
  mj_sim->camera.lookat[2] = static_cast<float>(lookat[2]);
  mj_sim->camera.distance = static_cast<float>(camera("distance", 6.0));
  mj_sim->camera.azimuth = static_cast<float>(camera("azimuth", -150.0));
  mj_sim->camera.elevation = static_cast<float>(camera("elevation", -20.0));
  mjv_defaultOption(&mj_sim->options);
  auto visualize = config("visualize", mc_rtc::Configuration{});
  mj_sim->options.geomgroup[0] = mj_sim->config.visualize_collisions.value_or(visualize("collisions", false));
  mj_sim->options.geomgroup[1] = mj_sim->config.visualize_visual.value_or(visualize("visuals", true));
  mj_sim->options.flags[mjVIS_CONTACTPOINT] = visualize("contact-points", false);
  mj_sim->options.flags[mjVIS_CONTACTFORCE] = visualize("contact-forces", false);
  mj_sim->options.flags[mjVIS_CONTACTSPLIT] = visualize("contact-split", false);
  mjv_defaultScene(&mj_sim->scene);
#ifndef USE_UI_ADAPTER
  mjr_defaultContext(&mj_sim->context);
#endif

  // create scene and context
  mjv_makeScene(mj_sim->model, &mj_sim->scene, 2000);
#ifdef USE_UI_ADAPTER
  mjr_makeContext(mj_sim->model, &mj_sim->platform_ui_adapter->mjr_context(), mjFONTSCALE_150);
#else
  mjr_makeContext(mj_sim->model, &mj_sim->context, mjFONTSCALE_150);
#endif

  // install GLFW event callback
#ifdef USE_UI_ADAPTER
  auto & uistate = mj_sim->platform_ui_adapter->state();
#else
  auto & uistate = mj_sim->uistate;
#endif
  uistate.userdata = static_cast<void *>(mj_sim);
#ifndef USE_UI_ADAPTER
#  if mjVERSION_HEADER >= 230
  uiSetCallback(mj_sim->window, &mj_sim->uistate, uiEvent, uiLayout, uiRender, nullptr);
#  else
  uiSetCallback(mj_sim->window, &mj_sim->uistate, uiEvent, uiLayout);
#  endif
#else
  mj_sim->platform_ui_adapter->SetEventCallback(uiEvent);
  mj_sim->platform_ui_adapter->SetLayoutCallback(uiLayout);
#endif
  uiLayout(&uistate);

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
  io.IniFilename = "/tmp/imgui.ini";

  ImGui::StyleColorsLight();
  auto & style = ImGui::GetStyle();
  style.FrameRounding = 6.0f;
  auto & bgColor = style.Colors[ImGuiCol_WindowBg];
  bgColor.w = 0.5f;
#ifdef USE_UI_ADAPTER
  auto & glfw_adapter = *dynamic_cast<mujoco::GlfwAdapter *>(mj_sim->platform_ui_adapter.get());
  ImGui_ImplGlfw_InitForOpenGL(glfw_adapter.window_, true);
#else
  ImGui_ImplGlfw_InitForOpenGL(mj_sim->window, true);
#endif
  ImGui_ImplOpenGL3_Init(glsl_version);
}

void mujoco_cleanup(MjSimImpl * mj_sim)
{
  if(mj_sim->config.with_visualization)
  {
    // Close the window
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

#ifndef USE_UI_ADAPTER
    glfwDestroyWindow(mj_sim->window);
#endif

    // free visualization storage
    mjv_freeScene(&mj_sim->scene);
#ifndef USE_UI_ADAPTER
    mjr_freeContext(&mj_sim->context);
#endif
  }

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
