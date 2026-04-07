include(FetchContent)

FetchContent_Declare(
  jvrc_mj_description
  GIT_REPOSITORY https://github.com/isri-aist/jvrc_mj_description.git
  GIT_TAG v1.0.0)
FetchContent_MakeAvailable(jvrc_mj_description)

FetchContent_Declare(
  env_mj_description
  GIT_REPOSITORY https://github.com/isri-aist/env_mj_description.git
  GIT_TAG v1.0.0)
FetchContent_MakeAvailable(env_mj_description)
