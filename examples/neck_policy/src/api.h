#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define SampleNeckPolicy_DLLIMPORT __declspec(dllimport)
#  define SampleNeckPolicy_DLLEXPORT __declspec(dllexport)
#else
#  define SampleNeckPolicy_DLLIMPORT __attribute__((visibility("default")))
#  define SampleNeckPolicy_DLLEXPORT __attribute__((visibility("default")))
#endif

#ifdef SampleNeckPolicy_EXPORTS
#  define SampleNeckPolicy_DLLAPI SampleNeckPolicy_DLLEXPORT
#else
#  define SampleNeckPolicy_DLLAPI SampleNeckPolicy_DLLIMPORT
#endif
