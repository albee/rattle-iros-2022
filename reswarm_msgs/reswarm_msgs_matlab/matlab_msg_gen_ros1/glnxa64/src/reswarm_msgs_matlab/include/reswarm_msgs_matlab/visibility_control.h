#ifndef RESWARM_MSGS_MATLAB__VISIBILITY_CONTROL_H_
#define RESWARM_MSGS_MATLAB__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RESWARM_MSGS_MATLAB_EXPORT __attribute__ ((dllexport))
    #define RESWARM_MSGS_MATLAB_IMPORT __attribute__ ((dllimport))
  #else
    #define RESWARM_MSGS_MATLAB_EXPORT __declspec(dllexport)
    #define RESWARM_MSGS_MATLAB_IMPORT __declspec(dllimport)
  #endif
  #ifdef RESWARM_MSGS_MATLAB_BUILDING_LIBRARY
    #define RESWARM_MSGS_MATLAB_PUBLIC RESWARM_MSGS_MATLAB_EXPORT
  #else
    #define RESWARM_MSGS_MATLAB_PUBLIC RESWARM_MSGS_MATLAB_IMPORT
  #endif
  #define RESWARM_MSGS_MATLAB_PUBLIC_TYPE RESWARM_MSGS_MATLAB_PUBLIC
  #define RESWARM_MSGS_MATLAB_LOCAL
#else
  #define RESWARM_MSGS_MATLAB_EXPORT __attribute__ ((visibility("default")))
  #define RESWARM_MSGS_MATLAB_IMPORT
  #if __GNUC__ >= 4
    #define RESWARM_MSGS_MATLAB_PUBLIC __attribute__ ((visibility("default")))
    #define RESWARM_MSGS_MATLAB_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RESWARM_MSGS_MATLAB_PUBLIC
    #define RESWARM_MSGS_MATLAB_LOCAL
  #endif
  #define RESWARM_MSGS_MATLAB_PUBLIC_TYPE
#endif
#endif  // RESWARM_MSGS_MATLAB__VISIBILITY_CONTROL_H_
// Generated 11-Aug-2021 11:29:38
// Copyright 2019-2020 The MathWorks, Inc.
