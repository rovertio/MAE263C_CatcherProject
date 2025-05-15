// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef JOINT_POSITIONS_NODE__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define JOINT_POSITIONS_NODE__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_joint_positions_node __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_joint_positions_node __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_joint_positions_node __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_joint_positions_node __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_joint_positions_node
    #define ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node ROSIDL_GENERATOR_C_EXPORT_joint_positions_node
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node ROSIDL_GENERATOR_C_IMPORT_joint_positions_node
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_joint_positions_node __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_joint_positions_node
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // JOINT_POSITIONS_NODE__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
