#ifndef PI3HAT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define PI3HAT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PI3HAT_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define PI3HAT_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define PI3HAT_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define PI3HAT_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef PI3HAT_HARDWARE_INTERFACE_BUILDING_DLL
#define PI3HAT_HARDWARE_INTERFACE_PUBLIC WERDNA_HARDWARE_INTERFACE_EXPORT
#else
#define PI3HAT_HARDWARE_INTERFACE_PUBLIC WERDNA_HARDWARE_INTERFACE_IMPORT
#endif
#define PI3HAT_HARDWARE_INTERFACE_PUBLIC_TYPE WERDNA_HARDWARE_INTERFACE_PUBLIC
#define PI3HAT_HARDWARE_INTERFACE_LOCAL
#else
#define PI3HAT_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define PI3HAT_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define PI3HAT_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define PI3HAT_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define PI3HAT_HARDWARE_INTERFACE_PUBLIC
#define PI3HAT_HARDWARE_INTERFACE_LOCAL
#endif
#define PI3HAT_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // DIFFDRIVE_ARDUINO__VISIBILITY_CONTROL_H_