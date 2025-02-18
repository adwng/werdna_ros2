#ifndef WERDNA_ODOMETRY_BROADCASTER__VISIBILITY_CONTROL_H_
#define WERDNA_ODOMETRY_BROADCASTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define WERDNA_ODOMETRY_BROADCASTER_EXPORT __attribute__((dllexport))
#define WERDNA_ODOMETRY_BROADCASTER_IMPORT __attribute__((dllimport))
#else
#define WERDNA_ODOMETRY_BROADCASTER_EXPORT __declspec(dllexport)
#define WERDNA_ODOMETRY_BROADCASTER_IMPORT __declspec(dllimport)
#endif
#ifdef WERDNA_ODOMETRY_BROADCASTER_BUILDING_DLL
#define WERDNA_ODOMETRY_BROADCASTER_PUBLIC WERDNA_ODOMETRY_BROADCASTER_EXPORT
#else
#define WERDNA_ODOMETRY_BROADCASTER_PUBLIC WERDNA_ODOMETRY_BROADCASTER_IMPORT
#endif
#define WERDNA_ODOMETRY_BROADCASTER_PUBLIC_TYPE WERDNA_ODOMETRY_BROADCASTER_PUBLIC
#define WERDNA_ODOMETRY_BROADCASTER_LOCAL
#else
#define WERDNA_ODOMETRY_BROADCASTER_EXPORT __attribute__((visibility("default")))
#define WERDNA_ODOMETRY_BROADCASTER_IMPORT
#if __GNUC__ >= 4
#define WERDNA_ODOMETRY_BROADCASTER_PUBLIC __attribute__((visibility("default")))
#define WERDNA_ODOMETRY_BROADCASTER_LOCAL __attribute__((visibility("hidden")))
#else
#define WERDNA_ODOMETRY_BROADCASTER_PUBLIC
#define WERDNA_ODOMETRY_BROADCASTER_LOCAL
#endif
#define WERDNA_ODOMETRY_BROADCASTER_PUBLIC_TYPE
#endif

#endif  // WERDNA_ODOMETRY_BROADCASTER__VISIBILITY_CONTROL_H_