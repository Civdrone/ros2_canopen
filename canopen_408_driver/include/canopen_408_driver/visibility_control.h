#ifndef CANOPEN_408_DRIVER__VISIBILITY_CONTROL_H_
#define CANOPEN_408_DRIVER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CANOPEN_408_DRIVER_EXPORT __attribute__((dllexport))
#define CANOPEN_408_DRIVER_IMPORT __attribute__((dllimport))
#else
#define CANOPEN_408_DRIVER_EXPORT __declspec(dllexport)
#define CANOPEN_408_DRIVER_IMPORT __declspec(dllimport)
#endif
#ifdef CANOPEN_408_DRIVER_BUILDING_LIBRARY
#define CANOPEN_408_DRIVER_PUBLIC CANOPEN_408_DRIVER_EXPORT
#else
#define CANOPEN_408_DRIVER_PUBLIC CANOPEN_408_DRIVER_IMPORT
#endif
#define CANOPEN_408_DRIVER_PUBLIC_TYPE CANOPEN_408_DRIVER_PUBLIC
#define CANOPEN_408_DRIVER_LOCAL
#else
#define CANOPEN_408_DRIVER_EXPORT __attribute__((visibility("default")))
#define CANOPEN_408_DRIVER_IMPORT
#if __GNUC__ >= 4
#define CANOPEN_408_DRIVER_PUBLIC __attribute__((visibility("default")))
#define CANOPEN_408_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define CANOPEN_408_DRIVER_PUBLIC
#define CANOPEN_408_DRIVER_LOCAL
#endif
#define CANOPEN_408_DRIVER_PUBLIC_TYPE
#endif

#endif  // CANOPEN_408_DRIVER__VISIBILITY_CONTROL_H_
