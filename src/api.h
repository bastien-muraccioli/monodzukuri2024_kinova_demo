#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define MonodzukuriKinovaDemo_DLLIMPORT __declspec(dllimport)
#  define MonodzukuriKinovaDemo_DLLEXPORT __declspec(dllexport)
#  define MonodzukuriKinovaDemo_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define MonodzukuriKinovaDemo_DLLIMPORT __attribute__((visibility("default")))
#    define MonodzukuriKinovaDemo_DLLEXPORT __attribute__((visibility("default")))
#    define MonodzukuriKinovaDemo_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define MonodzukuriKinovaDemo_DLLIMPORT
#    define MonodzukuriKinovaDemo_DLLEXPORT
#    define MonodzukuriKinovaDemo_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef MonodzukuriKinovaDemo_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define MonodzukuriKinovaDemo_DLLAPI
#  define MonodzukuriKinovaDemo_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef MonodzukuriKinovaDemo_EXPORTS
#    define MonodzukuriKinovaDemo_DLLAPI MonodzukuriKinovaDemo_DLLEXPORT
#  else
#    define MonodzukuriKinovaDemo_DLLAPI MonodzukuriKinovaDemo_DLLIMPORT
#  endif // MonodzukuriKinovaDemo_EXPORTS
#  define MonodzukuriKinovaDemo_LOCAL MonodzukuriKinovaDemo_DLLLOCAL
#endif // MonodzukuriKinovaDemo_STATIC
