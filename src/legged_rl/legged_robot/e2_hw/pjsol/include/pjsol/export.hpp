#pragma once

#if defined(_WIN32) || defined(__CYGWIN__)
#    ifdef PJSOL_BUILDING
#        define PJSOL_API __declspec(dllexport)
#    else
#        define PJSOL_API __declspec(dllimport)
#    endif
#else
#    if defined(__GNUC__) && __GNUC__ >= 4
#        define PJSOL_API __attribute__((visibility("default")))
#    else
#        define PJSOL_API
#    endif
#endif
