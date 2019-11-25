//
// Created by ziqwang on 2019-10-28.
//

#ifndef TOPOLOCKCREATOR_CSHARPCOMMON_H
#define TOPOLOCKCREATOR_CSHARPCOMMON_H

#include "stdio.h"

// Windows build
#if defined (_WIN32)
#if defined (SAMPLELIBRARY_DLL_EXPORTS)
#define CSharp_LIBRARY_CPP_CLASS __declspec(dllexport)
#define CSharp_LIBRARY_CPP_FUNCTION __declspec(dllexport)
#define CSharp_LIBRARY_C_FUNCTION extern "C" __declspec(dllexport)
#else
#define CSharp_LIBRARY_CPP_CLASS __declspec(dllimport)
#define CSharp_LIBRARY_CPP_FUNCTION __declspec(dllimport)
#define CSharp_LIBRARY_C_FUNCTION extern "C" __declspec(dllimport)
#endif // CSharp_LIBRARY_DLL_EXPORTS
#endif // _WIN32

// Apple build
#if defined(__APPLE__)
#define CSharp_LIBRARY_CPP_CLASS __attribute__ ((visibility ("default")))
#define CSharp_LIBRARY_CPP_FUNCTION __attribute__ ((visibility ("default")))
#define CSharp_LIBRARY_C_FUNCTION extern "C" __attribute__ ((visibility ("default")))
#endif // __APPLE__




#endif //TOPOLOCKCREATOR_CSHARPCOMMON_H
