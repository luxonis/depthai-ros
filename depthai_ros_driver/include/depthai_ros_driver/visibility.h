// Copyright (c) [2022] [Adam Serafin]

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#ifndef DEPTHAI_ROS_DRIVER__VISIBILITY_H_
#define DEPTHAI_ROS_DRIVER__VISIBILITY_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define DEPTHAI_ROS_DRIVER_EXPORT __attribute__((dllexport))
#define DEPTHAI_ROS_DRIVER_IMPORT __attribute__((dllimport))
#else
#define DEPTHAI_ROS_DRIVER_EXPORT __declspec(dllexport)
#define DEPTHAI_ROS_DRIVER_IMPORT __declspec(dllimport)
#endif

#ifdef DEPTHAI_ROS_DRIVER_DLL
#define DEPTHAI_ROS_DRIVER_PUBLIC DEPTHAI_ROS_DRIVER_EXPORT
#else
#define DEPTHAI_ROS_DRIVER_PUBLIC DEPTHAI_ROS_DRIVER_IMPORT
#endif

#define DEPTHAI_ROS_DRIVER_PUBLIC_TYPE DEPTHAI_ROS_DRIVER_PUBLIC

#define DEPTHAI_ROS_DRIVER_LOCAL

#else

#define DEPTHAI_ROS_DRIVER_EXPORT __attribute__((visibility("default")))
#define DEPTHAI_ROS_DRIVER_IMPORT

#if __GNUC__ >= 4
#define DEPTHAI_ROS_DRIVER_PUBLIC __attribute__((visibility("default")))
#define DEPTHAI_ROS_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define DEPTHAI_ROS_DRIVER_PUBLIC
#define DEPTHAI_ROS_DRIVER_LOCAL
#endif

#define DEPTHAI_ROS_DRIVER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DEPTHAI_ROS_DRIVER__VISIBILITY_H_
