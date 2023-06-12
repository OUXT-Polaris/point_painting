// Copyright (c) 2022 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POINTPAINTING_FUSIONCOMPONENT__VISIBILITY_CONTROL_H_
#define POINTPAINTING_FUSIONCOMPONENT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define POINTPAINTING_FUSIONCOMPONENT_EXPORT __attribute__((dllexport))
#define POINTPAINTING_FUSIONCOMPONENT_IMPORT __attribute__((dllimport))
#else
#define POINTPAINTING_FUSIONCOMPONENT_EXPORT __declspec(dllexport)
#define POINTPAINTING_FUSIONCOMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef POINTPAINTING_FUSIONCOMPONENT_BUILDING_LIBRARY
#define POINTPAINTING_FUSIONCOMPONENT_PUBLIC POINTPAINTING_FUSIONCOMPONENT_EXPORT
#else
#define POINTPAINTING_FUSIONCOMPONENT_PUBLIC POINTPAINTING_FUSIONCOMPONENT_IMPORT
#endif
#define POINTPAINTING_FUSIONCOMPONENT_PUBLIC_TYPE POINTPAINTING_FUSIONCOMPONENT_PUBLIC
#define POINTPAINTING_FUSIONCOMPONENT_LOCAL
#else
#define POINTPAINTING_FUSIONCOMPONENT_EXPORT __attribute__((visibility("default")))
#define POINTPAINTING_FUSIONCOMPONENT_IMPORT
#if __GNUC__ >= 4
#define POINTPAINTING_FUSIONCOMPONENT_PUBLIC __attribute__((visibility("default")))
#define POINTPAINTING_FUSIONCOMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define POINTPAINTING_FUSIONCOMPONENT_PUBLIC
#define POINTPAINTING_FUSIONCOMPONENT_LOCAL
#endif
#define POINTPAINTING_FUSIONCOMPONENT_PUBLIC_TYPE
#endif

#endif  // POINTPAINTING_FUSIONCOMPONENT__VISIBILITY_CONTROL_H_
