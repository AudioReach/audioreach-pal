/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef PLUGIN_UTILS_H
#define PLUGIN_UTILS_H

#include <string>

#ifndef PAL_PLUGIN_DIR
#define PAL_PLUGIN_DIR ""
#endif

static inline std::string getPalPluginPath(const std::string &libName)
{
    if (libName.find('/') != std::string::npos || PAL_PLUGIN_DIR[0] == '\0')
        return libName;

    std::string pluginPath = PAL_PLUGIN_DIR;
    if (pluginPath.back() != '/')
        pluginPath += "/";
    return pluginPath + libName;
}

#endif
