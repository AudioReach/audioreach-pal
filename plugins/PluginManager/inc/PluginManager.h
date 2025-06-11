/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef PLUGIN_MANAGER_H
#define PLUGIN_MANAGER_H

#include <string>
#include <vector>
#include <mutex>
#include <expat.h>
#include "PluginManagerIntf.h"

typedef struct {
    void* handle;
    std::string libName;
    std::vector<std::string> keyNames;
    std::string entryFunction;
    void* plugin;
    uint32_t refCount;
} pm_item_t;

class PluginManager
{
    private:
        static std::mutex mPluginManagerMutex;
        static std::vector<pm_item_t> registeredStreams;
        static std::vector<pm_item_t> registeredSessions;
        static std::vector<pm_item_t> registeredDevices;
        static std::vector<pm_item_t> registeredControls;
        static std::vector<pm_item_t> registeredConfigs;
        void deinitPluginItems(std::vector<pm_item_t>& items);
        void deinitStreamPlugins();
        void deinitSessionPlugins();
        void deinitDevicePlugins();
        void deinitControlPlugins();
        void deinitConfigPlugins();
        PluginManager();
        static int32_t registeredPlugin(pm_item_t item, pal_plugin_manager_t type);
        static void startElement(void* userData, const char* name, const char** attrs);
        static int32_t getRegisteredPluginList(pal_plugin_manager_t type, std::vector<pm_item_t> **pluginList);
        void getVendorConfigPath (char* config_file_path, int path_size);
        static void data_handler(void *userdata, const XML_Char *s, int len);
        int XmlParser(std::string xmlFile);
        PluginManager(const PluginManager&) = delete;
        PluginManager& operator=(const PluginManager&) = delete;

    public:
        static std::shared_ptr<PluginManager> getInstance();
        int32_t getVersion(){return PLUGIN_MANAGER_VERSION;};
        int32_t openPlugin(pal_plugin_manager_t pluginType, std::string keyName, void* &plugin);
        int32_t closePlugin(pal_plugin_manager_t pluginType, std::string keyName);
        ~PluginManager();
};

#endif
