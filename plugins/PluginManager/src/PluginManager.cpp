/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: PluginManager"

#include "PluginManager.h"
#include <dlfcn.h>
#include <iostream>
#include <fstream>
#include <map>
#include "PalCommon.h"
#include <cutils/properties.h>

std::shared_ptr<PluginManager> PluginManager::pm = nullptr;
std::mutex PluginManager::mPluginManagerMutex;
std::vector<pm_item_t> PluginManager::registeredStreams = {};
std::vector<pm_item_t> PluginManager::registeredSessions = {};
std::vector<pm_item_t> PluginManager::registeredDevices = {};

#define XML_PATH_MAX_LENGTH 100
#define PLUGIN_MANAGER_FILENAME "plugin_manager.xml"
#define VENDOR_CONFIG_PATH_MAX_LENGTH 128
char pimngr_xml_file[XML_PATH_MAX_LENGTH] = {0};
char pimngr_vendor_config_path[VENDOR_CONFIG_PATH_MAX_LENGTH] = {0};

static const std::map<std::string, pal_plugin_manager_t> PmNameToType
{
    { "stream",  PAL_PLUGIN_MANAGER_STREAM},
    { "session", PAL_PLUGIN_MANAGER_SESSION},
    { "device",  PAL_PLUGIN_MANAGER_DEVICE},
    { "config",  PAL_PLUGIN_MANAGER_CONFIG},
    { "effects", PAL_PLUGIN_MANAGER_EFFECTS}
};

struct xml_userdata {
    char data_buf[1024];
    size_t offs;
    XML_Parser parser;
};

PluginManager::PluginManager() {
        getVendorConfigPath(pimngr_vendor_config_path, sizeof(pimngr_vendor_config_path));
        snprintf(pimngr_xml_file, sizeof(pimngr_xml_file),
            "%s/%s", pimngr_vendor_config_path, PLUGIN_MANAGER_FILENAME);
        XmlParser(pimngr_xml_file);
}

PluginManager::~PluginManager() {
   deinitStreamPlugins();
}

void PluginManager::deinitStreamPlugins(){
    for (const auto& item : registeredStreams) {
        dlclose(item.handle);
    }
}

int32_t PluginManager::getRegisteredPluginList(pal_plugin_manager_t type, std::vector<pm_item_t> **pluginList){
    int32_t status = 0;
    switch (type) {
        case PAL_PLUGIN_MANAGER_STREAM:
            *pluginList = &registeredStreams;
            break;
        case PAL_PLUGIN_MANAGER_SESSION:
            *pluginList = &registeredSessions;
            break;
        case PAL_PLUGIN_MANAGER_DEVICE:
            *pluginList = &registeredDevices;
            break;
        default:
            PAL_ERR(LOG_TAG, "unsupported Plugin type %d", type);
            status = -EINVAL;
            break;
    }
    return status;
}

int32_t PluginManager::registeredPlugin(pm_item_t item, pal_plugin_manager_t type){
    void* handle = NULL;
    int32_t status = 0;
    bool_t foundLib = false;
    std::vector<pm_item_t> *pluginList = nullptr;

    status = getRegisteredPluginList(type, &pluginList);
    if (status || !pluginList) {
        PAL_ERR(LOG_TAG, "failed to get register plugin list");
        goto exit;
    }
        for (auto& reg : *pluginList) {
            if (item.libName == reg.libName) {
                PAL_DBG(LOG_TAG, "%s lib already mapped adding key %s", item.libName.c_str(), item.keyNames[0].c_str());
                reg.keyNames.push_back(item.keyNames[0]);
                foundLib = true;
                break;
            }
        }
        if (!foundLib){
            PAL_ERR(LOG_TAG, "%s registered", item.libName.c_str());
            pluginList->push_back(item);
        }

    exit:
    return status;
}

int32_t PluginManager::openPlugin(pal_plugin_manager_t type, std::string keyName, void* &plugin){
    int32_t status = 0;
    std::vector<pm_item_t> *pluginList = nullptr;

    PAL_DBG(LOG_TAG, "Enter");
    mPluginManagerMutex.lock();
    status = getRegisteredPluginList(type, &pluginList);
    if (status || !pluginList) {
        PAL_ERR(LOG_TAG, "failed to get register plugin list");
        goto exit;
    }
    for (auto& item : *pluginList) {
        for (auto& key : item.keyNames) {
            if (key == keyName) {
                /*if lib has not been opened yet open it*/
                if(!item.refCount){
                    try {
                        PAL_DBG(LOG_TAG, "Opening lib %s", item.libName.c_str());
                        item.handle = dlopen(item.libName.c_str(), RTLD_LAZY);
                        if (item.handle) {
                            item.plugin = (dlsym(item.handle, item.entryFunction.c_str()));
                            if (!item.plugin) {
                                PAL_ERR(LOG_TAG, "unable to load %s object pointer function",
                                key.c_str());
                                dlclose(item.handle);
                                status = -EINVAL;
                                goto exit;
                            } 
                        } else {
                            PAL_ERR(LOG_TAG, "dlopen failed for lib %s", item.libName.c_str());
                        }
                    } catch (const std::exception& e) {
                        PAL_ERR(LOG_TAG, "Dll loading of %s failed", key.c_str());
                        throw std::runtime_error(e.what());
                    }
                }
                item.refCount++;
                plugin = item.plugin;
                PAL_DBG(LOG_TAG, "found plugin object for  %s", keyName.c_str());
                break;
            }
            if (plugin)
                break;
        }
    }
    if (!plugin) {
        PAL_ERR(LOG_TAG, "cannot find a registered plugin for stream type %s",
                keyName.c_str());
        status = -EINVAL;
    }
    exit:
    PAL_DBG(LOG_TAG, "exit status: %d", status);
    mPluginManagerMutex.unlock();
    return status;
}

int32_t  PluginManager::closePlugin(pal_plugin_manager_t type, std::string keyName) {
    int32_t status = 0;
    bool_t found = false;
    std::vector<pm_item_t> *pluginList = nullptr;

    PAL_DBG(LOG_TAG, "Enter");
    mPluginManagerMutex.lock();
    status = getRegisteredPluginList(type, &pluginList);

    if (status || !pluginList) {
        PAL_ERR(LOG_TAG, "failed to get register plugin list");
        goto exit;
    }
    for (auto& item : *pluginList) {
        for (auto& key : item.keyNames) {
            if (key == keyName) {
                    item.refCount--;
                    if (!item.refCount){
                        dlclose(item.handle);
                        item.handle = nullptr;
                        item.plugin = nullptr;
                    }
                    found = true;
                    break;
            }
            if (found)
                break;
        }
    }
    if (!found) {
        PAL_ERR(LOG_TAG, "could not find a registered plugin for key %s cannot close", keyName.c_str())
        status = -EINVAL;
    }
    exit:
    mPluginManagerMutex.unlock();

    return status;
}


// Callback function for handling start elements
void PluginManager::startElement(void* userData, const char* name, const char** attrs) {
    if (strcmp(name, "stream") == 0 || strcmp(name, "session") == 0 || strcmp(name, "device") == 0) {
        pm_item_t item;
        // std::string stream;
        PAL_DBG(LOG_TAG, "enter");

        // Extract attributes
        for (int i = 0; attrs[i]; i += 2) {
            if (strcmp(attrs[i], "name") == 0){
                item.keyNames.push_back(attrs[i + 1]);
            }
            else if (strcmp(attrs[i], "lib") == 0)
                item.libName = attrs[i + 1];
            else if (strcmp(attrs[i], "entryFunction") == 0)
                item.entryFunction = attrs[i + 1];
        }
        if (item.keyNames[0].length() && item.libName.length() && item.entryFunction.length())
            registeredPlugin(item, PmNameToType.at(name));
        else
            PAL_ERR(LOG_TAG, "invalid parameters during parsing plugin manager xml");
    }
}

/* Function to get audio vendor configs path */
void PluginManager::getVendorConfigPath (char* config_file_path, int path_size)
{
   char vendor_sku[PROPERTY_VALUE_MAX] = {'\0'};
   if (property_get("ro.boot.product.vendor.sku", vendor_sku, "") <= 0) {
#if defined(FEATURE_IPQ_OPENWRT) || defined(LINUX_ENABLED)
       /* Audio configs are stored in /etc */
       snprintf(config_file_path, path_size, "%s", "/etc");
#else
       /* Audio configs are stored in /vendor/etc */
       snprintf(config_file_path, path_size, "%s", "/vendor/etc");
#endif
    } else {
       /* Audio configs are stored in /vendor/etc/audio/sku_${vendor_sku} */
       snprintf(config_file_path, path_size,
                       "%s%s", "/vendor/etc/audio/sku_", vendor_sku);
    }
}

void PluginManager::data_handler(void *userdata, const XML_Char *s, int len)
{
   struct xml_userdata *data = (struct xml_userdata *)userdata;

   if (len + data->offs >= sizeof(data->data_buf) ) {
       data->offs += len;
       /* string length overflow, return */
       return;
   } else {
       memcpy(data->data_buf + data->offs, s, len);
       data->offs += len;
   }
}

int PluginManager::XmlParser(std::string xmlFile)
{
    XML_Parser parser;
    FILE *file = NULL;
    int ret = 0;
    int bytes_read;
    void *buf = NULL;
    struct xml_userdata data;
    memset(&data, 0, sizeof(data));

    PAL_INFO(LOG_TAG, "XML parsing started - file name %s", xmlFile.c_str());
    file = fopen(xmlFile.c_str(), "r");
    if(!file) {
        ret = -ENOENT;
        PAL_ERR(LOG_TAG, "Failed to open xml file name %s ret %d", xmlFile.c_str(), ret);
        goto done;
    }

    parser = XML_ParserCreate(NULL);
    if (!parser) {
        ret = -EINVAL;
        PAL_ERR(LOG_TAG, "Failed to create XML ret %d", ret);
        goto closeFile;
    }

    data.parser = parser;
    XML_SetUserData(parser, &data);
    XML_SetElementHandler(parser, startElement, nullptr);
    XML_SetCharacterDataHandler(parser, data_handler);

    while (1) {
        buf = XML_GetBuffer(parser, 1024);
        if(buf == NULL) {
            ret = -EINVAL;
            PAL_ERR(LOG_TAG, "XML_Getbuffer failed ret %d", ret);
            goto freeParser;
        }

        bytes_read = fread(buf, 1, 1024, file);
        if(bytes_read < 0) {
            ret = -EINVAL;
            PAL_ERR(LOG_TAG, "fread failed ret %d", ret);
            goto freeParser;
        }

        if(XML_ParseBuffer(parser, bytes_read, bytes_read == 0) == XML_STATUS_ERROR) {
            ret = -EINVAL;
            PAL_ERR(LOG_TAG, "XML ParseBuffer failed for %s file ret %d", xmlFile.c_str(), ret);
            goto freeParser;
        }
        if (bytes_read == 0)
            break;
    }

freeParser:
    XML_ParserFree(parser);
closeFile:
    fclose(file);
done:
    return ret;
}

/*public APIs*/

std::shared_ptr<PluginManager> PluginManager::getInstance()
{
    if (!pm) {
        std::lock_guard<std::mutex> lock(PluginManager::mPluginManagerMutex);
        std::shared_ptr<PluginManager> sp(new PluginManager());
        pm = sp;
    }
    return pm;
}

