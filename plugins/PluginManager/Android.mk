LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libplugin_manager
LOCAL_MODULE_OWNER := qti
LOCAL_VENDOR_MODULE := true

LOCAL_CPPFLAGS += -fexceptions

ifeq ($(USE_PAL_STATIC_LINKING_MODULES),true)
    LOCAL_CFLAGS += -DUSE_STATIC_LINKING_MODULES
    LOCAL_SRC_FILES := \
        src/PluginManagerStatic.cpp
else
    LOCAL_SRC_FILES := \
        src/PluginManager.cpp
endif
LOCAL_SHARED_LIBRARIES := \
    liblog \
    libcutils \
    liblx-osal \
    libexpat

#need to add all lib here
ifeq ($(USE_PAL_STATIC_LINKING_MODULES),true)
    #LOCAL_STATIC_LIBRARIES +=
endif

LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/inc

LOCAL_HEADER_LIBRARIES := \
    libarpal_headers \
    libarpal_internalheaders

include $(BUILD_STATIC_LIBRARY)
