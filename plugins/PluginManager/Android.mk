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

LOCAL_HEADER_LIBRARIES := \
    libarpal_headers \
    libarpal_internalheaders

#used for static compilation
ifeq ($(USE_PAL_STATIC_LINKING_MODULES),true)

    LOCAL_C_INCLUDES += $(TOP)/system/media/audio_route/include
    LOCAL_C_INCLUDES += $(TOP)/vendor/qcom/proprietary/args/gsl/api/

    LOCAL_HEADER_LIBRARIES += \
        libarpal_headers \
        libspf-headers \
        libvui_dmgr_headers \
        libarvui_intf_headers \
        libaudiofeaturestats_headers \
        liblisten_headers \
        libacdb_headers \
        libaudioroute \
        libarpal_internalheaders \
        libarmemlog_headers \
        libstream_acd_headers \
        libstream_acdb_headers \
        libstream_common_headers \
        libstream_commonproxy_headers \
        libstream_compress_headers \
        libstream_contextproxy_headers \
        libstream_haptics_headers \
        libstream_incall_headers \
        libstream_nontunnel_headers \
        libstream_pcm_headers \
        libstream_sensorpcmdata_headers \
        libstream_sensorrenderer_headers \
        libstream_soundtrigger_headers \
        libstream_ultrasound_headers \
        libstream_asr_headers

    LOCAL_STATIC_LIBRARIES := \
        libstream_acd \
        libstream_acdb \
        libstream_common \
        libstream_commonproxy \
        libstream_compress \
        libstream_contextproxy \
        libstream_haptics \
        libstream_incall \
        libstream_nontunnel \
        libstream_pcm \
        libstream_sensorpcmdata \
        libstream_sensorrenderer \
        libstream_soundtrigger \
        libstream_ultrasound \
        libstream_asr
endif

include $(BUILD_STATIC_LIBRARY)
