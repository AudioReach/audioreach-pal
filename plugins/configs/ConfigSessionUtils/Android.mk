LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := libsession_config_utils
LOCAL_MODULE_OWNER := qti
LOCAL_VENDOR_MODULE := true

LOCAL_CPPFLAGS += -fexceptions  -frtti

LOCAL_SRC_FILES := \
    src/ConfigSessionUtils.cpp

LOCAL_C_INCLUDES += $(TOP)/system/media/audio_route/include

LOCAL_HEADER_LIBRARIES := \
    libarpal_headers \
    libarpal_internalheaders \
    libarvui_intf_headers \
    liblisten_headers \
    plugin_manager_headers \
    libaudiofeaturestats_headers \
    libspf-headers \
    libacdb_headers \
    libagm_headers \
    libsession_ar_headers \
    libsession_pcm_headers \
    libsession_config_utils_headers

LOCAL_SHARED_LIBRARIES := \
    liblog \
    libcutils \
    liblx-osal \
    libexpat \
    libar-pal \
    libsession_pcm

ifeq ($(TARGET_USES_QTI_TINYCOMPRESS),true)
LOCAL_SHARED_LIBRARIES += libqti-tinyalsa
else
LOCAL_SHARED_LIBRARIES += liboss_tinyalsa
endif

include $(BUILD_STATIC_LIBRARY)