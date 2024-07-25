LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := libstream_soundtrigger
LOCAL_MODULE_OWNER := qti
LOCAL_VENDOR_MODULE := true

LOCAL_CPPFLAGS  += -fexceptions -frtti

LOCAL_SRC_FILES := \
    src/StreamSoundTrigger.cpp \
	src/SoundTriggerEngine.cpp \
    src/SoundTriggerEngineCapi.cpp \
    src/SoundTriggerEngineGsl.cpp \
    src/PalRingBuffer.cpp \

LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/inc

LOCAL_C_INCLUDES += $(TOP)/system/media/audio_route/include
LOCAL_C_INCLUDES += $(TOP)/vendor/qcom/proprietary/args/gsl/api/

LOCAL_HEADER_LIBRARIES := \
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
	libcapiv2_headers


LOCAL_SHARED_LIBRARIES := \
    liblog \
    libcutils \
    liblx-osal \
    libar-pal \
    libexpat \
	libvui_intf

ifeq ($(USE_PAL_STATIC_LINKING_MODULES),true)
    include $(BUILD_STATIC_LIBRARY)
else
    include $(BUILD_SHARED_LIBRARY)
endif
