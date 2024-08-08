LOCAL_PATH := $(call my-dir)

#-------------------------------------------
#            Build SESSION_COMPRESS LIB
#-------------------------------------------

include $(CLEAR_VARS)

LOCAL_MODULE_TAGS   := optional
LOCAL_MODULE        := libsession_compress
LOCAL_MODULE_OWNER  := qti
LOCAL_VENDOR_MODULE := true

LOCAL_CPPFLAGS += -fexceptions -frtti

LOCAL_SRC_FILES := \
    src/SessionAlsaCompress.cpp

LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/inc

LOCAL_C_INCLUDES += $(TOP)/system/media/audio_route/include
LOCAL_C_INCLUDES += $(TOP)/vendor/qcom/proprietary/args/gsl/api/

LOCAL_HEADER_LIBRARIES := \
    libarpal_headers \
    libspf-headers \
    libacdb_headers \
    liblisten_headers \
    libarosal_headers \
    libaudiofeaturestats_headers \
    libarvui_intf_headers \
    libarmemlog_headers \
    libarpal_internalheaders \
    libagm_headers \
    libsession_ar_headers

LOCAL_SHARED_LIBRARIES := \
    liblog \
    libcutils \
    liblx-osal \
    libar-pal \
    libexpat \
    libutilscallstack \
    libvui_intf \
    libarmemlog \
    libsession_ar

 ifeq ($(TARGET_USES_QTI_TINYCOMPRESS),true)
 LOCAL_SHARED_LIBRARIES += libqti-tinyalsa libqti-tinycompress
 else
 LOCAL_C_INCLUDES       += $(TOP)/external/tinycompress/include
 LOCAL_SHARED_LIBRARIES += libtinyalsa libtinycompress
 endif

ifeq ($(USE_PAL_STATIC_LINKING_MODULES),true)
    include $(BUILD_STATIC_LIBRARY)
else
    include $(BUILD_SHARED_LIBRARY)
endif
