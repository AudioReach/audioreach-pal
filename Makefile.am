pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = pal.pc
EXTRA_DIST = $(pkgconfig_DATA)


if BUILDSYSTEM_OPENWRT

AM_CPPFLAGS := -I ./stream/inc
AM_CPPFLAGS += -I ./device/inc
AM_CPPFLAGS += -I ./session/inc
AM_CPPFLAGS += -I ./session/SessionAR/inc
AM_CPPFLAGS += -I ./context_manager/inc
AM_CPPFLAGS += -I ./plugins/PluginManager/inc
AM_CPPFLAGS += -I ./resource_manager/inc
AM_CPPFLAGS += -I ./utils/inc
AM_CPPFLAGS += -I ./inc
AM_CPPFLAGS += -I ./

AM_CPPFLAGS += $(SPF_CFLAGS)/
AM_CPPFLAGS += $(ACDBDATA_CFLAGS)
AM_CPPFLAGS += -I $(TOPDIR)/src/audio/mm-audio-headers/capiv2_api/
AM_CPPFLAGS += -I $(TOPDIR)/system/media/audio_route/include/


pal_sources = ./stream/src/Stream.cpp \
              ./device/src/Device.cpp \
              ./session/src/Session.cpp \
              ./context_manager/src/ContextManager.cpp \
              ./resource_manager/src/SndCardMonitor.cpp \
              ./resource_manager/src/ResourceManager.cpp \
              ./Pal.cpp \
              ./PalAR.cpp \
              ./utils/src/SoundTriggerPlatformInfo.cpp \
              ./utils/src/SignalHandler.cpp \
              ./utils/src/MetadataParser.cpp \
              ./utils/src/PerfLock.cpp \
              ./utils/src/VoiceUIPlatformInfo.cpp \
              ./utils/src/AudioHapticsInterface.cpp \
              ./utils/src/ACDPlatformInfo.cpp \
              ./utils/src/ASRPlatformInfo.cpp \
              ./utils/src/STUtils.cpp
else
AM_CPPFLAGS := -I $(top_srcdir)/stream/inc
AM_CPPFLAGS += -I $(top_srcdir)/device/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/SessionAR/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/StreamCommon/StreamUltrasSound/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/StreamCommon/inc/
AM_CPPFLAGS += -I $(top_srcdir)/plugins/PluginManager/inc
AM_CPPFLAGS += -I $(top_srcdir)/resource_manager/inc
AM_CPPFLAGS += -I $(top_srcdir)/utils/inc
AM_CPPFLAGS += -I $(top_srcdir)/context_manager/inc
AM_CPPFLAGS += -I $(top_srcdir)/inc
AM_CPPFLAGS += -I $(top_srcdir)/plugins/codecs
AM_CPPFLAGS += -I $(top_srcdir)/
AM_CPPFLAGS += -I $(top_srcdir)/plugins/vui_interface/api/vui-interface
AM_CPPFLAGS += @AGM_CFLAGS@
AM_CPPFLAGS += @PALHEADERS_CFLAGS@
AM_CPPFLAGS += @SPF_CFLAGS@
AM_CPPFLAGS += @KVH2XML_CFLAGS@
AM_CPPFLAGS += @AROUTE_CFLAGS@
AM_CPPFLAGS += -Wno-sign-compare -Wno-unused-result -fPIC


pal_sources = $(top_srcdir)/stream/src/Stream.cpp \
              $(top_srcdir)/device/src/Device.cpp \
              $(top_srcdir)/session/src/Session.cpp \
              $(top_srcdir)/context_manager/src/ContextManager.cpp \
              $(top_srcdir)/resource_manager/src/SndCardMonitor.cpp \
              $(top_srcdir)/resource_manager/src/ResourceManager.cpp \
              $(top_srcdir)/Pal.cpp \
              $(top_srcdir)/utils/src/SoundTriggerPlatformInfo.cpp \
              $(top_srcdir)/utils/src/MetadataParser.cpp \
              $(top_srcdir)/utils/src/VoiceUIPlatformInfo.cpp \
              $(top_srcdir)/utils/src/AudioHapticsInterface.cpp \
              $(top_srcdir)/utils/src/ACDPlatformInfo.cpp \
              $(top_srcdir)/utils/src/ASRPlatformInfo.cpp \
              $(top_srcdir)/utils/src/STUtils.cpp \
              $(top_srcdir)/utils/src/BTUtils.cpp

endif
AM_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED
AM_CPPFLAGS += -DCONFIG_GSL

library_includedir = $(includedir)/pal
lib_LTLIBRARIES     = libpal.la
libpal_la_SOURCES   = $(pal_sources)
if BUILDSYSTEM_OPENWRT
libpal_la_LIBADD    = $(GLIB_LIBS) -ltinyalsa -laudioroute -lar-osal -lar-spfhdrs -lexpat -ltinycompress -lagmclientwrapper
libpal_la_LIBADD   += -lar-gsl -lacdbdata
libpal_la_CPPFLAGS  = -DFEATURE_IPQ_OPENWRT
else
libpal_la_LIBADD    = $(GLIB_LIBS) -ltinyalsa -laudioroute -lar-osal -lar-spfhdrs -lexpat -ltinycompress
libpal_la_CPPFLAGS  =  -DPAL_USE_SYSLOG -DLINUX_ENABLED -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif
if IS_SDXLEMUR
libpal_la_LIBADD    += -lagmclientwrapper
endif
if IS_SA525M
libpal_la_LIBADD    += -lagmclientwrapper
endif
libpal_la_CPPFLAGS += $(AM_CPPFLAGS)
libpal_la_CPPFLAGS += -std=c++14
libpal_la_LDFLAGS   = -shared -avoid-version
libpal_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h
libpal_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"

if USE_SYSLOG
libpal_la_CPPFLAGS += -DPAL_USE_SYSLOG
endif

if USE_G_STR_FUNC
libpal_la_CPPFLAGS +=-Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
endif

if BUILDSYSTEM_OPENWRT
libpal_la_LIBADD += -lglib-2.0 -llog
endif
if COMPILE_COMPRESS
libpal_la_CPPFLAGS += -DSND_COMPRESS_DEC_HDR
endif
libpal_la_LIBADD   += -L${abs_top_builddir}/plugins/vui_interface/.libs
libpal_la_LIBADD   += ${abs_top_builddir}/plugins/PluginManager/.libs/libplugin_manager.a

if BUILDSYSTEM_QLI

libpal_la_list      = ./configs/qcom/IoT/qcm6490/mixer_paths_qcm6490_idp.xml \
                      ./configs/qcom/IoT/qcm6490/resourcemanager_qcm6490_idp.xml \
                      ./configs/qcom/IoT/qcm6490/mixer_paths_qcm6490_rb3.xml \
                      ./configs/qcom/IoT/qcm6490/resourcemanager_qcm6490_rb3.xml \
                      ./configs/qcom/IoT/qcm6490/mixer_paths_qcm6490_rb3_vision.xml \
                      ./configs/qcom/IoT/qcm6490/resourcemanager_qcm6490_rb3_vision.xml \
                      ./configs/qcom/IoT/qcm6490/mixer_paths_qcm6490_rb3_vc.xml \
                      ./configs/qcom/IoT/qcm6490/resourcemanager_qcm6490_rb3_vc.xml \
                      ./configs/qcom/IoT/qcm6490/usecaseKvManager.xml \
                      ./configs/qcom/IoT/qcm6490/mixer_paths_qcm6490_rb3_ia.xml \
                      ./configs/qcom/IoT/qcm6490/resourcemanager_qcm6490_rb3_ia.xml \
                      ./configs/qcom/IoT/qcs8300/mixer_paths_qcs8300_ridesx.xml \
                      ./configs/qcom/IoT/qcs8300/resourcemanager_qcs8300_ridesx.xml \
                      ./configs/qcom/IoT/qcs9075/mixer_paths_qcs9075_rb8.xml \
                      ./configs/qcom/IoT/qcs9075/resourcemanager_qcs9075_rb8.xml \
                      ./configs/qcom/IoT/qcs9100/mixer_paths_qcs9100_ridesx.xml \
                      ./configs/qcom/IoT/qcs9100/resourcemanager_qcs9100_ridesx.xml \
                      ./configs/qcom/IoT/qcs6490/mixer_paths_QCS6490_RB3Gen2.xml \
                      ./configs/qcom/IoT/qcs6490/resourcemanager_QCS6490_RB3Gen2.xml \
                      ./configs/qcom/IoT/qcs9075/mixer_paths_LEMANS_EVK.xml \
                      ./configs/qcom/IoT/qcs9075/resourcemanager_LEMANS_EVK.xml

# install essential xml files under /etc
root_etcdir      = "/etc"
root_etc_SCRIPTS = $(libpal_la_list)
install-data-hook:
	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_qcs8300_ridesx.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_qcs8300_ridesx.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_qcs9100_ridesx.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_qcs9100_ridesx.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_qcs9075_rb8.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_qcs9075_rb8.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_qcm6490_idp.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_qcm6490_idp.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_qcm6490_rb3.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_qcm6490_rb3.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_qcm6490_rb3_vision.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_qcm6490_rb3_vision.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_qcm6490_rb3_vc.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_qcm6490_rb3_vc.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/usecaseKvManager.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_qcm6490_rb3_ia.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_qcm6490_rb3_ia.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_QCS6490_RB3Gen2.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_QCS6490_RB3Gen2.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_LEMANS_EVK.xml
	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_LEMANS_EVK.xml

if COMPILE_ADSPRPCD
adsprpc_sources = adsprpcd/adsprpcd.c

bin_PROGRAMS = audioadsprpcd
audioadsprpcd_SOURCES = $(adsprpc_sources)
audioadsprpcd_LDADD = -ldl
audioadsprpcd_la_CFLAGS = -fPIC
endif

endif

SUBDIRS =  plugins . session stream device plugins/configs
