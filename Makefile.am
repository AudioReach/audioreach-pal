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
              ./utils/src/MemLogBuilder.cpp \
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
AM_CPPFLAGS += -I $(top_srcdir)/plugins/PluginManager/inc
AM_CPPFLAGS += -I $(top_srcdir)/resource_manager/inc
AM_CPPFLAGS += -I $(top_srcdir)/utils/inc
AM_CPPFLAGS += -I $(top_srcdir)/context_manager/inc
AM_CPPFLAGS += -I $(top_srcdir)/inc
AM_CPPFLAGS += -I $(top_srcdir)/
AM_CPPFLAGS += @AGM_CFLAGS@
AM_CPPFLAGS += @SPF_CFLAGS@
AM_CPPFLAGS += @MMHEADERS_CFLAGS@
AM_CPPFLAGS += -Wno-sign-compare -Wno-unused-result


pal_sources = $(top_srcdir)/stream/src/Stream.cpp \
              $(top_srcdir)/device/src/Device.cpp \
              $(top_srcdir)/session/src/Session.cpp \
              $(top_srcdir)/context_manager/src/ContextManager.cpp \
              $(top_srcdir)/resource_manager/src/SndCardMonitor.cpp \
              $(top_srcdir)/resource_manager/src/ResourceManager.cpp \
              $(top_srcdir)/Pal.cpp \
              $(top_srcdir)/PalAR.cpp \
              $(top_srcdir)/utils/src/SoundTriggerPlatformInfo.cpp \
              $(top_srcdir)/utils/src/SignalHandler.cpp \
              $(top_srcdir)/utils/src/MetadataParser.cpp \
              $(top_srcdir)/utils/src/MemLogBuilder.cpp \
              $(top_srcdir)/utils/src/PerfLock.cpp \
              $(top_srcdir)/utils/src/VoiceUIPlatformInfo.cpp \
              $(top_srcdir)/utils/src/AudioHapticsInterface.cpp \
              $(top_srcdir)/utils/src/ACDPlatformInfo.cpp \
	      $(top_srcdir)/utils/src/ASRPlatformInfo.cpp \
              $(top_srcdir)/utils/src/STUtils.cpp

endif
AM_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED
AM_CPPFLAGS += -DCONFIG_GSL

library_includedir = $(includedir)/pal
lib_LTLIBRARIES     = libpal.la
libpal_la_SOURCES   = $(pal_sources)
if BUILDSYSTEM_OPENWRT
libpal_la_LIBADD    = $(GLIB_LIBS) -ltinyalsa -laudioroute -lar-osal -lexpat -ltinycompress -lagmclientwrapper
libpal_la_LIBADD   += -lar-gsl -lacdbdata
else
libpal_la_LIBADD    = $(GLIB_LIBS) -ltinyalsa -laudioroute -lar_osal -lexpat -ltinycompress
endif
if IS_SDXLEMUR
libpal_la_LIBADD    += -lagmclientwrapper
endif
if IS_SA525M
libpal_la_LIBADD    += -lagmclientwrapper
endif
libpal_la_CPPFLAGS := $(AM_CPPFLAGS)
libpal_la_CPPFLAGS += -std=c++14
libpal_la_LDFLAGS   = -shared -avoid-version
libpal_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libpal_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libpal_la_list      = ./configs/$(MACHINE_ENABLED)/kvh2xml.xml \
                      ./configs/$(MACHINE_ENABLED)/mixer_paths_kona_mtp.xml \
                      ./configs/$(MACHINE_ENABLED)/resourcemanager_kona_mtp.xml \
                      ./configs/$(MACHINE_ENABLED)/usecaseKvManager.xml

if BUILDSYSTEM_OPENWRT
libpal_la_LIBADD += -lglib-2.0
endif
if COMPILE_COMPRESS
libpal_la_CPPFLAGS += -DSND_COMPRESS_DEC_HDR
endif
libpal_la_LIBADD   += -L./plugins/vui_interface/.libs -lvui_intf
libpal_la_LIBADD   += ./plugins/PluginManager/.libs/libplugin_manager.a

SUBDIRS = plugins stream session device

#lib_LTLIBRARIES     += libaudiocl.la
#libaudiocl_la_SOURCES   = $(acl_sources)
#libaudiocl_la_LIBADD    = @GLIB_LIBS@
#libaudiocl_la_CPPFLAGS := $(AM_CPPFLAGS)
#libaudiocl_la_LDFLAGS   = -shared -avoid-version -lcutils -llog
# install essential xml files under /etc
#root_etcdir      = "/etc"
#root_etc_SCRIPTS = $(libpal_la_list)
#install-data-hook:
#	chmod  go+r $(DESTDIR)$(root_etcdir)/kvh2xml.xml
#	chmod  go+r $(DESTDIR)$(root_etcdir)/mixer_paths_kona_mtp.xml
#	chmod  go+r $(DESTDIR)$(root_etcdir)/resourcemanager_kona_mtp.xml
#	chmod  go+r $(DESTDIR)$(root_etcdir)/usecaseKvManager.xml