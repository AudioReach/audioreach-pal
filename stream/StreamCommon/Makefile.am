pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = stream_common.pc
EXTRA_DIST = $(pkgconfig_DATA)

AM_CPPFLAGS := -I ./inc
AM_CPPFLAGS += -I ../../inc
AM_CPPFLAGS += -I ../../
AM_CPPFLAGS += -I ../../utils/inc
AM_CPPFLAGS += -I ../../device/inc
AM_CPPFLAGS += -I ../../session/inc
AM_CPPFLAGS += -I ../../stream/inc
AM_CPPFLAGS += -I ../../session/SessionAR/inc
AM_CPPFLAGS += -I ../../plugins/PluginManager/inc
AM_CPPFLAGS += -I ../../resource_manager/inc
AM_CPPFLAGS += -I ../../context_manager/inc


stream_common_sources = ./src/StreamCommon.cpp

library_includedir = $(includedir)/stream_common

lib_LTLIBRARIES     = libstream_common.la
libstream_common_la_SOURCES   = $(stream_common_sources)
libstream_common_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat
libstream_common_la_CPPFLAGS := $(AM_CPPFLAGS)
libstream_common_la_CPPFLAGS += -std=c++14
libstream_common_la_LDFLAGS   = -shared -avoid-version
libstream_common_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libstream_common_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libstream_common_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED
libstream_common_la_list      = ../../configs/$(MACHINE_ENABLED)/kvh2xml.xml \
                                 ../../configs/$(MACHINE_ENABLED)/usecaseKvManager.xml

if BUILDSYSTEM_OPENWRT
libstream_common_la_LIBADD += -lglib-2.0
endif

SUBDIRS = StreamUltrasSound StreamSensorRenderer StreamSensorPCMData StreamContextProxy StreamCommonProxy