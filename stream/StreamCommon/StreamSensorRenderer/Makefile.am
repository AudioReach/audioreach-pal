pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = stream_sensorrenderer.pc
EXTRA_DIST = $(pkgconfig_DATA)

AM_CPPFLAGS := -I ./inc
AM_CPPFLAGS += -I ../inc
AM_CPPFLAGS += -I ../../../inc
AM_CPPFLAGS += -I ../../../
AM_CPPFLAGS += -I ../../../utils/inc
AM_CPPFLAGS += -I ../../../device/inc
AM_CPPFLAGS += -I ../../../session/inc
AM_CPPFLAGS += -I ../../../stream/inc
AM_CPPFLAGS += -I ../../../session/SessionAR/inc
AM_CPPFLAGS += -I ../../../plugins/PluginManager/inc
AM_CPPFLAGS += -I ../../../resource_manager/inc
AM_CPPFLAGS += -I ../../../context_manager/inc


stream_sensorrenderer_sources = ./src/StreamSensorRenderer.cpp

library_includedir = $(includedir)/stream_sensorrenderer

lib_LTLIBRARIES     = libstream_sensorrenderer.la
libstream_sensorrenderer_la_SOURCES   = $(stream_sensorrenderer_sources)
libstream_sensorrenderer_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -L${abs_top_builddir}/stream/StreamCommon/.libs -lstream_common
libstream_sensorrenderer_la_CPPFLAGS := $(AM_CPPFLAGS)
libstream_sensorrenderer_la_CPPFLAGS += -std=c++14
libstream_sensorrenderer_la_LDFLAGS   = -shared -avoid-version
libstream_sensorrenderer_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libstream_sensorrenderer_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libstream_sensorrenderer_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libstream_sensorrenderer_la_LIBADD += -lglib-2.0
endif