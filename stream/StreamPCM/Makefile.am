pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = stream_pcm.pc
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


stream_pcm_sources = ./src/StreamPCM.cpp

library_includedir = $(includedir)/stream_pcm

lib_LTLIBRARIES     = libstream_pcm.la
libstream_pcm_la_SOURCES   = $(stream_pcm_sources)
libstream_pcm_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat
libstream_pcm_la_CPPFLAGS := $(AM_CPPFLAGS)
libstream_pcm_la_CPPFLAGS += -std=c++14
libstream_pcm_la_LDFLAGS   = -shared -avoid-version
libstream_pcm_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libstream_pcm_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libstream_pcm_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED


if BUILDSYSTEM_OPENWRT
libstream_pcm_la_LIBADD += -lglib-2.0
endif