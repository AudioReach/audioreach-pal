pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = streamsensorpcmdata_headers.pc
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


streamsensorpcmdata_headers_sources = ./src/StreamSensorPCMData.cpp

library_includedir = $(includedir)/streamsensorpcmdata_headers

lib_LTLIBRARIES     = libstreamsensorpcmdata_headers.la
libstreamsensorpcmdata_headers_la_SOURCES   = $(streamsensorpcmdata_headers_sources)
libstreamsensorpcmdata_headers_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -L${abs_top_builddir}/stream/StreamCommon/.libs -lstream_common
libstreamsensorpcmdata_headers_la_CPPFLAGS := $(AM_CPPFLAGS)
libstreamsensorpcmdata_headers_la_CPPFLAGS += -std=c++14
libstreamsensorpcmdata_headers_la_LDFLAGS   = -shared -avoid-version
libstreamsensorpcmdata_headers_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libstreamsensorpcmdata_headers_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libstreamsensorpcmdata_headers_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libstreamsensorpcmdata_headers_la_LIBADD += -lglib-2.0
endif