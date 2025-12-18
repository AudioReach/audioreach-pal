pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = stream_contextproxy.pc
EXTRA_DIST = $(pkgconfig_DATA)

if BUILDSYSTEM_OPENWRT
AM_CPPFLAGS := -I $(top_srcdir)/stream/StreamCommon/StreamContextProxy/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/StreamCommon/inc
AM_CPPFLAGS += -I $(top_srcdir)/inc
AM_CPPFLAGS += -I $(top_srcdir)/
AM_CPPFLAGS += -I $(top_srcdir)/utils/inc
AM_CPPFLAGS += -I $(top_srcdir)/device/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/SessionAR/inc
AM_CPPFLAGS += -I $(top_srcdir)/plugins/PluginManager/inc
AM_CPPFLAGS += -I $(top_srcdir)/resource_manager/inc
AM_CPPFLAGS += -I $(top_srcdir)/context_manager/inc
AM_CPPFLAGS += -I $(top_srcdir)/plugins/vui_interface/api/vui-interface

stream_contextproxy_sources = ./src/StreamContextProxy.cpp
else
AM_CPPFLAGS := -I $(top_srcdir)/stream/StreamCommon/StreamContextProxy/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/StreamCommon/inc
AM_CPPFLAGS += -I $(top_srcdir)/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/inc
AM_CPPFLAGS += -I $(top_srcdir)
AM_CPPFLAGS += -I $(top_srcdir)/utils/inc
AM_CPPFLAGS += -I $(top_srcdir)/device/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/SessionAR/inc
AM_CPPFLAGS += -I $(top_srcdir)/plugins/PluginManager/inc
AM_CPPFLAGS += -I $(top_srcdir)/resource_manager/inc
AM_CPPFLAGS += -I $(top_srcdir)/context_manager/inc
AM_CPPFLAGS += -I  $(top_srcdir)/plugins/vui_interface/api/vui-interface
AM_CPPFLAGS += @SPF_CFLAGS@
AM_CPPFLAGS += @AROUTE_CFLAGS@
AM_CPPFLAGS += @KVH2XML_CFLAGS@

stream_contextproxy_sources = $(top_srcdir)/stream/StreamCommon/StreamContextProxy/src/StreamContextProxy.cpp
endif

library_includedir = $(includedir)/stream_contextproxy

lib_LTLIBRARIES     = libstream_contextproxy.la
libstream_contextproxy_la_SOURCES   = $(stream_contextproxy_sources)
libstream_contextproxy_la_LIBADD    = @GLIB_LIBS@ -lexpat -L${abs_top_builddir}/stream/StreamCommon/.libs -lstream_common
libstream_contextproxy_la_LIBADD    += -L${abs_top_builddir}/.libs -lpal
libstream_contextproxy_la_CPPFLAGS := $(AM_CPPFLAGS)
libstream_contextproxy_la_CPPFLAGS += -std=c++14
libstream_contextproxy_la_LDFLAGS   = -shared -avoid-version
libstream_contextproxy_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h -DLINUX_ENABLED
libstream_contextproxy_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libstream_contextproxy_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libstream_contextproxy_la_LIBADD += -lglib-2.0 -llog
else
libstream_contextproxy_la_CPPFLAGS += -DPAL_USE_SYSLOG -DLINUX_ENABLED  -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if CUTILS_SUPPORTED
libstream_contextproxy_la_CPPFLAGS += -DPAL_CUTILS_SUPPORTED
libstream_contextproxy_la_LIBADD += -lcutils
endif

if USE_G_STR_FUNC
libstream_contextproxy_la_CPPFLAGS +=-Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
endif
