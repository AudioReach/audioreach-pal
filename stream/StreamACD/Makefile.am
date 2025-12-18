pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = stream_acd.pc
EXTRA_DIST = $(pkgconfig_DATA)

if BUILDSYSTEM_OPENWRT
AM_CPPFLAGS := -I $(top_srcdir)/stream/StreamACD/inc
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

stream_acd_sources = ./src/StreamACD.cpp
else
AM_CPPFLAGS := -I $(top_srcdir)/stream/StreamACD/inc
AM_CPPFLAGS += -I $(top_srcdir)/inc
AM_CPPFLAGS += -I $(top_srcdir)
AM_CPPFLAGS += -I $(top_srcdir)/utils/inc
AM_CPPFLAGS += -I $(top_srcdir)/device/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/SessionAR/inc
AM_CPPFLAGS += -I $(top_srcdir)/plugins/PluginManager/inc
AM_CPPFLAGS += -I $(top_srcdir)/resource_manager/inc
AM_CPPFLAGS += -I  $(top_srcdir)/plugins/vui_interface/api/vui-interface
AM_CPPFLAGS += -I $(top_srcdir)/plugins/codecs
AM_CPPFLAGS += -I $(top_srcdir)/context_manager/inc
AM_CPPFLAGS += @SPF_CFLAGS@
AM_CPPFLAGS += @AROUTE_CFLAGS@
AM_CPPFLAGS += @KVH2XML_CFLAGS@

stream_acd_sources = $(top_srcdir)/stream/StreamACD/src/StreamACD.cpp
endif

library_includedir = $(includedir)/stream_acd

lib_LTLIBRARIES     = libstream_acd.la
libstream_acd_la_SOURCES   = $(stream_acd_sources)
libstream_acd_la_LIBADD    = @GLIB_LIBS@ -lexpat
libstream_acd_la_LIBADD    += -L${abs_top_builddir}/.libs -lpal
libstream_acd_la_LIBADD    += -L${abs_top_builddir}/session/SessionAR/.libs -lsession_ar
libstream_acd_la_CPPFLAGS := $(AM_CPPFLAGS)
libstream_acd_la_CPPFLAGS += -std=c++14
libstream_acd_la_LDFLAGS   = -shared -avoid-version
libstream_acd_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h -DLINUX_ENABLED
libstream_acd_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libstream_acd_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libstream_acd_la_LIBADD += -lglib-2.0 -llog
else
libstream_acd_la_CPPFLAGS += -DPAL_USE_SYSLOG -DLINUX_ENABLED  -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if CUTILS_SUPPORTED
libstream_acd_la_CPPFLAGS += -DPAL_CUTILS_SUPPORTED
libstream_acd_la_LIBADD += -lcutils
endif

if USE_G_STR_FUNC
libstream_acd_la_CPPFLAGS +=-Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
endif
