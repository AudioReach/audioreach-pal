pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = streamsensorpcmdata_headers.pc
EXTRA_DIST = $(pkgconfig_DATA)

if BUILDSYSTEM_OPENWRT
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
else
AM_CPPFLAGS := -I $(top_srcdir)/stream/StreamCommon/StreamSensorPCMData/inc
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

streamsensorpcmdata_sources = $(top_srcdir)/stream/StreamCommon/StreamSensorPCMData/src/StreamSensorPCMData.cpp
endif
library_includedir = $(includedir)/streamsensorpcmdata_headers

lib_LTLIBRARIES     = libstream_sensorpcmdata.la
libstream_sensorpcmdata_la_SOURCES   = $(streamsensorpcmdata_sources)
libstream_sensorpcmdata_la_LIBADD    = @GLIB_LIBS@ -lexpat -L${abs_top_builddir}/stream/StreamCommon/.libs -lstream_common
libstream_sensorpcmdata_la_LIBADD    += -L${abs_top_builddir}/.libs -lpal
libstream_sensorpcmdata_la_CPPFLAGS := $(AM_CPPFLAGS)
libstream_sensorpcmdata_la_CPPFLAGS += -std=c++14
libstream_sensorpcmdata_la_LDFLAGS   = -shared -avoid-version
libstream_sensorpcmdata_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h -DLINUX_ENABLED
libstream_sensorpcmdata_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libstream_sensorpcmdata_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libstream_sensorpcmdata_la_LIBADD += -lglib-2.0 -llog
else
libstream_sensorpcmdata_la_CPPFLAGS += -DPAL_USE_SYSLOG -DLINUX_ENABLED  -DATRACE_UNSUPPORTED -DPAL_MEMLOG_UNSUPPORTED  -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if CUTILS_SUPPORTED
libstream_sensorpcmdata_la_CPPFLAGS += -DPAL_CUTILS_SUPPORTED
libstream_sensorpcmdata_la_LIBADD += -lcutils
endif

if USE_G_STR_FUNC
libstream_sensorpcmdata_la_CPPFLAGS +=-Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
endif
