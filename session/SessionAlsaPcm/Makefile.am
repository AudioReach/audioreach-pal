pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = session_pcm.pc
EXTRA_DIST = $(pkgconfig_DATA)

if BUILDSYSTEM_OPENWRT
AM_CPPFLAGS := -I $(top_srcdir)/session/SessionAlsaPcm/inc
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

session_pcm_sources = ./src/SessionAlsaPcm.cpp
else
AM_CPPFLAGS := -I $(top_srcdir)/session/SessionAlsaPcm/inc
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
AM_CPPFLAGS += -I $(top_srcdir)/plugins/codecs
AM_CPPFLAGS += @SPF_CFLAGS@
AM_CPPFLAGS += @AROUTE_CFLAGS@
AM_CPPFLAGS += @KVH2XML_CFLAGS@

session_pcm_sources = $(top_srcdir)/session/SessionAlsaPcm/src/SessionAlsaPcm.cpp
endif

library_includedir = $(includedir)/session_pcm

lib_LTLIBRARIES     = libsession_pcm.la
libsession_pcm_la_SOURCES   = $(session_pcm_sources)
libsession_pcm_la_LIBADD    = @GLIB_LIBS@ -lexpat -ltinyalsa -ltinycompress
libsession_pcm_la_LIBADD   += -L${abs_top_builddir}/.libs -lpal
libsession_pcm_la_LIBADD   += -L${abs_top_builddir}/session/SessionAR/.libs -lsession_ar
libsession_pcm_la_CPPFLAGS := $(AM_CPPFLAGS)
libsession_pcm_la_CPPFLAGS += -std=c++14
libsession_pcm_la_LDFLAGS   = -shared -avoid-version
libsession_pcm_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h -DAUDIO_FEATURE_STATS_UNSUPPORTED
libsession_pcm_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libsession_pcm_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libsession_pcm_la_LIBADD += -lglib-2.0 -llog -lagmclientwrapper
else
libsession_pcm_la_CPPFLAGS += -DPAL_USE_SYSLOG -DLINUX_ENABLED  -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if CUTILS_SUPPORTED
libsession_pcm_la_CPPFLAGS += -DPAL_CUTILS_SUPPORTED
libsession_pcm_la_LIBADD += -lcutils
endif

if USE_G_STR_FUNC
libsession_pcm_la_CPPFLAGS +=-Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
endif
