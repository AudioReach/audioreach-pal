pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = session_agm.pc
EXTRA_DIST = $(pkgconfig_DATA)

if BUILDSYSTEM_OPENWRT
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

session_agm_sources = ./src/SessionAgm.cpp
else
AM_CPPFLAGS := -I $(top_srcdir)/session/SessionAgm/inc
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
AM_CPPFLAGS += -I $(top_srcdir)/api/vui-interface
AM_CPPFLAGS += -I ${top_srcdir}/plugins/codecs
AM_CPPFLAGS += @SPF_CFLAGS@
AM_CPPFLAGS += @AROUTE_CFLAGS@
AM_CPPFLAGS += @KVH2XML_CFLAGS@

session_agm_sources = $(top_srcdir)/session/SessionAgm/src/SessionAgm.cpp
endif

library_includedir = $(includedir)/session_agm

lib_LTLIBRARIES     = libsession_agm.la
libsession_agm_la_SOURCES   = $(session_agm_sources)
libsession_agm_la_LIBADD    = @GLIB_LIBS@ -lexpat -ltinyalsa -ltinycompress
libsession_agm_la_LIBADD    += -L${abs_top_builddir}/.libs -lpal
libsession_agm_la_LIBADD    += -L${abs_top_builddir}/session/SessionAR/.libs -lsession_ar
libsession_agm_la_CPPFLAGS := $(AM_CPPFLAGS)
libsession_agm_la_CPPFLAGS += -std=c++14
libsession_agm_la_LDFLAGS   = -shared -avoid-version
libsession_agm_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h
libsession_agm_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libsession_agm_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\)

if BUILDSYSTEM_QLI
libsession_agm_la_CPPFLAGS += -DPAL_USE_SYSLOG -DLINUX_ENABLED -DPAL_MEMLOG_UNSUPPORTED -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if BUILDSYSTEM_OPENWRT
libsession_agm_la_LIBADD += -lglib-2.0 -llogs -lagmclientwrapper
endif

if CUTILS_SUPPORTED
libsession_agm_la_CPPFLAGS += -DPAL_CUTILS_SUPPORTED
libsession_agm_la_LIBADD += -lcutils
endif

if USE_G_STR_FUNC
libsession_agm_la_CPPFLAGS +=-Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
endif
