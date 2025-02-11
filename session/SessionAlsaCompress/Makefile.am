pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = session_compress.pc
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

session_compress_sources = ./src/SessionAlsaCompress.cpp
else
AM_CPPFLAGS := -I $(top_srcdir)/session/SessionAlsaCompress/inc
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

session_compress_sources = $(top_srcdir)/session/SessionAlsaCompress/src/SessionAlsaCompress.cpp
endif


library_includedir = $(includedir)/session_compress

lib_LTLIBRARIES     = libsession_compress.la
libsession_compress_la_SOURCES   = $(session_compress_sources)
libsession_compress_la_LIBADD    = @GLIB_LIBS@ -lexpat -ltinyalsa -ltinycompress
libsession_compress_la_LIBADD    += -L${abs_top_builddir}/session/SessionAR/.libs -lsession_ar -L${abs_top_builddir}/.libs -lpal
libsession_compress_la_CPPFLAGS := $(AM_CPPFLAGS)
libsession_compress_la_CPPFLAGS += -std=c++14
libsession_compress_la_LDFLAGS   = -shared -avoid-version
libsession_compress_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h
libsession_compress_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libsession_compress_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_QLI
libsession_compress_la_CPPFLAGS += -DPAL_USE_SYSLOG -DLINUX_ENABLED -DPAL_MEMLOG_UNSUPPORTED -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if BUILDSYSTEM_OPENWRT
libsession_compress_la_LIBADD += -lglib-2.0 -llog
endif

if CUTILS_SUPPORTED
libsession_compress_la_CPPFLAGS += -DPAL_CUTILS_SUPPORTED
libsession_compress_la_LIBADD += -lcutils
endif

if USE_G_STR_FUNC
libsession_compress_la_CPPFLAGS +=-Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
endif
