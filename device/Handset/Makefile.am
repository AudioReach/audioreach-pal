pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = dev_handset.pc
EXTRA_DIST = $(pkgconfig_DATA)

if BUILDSYSTEM_OPENWRT
AM_CPPFLAGS := -I $(top_srcdir)/device/Handset/inc
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
AM_CPPFLAGS += -I $(top_srcdir)/device/Speaker/inc
AM_CPPFLAGS += -I $(top_srcdir)/plugins/vui_interface/api/vui-interface

dev_handset_sources = ./src/Handset.cpp
else
AM_CPPFLAGS := -I $(top_srcdir)/device/Handset/inc
AM_CPPFLAGS += -I $(top_srcdir)/device/Speaker/inc
AM_CPPFLAGS += -I $(top_srcdir)/inc
AM_CPPFLAGS += -I $(top_srcdir)
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

dev_handset_sources = $(top_srcdir)/device/Handset/src/Handset.cpp
endif

library_includedir = $(includedir)/dev_handset

lib_LTLIBRARIES     = libdev_handset.la
libdev_handset_la_SOURCES   = $(dev_handset_sources)
libdev_handset_la_LIBADD    = @GLIB_LIBS@ -lexpat -ltinyalsa -ltinycompress
libdev_handset_la_LIBADD    += -L${abs_top_builddir}/device/Speaker/.libs -ldev_speaker
libdev_handset_la_LIBADD   += -L${abs_top_builddir}/.libs -lpal
libdev_handset_la_CPPFLAGS := $(AM_CPPFLAGS)
libdev_handset_la_CPPFLAGS += -std=c++14
libdev_handset_la_LDFLAGS   = -shared -avoid-version

libdev_handset_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h

libdev_handset_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libdev_handset_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED


if BUILDSYSTEM_OPENWRT
libdev_handset_la_LIBADD += -lglib-2.0 -llog
else
libdev_handset_la_CPPFLAGS += -DPAL_USE_SYSLOG -DLINUX_ENABLED  -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if CUTILS_SUPPORTED
libdev_handset_la_CPPFLAGS += -DPAL_CUTILS_SUPPORTED
libdev_handset_la_LIBADD += -lcutils
endif

if USE_G_STR_FUNC
libdev_handset_la_CPPFLAGS +=-Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
endif
