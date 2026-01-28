pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = dev_headset_va.pc
EXTRA_DIST = $(pkgconfig_DATA)

if BUILDSYSTEM_OPENWRT
AM_CPPFLAGS := -I $(top_srcdir)/device/HeadsetVaMic/inc
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

dev_headset_va_sources = ./src/HeadsetVaMic.cpp
else
AM_CPPFLAGS := -I $(top_srcdir)/device/HeadsetVaMic/inc
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

dev_headset_va_sources = $(top_srcdir)/device/HeadsetVaMic/src/HeadsetVaMic.cpp
endif

library_includedir = $(includedir)/dev_headset_va

lib_LTLIBRARIES     = libdev_headset_va.la
libdev_headset_va_la_SOURCES   = $(dev_headset_va_sources)
libdev_headset_va_la_LIBADD    = @GLIB_LIBS@ -lexpat -ltinyalsa -ltinycompress
libdev_headset_va_la_LIBADD   += -L${abs_top_builddir}/.libs -lpal
libdev_headset_va_la_CPPFLAGS := $(AM_CPPFLAGS)
libdev_headset_va_la_CPPFLAGS += -std=c++14
libdev_headset_va_la_LDFLAGS   = -shared -avoid-version

libdev_headset_va_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h

libdev_headset_va_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libdev_headset_va_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libdev_headset_va_la_LIBADD += -lglib-2.0 -llog
else
libdev_headset_va_la_CPPFLAGS += -DPAL_USE_SYSLOG -DLINUX_ENABLED  -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if CUTILS_SUPPORTED
libdev_headset_va_la_CPPFLAGS += -DPAL_CUTILS_SUPPORTED
libdev_headset_va_la_LIBADD += -lcutils
endif

if USE_G_STR_FUNC
libdev_headset_va_la_CPPFLAGS +=-Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
endif
