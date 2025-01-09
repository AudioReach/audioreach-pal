pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = session_utils_config.pc
EXTRA_DIST = $(pkgconfig_DATA)

AM_CPPFLAGS = -I ${top_srcdir}
AM_CPPFLAGS += -I ${top_srcdir}/api
AM_CPPFLAGS += -I ${top_srcdir}/inc
AM_CPPFLAGS += -I ${top_srcdir}/utils/inc
AM_CPPFLAGS += -I ${top_srcdir}/device/inc
AM_CPPFLAGS += -I ${top_srcdir}/stream/inc
AM_CPPFLAGS += -I ${top_srcdir}/session/inc
AM_CPPFLAGS += -I ${top_srcdir}/context_manager/inc
AM_CPPFLAGS += -I ${top_srcdir}/resource_manager/inc
AM_CPPFLAGS += -I ${top_srcdir}/plugins/PluginManager/inc
AM_CPPFLAGS += -I ${top_srcdir}/session/SessionAlsaPcm/inc
AM_CPPFLAGS += -I ${top_srcdir}/session/SessionAR/inc
AM_CPPFLAGS += -I ${top_srcdir}/api/vui-interface
AM_CPPFLAGS += -I ${top_srcdir}/plugins/codecs/
AM_CPPFLAGS += -I ${top_srcdir}/plugins/configs/ConfigSessionUtils/inc
AM_CPPFLAGS += @AGM_CFLAGS@
AM_CPPFLAGS += @PALHEADERS_CFLAGS@
AM_CPPFLAGS += @SPFHEADERS_CFLAGS@
AM_CPPFLAGS += @KVH2XML_CFLAGS@
AM_CPPFLAGS += @AROUTE_CFLAGS@


session_utils_sources = ./src/ConfigSessionUtils.cpp

library_includedir = $(includedir)/session_utils_config
lib_LTLIBRARIES     = libsession_utils_config.la
libsession_utils_config_la_SOURCES   = $(session_utils_sources)
libsession_utils_config_la_LIBADD    = @GLIB_LIBS@ -lexpat -ltinyalsa
libsession_utils_config_la_CPPFLAGS := $(AM_CPPFLAGS)
libsession_utils_config_la_CPPFLAGS += -std=c++14
libsession_utils_config_la_LDFLAGS   = --disable-shared -avoid-version
libsession_utils_config_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h
libsession_utils_config_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libsession_utils_config_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DPAL_USE_SYSLOG -DLINUX_ENABLED -DPAL_MEMLOG_UNSUPPORTED -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED

libsession_utils_config_la_LIBADD   += -L${abs_top_builddir}/.libs -lpal
libsession_utils_config_la_LIBADD   += -L${abs_top_builddir}/session/SessionAlsaPcm/.libs -lsession_pcm

if BUILDSYSTEM_OPENWRT
libsession_utils_config_la_LIBADD += -lglib-2.0
endif
