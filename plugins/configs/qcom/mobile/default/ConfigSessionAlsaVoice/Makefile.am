pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = session_voice_config.pc
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
AM_CPPFLAGS += -I ${top_srcdir}/session/SessionAlsaVoice/inc
AM_CPPFLAGS += -I ${top_srcdir}/session/SessionAR/inc
AM_CPPFLAGS += -I ${top_srcdir}/api/vui-interface
AM_CPPFLAGS += -I ${top_srcdir}/plugins/codecs/
AM_CPPFLAGS += -I ${top_srcdir}/plugins/configs/ConfigSessionUtils/inc
AM_CPPFLAGS += -I ${top_srcdir}/plugins/configs/ConfigSessionAlsaVoice/inc
AM_CPPFLAGS += @AGM_CFLAGS@
AM_CPPFLAGS += @PALHEADERS_CFLAGS@
AM_CPPFLAGS += @SPFHEADERS_CFLAGS@
AM_CPPFLAGS += @KVH2XML_CFLAGS@
AM_CPPFLAGS += @AROUTE_CFLAGS@


session_voice_config_sources = ./src/ConfigSessionAlsaVoice.cpp

library_includedir = $(includedir)/session_voice_config
lib_LTLIBRARIES     = libsession_voice_config.la
libsession_voice_config_la_SOURCES   = $(session_voice_config_sources)
libsession_voice_config_la_LIBADD    = @GLIB_LIBS@ -lexpat -ltinyalsa
libsession_voice_config_la_CPPFLAGS := $(AM_CPPFLAGS)
libsession_voice_config_la_CPPFLAGS += -std=c++14
libsession_voice_config_la_LDFLAGS   = -shared -avoid-version
libsession_voice_config_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h  -DPAL_USE_SYSLOG -DLINUX_ENABLED -DPAL_MEMLOG_UNSUPPORTED -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
libsession_voice_config_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libsession_voice_config_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

libsession_voice_config_la_LIBADD   += -L${abs_top_builddir}/.libs -lpal
libsession_voice_config_la_LIBADD   += -L${abs_top_builddir}/session/SessionAR/.libs -lsession_ar
libsession_voice_config_la_LIBADD   += -L${abs_top_builddir}/session/SessionAlsaVoice/.libs -lsession_voice
libsession_voice_config_la_LIBADD   += ${abs_top_builddir}/plugins/configs/ConfigSessionUtils/.libs/libsession_utils_config.a

if BUILDSYSTEM_OPENWRT
libsession_voice_config_la_LIBADD += -lglib-2.0
endif
