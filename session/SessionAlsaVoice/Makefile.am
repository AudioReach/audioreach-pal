pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = session_voice.pc
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

session_voice_sources = ./src/SessionAlsaVoice.cpp
else
AM_CPPFLAGS := -I $(top_srcdir)/session/SessionAlsaVoice/inc
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
AM_CPPFLAGS += -I ${top_srcdir}/plugins/codecs
AM_CPPFLAGS += @SPF_CFLAGS@
AM_CPPFLAGS += @AROUTE_CFLAGS@
AM_CPPFLAGS += @KVH2XML_CFLAGS@

session_voice_sources = $(top_srcdir)/session/SessionAlsaVoice/src/SessionAlsaVoice.cpp

endif

library_includedir = $(includedir)/session_voice

lib_LTLIBRARIES     = libsession_voice.la
libsession_voice_la_SOURCES   = $(session_voice_sources)
libsession_voice_la_LIBADD    = @GLIB_LIBS@ -lexpat -ltinyalsa -ltinycompress
libsession_voice_la_LIBADD    += -L${abs_top_builddir}/.libs -lpal
libsession_voice_la_LIBADD    += -L${abs_top_builddir}/session/SessionAR/.libs -lsession_ar
libsession_voice_la_CPPFLAGS := $(AM_CPPFLAGS)
libsession_voice_la_CPPFLAGS += -std=c++14
libsession_voice_la_LDFLAGS   = -shared -avoid-version
libsession_voice_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h

libsession_voice_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libsession_voice_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libsession_voice_la_LIBADD += -lglib-2.0 -llog
else
libsession_voice_la_CPPFLAGS += -DPAL_USE_SYSLOG -DLINUX_ENABLED  -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if CUTILS_SUPPORTED
libsession_voice_la_CPPFLAGS += -DPAL_CUTILS_SUPPORTED
libsession_voice_la_LIBADD += -lcutils
endif

if USE_G_STR_FUNC
libsession_voice_la_CPPFLAGS +=-Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
endif
