pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = session_voice.pc
EXTRA_DIST = $(pkgconfig_DATA)


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

library_includedir = $(includedir)/session_voice

lib_LTLIBRARIES     = libsession_voice.la
libsession_voice_la_SOURCES   = $(session_voice_sources)
libsession_voice_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ltinyalsa -ltinycompress
libsession_voice_la_LIBADD    += -L${abs_top_builddir}/.libs -lpal
libsession_voice_la_LIBADD    += -L${abs_top_builddir}/session/SessionAR/.libs -lsession_ar
libsession_voice_la_CPPFLAGS := $(AM_CPPFLAGS)
libsession_voice_la_CPPFLAGS += -std=c++14
libsession_voice_la_LDFLAGS   = -shared -avoid-version
libsession_voice_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libsession_voice_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libsession_voice_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libsession_voice_la_LIBADD += -lglib-2.0
endif