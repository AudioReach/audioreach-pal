pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = session_compress.pc
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


session_compress_sources = ./src/SessionAlsaCompress.cpp

library_includedir = $(includedir)/session_compress

lib_LTLIBRARIES     = libsession_compress.la
libsession_compress_la_SOURCES   = $(session_compress_sources)
libsession_compress_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ltinyalsa -ltinycompress
libsession_compress_la_LIBADD    += -L${abs_top_builddir}/plugins/vui_interface/.libs -lvui_intf
libsession_compress_la_LIBADD    += -L${abs_top_builddir}/session/SessionAR/.libs -lsession_ar -L${abs_top_builddir}/.libs -lpal
libsession_compress_la_CPPFLAGS := $(AM_CPPFLAGS)
libsession_compress_la_CPPFLAGS += -std=c++14
libsession_compress_la_LDFLAGS   = -shared -avoid-version
libsession_compress_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libsession_compress_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libsession_compress_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libsession_compress_la_LIBADD += -lglib-2.0
endif