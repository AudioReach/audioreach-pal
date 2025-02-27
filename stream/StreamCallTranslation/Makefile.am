pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = stream_calltranslation.pc
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


stream_calltranslation_sources = ./src/StreamCallTranslation.cpp

library_includedir = $(includedir)/stream_calltranslation

lib_LTLIBRARIES     = libstream_calltranslation.la
libstream_calltranslation_la_SOURCES   = $(stream_calltranslation_sources)
libstream_calltranslation_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat
libstream_calltranslation_la_CPPFLAGS := $(AM_CPPFLAGS)
libstream_calltranslation_la_CPPFLAGS += -std=c++14
libstream_calltranslation_la_LDFLAGS   = -shared -avoid-version
libstream_calltranslation_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libstream_calltranslation_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libstream_calltranslation_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libstream_calltranslation_la_LIBADD += -lglib-2.0
endif