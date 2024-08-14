pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = stream_incall.pc
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


stream_incall_sources = ./src/StreamInCall.cpp

library_includedir = $(includedir)/stream_incall

lib_LTLIBRARIES     = libstream_incall.la
libstream_incall_la_SOURCES   = $(stream_incall_sources)
libstream_incall_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat
libstream_incall_la_CPPFLAGS := $(AM_CPPFLAGS)
libstream_incall_la_CPPFLAGS += -std=c++14
libstream_incall_la_LDFLAGS   = -shared -avoid-version
libstream_incall_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libstream_incall_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libstream_incall_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED
libstream_incall_la_list      = ../../configs/$(MACHINE_ENABLED)/kvh2xml.xml \
                                 ../../configs/$(MACHINE_ENABLED)/usecaseKvManager.xml

if BUILDSYSTEM_OPENWRT
libstream_incall_la_LIBADD += -lglib-2.0
endif