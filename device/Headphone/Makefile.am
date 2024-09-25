pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = dev_headphone.pc
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


dev_headphone_sources = ./src/Headphone.cpp

library_includedir = $(includedir)/dev_headphone

lib_LTLIBRARIES     = libdev_headphone.la
libdev_headphone_la_SOURCES   = $(dev_headphone_sources)
libdev_headphone_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ltinyalsa -ltinycompress
libdev_headphone_la_CPPFLAGS := $(AM_CPPFLAGS)
libdev_headphone_la_CPPFLAGS += -std=c++14
libdev_headphone_la_LDFLAGS   = -shared -avoid-version
libdev_headphone_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libdev_headphone_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libdev_headphone_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libdev_headphone_la_LIBADD += -lglib-2.0
endif
