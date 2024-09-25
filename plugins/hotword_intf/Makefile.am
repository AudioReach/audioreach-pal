pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = hotword_intf.pc
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
AM_CPPFLAGS += -I ../../api/vui-interface

hotword_intf_sources = ./src/HotwordInterface.cpp

library_includedir = $(includedir)/hotword_intf

lib_LTLIBRARIES     = libhotword_intf.la
libhotword_intf_la_SOURCES   = $(hotword_intf_sources)
libhotword_intf_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat
libhotword_intf_la_CPPFLAGS := $(AM_CPPFLAGS)
libhotword_intf_la_CPPFLAGS += -std=c++14
libhotword_intf_la_LDFLAGS   = -shared -avoid-version
libhotword_intf_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libhotword_intf_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libhotword_intf_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libhotword_intf_la_LIBADD += -lglib-2.0
endif