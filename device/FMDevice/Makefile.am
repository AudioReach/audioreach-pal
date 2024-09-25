pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = dev_fm.pc
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


dev_fm_sources = ./src/FMDevice.cpp

library_includedir = $(includedir)/dev_fm

lib_LTLIBRARIES     = libdev_fm.la
libdev_fm_la_SOURCES   = $(dev_fm_sources)
libdev_fm_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ltinyalsa -ltinycompress
libdev_fm_la_CPPFLAGS := $(AM_CPPFLAGS)
libdev_fm_la_CPPFLAGS += -std=c++14
libdev_fm_la_LDFLAGS   = -shared -avoid-version
libdev_fm_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libdev_fm_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libdev_fm_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libdev_fm_la_LIBADD += -lglib-2.0
endif
