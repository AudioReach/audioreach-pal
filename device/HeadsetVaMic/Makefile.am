pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = dev_headset_va.pc
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


dev_headset_va_sources = ./src/HeadsetVaMic.cpp

library_includedir = $(includedir)/dev_headset_va

lib_LTLIBRARIES     = libdev_headset_va.la
libdev_headset_va_la_SOURCES   = $(dev_headset_va_sources)
libdev_headset_va_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ltinyalsa -ltinycompress
libdev_headset_va_la_CPPFLAGS := $(AM_CPPFLAGS)
libdev_headset_va_la_CPPFLAGS += -std=c++14
libdev_headset_va_la_LDFLAGS   = -shared -avoid-version
libdev_headset_va_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libdev_headset_va_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libdev_headset_va_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED
libdev_headset_va_la_list      = ../../configs/$(MACHINE_ENABLED)/kvh2xml.xml \
                                 ../../configs/$(MACHINE_ENABLED)/usecaseKvManager.xml

if BUILDSYSTEM_OPENWRT
libdev_headset_va_la_LIBADD += -lglib-2.0
endif