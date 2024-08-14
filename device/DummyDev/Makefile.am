pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = dev_dummy.pc
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


dev_dummy_sources = ./src/DummyDev.cpp

library_includedir = $(includedir)/dev_dummy

lib_LTLIBRARIES     = libdev_dummy.la
libdev_dummy_la_SOURCES   = $(dev_dummy_sources)
libdev_dummy_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ltinyalsa -ltinycompress
libdev_dummy_la_CPPFLAGS := $(AM_CPPFLAGS)
libdev_dummy_la_CPPFLAGS += -std=c++14
libdev_dummy_la_LDFLAGS   = -shared -avoid-version
libdev_dummy_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libdev_dummy_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libdev_dummy_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED
libdev_dummy_la_list      = ../../configs/$(MACHINE_ENABLED)/kvh2xml.xml \
                                 ../../configs/$(MACHINE_ENABLED)/usecaseKvManager.xml

if BUILDSYSTEM_OPENWRT
libdev_dummy_la_LIBADD += -lglib-2.0
endif