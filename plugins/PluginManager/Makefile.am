pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = plugin_manager.pc
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


plugin_manager_sources = ./src/PluginManager.cpp

library_includedir = $(includedir)/plugin_manager

$(info MACHINE_ENABLED is $(MACHINE_ENABLED))

lib_LTLIBRARIES     = libplugin_manager.la
libplugin_manager_la_SOURCES   = $(plugin_manager_sources)
libplugin_manager_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat
libplugin_manager_la_CPPFLAGS := $(AM_CPPFLAGS)
libplugin_manager_la_CPPFLAGS += -std=c++14
libplugin_manager_la_LDFLAGS   = --disable-shared -avoid-version
libplugin_manager_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libplugin_manager_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libplugin_manager_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED
libplugin_manager_la_list      = ../../configs/$(MACHINE_ENABLED)/kvh2xml.xml \
                                 ../../configs/$(MACHINE_ENABLED)/usecaseKvManager.xml

if BUILDSYSTEM_OPENWRT
libplugin_manager_la_LIBADD += -lglib-2.0
endif