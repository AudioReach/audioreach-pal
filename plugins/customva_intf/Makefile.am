pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = customva_intf.pc
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
AM_CPPFLAGS += -I ../../api/vui-interface
AM_CPPFLAGS += -I ../../resource_manager/inc
AM_CPPFLAGS += -I ../../context_manager/inc


customva_intf_sources = ./src/CustomVAInterface.cpp

library_includedir = $(includedir)/customva_intf

lib_LTLIBRARIES     = libcustomva_intf.la
libcustomva_intf_la_SOURCES   = $(customva_intf_sources)
libcustomva_intf_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat
libcustomva_intf_la_CPPFLAGS := $(AM_CPPFLAGS)
libcustomva_intf_la_CPPFLAGS += -std=c++14
libcustomva_intf_la_LDFLAGS   = -shared -avoid-version
libcustomva_intf_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libcustomva_intf_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libcustomva_intf_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED
libcustomva_intf_la_list      = ../../configs/$(MACHINE_ENABLED)/kvh2xml.xml \
                                 ../../configs/$(MACHINE_ENABLED)/usecaseKvManager.xml

if BUILDSYSTEM_OPENWRT
libcustomva_intf_la_LIBADD += -lglib-2.0
endif