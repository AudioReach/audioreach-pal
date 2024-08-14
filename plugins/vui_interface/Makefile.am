pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = vui_interface.pc
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

vui_interface_sourcess := ./src/SoundTriggerUtils.cpp$
vui_interface_sourcess += ./src/SVAExtension.cpp$
vui_interface_sourcess += ./src/SVAInterface.cpp$
vui_interface_sourcess += ./src/VoiceUIInterface.cpp$

library_includedir = $(includedir)/vui_interface
lib_LTLIBRARIES     = libvui_intf.la
libvui_intf_la_SOURCES   = $(vui_interface_sources)
libvui_intf_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat
libvui_intf_la_CPPFLAGS := $(AM_CPPFLAGS)
libvui_intf_la_CPPFLAGS += -std=c++14
libvui_intf_la_LDFLAGS   = -shared -avoid-version
libvui_intf_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libvui_intf_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libvui_intf_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED
libvui_intf_la_list      = ../../configs/$(MACHINE_ENABLED)/kvh2xml.xml \
                           ../../configs/$(MACHINE_ENABLED)/usecaseKvManager.xml

if BUILDSYSTEM_OPENWRT
libvui_intf_la_LIBADD += -lglib-2.0
endif