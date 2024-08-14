pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = session_pcm.pc
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


session_pcm_sources = ./src/SessionAlsaPcm.cpp

library_includedir = $(includedir)/session_pcm

lib_LTLIBRARIES     = libsession_pcm.la
libsession_pcm_la_SOURCES   = $(session_pcm_sources)
libsession_pcm_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ltinyalsa -ltinycompress
libsession_pcm_la_CPPFLAGS := $(AM_CPPFLAGS)
libsession_pcm_la_CPPFLAGS += -std=c++14
libsession_pcm_la_LDFLAGS   = -shared -avoid-version
libsession_pcm_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libsession_pcm_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libsession_pcm_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED
libsession_pcm_la_list      = ../../configs/$(MACHINE_ENABLED)/kvh2xml.xml \
                                 ../../configs/$(MACHINE_ENABLED)/usecaseKvManager.xml

if BUILDSYSTEM_OPENWRT
libsession_pcm_la_LIBADD += -lglib-2.0
endif