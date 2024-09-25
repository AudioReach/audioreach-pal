pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = dev_speaker_mic.pc
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


dev_speaker_mic_sources = ./src/SpeakerMic.cpp

library_includedir = $(includedir)/dev_speaker_mic

lib_LTLIBRARIES     = libdev_speaker_mic.la
libdev_speaker_mic_la_SOURCES   = $(dev_speaker_mic_sources)
libdev_speaker_mic_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ltinyalsa -ltinycompress
libdev_speaker_mic_la_CPPFLAGS := $(AM_CPPFLAGS)
libdev_speaker_mic_la_CPPFLAGS += -std=c++14
libdev_speaker_mic_la_LDFLAGS   = -shared -avoid-version
libdev_speaker_mic_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libdev_speaker_mic_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libdev_speaker_mic_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libdev_speaker_mic_la_LIBADD += -lglib-2.0
endif
