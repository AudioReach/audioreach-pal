pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = stream_calltranslation.pc
EXTRA_DIST = $(pkgconfig_DATA)


if BUILDSYSTEM_OPENWRT
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

stream_calltranslation_sources = ./src/StreamCallTranslation.cpp \
                                 ./src/CallTranslationNMTEngine.cpp
else
AM_CPPFLAGS := -I $(top_srcdir)/stream/StreamCommon/StreamSensorPCMData/inc
AM_CPPFLAGS := -I $(top_srcdir)/stream/StreamCallTranslation/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/StreamCommon/inc
AM_CPPFLAGS += -I $(top_srcdir)/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/inc
AM_CPPFLAGS += -I $(top_srcdir)
AM_CPPFLAGS += -I $(top_srcdir)/utils/inc
AM_CPPFLAGS += -I $(top_srcdir)/device/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/inc
AM_CPPFLAGS += -I $(top_srcdir)/stream/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/SessionAR/inc
AM_CPPFLAGS += -I $(top_srcdir)/plugins/PluginManager/inc
AM_CPPFLAGS += -I $(top_srcdir)/resource_manager/inc
AM_CPPFLAGS += -I $(top_srcdir)/context_manager/inc
AM_CPPFLAGS += -I $(top_srcdir)/plugins/vui_interface/api/vui-interface
AM_CPPFLAGS += -I $(top_srcdir)/plugins/codecs
AM_CPPFLAGS += @SPF_CFLAGS@
AM_CPPFLAGS += @AROUTE_CFLAGS@
AM_CPPFLAGS += @KVH2XML_CFLAGS@

stream_calltranslation_sources = $(top_srcdir)/stream/StreamCallTranslation/src/StreamCallTranslation.cpp
stream_calltranslation_sources += $(top_srcdir)/stream/StreamCallTranslation/src/CallTranslationNMTEngine.cpp
endif



library_includedir = $(includedir)/stream_calltranslation

lib_LTLIBRARIES     = libstream_calltranslation.la
libstream_calltranslation_la_SOURCES   = $(stream_calltranslation_sources)
libstream_calltranslation_la_LIBADD    = @GLIB_LIBS@ -lexpat -L${abs_top_builddir}/stream/StreamCommon/.libs -lstream_common
libstream_calltranslation_la_LIBADD   += -L${abs_top_builddir}/session/SessionAR/.libs -lsession_ar
libstream_calltranslation_la_CPPFLAGS := $(AM_CPPFLAGS)
libstream_calltranslation_la_CPPFLAGS += -std=c++14
libstream_calltranslation_la_LDFLAGS   = -shared -avoid-version
libstream_calltranslation_la_CPPFLAGS += @GLIB_CFLAGS@ -include glib.h
libstream_calltranslation_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libstream_calltranslation_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libstream_calltranslation_la_LIBADD += -lglib-2.0
else
libstream_calltranslation_la_CPPFLAGS +=  -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if USE_SYSLOG
libstream_calltranslation_la_CPPFLAGS += -DPAL_USE_SYSLOG
endif