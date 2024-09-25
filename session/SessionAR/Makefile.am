pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = session_ar.pc
EXTRA_DIST = $(pkgconfig_DATA)


AM_CPPFLAGS := -I ./inc
AM_CPPFLAGS += -I ../../inc
AM_CPPFLAGS += -I ../../
AM_CPPFLAGS += -I ../../utils/inc
AM_CPPFLAGS += -I ../../device/inc
AM_CPPFLAGS += -I ../../session/inc
AM_CPPFLAGS += -I ../../stream/inc
AM_CPPFLAGS += -I ../../plugins/PluginManager/inc
AM_CPPFLAGS += -I ../../resource_manager/inc
AM_CPPFLAGS += -I ../../context_manager/inc


session_ar_sources = ./src/SessionAR.cpp \
                     ./src/SessionAlsaUtils.cpp \
                     ./src/PayloadBuilder.cpp

library_includedir = $(includedir)/session_ar

lib_LTLIBRARIES     = libsession_ar.la
libsession_ar_la_SOURCES   = $(session_ar_sources)
libsession_ar_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ltinyalsa -ltinycompress -lar-gsl
libsession_ar_la_LIBADD   += -L${abs_top_builddir}/.libs -lpal
libsession_ar_la_CPPFLAGS := $(AM_CPPFLAGS)
libsession_ar_la_CPPFLAGS += -std=c++14
libsession_ar_la_LDFLAGS   = -shared -avoid-version
libsession_ar_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libsession_ar_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libsession_ar_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libsession_ar_la_LIBADD += -lglib-2.0 -lagmclientwrapper
endif