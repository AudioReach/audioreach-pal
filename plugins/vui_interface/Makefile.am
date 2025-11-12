pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = vui_interface.pc
EXTRA_DIST = $(pkgconfig_DATA)

h_sources := $(top_srcdir)/inc/SVAExtension.h
h_sources += $(top_srcdir)/inc/SVAInterface.h


AM_CPPFLAGS := -DVUI_USE_SYSLOG @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
AM_CPPFLAGS += -I $(top_srcdir)/src/
AM_CPPFLAGS += -I $(top_srcdir)/inc/

library_include_HEADERS = $(h_sources)
library_includedir = $(includedir)/

AM_CPPFLAGS += @PALHEADERS_CFLAGS@
AM_CPPFLAGS += @SPFHEADER_CFLAGS@
AM_CPPFLAGS += @KVH2XML_CFLAGS@
AM_CPPFLAGS += @AUDIOLISTEN_CFLAGS@
AM_CPPFLAGS += @AROSAL_CFLAGS@
AM_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\)


c_sources := $(top_srcdir)/src/SoundTriggerUtils.cpp
c_sources += $(top_srcdir)/src/SVAExtension.cpp
c_sources += $(top_srcdir)/src/SVAInterface.cpp
c_sources += $(top_srcdir)/src/VoiceUIInterface.cpp

lib_LTLIBRARIES = libvuiinterface.la
libvuiinterface_la_SOURCES = $(c_sources)
libvuiinterface_la_CPPFLAGS = $(AM_CPPFLAGS)
libvuiinterface_la_LDFLAGS  = -shared -avoid-version
libvuiinterface_la_CPPFLAGS += -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat
libsvapdk7vendor_la_LIBADD = -lar_osal
