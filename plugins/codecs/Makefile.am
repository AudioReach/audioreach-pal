pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = codecs.pc
EXTRA_DIST = $(pkgconfig_DATA)

library_includedir = $(includedir)/codecs
lib_LTLIBRARIES     = libbt_bundle.la

bt_bundle_sources = \
              bt_base.c \
	      bt_bundle.c

libbt_bundle_la_CPPFLAGS = -I $(top_srcdir)/plugins/codecs/inc
libbt_bundle_la_SOURCES   = $(bt_bundle_sources)
libbt_bundle_la_LIBADD    = @GLIB_LIBS@ -lexpat -ldl
libbt_bundle_la_LDFLAGS = -shared -avoid-version
libbt_bundle_la_CPPFLAGS += @GLIB_CFLAGS@ @SPF_CFLAGS@ -include glib.h
libbt_bundle_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libbt_bundle_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_QLI
libbt_bundle_la_CPPFLAGS +=   -DPAL_USE_SYSLOG -DPAL_MEMLOG_UNSUPPORTED -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif


if BUILDSYSTEM_OPENWRT
libbt_bundle_la_LIBADD += -lglib-2.0
endif

if USE_SYSLOG
libbt_bundle_la_CPPFLAGS += -DPAL_USE_SYSLOG
endif

library_includedir = $(includedir)/codecs
lib_LTLIBRARIES     += libbt_ble.la

bt_ble_sources = \
              bt_base.c \
	      bt_ble.c

libbt_ble_la_CPPFLAGS = -I $(top_srcdir)/plugins/codecs/inc
libbt_ble_la_SOURCES   = $(bt_ble_sources)
libbt_ble_la_LIBADD    = @GLIB_LIBS@ -lexpat -ldl
libbt_ble_la_LDFLAGS = -shared -avoid-version
libbt_ble_la_CPPFLAGS += @GLIB_CFLAGS@ @SPF_CFLAGS@ -include glib.h
libbt_ble_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libbt_ble_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if USE_SYSLOG
libbt_ble_la_CPPFLAGS += -DPAL_USE_SYSLOG
endif

if BUILDSYSTEM_QLI
libbt_ble_la_CPPFLAGS += -DPAL_USE_SYSLOG -DPAL_MEMLOG_UNSUPPORTED -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if BUILDSYSTEM_OPENWRT
libbt_ble_la_LIBADD += -lglib-2.0
endif

library_includedir = $(includedir)/codecs
lib_LTLIBRARIES     += libbt_aptx.la

bt_aptx_sources = \
              bt_base.c \
	      bt_aptx.c

libbt_aptx_la_CPPFLAGS = -I $(top_srcdir)/plugins/codecs/inc
libbt_aptx_la_SOURCES   = $(bt_aptx_sources)
libbt_aptx_la_LIBADD    = @GLIB_LIBS@ -lexpat -ldl
libbt_aptx_la_LDFLAGS = -shared -avoid-version
libbt_aptx_la_CPPFLAGS += @GLIB_CFLAGS@ @SPFH_CFLAGS@ -include glib.h
libbt_aptx_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libbt_aptx_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_QLI
libbt_ble_la_CPPFLAGS += -DPAL_USE_SYSLOG -DPAL_MEMLOG_UNSUPPORTED -DATRACE_UNSUPPORTED -DCARD_STATE_UNSUPPORTED -DPAL_SIGNAL_HANDLER_UNSUPPORTED -DA2DP_SINK_SUPPORTED -DVUI_DMGR_AUDIO_UNSUPPORTED -DAUDIO_FEATURE_STATS_UNSUPPORTED
endif

if USE_SYSLOG
libbt_aptx_la_CPPFLAGS += -DPAL_USE_SYSLOG
endif

if BUILDSYSTEM_OPENWRT
libbt_aptx_la_LIBADD += -lglib-2.0
endif
