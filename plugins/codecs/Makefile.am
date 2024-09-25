pkgconfigdir = $(libdir)/pkgconfig$
pkgconfig_DATA = codecs.pc
EXTRA_DIST = $(pkgconfig_DATA)$

lib_LTLIBRARIES     = libbt_bundle.la

bt_bundle_sources = \
              bt_base.c \
	      bt_bundle.c

libbt_bundle_la_CPPFLAGS = -I ./inc
libbt_bundle_la_SOURCES   = $(bt_bundle_sources)
libbt_bundle_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ldl
libbt_bundle_la_LDFLAGS = -shared -avoid-version
libbt_bundle_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libbt_bundle_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libbt_bundle_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libbt_bundle_la_LIBADD += -lglib-2.0
endif


lib_LTLIBRARIES     += libbt_ble.la

bt_ble_sources = \
              bt_base.c \
	      bt_ble.c

libbt_ble_la_CPPFLAGS = -I ./inc
libbt_ble_la_SOURCES   = $(bt_ble_sources)
libbt_ble_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ldl
libbt_ble_la_LDFLAGS = -shared -avoid-version
libbt_ble_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libbt_ble_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libbt_ble_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libbt_ble_la_LIBADD += -lglib-2.0
endif


lib_LTLIBRARIES     += libbt_aptx.la

bt_aptx_sources = \
              bt_base.c \
	      bt_aptx.c

libbt_aptx_la_CPPFLAGS = -I ./inc
libbt_aptx_la_SOURCES   = $(bt_aptx_sources)
libbt_aptx_la_LIBADD    = @GLIB_LIBS@ -llog -lcutils -lexpat -ldl
libbt_aptx_la_LDFLAGS = -shared -avoid-version
libbt_aptx_la_CPPFLAGS += @GLIB_CFLAGS@ -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libbt_aptx_la_CPPFLAGS += -DACD_SM_FILEPATH=\"/etc/models/acd/\"
libbt_aptx_la_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\) -DLINUX_ENABLED

if BUILDSYSTEM_OPENWRT
libbt_aptx_la_LIBADD += -lglib-2.0
endif
