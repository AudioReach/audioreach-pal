pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = pal-headers.pc
EXTRA_DIST = $(pkgconfig_DATA)

h_sources = ${top_srcdir}/PalDefs.h \
            ${top_srcdir}/PalApi.h \
            ${top_srcdir}/PalMappings.h \
            ${top_srcdir}/PalARDefs.h

library_includedir = $(includedir)/pal
library_include_HEADERS = $(h_sources)
