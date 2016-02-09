#
#
#

### Subdirectories
APPDIR		= app
CMSIS			= cmsis
FREERTOS 	= freertos
PLIB			= plib

include common.mk

SUBDIRS = $(APPDIR) $(CMSIS) $(FREERTOS) $(PLIB)

.PHONY: subdirs $(SUBDIRS)

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

clean:
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean; \
	done
