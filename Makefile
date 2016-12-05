#
#
#

### Subdirectories
APPDIR		= app
CMSIS			= cmsis
FREERTOS 	= freertos
PLIB			= plib

include common.mk

SUBDIRS = $(CMSIS) $(PLIB) $(FREERTOS) $(APPDIR) 

.PHONY: subdirs $(SUBDIRS)

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

clean:
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean; \
	done
