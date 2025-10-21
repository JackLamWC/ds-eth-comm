FLASHDBINC = $(FLASHDBDIR)/fal/inc \
		     $(FLASHDBDIR)/inc \
			 $(FLASHDBDIR)

FLASHDBSRC = $(FLASHDBDIR)/fal/src/fal_flash.c \
			 $(FLASHDBDIR)/fal/src/fal_partition.c \
			 $(FLASHDBDIR)/fal/src/fal_rtt.c \
			 $(FLASHDBDIR)/fal/src/fal.c \
			 $(FLASHDBDIR)/src/fdb_file.c \
			 $(FLASHDBDIR)/src/fdb_kvdb.c \
			 $(FLASHDBDIR)/src/fdb_tsdb.c \
			 $(FLASHDBDIR)/src/fdb_utils.c \
			 $(FLASHDBDIR)/src/fdb.c \
			 $(FLASHDBDIR)/fdb_port.c

ALLCSRC += $(FLASHDBSRC)
ALLINC += $(FLASHDBINC)