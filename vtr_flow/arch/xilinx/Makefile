ARCH = $(wildcard *.xml)
RRG = $(ARCH:%.xml=%.rrg.gz)
DEVICES = xc6vlx240t
DBS = $(DEVICES) Virtex6
DBFILES = $(addsuffix .db,$(DBS))
PTCFILES = $(addsuffix .ptc2name,$(DEVICES))
PKG = $(ARCH:%.xml=%.pkg)
BENCH = majority.blif

XDLRC2VPR = ../../../xdlrc2vpr/xdlrc2vpr

all: $(PKG) $(RRG)

%.pkg: %.xml partgen
	partgen -p $(@:%.pkg=%)
	rm -f partlist.xct partlist.xml

.PHONY: partlist.rrg.gz
%.rrg.gz: %.xml $(BENCH) $(DBFILES) $(XDLRC2VPR)
	rm -f $@
	../../../vpr/vpr $(word 1,$^) $(word 2,$^) --route_chan_width 18 --dump_vtb_rrg --nodisp
	$(eval WIDTH := $(shell GREP_OPTIONS="" grep -Po '<layout width="\K[^"]+' $(word 1,$^)))
	$(eval HEIGHT := $(shell GREP_OPTIONS="" grep -Po '<layout width="[^"]+" height="\K[^"]+' $(word 1,$^)))
	( /usr/bin/time -v $(XDLRC2VPR) $(@:%.rrg.gz=%) $(WIDTH) $(HEIGHT) 2>&1 ) | tee xdlrc2vpr.out
	cat $(@:%.rrg.gz=%.header_and_shim.gz) $(@:%.rrg.gz=%.nodes.gz) $(@:%.rrg.gz=%.edge_switch.gz) > $@
	rm $(@:%.rrg.gz=%.header_and_shim.gz) \
		$(@:%.rrg.gz=%.nodes.gz) \
		$(@:%.rrg.gz=%.edge_switch.gz) \
		$(@:%.rrg.gz=%.vpr) \
		*.net

$(XDLRC2VPR):
	$(MAKE) -C $(dir XDLRC2VPR)

$(DBFILES):
	ln -sf ../../../torc/src/torc/devices/$@ .

$(BENCH):
	ln -sf ../../benchmarks/blif/6/$@ .

partgen:
ifeq ($(shell which partgen),)
	$(error "Xilinx ISE tool 'partgen' not found on $$PATH, please source Xilinx settings script!")
endif

clean:
	rm -f *.rrg.gz *.echo *.net *.vpr *.out *.log *.pkg *.tws *.tws.txt partlist.xct partlist.xml $(DBFILES) $(BENCH)
