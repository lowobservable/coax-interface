YOSYS ?= yosys
NEXTPNR ?= nextpnr-ice40
ICEPACK ?= icepack

all: top.bin

top.json: top.v coax_buffer.v coax_tx_bit_timer.v coax_tx.v coax_buffered_tx.v coax_tx_distorter.v coax_rx_bit_timer.v coax_rx.v coax_buffered_rx.v control.v fifo_sync_ram.v ram_sdp.v spi_device.v strobe_cdc.v

clean:
	rm -f *.json *.asc *.bin

%.json:
	$(YOSYS) -p 'synth_ice40 -top top -json $@' $^

%.asc: %.json pins.pcf
	$(NEXTPNR) --up5k --package sg48 --json $< --pcf pins.pcf --asc $@

%.bin: %.asc
	$(ICEPACK) $< $@

.PHONY: all clean
