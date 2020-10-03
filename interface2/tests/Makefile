IVERILOG ?= iverilog

RTL = ../rtl
TESTS = $(patsubst %.v,%,$(wildcard *_tb.v))

all: test

coax_buffered_rx_tb: coax_buffered_rx_tb.v $(RTL)/coax_buffered_rx.v $(RTL)/coax_rx.v $(RTL)/coax_rx_bit_timer.v $(RTL)/fifo_sync_ram.v $(RTL)/ram_sdp.v
coax_buffered_tx_tb: coax_buffered_tx_tb.v $(RTL)/coax_buffered_tx.v $(RTL)/coax_tx.v $(RTL)/coax_tx_bit_timer.v $(RTL)/fifo_sync_ram.v $(RTL)/ram_sdp.v
coax_rx_bit_timer_tb: coax_rx_bit_timer_tb.v $(RTL)/coax_rx_bit_timer.v
coax_rx_tb: coax_rx_tb.v $(RTL)/coax_rx.v $(RTL)/coax_rx_bit_timer.v
coax_tx_bit_timer_tb: coax_tx_bit_timer_tb.v $(RTL)/coax_tx_bit_timer.v
coax_tx_distorter_tb: coax_tx_distorter_tb.v $(RTL)/coax_tx_distorter.v
coax_tx_tb: coax_tx_tb.v $(RTL)/coax_tx.v $(RTL)/coax_tx_bit_timer.v
control_tb: control_tb.v $(RTL)/control.v

test: $(TESTS)
	$(foreach tb,$(TESTS),./run_test.sh $(tb);)

clean:
	rm -f *_tb *.vcd

%_tb:
	$(IVERILOG) -o $@ $^

.PHONY: all, clean
