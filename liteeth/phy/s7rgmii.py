#
# This file is part of LiteEth.
#
# Copyright (c) 2015-2023 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# RGMII PHY for 7-Series Xilinx FPGA

import math

from migen import *
from migen.genlib.cdc import PulseSynchronizer
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.cores.clock.xilinx_s7 import S7MMCM
from litex.soc.cores.clock import S7PLL

from liteeth.common import *
from liteeth.phy.common import *


# Constants ----------------------------------------------------------------------------------------

speeds = {
    "10"   : 0,
    "100"  : 1,
    "1000" : 2,
}

# LiteEth PHY RGMII TX -----------------------------------------------------------------------------

class LiteEthPHYRGMIITX(LiteXModule):
    pll_clk_speed = 125e6
    def __init__(self, pads, speed=speeds["1000"], cm_buf_type="BUFH"):
        self.sink = sink = stream.Endpoint(eth_phy_description(8))

        # Create clock counters
        def divider(div, size):
            counter = Signal(size)
            signal1 = Signal(reset=1)
            signal2 = Signal(reset=1)
            overflow = Signal()

            limit = int(div - 1)
            half1 = int((div+1)/2)
            half2 = int(div/2)
            self.comb += [
                overflow.eq(counter == limit),
                signal1.eq(counter < half1),
                signal2.eq(counter < half2)
            ]
            self.sync.eth_tx_full += If(overflow, [
                counter.eq(0),
            ]).Else([
                counter.eq(counter + 1),
            ])

            return counter, overflow, signal1, signal2

        clock_speeds = {speeds["1000"]: (125e6, 125e6), speeds["100"]: (25e6, 12.5e6), speeds["10"]: (2.5e6, 1.25e6)}  # (Ext signal frequency, Int clock frequency)
        counter_size = math.floor(math.log2(self.pll_clk_speed/min([min(x) for x in clock_speeds.values()]))) + 1
        dividers = {k: (divider(self.pll_clk_speed/v[0], counter_size), divider(self.pll_clk_speed/v[1], counter_size)) for k,v in clock_speeds.items()}
        self.ext_clk_counter = Signal()
        self.ext_clk_overflow = Signal()
        self.ext_clk_signal1 = Signal()
        self.ext_clk_signal2 = Signal()
        self.comb += Case(speed, {k: [self.ext_clk_counter.eq(v[0][0]), self.ext_clk_overflow.eq(v[0][1]), self.ext_clk_signal1.eq(v[0][2]), self.ext_clk_signal2.eq(v[0][3])] for k,v in dividers.items()})
        self.int_clk_counter = Signal()
        self.int_clk_overflow = Signal()
        self.int_clk_signal1 = Signal()
        self.int_clk_signal2 = Signal()
        self.comb += Case(speed, {k: [self.int_clk_counter.eq(v[1][0]), self.int_clk_overflow.eq(v[1][1]), self.int_clk_signal1.eq(v[1][2]), self.int_clk_signal2.eq(v[1][3])] for k,v in dividers.items()})

        # # #
        tx_ctl_obuf  = Signal()
        self.specials += [
            Instance("ODDR",
                p_DDR_CLK_EDGE = "SAME_EDGE",
                i_C  = ClockSignal("eth_tx_full"),
                i_CE = 1,
                i_S  = 0,
                i_R  = 0,
                i_D1 = sink.valid,
                i_D2 = sink.valid,
                o_Q  = tx_ctl_obuf,
            ),
            Instance("OBUF",
                i_I = tx_ctl_obuf,
                o_O = pads.tx_ctl,
            ),
        ]

        tx_data_obuf = Signal(4)
        for i in range(4):
            d1 = Signal()
            d2 = Signal()
            self.comb += [
                d1.eq(Mux(self.int_clk_signal1, sink.data[i], sink.data[4+i])),
                d2.eq(Mux(self.int_clk_signal2, sink.data[i], sink.data[4+i])),
            ]
            self.specials += [
                Instance("ODDR",
                    p_DDR_CLK_EDGE = "SAME_EDGE",
                    i_C  = ClockSignal("eth_tx_full"),
                    i_CE = 1,
                    i_S  = 0,
                    i_R  = 0,
                    i_D1 = d1,
                    i_D2 = d2,
                    o_Q  = tx_data_obuf[i],
                ),
                Instance("OBUF",
                    i_I = tx_data_obuf[i],
                    o_O = pads.tx_data[i],
                )
            ]
        self.comb += sink.ready.eq(1)

        clk_d1, clk_d2 = Signal(), Signal()
        self.sync.eth_tx_full += [
            clk_d1.eq(self.ext_clk_signal1),
            clk_d2.eq(self.ext_clk_signal2),
        ]

        self.clk_oddr = Instance("ODDR",
            p_DDR_CLK_EDGE = "SAME_EDGE",
            i_C  = ClockSignal("eth_tx_delayed"),
            i_CE = 1,
            i_S  = 0,
            i_R  = 0,
            i_D1 = clk_d1,
            i_D2 = clk_d2,
            o_Q  = ClockSignal("eth_tx_io"),
        )
        self.specials += [
            Instance(f"{cm_buf_type}CE",
                i_I=ClockSignal("eth_tx_full"),
                i_CE=self.int_clk_overflow,
                o_O=ClockSignal("eth_tx")
            ),
            self.clk_oddr
        ]

# LiteEth PHY RGMII RX -----------------------------------------------------------------------------

class LiteEthPHYRGMIIRX(LiteXModule):
    def __init__(self, pads, rx_delay=2e-9, iodelay_clk_freq=200e6, speed=speeds["1000"], cm_buf_type="BUFG"):
        self.source = source = stream.Endpoint(eth_phy_description(8))

        low_speed = Signal()
        self.comb += low_speed.eq(speed != speeds["1000"])

        # # #

        assert iodelay_clk_freq in [200e6, 300e6, 400e6]
        iodelay_tap_average = 1 / (2*32 * iodelay_clk_freq)
        rx_delay_taps = round(rx_delay / iodelay_tap_average)
        assert rx_delay_taps < 32, "Exceeded IDELAYE2 max value: {} >= 32".format(rx_delay_taps)

        self.rx_ctl_ibuf      = rx_ctl_ibuf      = Signal()
        self.rx_ctl_idelay    = rx_ctl_idelay    = Signal()
        self.rx_ctl           = rx_ctl           = Signal()
        self.rx_data_ibuf     = rx_data_ibuf     = Signal(4)
        self.rx_data_idelay   = rx_data_idelay   = Signal(4)
        self.rx_data_buffer   = rx_data_buffer   = Signal(8)
        self.rx_data_previous = rx_data_previous = Signal(8)
        self.rx_data          = rx_data          = Signal(8)

        self.specials += [
            Instance("IBUF", i_I=pads.rx_ctl, o_O=rx_ctl_ibuf),
            Instance("IDELAYE2",
                p_IDELAY_TYPE      = "FIXED",
                p_IDELAY_VALUE     = rx_delay_taps,
                p_REFCLK_FREQUENCY = iodelay_clk_freq/1e6,
                i_C        = 0,
                i_LD       = 0,
                i_CE       = 0,
                i_LDPIPEEN = 0,
                i_INC      = 0,
                i_IDATAIN  = rx_ctl_ibuf,
                o_DATAOUT  = rx_ctl_idelay,
            ),
            Instance("IDDR",
                p_DDR_CLK_EDGE = "SAME_EDGE_PIPELINED",
                i_C  = ClockSignal("eth_rx_io"),
                i_CE = 1,
                i_S  = 0,
                i_R  = 0,
                i_D  = rx_ctl_idelay,
                o_Q1 = rx_ctl,
                o_Q2 = Signal(),
            )
        ]

        for i in range(4):
            self.specials += [
                Instance("IBUF",
                    i_I = pads.rx_data[i],
                    o_O = rx_data_ibuf[i],
                ),
                Instance("IDELAYE2",
                    p_IDELAY_TYPE      = "FIXED",
                    p_IDELAY_VALUE     = rx_delay_taps,
                    p_REFCLK_FREQUENCY = iodelay_clk_freq/1e6,
                    i_C        = 0,
                    i_LD       = 0,
                    i_CE       = 0,
                    i_LDPIPEEN = 0,
                    i_INC      = 0,
                    i_IDATAIN  = rx_data_ibuf[i],
                    o_DATAOUT  = rx_data_idelay[i],
                ),
                Instance("IDDR",
                    p_DDR_CLK_EDGE = "SAME_EDGE_PIPELINED",
                    i_C  = ClockSignal("eth_rx_io"),
                    i_CE = 1,
                    i_S  = 0,
                    i_R  = 0,
                    i_D  = rx_data_idelay[i],
                    o_Q1 = rx_data_buffer[i],
                    o_Q2 = rx_data_buffer[i+4],
                )
            ]

            # Save previous nibble
            self.sync.eth_rx_full += [
                rx_data_previous[i].eq(rx_data_buffer[i]),
            ]

            # Use rising clock edge of current and previous one when speed is not 1000mbits
            self.comb += rx_data[i].eq(Mux(low_speed, rx_data_previous[i], rx_data_buffer[i]))
            self.comb += rx_data[i+4].eq(Mux(low_speed, rx_data_buffer[i], rx_data_buffer[i+4]))

        # Detect rising on rx_ctl
        first = Signal()
        rx_ctl_first = Signal()
        self.comb += first.eq(rx_ctl & ~rx_ctl_first)
        self.sync.eth_rx_full += rx_ctl_first.eq(rx_ctl),

        # Divide by 2: On first clock after rx_ctl, force the next value (next nibble) to 1
        eth_rx_buffer = Signal()
        self.sync.eth_rx_full += If(first, eth_rx_buffer.eq(1)).Else(eth_rx_buffer.eq(~eth_rx_buffer))

        # Always enable clock when speed is 1000mbits, otherwise force reset on first clock after rx_ctl rising and use clock divided by 2
        ce_ff = Signal()
        self.comb += ce_ff.eq(Mux(low_speed, Mux(first, 0, eth_rx_buffer), 1))

        # Create the internal clock
        self.specials += Instance(f"{cm_buf_type}CE",
            i_I=ClockSignal("eth_rx_full"),
            i_CE=ce_ff,
            o_O=ClockSignal("eth_rx")
        )

        last = Signal()
        rx_ctl_last = Signal()
        self.comb += last.eq(~rx_ctl & rx_ctl_last)
        self.sync += [
            rx_ctl_last.eq(rx_ctl),
            source.valid.eq(rx_ctl),
            source.data.eq(rx_data)
        ]
        self.comb += source.last.eq(last)

# LiteEth PHY RGMII CRG ----------------------------------------------------------------------------

class LiteEthPHYRGMIICRG(LiteXModule):
    pll_clk_speed = 125e6
    def __init__(self, clock_pads, pads, with_hw_init_reset, tx_delay=2e-9, hw_reset_cycles=256, clk_freq=100e6, cm_type="PLL"):
        self._reset = CSRStorage()

        # RX clock.
        self.cd_eth_rx_io = ClockDomain()
        self.cd_eth_rx_full = ClockDomain()
        self.cd_eth_rx      = ClockDomain()
        rx_clk_ibuf  = Signal()
        self.specials += [
            Instance("IBUF",
                i_I = clock_pads.rx,
                o_O = rx_clk_ibuf,
            ),
            Instance("BUFR",
                i_I = rx_clk_ibuf,
                o_O = self.cd_eth_rx_full.clk,
            ),
            Instance("BUFIO",
                i_I = rx_clk_ibuf,
                o_O = self.cd_eth_rx_io.clk,
            ),
        ]

        # TX clock.
        self.cd_eth_tx         = ClockDomain()
        self.cd_eth_tx_full    = ClockDomain()
        self.cd_eth_tx_delayed = ClockDomain(reset_less=True)
        self.cd_eth_tx_io      = ClockDomain(reset_less=True)

        # PLL
        tx_phase = self.pll_clk_speed*tx_delay*360
        assert tx_phase < 360
        self.pll = pll = {"PLL": S7PLL, "MMCM": S7MMCM}[cm_type]()
        pll.register_clkin(ClockSignal("sys"),    clk_freq)
        pll.create_clkout(self.cd_eth_tx_full,    self.pll_clk_speed)                  # 125 Mhz
        pll.create_clkout(self.cd_eth_tx_delayed, self.pll_clk_speed, phase=tx_phase)  # 125 Mhz delayed TX

        self.specials += [
            Instance("OBUF",
                i_I = self.cd_eth_tx_io.clk,
                o_O = clock_pads.tx,
            )
        ]

        # Reset
        self.reset = reset = Signal()
        if with_hw_init_reset:
            self.hw_reset = LiteEthPHYHWReset(cycles=hw_reset_cycles)
            self.comb += reset.eq(self._reset.storage | self.hw_reset.reset)
        else:
            self.comb += reset.eq(self._reset.storage)
        if hasattr(pads, "rst_n"):
            self.comb += pads.rst_n.eq(~reset)
        self.specials += [
            AsyncResetSynchronizer(self.cd_eth_tx, reset),
            AsyncResetSynchronizer(self.cd_eth_rx, reset),
        ]

# LiteEth PHY RGMII Speed Detection --------------------------------------------------------------

class LiteEthPHYRGMIISpeedDetection(LiteXModule):
    def __init__(self, clk_freq):
        self.speed  = Signal(2)
        self._speed = CSRStatus(2)

        # # #

        speed        = Signal(2)
        update_speed = Signal()
        self.sync += If(update_speed, self.speed.eq(speed))
        self.comb += self._speed.status.eq(self.speed)

        # Principle:
        #  sys_clk >= 125MHz.
        #  eth_rx  <= 125Mhz.
        # We generate ticks every 1024 clock cycles in eth_rx domain
        # and measure ticks period in sys_clk domain.

        # Generate a tick every 1024 clock cycles (eth_rx clock domain).
        eth_tick    = Signal()
        eth_counter = Signal(10, reset_less=True)
        self.sync.eth_rx_full += eth_counter.eq(eth_counter + 1)
        self.comb += eth_tick.eq(eth_counter == 0)

        # Synchronize tick (sys clock domain).
        self.sys_tick = sys_tick   = Signal()
        self.eth_ps   = eth_ps     = PulseSynchronizer("eth_rx_full", "sys")
        self.comb += [
            eth_ps.i.eq(eth_tick),
            sys_tick.eq(eth_ps.o)
        ]

        # sys_clk domain counter.
        self.sys_counter       = sys_counter       = Signal(24, reset_less=True)
        self.sys_counter_reset = sys_counter_reset = Signal()
        self.sys_counter_ce    = sys_counter_ce    = Signal()
        self.sync += [
            If(sys_counter_reset,
               sys_counter.eq(0)
            ).Elif(sys_counter_ce,
                sys_counter.eq(sys_counter + 1)
            )
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            sys_counter_reset.eq(1),
            If(sys_tick,
                NextState("COUNT")
            )
        )
        fsm.act("COUNT",
            sys_counter_ce.eq(1),
            If(sys_tick,
                NextState("DETECTION")
            )
        )
        fsm.act("DETECTION",
            update_speed.eq(1),
            If(sys_counter > int(0.95 * clk_freq / 2_500_000 * 1024),
                speed.eq(speeds["10"])   # 2.5 MHz
            ).Elif(sys_counter > int(0.95 * clk_freq / 25_000_000 * 1024),
                speed.eq(speeds["100"])   # 25 MHz
            ).Else(
                speed.eq(speeds["1000"])   # 125 MHz
            ),
            NextState("IDLE")
        )

# LiteEth PHY RGMII --------------------------------------------------------------------------------

class LiteEthPHYRGMII(LiteXModule):
    dw          = 8
    tx_clk_freq = 125e6
    rx_clk_freq = 125e6
    def __init__(self, clock_pads, pads, with_hw_init_reset=True, tx_delay=2e-9, rx_delay=2e-9,
            iodelay_clk_freq=200e6, hw_reset_cycles=256, clk_freq=100e6, cm_type="PLL", tx_cm_buf_type="BUFH", rx_cm_buf_type="BUFG"):
        self.clock_pads = clock_pads
        self.pads = pads
        self.speed_detection = LiteEthPHYRGMIISpeedDetection(clk_freq)
        speed = self.speed_detection.speed
        self.crg = LiteEthPHYRGMIICRG(clock_pads, pads, with_hw_init_reset, tx_delay, hw_reset_cycles, clk_freq, cm_type)
        self.tx  = ClockDomainsRenamer("eth_tx")(LiteEthPHYRGMIITX(pads, speed, tx_cm_buf_type))
        self.rx  = ClockDomainsRenamer("eth_rx")(LiteEthPHYRGMIIRX(pads, rx_delay, iodelay_clk_freq, speed, rx_cm_buf_type))
        self.sink, self.source = self.tx.sink, self.rx.source

        if hasattr(pads, "mdc"):
            self.mdio = LiteEthPHYMDIO(pads)

    def add_timing_constraints(self, platform, sys_clk, min_t_setup=1.0e-9, min_t_hold=1.0e-9):
        rx_period = 1e9/self.rx_clk_freq
        platform.add_platform_command("create_clock -name {{clk}} -period {} [get_ports {{clk}}]".format(str(rx_period)), clk=self.clock_pads.rx)
        platform.add_platform_command("create_generated_clock -name {clk} -source [get_pins {source}/C] -divide_by 1 [get_ports {clk}]",  source=self.tx.clk_oddr, clk=self.clock_pads.tx)
        platform.add_false_path_constraints(sys_clk, self.clock_pads.rx)

        max_ns = (rx_period / 2) - (min_t_setup / 1.0e-9)
        min_ns = min_t_hold / 1.0e-9
        for signal in [self.pads.rx_data, self.pads.rx_ctl]:
            for edge in ["", "-clock_fall "]:
                card = '[*]' if signal.nbits > 1 else ''
                platform.add_platform_command("set_input_delay -clock [get_clocks {{clk}}] {}-min -add_delay {} [get_ports {{signal}}{}]".format(edge, min_ns, card), clk=self.clock_pads.rx, signal=signal)
                platform.add_platform_command("set_input_delay -clock [get_clocks {{clk}}] {}-max -add_delay {} [get_ports {{signal}}{}]".format(edge, max_ns, card), clk=self.clock_pads.rx, signal=signal)

        max_ns = min_t_setup / 1.0e-9
        min_ns = -min_t_hold / 1.0e-9
        for signal in [self.pads.tx_data, self.pads.tx_ctl]:
            for edge in ["", "-clock_fall "]:
                card = '[*]' if signal.nbits > 1 else ''
                platform.add_platform_command("set_output_delay -clock [get_clocks {{clk}}] {}-min -add_delay {} [get_ports {{signal}}{}]".format(edge, min_ns, card), clk=self.clock_pads.tx, signal=signal)
                platform.add_platform_command("set_output_delay -clock [get_clocks {{clk}}] {}-max -add_delay {} [get_ports {{signal}}{}]".format(edge, max_ns, card), clk=self.clock_pads.tx, signal=signal)