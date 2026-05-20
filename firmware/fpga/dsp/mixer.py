#
# This file is part of HackRF.
#
# Copyright (c) 2025 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from amaranth               import Module, Signal
from amaranth.lib           import wiring, stream
from amaranth.lib.wiring    import In, Out

from dsp.fir_mac16          import iCE40Multiplier
from dsp.round              import convergent_round


class ComplexMultiplier(wiring.Component):

    def __init__(self, a_shape, b_shape, c_shape, always_ready=False):
        super().__init__({
            "a": In(stream.Signature(a_shape, always_ready=always_ready)),
            "b": In(stream.Signature(b_shape, always_ready=always_ready)),
            "c": Out(stream.Signature(c_shape, always_ready=always_ready)),
        })
        self.always_ready = always_ready

    def elaborate(self, platform):
        m = Module()

        A = self.a.p.i
        B = self.a.p.q
        C = self.b.p.i
        D = self.b.p.q

        a_width = A.shape().width
        b_width = B.shape().width
        c_width = C.shape().width
        d_width = D.shape().width

        o_shape = (A * C).shape()
        o_width = o_shape.width
        o_shape2 = ((A+B)*(C+D)).shape()
        o_width2 = o_shape2.width

        m.submodules.dsp1 = dsp1 = iCE40Multiplier(a_width=a_width, b_width=c_width, p_width=0, o_width=o_width, always_ready=self.always_ready)
        m.submodules.dsp2 = dsp2 = iCE40Multiplier(a_width=b_width, b_width=d_width, p_width=0, o_width=o_width, always_ready=self.always_ready)
        m.submodules.dsp3 = dsp3 = iCE40Multiplier(a_width=a_width+1, b_width=d_width+1, p_width=0, o_width=o_width2, always_ready=self.always_ready)

        valid_in = self.a.valid & self.b.valid
        m.d.comb += [
            dsp1.a.eq(A),
            dsp1.b.eq(C),
            dsp1.valid_in.eq(valid_in),
            dsp2.a.eq(B),
            dsp2.b.eq(D),
            dsp2.valid_in.eq(valid_in),
            dsp3.a.eq(A+B),
            dsp3.b.eq(C+D),
            dsp3.valid_in.eq(valid_in),
        ]

        ready_in = dsp1.ready_in
        if not self.always_ready:
            m.d.comb += self.a.ready.eq(ready_in & self.b.valid)
            m.d.comb += self.b.ready.eq(ready_in & self.a.valid)

        mid_valid = Signal()
        mid_ready = Signal()
        mid_sum0 = Signal(o_shape)
        mid_sum1 = Signal(o_shape)
        mid_dsp2 = Signal(o_shape)
        with m.If(~mid_valid | mid_ready):
            m.d.sync += mid_valid.eq(dsp1.valid_out)
            with m.If(mid_valid):
                m.d.sync += mid_sum0.eq(dsp1.o - dsp2.o)
                m.d.sync += mid_sum1.eq(dsp3.o - dsp1.o)
                m.d.sync += mid_dsp2.eq(dsp2.o)
            m.d.comb += dsp1.ready_out.eq(1)
            m.d.comb += dsp2.ready_out.eq(1)
            m.d.comb += dsp3.ready_out.eq(1)

        with m.If(~self.c.valid | self.c.ready):
            m.d.sync += self.c.valid.eq(mid_valid)
            with m.If(mid_valid):
                m.d.sync += self.c.p.i.eq(convergent_round(mid_sum0, o_width - self.c.p.i.shape().width - 1))
                m.d.sync += self.c.p.q.eq(convergent_round(mid_sum1 - mid_dsp2, o_width - self.c.p.q.shape().width - 1))
            m.d.comb += mid_ready.eq(1)
        
        return m

