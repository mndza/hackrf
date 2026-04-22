#
# This file is part of HackRF.
#
# Copyright (c) 2026 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import math

from amaranth       import Elaboratable, Module, Signal, Mux, unsigned, signed
from amaranth.sim   import Simulator


def rad2int(angle, w):
    return round(angle * (1 << w) / (2*math.pi))

def deg2int(angle, w):
    return round(angle * (1 << w) / 360)


class TrivialRotation(Elaboratable):
    def __init__(self, w):
        self.w = w
        # inputs
        self.i_x = Signal(signed(w))
        self.i_y = Signal(signed(w))
        self.i_z = Signal(w)               # unsigned 0..360
        # outputs
        self.o_x = Signal(signed(w))
        self.o_y = Signal(signed(w))
        self.o_z = Signal(signed(w - 2))   # remaining angle +-45°

    def elaborate(self, platform):
        m = Module()

        w = self.w
        half_quad = (1 << (w - 3))    # 45°
        quadrant = (self.i_z + half_quad)[-3:-1]  # -1 because amaranth adds a bit!
        
        with m.Switch(quadrant):
            with m.Case(0b00):
                m.d.comb += self.o_x.eq( self.i_x)
                m.d.comb += self.o_y.eq( self.i_y)
            with m.Case(0b01):
                m.d.comb += self.o_x.eq(-self.i_y)
                m.d.comb += self.o_y.eq( self.i_x)
            with m.Case(0b10):
                m.d.comb += self.o_x.eq(-self.i_x)
                m.d.comb += self.o_y.eq(-self.i_y)
            with m.Case(0b11):
                m.d.comb += self.o_x.eq( self.i_y)
                m.d.comb += self.o_y.eq(-self.i_x)

        # Remaining angle: lower w-2 bits,
        # re-interpreted as signed to give [-45°, +45°].
        m.d.comb += self.o_z.eq(self.i_z[:-2].as_signed())
        
        return m


class FriendAngles(Elaboratable):
    """
    Stage 2: Friend Angles
    Kernel: [P0=25, P1=24+j7, P2=20+j15]
    Same abs = 25, so no rotation error.
    Angles: 0°, arctan(7/24) = 16.260°, arctan(15/20) = 36.870°
    After this stage the remaining angle is in [-16.260°, +16.260°].
    """
    def __init__(self, w):
        self.w = w
        W = w + 2  # 2 extra guard bits for the 25x scale growth
        self.i_x = Signal(signed(W))
        self.i_y = Signal(signed(W))
        self.i_z = Signal(signed(w - 2))   # remaining angle +-45°
        self.o_x = Signal(signed(W))
        self.o_y = Signal(signed(W))
        self.o_z = Signal(signed(w - 3))   # remaining angle +-16.260°

    def elaborate(self, platform):
        m = Module()
        PW = self.w - 2

        x = self.i_x
        y = self.i_y
        z = self.i_z

        # Scaled x.
        x16 = x << 4
        x8  = x << 3
        x4  = x << 2
        x25 = x16 + x8 + x
        x24 = x16 + x8
        x20 = x16 + x4
        x15 = x16 - x
        x7  = x8 - x

        # Scaled y.
        y16 = y << 4
        y8  = y << 3
        y4  = y << 2
        y25 = y16 + y8 + y
        y24 = y16 + y8
        y20 = y16 + y4
        y15 = y16 - y
        y7  = y8 - y

        OW = self.w + 5
        ox = Signal(signed(OW))
        oy = Signal(signed(OW))
        oz = Signal(signed(PW))

        z_abs  = Signal(unsigned(PW))
        a1_fp  = rad2int(math.atan2(7, 24), self.w)  # 16.260°
        a2_fp  = rad2int(math.atan2(15, 20), self.w)  # 36.870°
        th_mid = rad2int((math.atan2(7, 24) + math.atan2(15, 20)) / 2, self.w)

        m.d.comb += z_abs.eq(Mux(z[-1], -z, z))

        with m.If(z_abs <= (a1_fp >> 1)):
            # |z| <= 8.13°: do not rotate
            m.d.comb += [
                ox.eq(x25),
                oy.eq(y25),
                oz.eq(z),
            ]
        with m.Elif(z >= 0):
            with m.If(z_abs <= th_mid):
                # 8.13° < z <= 26.56°: rotate by 16.260°
                m.d.comb += [
                    ox.eq(x24 - y7),
                    oy.eq(y24 + x7),
                    oz.eq(z - a1_fp),
                ]
            with m.Else():
                # z > 26.56°: rotate by 36.870°
                m.d.comb += [
                    ox.eq(x20 - y15),
                    oy.eq(y20 + x15),
                    oz.eq(z - a2_fp),
                ]
        with m.Else():
            with m.If(z_abs <= th_mid):
                m.d.comb += [
                    ox.eq(x24 + y7),
                    oy.eq(y24 - x7),
                    oz.eq(z + a1_fp),
                ]
            with m.Else():
                m.d.comb += [
                    ox.eq(x20 + y15),
                    oy.eq(y20 - x15),
                    oz.eq(z + a2_fp),
                ]

        m.d.comb += self.o_x.eq(ox >> 5)
        m.d.comb += self.o_y.eq(oy >> 5)
        m.d.comb += self.o_z.eq(oz)
        return m


class USRCORDICStage(Elaboratable):
    def __init__(self, w, k):
        self.w = w
        self.k = k

        W = w + 2
        self.i_x = Signal(signed(W))
        self.i_y = Signal(signed(W))
        self.i_z = Signal(signed(w - 3))
        self.o_x = Signal(signed(W))
        self.o_y = Signal(signed(W))
        self.o_z = Signal(signed(w - 3))

    def elaborate(self, platform):
        m = Module()
        W = self.w + 2
        k = self.k

        x = self.i_x
        y = self.i_y
        z = self.i_z

        alpha_fp = rad2int(math.atan2(2**(-k+1), 1), self.w)
        alpha_th = rad2int(math.atan2(2**(-k+1), 1) / 2, self.w)

        z_abs = Signal(unsigned(self.w - 3))
        m.d.comb += z_abs.eq(Mux(z[-1], -z, z))

        OW = W + 2*k
        ox = Signal(signed(OW))
        oy = Signal(signed(OW))
        oz = Signal(signed(self.w - 3))

        with m.If(z_abs <= alpha_th):
            m.d.comb += [
                ox.eq((x << (2*k - 1)) + x),
                oy.eq((y << (2*k - 1)) + y),
                oz.eq(z),
            ]
        with m.Elif(z >= 0):
            m.d.comb += [
                ox.eq((x << (2*k - 1)) - (y << k)),
                oy.eq((y << (2*k - 1)) + (x << k)),
                oz.eq(z - alpha_fp),
            ]
        with m.Else():
            m.d.comb += [
                ox.eq((x << (2*k - 1)) + (y << k)),
                oy.eq((y << (2*k - 1)) - (x << k)),
                oz.eq(z + alpha_fp),
            ]

        #
        m.d.comb += self.o_x.eq(ox >> 7)
        m.d.comb += self.o_y.eq(oy >> 7)
        m.d.comb += self.o_z.eq(oz)
        return m


class CORDICStage(Elaboratable):
    """
    Classical CORDIC micro-rotation.
    P1 = C + j*S,  where S/C = 2^(-shift).
    Direction = sign(z_in).
    """
    def __init__(self, w, shift):
        self.w         = w
        self.shift     = shift

        W = w + 2
        self.i_x = Signal(signed(W))
        self.i_y = Signal(signed(W))
        self.i_z = Signal(signed(w - 3))
        self.o_x = Signal(signed(W))
        self.o_y = Signal(signed(W))
        self.o_z = Signal(signed(w - 3))

    def elaborate(self, platform):
        m = Module()
        W = self.w + 2

        x = self.i_x
        y = self.i_y
        z = self.i_z

        alpha_fp = rad2int(math.atan2(1, 1 << self.shift), self.w)

        # δ = sign(z): if z >= 0, rotate forward (subtract from z), else backward
        with m.If(z[-1] == 0):   # z >= 0
            m.d.comb += [
                self.o_x.eq(x - (y >> (self.shift))),
                self.o_y.eq(y + (x >> (self.shift))),
                self.o_z.eq(z - alpha_fp),
            ]
        with m.Else():            # z < 0
            m.d.comb += [
                self.o_x.eq(x + (y >> (self.shift))),
                self.o_y.eq(y - (x >> (self.shift))),
                self.o_z.eq(z + alpha_fp),
            ]

        return m


class NanoRotation(Elaboratable):
    """
    Nano-rotation stage.
    Pk = 512 + j*k, k=0..8

    The rotation is:
      x' = x - k*(y >> shift)
      y' = y + k*(x >> shift)
    """
    def __init__(self, w, shift):
        self.w = w
        self.shift = shift
        W = w + 2
        self.i_x = Signal(signed(W))
        self.i_y = Signal(signed(W))
        self.i_z = Signal(signed(w - 3))
        self.o_x = Signal(signed(W))
        self.o_y = Signal(signed(W))
        self.o_z = Signal(signed(w - 3))

    def elaborate(self, platform):
        m = Module()
        W = self.w + 2

        x = self.i_x
        y = self.i_y
        z = self.i_z

        # Per-angle step in fixed-point units:
        alpha1_fp = rad2int(math.atan2(1, 1 << self.shift), self.w)

        k = Signal(range(8+1))
        z_abs = Signal(signed(self.w - 3))
        z_pos = Signal()

        m.d.comb += z_abs.eq(Mux(z[-1], -z, z))
        m.d.comb += z_pos.eq(~z[-1])  # 1 if z>=0

        # Decode k by comparing z_abs against multiples of alpha1_fp
        statement = m.If
        for i in range(8):
            with statement(z_abs < rad2int(math.atan2(1, 1 << self.shift) * (i + 0.5), self.w)):
                m.d.comb += k.eq(i)
            statement = m.Elif
        with m.Else():
            m.d.comb += k.eq(8)

        x_sh = x >> self.shift   # x / 2**shift
        y_sh = y >> self.shift   # y / 2**shift

        x8 = x << 3
        x4 = x << 2
        x2 = x << 1
        y8 = y << 3
        y4 = y << 2
        y2 = y << 1

        inputs_scaled = [
            (0, 0),
            (x, y),
            (x2, y2),
            (x2+x, y2+y),
            (x4, y4),
            (x4+x, y4+y),
            (x4+x2, y4+y2),
            (x8-x, y8-y),
            (x8, y8),
        ]

        ky = Signal(signed(W + 4))
        kx = Signal(signed(W + 4))
        with m.Switch(k):
            for i in range(9):
                with m.Case(i):
                    #m.d.comb += ky.eq(i * y_sh)
                    #m.d.comb += kx.eq(i * x_sh)
                    m.d.comb += kx.eq(inputs_scaled[i][0] >> self.shift)
                    m.d.comb += ky.eq(inputs_scaled[i][1] >> self.shift)

        # Apply rotation: sign of z determines direction.
        with m.If(z_pos):
            m.d.comb += [
                self.o_x.eq(x - ky),
                self.o_y.eq(y + kx),
                self.o_z.eq(z - k * alpha1_fp),
            ]
        with m.Else():
            m.d.comb += [
                self.o_x.eq(x + ky),
                self.o_y.eq(y - kx),
                self.o_z.eq(z + k * alpha1_fp),
            ]

        return m


class CORDIC_II(Elaboratable):
    """
    Pipelined CORDIC-II rotator (rotation mode).

    TODO: fix output scale.
    """
    def __init__(self, w=16):
        self.w = w

        self.i_x   = Signal(signed(w))
        self.i_y   = Signal(signed(w))
        self.i_z   = Signal(w)        # unsigned angle 0..360°
        self.i_vld = Signal()

        self.o_x   = Signal(signed(w))
        self.o_y   = Signal(signed(w))
        self.o_vld = Signal()

    def elaborate(self, platform):
        m = Module()
        w = self.w

        stages = [
            TrivialRotation(w),
            FriendAngles(w),            # 0°, 16.260°, 36.870°
            USRCORDICStage(w, k=4),     # 7.125°
            CORDICStage(w, shift=5),    # atan(1/32)
            CORDICStage(w, shift=6),    # atan(1/64)
            CORDICStage(w, shift=7),    # atan(1/128)
            NanoRotation(w, shift=10),  # atan(k/1024), k=0..8
        ]
        m.submodules += stages

        num_stages = len(stages)

        vld_r = [ Signal(name=f"vld_r{i}") for i in range(num_stages) ]
        for i in range(num_stages):
            m.d.sync += vld_r[i].eq(vld_r[i-1] if i > 0 else self.i_vld)
        m.d.comb += self.o_vld.eq(vld_r[-1])

        m.d.comb += [
            stages[0].i_x.eq(self.i_x),
            stages[0].i_y.eq(self.i_y),
            stages[0].i_z.eq(self.i_z),
        ]
        for i in range(1, num_stages):
            m.d.sync += [
                stages[i].i_x.eq(stages[i-1].o_x),
                stages[i].i_y.eq(stages[i-1].o_y),
                stages[i].i_z.eq(stages[i-1].o_z),
            ]
        m.d.sync += [
            self.o_x.eq(stages[-1].o_x),
            self.o_y.eq(stages[-1].o_y),
        ]

        return m


def sim_test():
    w = 16
    FULL = (1 << (w - 1)) - 1  # max positive value

    dut = CORDIC_II(w=w)
    sim = Simulator(dut)
    sim.add_clock(1e-8)

    angle_deg = 45.0
    x_in = round(0.8 * FULL)
    y_in = 0

    captured = []

    if 1:
        num_samples = 8 * (1<<16)
        samples = range(8 * (1 << 16))
    else:
        num_samples = 1
        samples = [round(angle_deg / 360.0 * (1 << w)) & ((1 << w) - 1)]

    async def input_process(ctx):
        for i, sample in enumerate(samples):
            ctx.set(dut.i_x, x_in)
            ctx.set(dut.i_y, y_in)
            ctx.set(dut.i_z, sample)
            ctx.set(dut.i_vld, 1)
            await ctx.tick()
        ctx.set(dut.i_vld, 0)

    async def output_process(ctx):
        while len(captured) < num_samples:
            ox, oy = await ctx.tick().sample(dut.o_x, dut.o_y).until(dut.o_vld)
            captured.append((ox, oy))

    sim.add_testbench(input_process)
    sim.add_testbench(output_process)
    with sim.write_vcd("cordic2_sim.vcd"):
        sim.run()

    if captured:
        ox, oy = captured[0]
        mag = math.sqrt(ox**2 + oy**2)
        gain = mag / x_in
        print("\nCORDIC-II simulation")
        print(f"input:  ({x_in}, {y_in}), angle={angle_deg}°")
        print(f"output: ({ox}, {oy})")
        print(f"|output|/|input| = {gain:.4f}  (expected ~1.565)")
        print(f"phase angle out  = {math.degrees(math.atan2(oy, ox)):.3f}°  (expected ~45°)")
        phase_err = abs(math.degrees(math.atan2(oy, ox)) - angle_deg)
        print(f"phase error      = {phase_err:.4f}°")


if __name__ == "__main__":
    sim_test()
