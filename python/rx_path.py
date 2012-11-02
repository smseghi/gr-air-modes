#
# Copyright 2012 Corgan Labs
# 
# This file is part of gr-air-modes
# 
# gr-air-modes is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# gr-air-modes is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with gr-air-modes; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

from gnuradio import gr
import air_modes_swig

class rx_path(gr.hier_block2):

    def __init__(self, rate, threshold, queue, use_pmf=False):
        gr.hier_block2.__init__(self, "modes_rx_path",
                                gr.io_signature(1, 1, gr.sizeof_gr_complex),
                                gr.io_signature(0,0,0))

        self._rate = int(rate)
        self._threshold = threshold
        self._queue = queue
        self._spc = int(rate/2e6)

        # Convert incoming I/Q baseband to amplitude
        self._demod = gr.complex_to_mag()
        self._bb = self._demod

        # Pulse matched filter for 0.5us pulses
        if use_pmf:
            self._pmf = gr.moving_average_ff(self._spc, 1.0/self._spc, self._rate)
            self.connect(self._demod, self._pmf)
            self._bb = self._pmf

        # Establish baseline amplitude (noise, interference)
        self._avg = gr.moving_average_ff(75*self._spc, 1.0/(75*self._spc), self._rate) # First ACF minima

        # Synchronize to Mode-S preamble
        #self._sync = air_modes_swig.modes_preamble(self._rate, self._threshold)
        self._sync = air_modes_swig.modes_sync(self._rate, self._threshold)

        # Slice Mode-S bits and send to message queue
        self._slicer = air_modes_swig.modes_slicer(self._rate, self._queue)

        # Wire up the flowgraph
        self.connect(self, self._demod)
        self.connect(self._bb, (self._sync, 0))
        self.connect(self._bb, self._avg, (self._sync, 1))
        self.connect(self._sync, self._slicer)

        # DEBUG
        self.connect(self._sync, gr.file_sink(gr.sizeof_float, "sync.dat"))
