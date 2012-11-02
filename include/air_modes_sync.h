/*
  Copyright 2012 Corgan Labs
  
  This file is part of gr-air-modes
  
  gr-air-modes is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3, or (at your option)
  any later version.
  
  gr-air-modes is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with gr-air-modes; see the file COPYING.  If not, write to
  the Free Software Foundation, Inc., 51 Franklin Street,
  Boston, MA 02110-1301, USA.
*/

#ifndef INCLUDED_AIR_MODES_SYNC_H
#define INCLUDED_AIR_MODES_SYNC_H

#include <gr_block.h>
#include <air_modes_api.h>

class air_modes_sync;
typedef boost::shared_ptr<air_modes_sync> air_modes_sync_sptr;

AIR_MODES_API air_modes_sync_sptr air_make_modes_sync(float rate, float threshold);

class AIR_MODES_API air_modes_sync : public gr_block
{
private:
  float        d_rate;		// Incoming sample rate
  unsigned int d_spc;		// Samples per chip
  float        d_threshold;     // Threshold for edge detection

  unsigned int d_samples;       // Total samples processed
  unsigned int d_edges;         // Total edges seen 
  unsigned int d_preambles;     // Total preambles seen

  pmt::pmt_t   d_me;		// Our source block id
  pmt::pmt_t   d_rx;            // Upstream timestamp tag key
  pmt::pmt_t   d_key;		// Burst output timestamp tag key

  unsigned int d_state;		// Finite state machine
  unsigned int d_count;         // Burst sample count
  gr_tag_t     d_timestamp;	// Latest timestamp from upstream

  friend air_modes_sync_sptr air_make_modes_sync(float rate, float threshold);
  air_modes_sync(float rate, float threshold);

  double tag_to_timestamp(uint64_t abs_sample_cnt);

  virtual void forecast(int noutput_items,
			gr_vector_int &ninput_items_required);

public:
  ~air_modes_sync();

  int general_work(int noutput_items,
		   gr_vector_int &ninput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items);
};

#endif /* INCLUDED_AIR_MODES_SYNC_H */
