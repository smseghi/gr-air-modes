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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <air_modes_sync.h>
#include <gr_io_signature.h>
#include <gr_tags.h>

// These should eventually end up in a constants include file
#define MODE_S_CHIP_TIME 0.5e-6
#define EPSILON          1e-17
#define PREAMBLE_LEN     16
#define BURST_LEN        240
#define PREAMBLE_SLICE   0.5

static const bool preamble_bits[] = {1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0};

// Finite state machine states
#define ST_IDLE          0
#define ST_RISING        1
#define ST_PEAK          2
#define ST_BURSTING      3

air_modes_sync_sptr
air_make_modes_sync(float rate, float threshold)
{
  return gnuradio::get_initial_sptr(new air_modes_sync(rate, threshold));
}

air_modes_sync::air_modes_sync(float rate, float threshold) :
  gr_block("air_modes_sync",
	   gr_make_io_signature(2, 2, sizeof(float)),
	   gr_make_io_signature(1, 1, sizeof(float))),
  d_rate(rate),
  d_samples(0),
  d_edges(0),
  d_preambles(0),
  d_count(0)
{
  // Set up configuration parameters
  d_spc = (unsigned int)(d_rate*MODE_S_CHIP_TIME);
  d_threshold = powf(10.0, threshold/20.0);
  set_history(18*d_spc); // preamble length + 2 chips

  // Get unique name of this block into str
  std::stringstream str;
  str << name() << unique_id();

  // Create atoms for stream tagging
  d_me  = pmt::pmt_string_to_symbol(str.str());
  d_key = pmt::pmt_string_to_symbol("preamble_found");
  d_rx  = pmt::pmt_string_to_symbol("rx_time");

  // Kick off the state machine
  d_state = ST_IDLE;
}

air_modes_sync::~air_modes_sync()
{
  std::cerr << std::endl 
	    << "d_samples   = " << d_samples << std::endl
	    << "d_edges     = " << d_edges << std::endl
	    << "d_preambles = " << d_preambles << std::endl
	    << "S/E         = " << float(d_samples)/d_edges << std::endl
	    << "E/P         = " << float(d_edges)/d_preambles << std::endl
	    << "S/P         = " << float(d_samples)/d_preambles << std::endl;
}

void
air_modes_sync::forecast(int noutput_items,
			 gr_vector_int &ninput_items_required)
{
  // This is appropriate for bursting and doesn't hurt otherwise.
  ninput_items_required[0] = d_spc*noutput_items;
  ninput_items_required[1] = ninput_items_required[0];
}

double 
air_modes_sync::tag_to_timestamp(uint64_t abs_sample_cnt)
{
  if (d_timestamp.key == NULL || d_timestamp.key != d_rx)
    return 0;

  uint64_t last_whole_stamp = pmt::pmt_to_uint64(pmt::pmt_tuple_ref(d_timestamp.value, 0));
  double   last_frac_stamp  = pmt::pmt_to_double(pmt::pmt_tuple_ref(d_timestamp.value, 1));
  double   tstime           = double(abs_sample_cnt)/d_rate + double(last_whole_stamp) + last_frac_stamp;

  return tstime;
}

int
air_modes_sync::general_work(int noutput_items,
			     gr_vector_int &ninput_items,
			     gr_vector_const_void_star &input_items,
			     gr_vector_void_star &output_items)
{
  const float *in0 = (const float *) input_items[0];
  const float *in1 = (const float *) input_items[1];
  float *out = (float *) output_items[0];
  const size_t ninputs = std::min<size_t>(ninput_items[0], ninput_items[1]);

  // Maintain time
  uint64_t abs_sample_cnt = nitems_read(0);
  std::vector<gr_tag_t> tstamp_tags;

  get_tags_in_range(tstamp_tags, 0, 
		    abs_sample_cnt, abs_sample_cnt+ninputs,
		    d_rx);

  if (tstamp_tags.size() > 0)
    d_timestamp = tstamp_tags.back();
 
  // Iterate over as many samples as we can, updating state along the way
  int i = 0, j = 0;
  while (i < ninputs && j < noutput_items) {

    if (d_state == ST_IDLE) {
      if (in0[i] > in1[i]*d_threshold) { // Rising edge detect
	d_state = ST_RISING;
	d_edges++;
      }
      else {
	i++;			         // Otherwise consume sample 
	continue;
      }
    }

    if (d_state == ST_RISING) {
      if (in0[i] > in0[i+1]) {	         // No longer rising
	d_state = ST_PEAK;
      }
      else {
	i++;			         // Still rising, consume sample
	continue;
      }
    }

    if (d_state == ST_PEAK) {

      float ampl = (in0[i]+in1[i])*PREAMBLE_SLICE;
      bool valid_preamble = true;
      int k = 0;

      while (valid_preamble && k < 16) {
	float sample = in0[i+k*d_spc];
	valid_preamble &= 
	  preamble_bits[k++] ? sample > ampl : sample < ampl;
      }

      if (valid_preamble) {
	d_preambles++;
	d_count = BURST_LEN*d_spc;
	d_state = ST_BURSTING;

	double tstamp = tag_to_timestamp(abs_sample_cnt+i);
	add_item_tag(0,			           // output stream number
		     nitems_written(0),            // output sample number
		     d_key,                        // frame_info
		     pmt::pmt_from_double(tstamp), // timestamp value
		     d_me);                        // source block id
      }
      else {
	i++;
	d_state = ST_IDLE;
	continue;
      }
    }

    if (d_state == ST_BURSTING) {
      if (d_count > 0) {
	if (d_count % d_spc == 0)
	  out[j++] = in0[i];
	d_count--;
      }
      else {
	d_state = ST_IDLE;
      }

      i++;
    }

  } /* while I/O possible */
  
  consume_each(i);
  d_samples += i;

  return j;
}
