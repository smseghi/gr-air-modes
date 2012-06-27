#
# Copyright 2010, 2012 Nick Foster
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

import time, os, sys
from string import split, join
import modes_parse
from modes_exceptions import *
import math

class modes_output_print(modes_parse.modes_parse):
  def __init__(self, mypos):
      modes_parse.modes_parse.__init__(self, mypos)
      
  def parse(self, message):
    [data, ecc, reference, timestamp] = message.split()

    data = modes_parse.modes_reply(long(data, 16))
    ecc = long(ecc, 16)
    reference = float(reference)
    timestamp = float(timestamp)

    #TODO this is suspect
    if reference == 0.0:
      refdb = -150.0
    else:
      refdb = 10.0*math.log10(reference)
    output = "(%.0f %.10f) " % (refdb, timestamp);

    try:
      msgtype = data["df"]
      if msgtype == 0:
        output += self.print0(data, ecc)
      elif msgtype == 4:
        output += self.print4(data, ecc)
      elif msgtype == 5:
        output += self.print5(data, ecc)
      elif msgtype == 11:
        output += self.print11(data, ecc)
      elif msgtype == 17:
        output += self.print17(data)
      else:
        output += "No handler for message type %i from %x (but it's in modes_parse)" % (msgtype, ecc)
      print output
    except NoHandlerError as e:
      output += "No handler for message type %i from %x" % (msgtype, ecc)
      print output
    except MetricAltError:
      pass
    except CPRNoPositionError:
      pass

  def print0(self, shortdata, ecc):
    [vs, cc, sl, ri, altitude] = self.parse0(shortdata)
	
    retstr = "Type 0 (short A-A surveillance) from %x at %ift" % (ecc, altitude)
    # the ri values below 9 are used for other things. might want to print those someday.
    if ri == 0:
      retstr += " (No TCAS)"
    elif ri == 2:
      retstr += " (TCAS resolution inhibited)"
    elif ri == 3:
      retstr += " (Vertical TCAS resolution only)"
    elif ri == 4:
      retstr += " (Full TCAS resolution)"
    elif ri == 9:
      retstr += " (speed <75kt)"
    elif ri > 9:
      retstr += " (speed %i-%ikt)" % (75 * (1 << (ri-10)), 75 * (1 << (ri-9)))

    if vs is True:
      retstr += " (aircraft is on the ground)"

    return retstr

  def print4(self, shortdata, ecc):

    [fs, dr, um, altitude] = self.parse4(shortdata)

    retstr = "Type 4 (short surveillance altitude reply) from %x at %ift" % (ecc, altitude)

    if fs == 1:
      retstr += " (aircraft is on the ground)"
    elif fs == 2:
      retstr += " (AIRBORNE ALERT)"
    elif fs == 3:
      retstr += " (GROUND ALERT)"
    elif fs == 4:
      retstr += " (SPI ALERT)"
    elif fs == 5:
      retstr += " (SPI)"

    return retstr

  def print5(self, shortdata, ecc):
    [fs, dr, um, ident] = self.parse5(shortdata)

    retstr = "Type 5 (short surveillance ident reply) from %x with ident %i" % (ecc, ident)

    if fs == 1:
      retstr += " (aircraft is on the ground)"
    elif fs == 2:
      retstr += " (AIRBORNE ALERT)"
    elif fs == 3:
      retstr += " (GROUND ALERT)"
    elif fs == 4:
      retstr += " (SPI ALERT)"
    elif fs == 5:
      retstr += " (SPI)"

    return retstr

  def print11(self, data, ecc):
    [icao24, interrogator, ca] = self.parse11(data, ecc)
    retstr = "Type 11 (all call reply) from %x in reply to interrogator %i" % (icao24, interrogator)
    return retstr

  def print17(self, data):
    icao24 = data["aa"]
    subtype = data["me"]["ftc"]

    retstr = None

    if 1 <= subtype <= 4:
      (msg, typestring) = self.parseBDS08(data)
      retstr = "Type 17 subtype 04 (ident) from %x type %s ident %s" % (icao24, typestring, msg)

    elif subtype >= 5 and subtype <= 8:
      [altitude, decoded_lat, decoded_lon, rnge, bearing] = self.parseBDS06(data)
      retstr = "Type 17 subtype 06 (surface report) from %x at (%.6f, %.6f)" % (icao24, decoded_lat, decoded_lon)
      if rnge is not None and bearing is not None:
        retstr += " (%.2f @ %.0f)" % (rnge, bearing)

    elif subtype >= 9 and subtype <= 18:
      [altitude, decoded_lat, decoded_lon, rnge, bearing] = self.parseBDS05(data)
      retstr = "Type 17 subtype 05 (position report) from %x at (%.6f, %.6f)" % (icao24, decoded_lat, decoded_lon)
      if rnge is not None and bearing is not None:
        retstr += " (" + "%.2f" % rnge + " @ " + "%.0f" % bearing + ")"
      retstr += " at " + str(altitude) + "ft"

    elif subtype == 19:
      subsubtype = data["me"]["bds09"]["sub"]
      if subsubtype == 0:
        [velocity, heading, vert_spd] = self.parseBDS09_0(data)
        retstr = "Type 17 subtype 09-0 (track report) from %x with velocity %.0fkt heading %.0f VS %.0f" % (icao24, velocity, heading, vert_spd)
      elif 1 <= subsubtype <= 2:
        [velocity, heading, vert_spd] = self.parseBDS09_1(data)
        retstr = "Type 17 subtype 09-%i (track report) from %x with velocity %.0fkt heading %.0f VS %.0f" % (subsubtype, icao24, velocity, heading, vert_spd)
      else:
        retstr = "Type 17 subtype 09-%i from %x not implemented" % (subsubtype, icao24)

    elif subtype == 28:
      emerg_str = self.parseBDS62(data)
      retstr = "Type 17 subtype 28 (emergency) from %x type %s" % (icao24, emerg_str)
      
    else:
      retstr = "Type 17 subtype %i from %x not implemented" % (subtype, icao24)

    return retstr

  def print20(self, data, ecc):
    [fs, dr, um, alt] = self.parse4(data)
    mb_fields = data["mb"].get_fields()
    bds1 = mb_fields["bds1"]
    bds2 = mb_fields["bds2"]

    if bds2 != 0:
      retstr = "No handler for BDS2 == %i from %x" % (bds2, ecc)

    elif bds1 == 0:
      retstr = "No handler for BDS1 == 0 from %x" % ecc
    elif bds1 == 1:
      retstr = "Type 20 link capability report from %x: ACS: 0x%x, BCS: 0x%x, ECS: 0x%x, continues %i" \
                % (ecc, mb_fields["acs"], mb_fields["bcs"], mb_fields["ecs"], mb_fields["cfs"])
    elif bds1 == 2:
      retstr = "Type 20 identification from %x with text %s" % (ecc, self.parseMB_id(data))
    elif bds2 == 3:
      retstr = "TCAS report from %x: " % ecc
      tti = mb_fields["tcas"].get_type()
      if tti == 1:
        (resolutions, complements, rat, mte, threat_id) = self.parseMB_TCAS_threatid(data)
        retstr += "threat ID: %x advised: %s complement: %s" % (threat_id, resolutions, complements)
      elif tti == 2:
        (resolutions, complements, rat, mte, threat_alt, threat_range, threat_bearing) = self.parseMB_TCAS_threatloc(data)
        retstr += "range: %i bearing: %i alt: %i advised: %s complement: %s" % (threat_range, threat_bearing, threat_alt, resolutions, complements)
      else:
        retstr += " (no handler for TTI=%i)" % tti
      if mte == 1:
        retstr += " (multiple threats)"
      if rat == 1:
        retstr += " (resolved)"
    else:
      retstr = "No handler for BDS1 == %i from %x" % (bds1, ecc)
      
    return retstr
