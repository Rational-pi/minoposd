/**
 ******************************************************************************
 *
 * @file       AnalogRssi.ino
 * @author     Philippe Vanhaesnedonck
 * @brief      Implements RSSI report on the Ardupilot Mega MinimOSD
 *             using built-in ADC reference.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/> or write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


// !!! For using this, you have to solder a little bit on the MinimOSD, see the wiki !!!


#include "AnalogRssi.h"

#ifdef A7105_ANALOG_RSSI
#define DECAY_RATE          1  // takes 255 cycles (~84ms for 336us channel sample period) to drop 1.1V

static int16_t peak_rssi = 0;
static float avg_peak_rssi = 0;

void negativePeakHold(uint16_t result, uint16_t sampleTimeMicros)
{
    if (result == 1023) return; // Ignore overflows i.e >= our 1.1V reference
    uint8_t raw_rssi = result >> 2; // reduce to 8 bit precision to match OSD config`
    peak_rssi = max(peak_rssi - DECAY_RATE, 255 - raw_rssi); // hold negative (inverted) peak or decay to find peak
}
#endif // A7105_ANALOG_RSSI

void analog_rssi_init(void)
{
#ifdef A7105_ANALOG_RSSI
    analog.handle(RSSI_PIN, INTERNAL, negativePeakHold); // INTERNAL: a built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328
#else
    analog.reference(RSSI_PIN, INTERNAL);
#endif // A7105_ANALOG_RSSI
}

void analog_rssi_read(void)
{
#ifdef A7105_ANALOG_RSSI
    if (rssiraw_on) {
        osd_rssi = peak_rssi;
    } else {
        avg_peak_rssi = peak_rssi * .2 + avg_peak_rssi * .8; // Smooth input
        osd_rssi = constrain((int)avg_peak_rssi, rssipersent, rssical);
    }
#else
    if (rssiraw_on) {
        osd_rssi = analog.read(RSSI_PIN) / 4; // Just raw value, 0-255. We use this range to better align
                                             // with the original code.
    } else {
#ifdef JR_SPECIALS
// SEARCH GLITCH
        osd_rssi = analog.read(RSSI_PIN) / 4; // 1:1 input
#else
        osd_rssi = analog.read(RSSI_PIN) * .2 / 4 + osd_rssi * .8; // Smooth input
#endif
        osd_rssi = constrain(osd_rssi, rssipersent, rssical); // Ensure we stay in range
    }
#endif // A7105_ANALOG_RSSI
}
