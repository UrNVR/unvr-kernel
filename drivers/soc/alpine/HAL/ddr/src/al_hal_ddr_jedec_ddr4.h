/*******************************************************************************
Copyright (C) 2015 Annapurna Labs Ltd.

This file may be licensed under the terms of the Annapurna Labs Commercial
License Agreement.

Alternatively, this file can be distributed under the terms of the GNU General
Public License V2 as published by the Free Software Foundation and can be
found at http://www.gnu.org/licenses/gpl-2.0.html

Alternatively, redistribution and use in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:

    *     Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

    *     Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

/**
 * @addtogroup groupddr
 *
 *  @{
 * @file   al_hal_ddr_jedec_ddr4.h
 *
 * @brief  DDR JEDEC registers bits defines
 *
 */
#ifndef __AL_HAL_DDR_JEDEC_DDR4_H__
#define __AL_HAL_DDR_JEDEC_DDR4_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Mode Register 0 */
#define AL_DDR_JEDEC_DDR4_MR0_BL_MASK		0x00000003
#define AL_DDR_JEDEC_DDR4_MR0_BL_SHIFT		0

#define AL_DDR_JEDEC_DDR4_MR0_BL_8_FIXED		\
	(0 << AL_DDR_JEDEC_DDR4_MR0_BL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR0_BC_4_8		\
	(1 << AL_DDR_JEDEC_DDR4_MR0_BL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR0_BC_4		\
	(2 << AL_DDR_JEDEC_DDR4_MR0_BL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR0_BT_MASK		0x00000008
#define AL_DDR_JEDEC_DDR4_MR0_BT_SHIFT		3

#define AL_DDR_JEDEC_DDR4_MR0_BT_SEQ		\
	(0 << AL_DDR_JEDEC_DDR4_MR0_BT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR0_BT_INTRLVD		\
	(1 << AL_DDR_JEDEC_DDR4_MR0_BT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR0_CL_MASK		0x00000074
#define AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT		2
#define AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2		4

#define AL_DDR_JEDEC_DDR4_MR0_CL_9		\
	((0x0 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x0 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_10		\
	((0x1 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x0 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_11		\
	((0x0 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x1 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_12		\
	((0x1 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x1 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_13		\
	((0x0 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x2 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_14		\
	((0x1 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x2 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_15		\
	((0x0 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x3 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_16		\
	((0x1 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x3 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_18		\
	((0x0 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x4 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_20		\
	((0x1 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x4 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_22		\
	((0x0 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x5 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_24		\
	((0x1 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x5 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_23		\
	((0x0 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x6 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_17		\
	((0x1 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x6 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_19		\
	((0x0 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x7 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_CL_21		\
	((0x1 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT) |\
	(0x7 << AL_DDR_JEDEC_DDR4_MR0_CL_SHIFT2))

#define AL_DDR_JEDEC_DDR4_MR0_DR			0x00000100

#define AL_DDR_JEDEC_DDR4_MR0_WR_MASK		0x00000E00
#define AL_DDR_JEDEC_DDR4_MR0_WR_SHIFT		9

#define AL_DDR_JEDEC_DDR4_MR0_WR_10		\
	(0 << AL_DDR_JEDEC_DDR4_MR0_WR_SHIFT)
#define AL_DDR_JEDEC_DDR4_MR0_WR_12		\
	(1 << AL_DDR_JEDEC_DDR4_MR0_WR_SHIFT)
#define AL_DDR_JEDEC_DDR4_MR0_WR_14		\
	(2 << AL_DDR_JEDEC_DDR4_MR0_WR_SHIFT)
#define AL_DDR_JEDEC_DDR4_MR0_WR_16		\
	(3 << AL_DDR_JEDEC_DDR4_MR0_WR_SHIFT)
#define AL_DDR_JEDEC_DDR4_MR0_WR_18		\
	(4 << AL_DDR_JEDEC_DDR4_MR0_WR_SHIFT)
#define AL_DDR_JEDEC_DDR4_MR0_WR_20		\
	(5 << AL_DDR_JEDEC_DDR4_MR0_WR_SHIFT)
#define AL_DDR_JEDEC_DDR4_MR0_WR_24		\
	(6 << AL_DDR_JEDEC_DDR4_MR0_WR_SHIFT)


/* Mode Register 1 */

#define AL_DDR_JEDEC_DDR4_MR1_DE_MASK		0x00000001
#define AL_DDR_JEDEC_DDR4_MR1_DE_SHIFT		0

#define AL_DDR_JEDEC_DDR4_MR1_DE_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR1_DE_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_DE_EN		\
	(1 << AL_DDR_JEDEC_DDR4_MR1_DE_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_DIC_MASK		0x00000006
#define AL_DDR_JEDEC_DDR4_MR1_DIC_SHIFT		1

#define AL_DDR_JEDEC_DDR4_MR1_DIC_RZQ7		\
	(0x0 << AL_DDR_JEDEC_DDR4_MR1_DIC_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_DIC_RZQ5		\
	(0x1 << AL_DDR_JEDEC_DDR4_MR1_DIC_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_AL_MASK		0x00000018
#define AL_DDR_JEDEC_DDR4_MR1_AL_SHIFT		3

#define AL_DDR_JEDEC_DDR4_MR1_AL_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR1_AL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_AL_CL_MINUS_1	\
	(1 << AL_DDR_JEDEC_DDR4_MR1_AL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_AL_CL_MINUS_2	\
	(2 << AL_DDR_JEDEC_DDR4_MR1_AL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_RTT_MASK		0x00000700
#define AL_DDR_JEDEC_DDR4_MR1_RTT_SHIFT		8

#define AL_DDR_JEDEC_DDR4_MR1_RTT_ODT_DIS		\
	(0x0 << AL_DDR_JEDEC_DDR4_MR1_RTT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_RTT_RZQ4		\
	(0x1 << AL_DDR_JEDEC_DDR4_MR1_RTT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_RTT_RZQ2		\
	(0x2 << AL_DDR_JEDEC_DDR4_MR1_RTT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_RTT_RZQ6		\
	(0x3 << AL_DDR_JEDEC_DDR4_MR1_RTT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_RTT_RZQ1		\
	(0x4 << AL_DDR_JEDEC_DDR4_MR1_RTT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_RTT_RZQ5		\
	(0x5 << AL_DDR_JEDEC_DDR4_MR1_RTT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_RTT_RZQ3		\
	(0x6 << AL_DDR_JEDEC_DDR4_MR1_RTT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR1_RTT_RZQ7		\
	(0x7 << AL_DDR_JEDEC_DDR4_MR1_RTT_SHIFT)

/* Mode Register 2 */

#define AL_DDR_JEDEC_DDR4_MR2_CWL_MASK		0x00000038
#define AL_DDR_JEDEC_DDR4_MR2_CWL_SHIFT		3

#define AL_DDR_JEDEC_DDR4_MR2_CWL_9		\
	(0 << AL_DDR_JEDEC_DDR4_MR2_CWL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_CWL_10		\
	(1 << AL_DDR_JEDEC_DDR4_MR2_CWL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_CWL_11		\
	(2 << AL_DDR_JEDEC_DDR4_MR2_CWL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_CWL_12		\
	(3 << AL_DDR_JEDEC_DDR4_MR2_CWL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_CWL_14		\
	(4 << AL_DDR_JEDEC_DDR4_MR2_CWL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_CWL_16		\
	(5 << AL_DDR_JEDEC_DDR4_MR2_CWL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_CWL_18		\
	(6 << AL_DDR_JEDEC_DDR4_MR2_CWL_SHIFT)


#define AL_DDR_JEDEC_DDR4_MR2_RTTWR_MASK	0x00000600
#define AL_DDR_JEDEC_DDR4_MR2_RTTWR_SHIFT	9

#define AL_DDR_JEDEC_DDR4_MR2_RTTWR_ODT_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR2_RTTWR_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_RTTWR_RZQ2		\
	(1 << AL_DDR_JEDEC_DDR4_MR2_RTTWR_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_RTTWR_RZQ1		\
	(2 << AL_DDR_JEDEC_DDR4_MR2_RTTWR_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_RTTWR_HI_Z		\
	(3 << AL_DDR_JEDEC_DDR4_MR2_RTTWR_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_WRITE_CRC_MASK		0x00001000
#define AL_DDR_JEDEC_DDR4_MR2_WRITE_CRC_SHIFT		12

#define AL_DDR_JEDEC_DDR4_MR2_WRITE_CRC_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR2_WRITE_CRC_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR2_WRITE_CRC_EN		\
	(1 << AL_DDR_JEDEC_DDR4_MR2_WRITE_CRC_SHIFT)

/* Mode Register 3 */

#define AL_DDR_JEDEC_DDR4_MR3_MPR_PAGE_SEL_MASK		0x00000003
#define AL_DDR_JEDEC_DDR4_MR3_MPR_PAGE_SEL_SHIFT	0

#define AL_DDR_JEDEC_DDR4_MR3_MPR_OPER_MASK	0x00000004
#define AL_DDR_JEDEC_DDR4_MR3_MPR_OPER_SHIFT	2

#define AL_DDR_JEDEC_DDR4_MR3_MPR_OPER_NORM		\
	(0 << AL_DDR_JEDEC_DDR4_MR3_MPR_OPER_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_MPR_OPER_MPR		\
	(1 << AL_DDR_JEDEC_DDR4_MR3_MPR_OPER_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_PDA_MASK		0x00000010
#define AL_DDR_JEDEC_DDR4_MR3_PDA_SHIFT		4

#define AL_DDR_JEDEC_DDR4_MR3_PDA_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR3_PDA_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_PDA_EN		\
	(1 << AL_DDR_JEDEC_DDR4_MR3_PDA_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_MASK 0x000001c0
#define AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_SHIFT 6

#define AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_FIXED_1X \
	(0 << AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_FIXED_2X \
	(1 << AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_FIXED_4X \
	(2 << AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_ON_THE_FLY_2X \
	(5 << AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_ON_THE_FLY_4X \
	(6 << AL_DDR_JEDEC_DDR4_MR3_FINE_GRANULARITY_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_CRC_DM_WCL_MASK		0x00000600
#define AL_DDR_JEDEC_DDR4_MR3_CRC_DM_WCL_SHIFT		9

#define AL_DDR_JEDEC_DDR4_MR3_CRC_DM_WCL_4	\
	(0 << AL_DDR_JEDEC_DDR4_MR3_CRC_DM_WCL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_CRC_DM_WCL_5	\
	(1 << AL_DDR_JEDEC_DDR4_MR3_CRC_DM_WCL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_CRC_DM_WCL_6	\
	(2 << AL_DDR_JEDEC_DDR4_MR3_CRC_DM_WCL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_MPR_RD_FORMAT_MASK	0x00001800
#define AL_DDR_JEDEC_DDR4_MR3_MPR_RD_FORMAT_SHIFT	11

#define AL_DDR_JEDEC_DDR4_MR3_MPR_RD_FORMAT_SERIAL	\
	(0 << AL_DDR_JEDEC_DDR4_MR3_MPR_RD_FORMAT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_MPR_RD_FORMAT_PARALLEL	\
	(1 << AL_DDR_JEDEC_DDR4_MR3_MPR_RD_FORMAT_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR3_MPR_RD_FORMAT_STAGGERED	\
	(2 << AL_DDR_JEDEC_DDR4_MR3_MPR_RD_FORMAT_SHIFT)

/* Mode Register 4 */

#define AL_DDR_JEDEC_DDR4_MR4_MPDM_MASK		0x00000002
#define AL_DDR_JEDEC_DDR4_MR4_MPDM_SHIFT	1

#define AL_DDR_JEDEC_DDR4_MR4_MPDM_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR4_MPDM_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_MPDM_EN		\
	(1 << AL_DDR_JEDEC_DDR4_MR4_MPDM_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_CAL_MASK		0x000001c0
#define AL_DDR_JEDEC_DDR4_MR4_CAL_SHIFT		6

#define AL_DDR_JEDEC_DDR4_MR4_CAL_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR4_CAL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_CAL_3		\
	(1 << AL_DDR_JEDEC_DDR4_MR4_CAL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_CAL_4		\
	(2 << AL_DDR_JEDEC_DDR4_MR4_CAL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_CAL_5		\
	(3 << AL_DDR_JEDEC_DDR4_MR4_CAL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_CAL_6		\
	(4 << AL_DDR_JEDEC_DDR4_MR4_CAL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_CAL_8		\
	(5 << AL_DDR_JEDEC_DDR4_MR4_CAL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_RD_PREAMBLE_MASK	0x00000800
#define AL_DDR_JEDEC_DDR4_MR4_RD_PREAMBLE_SHIFT	11

#define AL_DDR_JEDEC_DDR4_MR4_RD_PREAMBLE_1CK		\
	(0 << AL_DDR_JEDEC_DDR4_MR4_RD_PREAMBLE_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_RD_PREAMBLE_2CK		\
	(1 << AL_DDR_JEDEC_DDR4_MR4_RD_PREAMBLE_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_WR_PREAMBLE_MASK	0x00001000
#define AL_DDR_JEDEC_DDR4_MR4_WR_PREAMBLE_SHIFT	12

#define AL_DDR_JEDEC_DDR4_MR4_WR_PREAMBLE_1CK		\
	(0 << AL_DDR_JEDEC_DDR4_MR4_WR_PREAMBLE_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR4_WR_PREAMBLE_2CK		\
	(1 << AL_DDR_JEDEC_DDR4_MR4_WR_PREAMBLE_SHIFT)

/* Mode Register 5 */

#define AL_DDR_JEDEC_DDR4_MR5_PL_MASK		0x00000007
#define AL_DDR_JEDEC_DDR4_MR5_PL_SHIFT		0

#define AL_DDR_JEDEC_DDR4_MR5_PL_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR5_PL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_PL_4		\
	(1 << AL_DDR_JEDEC_DDR4_MR5_PL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_PL_5		\
	(2 << AL_DDR_JEDEC_DDR4_MR5_PL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_PL_6		\
	(3 << AL_DDR_JEDEC_DDR4_MR5_PL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_MASK	0x000001c0
#define AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_SHIFT	6

#define AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_RZQ4		\
	(1 << AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_RZQ2		\
	(2 << AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_RZQ6		\
	(3 << AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_RZQ1		\
	(4 << AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_RZQ5		\
	(5 << AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_RZQ3		\
	(6 << AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_RZQ7		\
	(7 << AL_DDR_JEDEC_DDR4_MR5_RTT_PARK_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_CA_PAR_PERS_MASK	0x00000200
#define AL_DDR_JEDEC_DDR4_MR5_CA_PAR_PERS_SHIFT	9

#define AL_DDR_JEDEC_DDR4_MR5_CA_PAR_PERS_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR5_CA_PAR_PERS_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_CA_PAR_PERS_EN		\
	(1 << AL_DDR_JEDEC_DDR4_MR5_CA_PAR_PERS_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_DM_MASK		0x00000400
#define AL_DDR_JEDEC_DDR4_MR5_DM_SHIFT		10

#define AL_DDR_JEDEC_DDR4_MR5_DM_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR5_DM_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_DM_EN		\
	(1 << AL_DDR_JEDEC_DDR4_MR5_DM_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_WRITE_DBI_MASK	0x00000800
#define AL_DDR_JEDEC_DDR4_MR5_WRITE_DBI_SHIFT	11

#define AL_DDR_JEDEC_DDR4_MR5_WRITE_DBI_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR5_WRITE_DBI_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_WRITE_DBI_EN		\
	(1 << AL_DDR_JEDEC_DDR4_MR5_WRITE_DBI_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_READ_DBI_MASK	0x00001000
#define AL_DDR_JEDEC_DDR4_MR5_READ_DBI_SHIFT	12

#define AL_DDR_JEDEC_DDR4_MR5_READ_DBI_DIS		\
	(0 << AL_DDR_JEDEC_DDR4_MR5_READ_DBI_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR5_READ_DBI_EN		\
	(1 << AL_DDR_JEDEC_DDR4_MR5_READ_DBI_SHIFT)

/* Mode Register 6 */

#define AL_DDR_JEDEC_DDR4_MR6_VREFDQ_VAL_MASK		0x0000003f
#define AL_DDR_JEDEC_DDR4_MR6_VREFDQ_VAL_SHIFT		0

#define AL_DDR_JEDEC_DDR4_MR6_VREFDQ_RANGE_MASK		0x00000040
#define AL_DDR_JEDEC_DDR4_MR6_VREFDQ_RANGE_SHIFT	6

#define AL_DDR_JEDEC_DDR4_MR6_CCDL_MASK		0x00001c00
#define AL_DDR_JEDEC_DDR4_MR6_CCDL_SHIFT	10

#define AL_DDR_JEDEC_DDR4_MR6_CCDL_4		\
	(0 << AL_DDR_JEDEC_DDR4_MR6_CCDL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR6_CCDL_5		\
	(1 << AL_DDR_JEDEC_DDR4_MR6_CCDL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR6_CCDL_6		\
	(2 << AL_DDR_JEDEC_DDR4_MR6_CCDL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR6_CCDL_7		\
	(3 << AL_DDR_JEDEC_DDR4_MR6_CCDL_SHIFT)

#define AL_DDR_JEDEC_DDR4_MR6_CCDL_8		\
	(4 << AL_DDR_JEDEC_DDR4_MR6_CCDL_SHIFT)

/* Converting VREF range #1 array from idx to percentages as shown in JEDEC (e.g 6065 = 60.65%) */
static const unsigned int vref_range1_val[] = {
	6000, 6065, 6130, 6195, 6260, 6325, 6390, 6455, 6520, 6585, 6650, 6715, 6780,
	6845, 6910, 6975, 7040, 7105, 7170, 7235, 7300, 7365, 7430, 7495, 7560, 7625,
	7690, 7755, 7820, 7885, 7950, 8015, 8080, 8145, 8210, 8275, 8340, 8405, 8470,
	8535, 8600, 8665, 8730, 8795, 8860, 8925, 8990, 9055, 9120, 9185, 9250};
/* Converting VREF range #2 array from idx to percentages as shown in JEDEC (e.g 6065 = 60.65%) */
static const unsigned int vref_range2_val[] = {
	4500, 4565, 4630, 4695, 4760, 4825, 4890, 4955, 5020, 5085, 5150, 5215, 5280,
	5345, 5410, 5475, 5540, 5605, 5670, 5735, 5800, 5865, 5930, 5995, 6060, 6125,
	6190, 6255, 6320, 6385, 6450, 6515, 6580, 6645, 6710, 6775, 6840, 6905, 6970,
	7035, 7100, 7165, 7230, 7295, 7360, 7425, 7490, 7555, 7620, 7685, 7750};

/*
 * The following arrays specifies the encoding used for the status register field
 * ECCSTAT.corrected_bit_num (DDR controller), which indicates the bit that is corrected
 */
static const unsigned int ecc_corrected_bit_map[] = {
64, 65, 66, 0, 67, 1, 2, 3, 68, 4, 5, 6, 7, 8, 9, 10, 69, 11, 12, 13, 14, 15, 16, 17, 18, 19,
20, 21, 22, 23, 24, 25, 70, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 71, 57, 58, 59, 60, 61, 62, 63
};

#ifdef __cplusplus
}
#endif

#endif

/** @} end of DDR group */

