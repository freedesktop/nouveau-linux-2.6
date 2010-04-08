/*
 * Copyright (C) 2010 Jimmy Rentz
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER(S) AND/OR ITS SUPPLIERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
 
#ifndef __NOUVEAU_VPE_HW_H__
#define __NOUVEAU_VPE_HW_H__

/* VPE is the video decoder engine that is found in nv30, nv40 and some 
 * older hardware (geforce 4 and higher I believe).  
 * It contains an mpeg2 decoder with the following properties:
 * (-) Decodes at the idct level.  However, I believe older cards only
 * support mc level.
 * (-) 32x64 to 2032x2032 profiles.
 * (-) 4:2:0 chroma sampling.
 * (-) Only one set of registers so only one user unless some type of
 * context/channel switching is added.*/

#define NV_VPE_MAX_CHANNELS           1
#define NV_VPE_MAX_SURFACES           8
#define NV_VPE_MIN_WIDTH              32
#define NV_VPE_MIN_HEIGHT             64
#define NV_VPE_MAX_WIDTH              2032
#define NV_VPE_MAX_HEIGHT             2032
#define NV_VPE_PUSHBUFFER_SIZE        1 * 1024 * 1024

#define NV_VPE_CMD_ALIGNMENT         16 

#define NV_VPE_MAX_MB_BATCH          16

#define NV_VPE_MAX_MB_HEADER         20
#define NV_VPE_MAX_MB_DCT            (33 * 6)
#define NV_VPE_MAX_MB                (NV_VPE_MAX_MB_HEADER + NV_VPE_MAX_MB_DCT)

#define NV_VPE_CMD_TYPE_SHIFT          28

#define NV_VPE_CMD_NOP                0x1

#define NV_VPE_CMD_INIT_SURFACE       0x2
  #define NV_VPE_CMD_INIT_SURFACE_LUMA(index) ( (index * 2) << 24)
  #define NV_VPE_CMD_INIT_SURFACE_CHROMA(index) ( ( (index * 2) + 1) << 24)
  #define NV_VPE_CMD_INIT_SURFACE_OFFSET_DIV(offset) (offset >> 5)
  
#define NV_VPE_CMD_INIT_CHANNEL       0x3
  #define NV_VPE_CMD_INIT_CHANNEL_SURFACE_GROUP_INFO   0x1 /* ( (width round to 112) / 32 */
  #define NV_VPE_CMD_INIT_CHANNEL_ACCEL                0x2 /* (0x1 to turn on idct operations). */
         #define NV_VPE_CMD_INIT_CHANNEL_ACCEL_IDCT 0x1
    
#define NV_VPE_CMD_DCT_SEPARATOR      0x6
#define NV_VPE_CMD_END_SEQUENCE	      0x7

	#define NV_VPE_CMD_SEQUENCE       0x1

/* DCT Blocks */
#define NV_VPE_CMD_DCT_CHROMA_HEADER  0x8  
#define NV_VPE_CMD_DCT_LUMA_HEADER    0x9
	/* The block pattern is used for chroma and luma blocks */
	#define NV_VPE_CMD_DCT_BLOCK_PATTERN(p)  ( (p) << 24)
    /* Not sure what this is for. This is always set in the dct block header */
	#define NV_VPE_CMD_DCT_BLOCK_UNKNOWN  0x10000
    /* Target surface index. Is 0 based. */
	#define NV_VPE_CMD_DCT_BLOCK_TARGET_SURFACE(s)	(s << 20)
    /* If picture element is frame */
	#define NV_VPE_CMD_PICT_FRAME    0x80000
    /* If field based encoding and a luma block */
    #define NV_VPE_CMD_PICT_FRAME_FIELD 0x800000
    /* If picture element or field encoding is bottom field */
    #define NV_VD_VPE_CMD_BOTTOM_FIELD      0x20000
    /* If macroblock x coordinate is even */
	#define NV_VD_VPE_CMD_EVEN_X_COORD 	    0x8000
	
/* Used to terminate a set of dct data blocks.*/
#define NV_VPE_DCT_BLOCK_TERMINATOR   0x1
	
/* Used to designate dct data blocks that are all zero.*/
#define NV_VPE_DCT_BLOCK_NULL         (0x80040000 | NV_VPE_DCT_BLOCK_TERMINATOR)

/* Coordinates of dct */
#define NV_VPE_CMD_DCT_COORDINATE     0xA
    /* Luma */
	#define NV_VPE_DCT_POINTS_LUMA(x,y,p) ( ( (y * 16 * p) << 12 ) | (x * 16) )
    /* Chroma */
	#define NV_VPE_DCT_POINTS_CHROMA(x,y,p) ( ( (y * 8 * p) << 12 ) | (x * 16) )


/* Motion Vectors */
#define NV_VPE_CMD_LUMA_MOTION_VECTOR_HEADER   0xD
#define NV_VPE_CMD_CHROMA_MOTION_VECTOR_HEADER 0xC
#define NV_VPE_CMD_MOTION_VECTOR               0xE

    /* Motion Vector Header */
    
    /* Set if 2 motion vectors exist for this header. Otherwise, it is cleared and only 1 exists.*/
	#define NV_VPE_CMD_MC_MV_COUNT_2               (0x1 << 16)
	
	/* [Field Picture or Field Motion Only] motion_vertical_field_select is set here.  
	 * This means that the bottom field is selected for the given vertical vector. 
	 * However, dual-prime blocks do not follow this rule.
	 * It is treated speciallly for them.*/
	#define NV_VPE_CMD_BOTTOM_FIELD_VERTICAL_MOTION_SELECT_FIRST     (0x1 << 17)
	
	/* [Frame Picture and Frame Motion Type only] */
	#define NV_VPE_CMD_FRAME_PICT_FRAME_MOTION        (0x1 << 19)
	
	/* MC prediction surface index. Is 0 based. */
	#define NV_VPE_CMD_PREDICTION_SURFACE(s) 		    (s << 20)
	
	/* Set if this is a second motion vector. Otherwise, the first one is assumed.*/
	#define NV_VPE_CMD_MOTION_VECTOR_TYPE_SECOND      (0x1 << 23)
	
	/* [Frame Picture and Frame Motion Type OR Field Picture only]*/
	#define NV_VPE_CMD_FRAME_FRAME_PICT_OR_FIELD      (0x1 << 24)
	
	/* If Vertical Motion Vector is odd then set. This is before any operations are done. */
	#define NV_VPE_CMD_ODD_VERTICAL_MOTION_VECTOR     (0x1 << 25)
	
	/* If Horizontal Motion Vector is odd then set. This is before any operations are done. */
	#define NV_VPE_CMD_ODD_HORIZONTAL_MOTION_VECTOR   (0x1 << 26)
	
	/* If set then the motion vectors are backward.  Otherwise, they are forward.*/
	#define NV_VPE_CMD_MOTION_VECTOR_BACKWARD         (0x1 << 27)
	
	/* Motion Vectors. This is the equation used for each motion vector.
	 * d is only used as a second vector displacement in a couple of cases.
	 */
	#define NV_VPE_MOTION_VECTOR_VERTICAL(y, c, v, q, d)          ( ( (y * c) + (v / q) + d) << 12)
	#define NV_VPE_MOTION_VECTOR_HORIZONTAL(x, c, v, q, d)        ( (x * c) + (v / q) + d)

#endif
