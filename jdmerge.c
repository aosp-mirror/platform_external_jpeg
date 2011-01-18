/*
 * jdmerge.c
 *
 * Copyright (C) 1994-1996, Thomas G. Lane.
 * This file is part of the Independent JPEG Group's software.
 * For conditions of distribution and use, see the accompanying README file.
 *
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Added neon optimized merged upsampling/color conversion for often used
 * color formats.
 *
 * Author: Henrik Smiding henrik.smiding@stericsson.com for
 * ST-Ericsson.
 *
 * This file contains code for merged upsampling/color conversion.
 *
 * This file combines functions from jdsample.c and jdcolor.c;
 * read those files first to understand what's going on.
 *
 * When the chroma components are to be upsampled by simple replication
 * (ie, box filtering), we can save some work in color conversion by
 * calculating all the output pixels corresponding to a pair of chroma
 * samples at one time.  In the conversion equations
 *	R = Y           + K1 * Cr
 *	G = Y + K2 * Cb + K3 * Cr
 *	B = Y + K4 * Cb
 * only the Y term varies among the group of pixels corresponding to a pair
 * of chroma samples, so the rest of the terms can be calculated just once.
 * At typical sampling ratios, this eliminates half or three-quarters of the
 * multiplications needed for color conversion.
 *
 * This file currently provides implementations for the following cases:
 *	YCbCr => RGB color conversion only.
 *	Sampling ratios of 2h1v or 2h2v.
 *	No scaling needed at upsample time.
 *	Corner-aligned (non-CCIR601) sampling alignment.
 * Other special cases could be added, but in most applications these are
 * the only common cases.  (For uncommon cases we fall back on the more
 * general code in jdsample.c and jdcolor.c.)
 */

#define JPEG_INTERNALS
#include "jinclude.h"
#include "jpeglib.h"

#ifdef UPSAMPLE_MERGING_SUPPORTED

/* Private subobject */

typedef struct {
  struct jpeg_upsampler pub;	/* public fields */

  /* Pointer to routine to do actual upsampling/conversion of one row group */
  JMETHOD(void, upmethod, (j_decompress_ptr cinfo,
			   JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
			   JSAMPARRAY output_buf));

  /* Private state for YCC->RGB conversion */
  int * Cr_r_tab;		/* => table for Cr to R conversion */
  int * Cb_b_tab;		/* => table for Cb to B conversion */
  INT32 * Cr_g_tab;		/* => table for Cr to G conversion */
  INT32 * Cb_g_tab;		/* => table for Cb to G conversion */

  /* For 2:1 vertical sampling, we produce two output rows at a time.
   * We need a "spare" row buffer to hold the second output row if the
   * application provides just a one-row buffer; we also use the spare
   * to discard the dummy last row if the image height is odd.
   */
  JSAMPROW spare_row;
  boolean spare_full;		/* T if spare buffer is occupied */

  JDIMENSION out_row_width;	/* samples per output row */
  JDIMENSION rows_to_go;	/* counts rows remaining in image */
} my_upsampler;

typedef my_upsampler * my_upsample_ptr;

#define SCALEBITS	16	/* speediest right-shift on some machines */
#define ONE_HALF	((INT32) 1 << (SCALEBITS-1))
#define FIX(x)		((INT32) ((x) * (1L<<SCALEBITS) + 0.5))

#ifdef ANDROID_RGB

/* Declarations for ordered dithering.
 *
 * We use 4x4 ordered dither array packed into 32 bits. This array is
 * sufficent for dithering RGB_888 to RGB_565.
 */

#define DITHER_MASK         0x3
#define DITHER_ROTATE(x)    (((x)<<24) | (((x)>>8)&0x00FFFFFF))
static const INT32 dither_matrix[4] = {
  0x0008020A,
  0x0C040E06,
  0x030B0109,
  0x0F070D05
};

#if defined(__ARM_HAVE_NEON)
#include <arm_neon.h>

#if BITS_IN_JSAMPLE == 8
/* Defines to control which formats to use NEON optimizations for.
 * Comment out to disable
 */
#define ENABLE_NEON_YCC_RGB_TABLE
#define ENABLE_NEON_H2V2
#define ENABLE_NEON_H2V2_8888
#define ENABLE_NEON_H2V2_565
#define ENABLE_NEON_H2V2_565D
#define ENABLE_NEON_H2V1
#define ENABLE_NEON_H2V1_565
#define ENABLE_NEON_H2V1_565
#define ENABLE_NEON_H2V1_565D

/* Declarations for ordered dithering for NEON code.
 *
 * We use 4x4 ordered dither array, repeated three times to
 * allow an eight byte load from offsets 0 to 3.
 * sufficent for dithering eight pixels to RGB_565.
 */
static const uint8_t dither_matrix_neon[48] = {
  10,  2,  8,  0, 10,  2,  8,  0, 10,  2,  8,  0,
   6, 14,  4, 12,  6, 14,  4, 12,  6, 14,  4, 12,
   9,  1, 11,  3,  9,  1, 11,  3,  9,  1, 11,  3,
   5, 13,  7, 15,  5, 13,  7, 15,  5, 13,  7, 15
};

#define CR_R_CONST  "91881"   // Cr_r constant for red part of YCrCb to RGB conversion (1.40200 * 65536)
#define CR_G_CONST "-46802"   // Cr_g constant for green part of YCrCb to RGB conversion (-0.71414 * 65536)
#define CB_G_CONST "-22554"   // Cb_g constant for green part of YCrCb to RGB conversion (-0.34414 * 65536)
#define CB_B_CONST "116130"   // Cb_b constant for blue part of YCrCb to RGB conversion (1.77200 * 65536)

LOCAL(void)
clear_ycc_rgb_table(j_decompress_ptr cinfo)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;

  upsample->Cr_r_tab = NULL;
  upsample->Cb_b_tab = NULL;
  upsample->Cr_g_tab = NULL;
  upsample->Cb_g_tab = NULL;
}
#endif
#endif
#endif


/*
 * Initialize tables for YCC->RGB colorspace conversion.
 * This is taken directly from jdcolor.c; see that file for more info.
 */

LOCAL(void)
build_ycc_rgb_table (j_decompress_ptr cinfo)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;

  upsample->Cr_r_tab = (int *)
    (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE,
				(MAXJSAMPLE+1) * SIZEOF(int));
  upsample->Cb_b_tab = (int *)
    (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE,
				(MAXJSAMPLE+1) * SIZEOF(int));
  upsample->Cr_g_tab = (INT32 *)
    (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE,
				(MAXJSAMPLE+1) * SIZEOF(INT32));
  upsample->Cb_g_tab = (INT32 *)
    (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE,
				(MAXJSAMPLE+1) * SIZEOF(INT32));

#if defined(ENABLE_NEON_YCC_RGB_TABLE)
  static const INT32 start_values[4] = {-128, -127, -126, -125};

  asm volatile (
                // Setup constants
                "vld1.32        {q12}, [%[start_values]]        \n\t"   // Load initial values
                "vmov.i32       q10, #32768                     \n\t"   // Load ONE_HALF constant
                "ldr            r0, ="CR_R_CONST"               \n\t"   // Load Cr_r constant
                "vdup.32        q0, r0                          \n\t"   //
                "ldr            r0, ="CR_G_CONST"               \n\t"   // Load Cr_g constant
                "vmul.i32       q8, q0, q12                     \n\t"   // Calculate Cr_r start values
                "vdup.32        q1, r0                          \n\t"   //
                "vshl.i32       q0, #2                          \n\t"   // Calculate Cr_r step values
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant
                "vmul.i32       q9, q1, q12                     \n\t"   // Calculate Cr_g start values
                "vdup.32        q2, r0                          \n\t"   //
                "vshl.i32       q1, #2                          \n\t"   // Calculate Cb_b step values
                "ldr            r0, ="CB_B_CONST"               \n\t"   // Load Cb_b constant
                "vmla.i32       q10, q2, q12                    \n\t"   // Calculate Cb_g start values
                "vdup.32        q3, r0                          \n\t"   //
                "vshl.i32       q2, #2                          \n\t"   // Calculate Cr_g step values
                "vmul.i32       q11, q3, q12                    \n\t"   // Calculate Cb_b start values
                "vshl.i32       q3, #2                          \n\t"   // Calculate Cb_g step values
                "mov            r0, #64                         \n\t"   // Loop counter
                "1:                                             \n\t"   //
                "vrshr.s32      q12, q8, #16                    \n\t"   // Shift and round Cr_r values
                "vst1.32        {q12}, [%[cr_r_tab]]!           \n\t"   // Store Cr_r values
                "vadd.i32       q8, q0                          \n\t"   // Step Cr_r values
                "vst1.32        {q9}, [%[cr_g_tab]]!            \n\t"   // Store Cr_g values
                "vrshr.s32      q12, q11, #16                   \n\t"   // Shift and round Cb_b values
                "vst1.32        {q10}, [%[cb_g_tab]]!           \n\t"   // Store Cb_g values
                "vadd.i32       q11, q3                         \n\t"   // Step Cb_b values
                "vadd.i32       q9, q1                          \n\t"   // Step Cr_g values
                "vst1.32        {q12}, [%[cb_b_tab]]!           \n\t"   // Store Cb_b values
                "subs           r0, r0, #1                      \n\t"   // Decrement loop counter
                "vadd.i32       q10, q2                         \n\t"   // Step Cb_g values
                "bne            1b                              \n\t"   // If loop counter != 0, loop
                :
                : [start_values] "r" (start_values), [cr_r_tab] "r" (upsample->Cr_r_tab), [cr_g_tab] "r" (upsample->Cr_g_tab),
                  [cb_g_tab] "r" (upsample->Cb_g_tab), [cb_b_tab] "r" (upsample->Cb_b_tab)
                : "cc", "memory", "r0", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23", "d24", "d25"
                );
#else
  int i;
  INT32 x;
  SHIFT_TEMPS

  for (i = 0, x = -CENTERJSAMPLE; i <= MAXJSAMPLE; i++, x++) {
    /* i is the actual input pixel value, in the range 0..MAXJSAMPLE */
    /* The Cb or Cr value we are thinking of is x = i - CENTERJSAMPLE */
    /* Cr=>R value is nearest int to 1.40200 * x */
    upsample->Cr_r_tab[i] = (int)
		    RIGHT_SHIFT(FIX(1.40200) * x + ONE_HALF, SCALEBITS);
    /* Cb=>B value is nearest int to 1.77200 * x */
    upsample->Cb_b_tab[i] = (int)
		    RIGHT_SHIFT(FIX(1.77200) * x + ONE_HALF, SCALEBITS);
    /* Cr=>G value is scaled-up -0.71414 * x */
    upsample->Cr_g_tab[i] = (- FIX(0.71414)) * x;
    /* Cb=>G value is scaled-up -0.34414 * x */
    /* We also add in ONE_HALF so that need not do it in inner loop */
    upsample->Cb_g_tab[i] = (- FIX(0.34414)) * x + ONE_HALF;
  }
#endif
}


/*
 * Initialize for an upsampling pass.
 */

METHODDEF(void)
start_pass_merged_upsample (j_decompress_ptr cinfo)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;

  /* Mark the spare buffer empty */
  upsample->spare_full = FALSE;
  /* Initialize total-height counter for detecting bottom of image */
  upsample->rows_to_go = cinfo->output_height;
}


/*
 * Control routine to do upsampling (and color conversion).
 *
 * The control routine just handles the row buffering considerations.
 */

METHODDEF(void)
merged_2v_upsample (j_decompress_ptr cinfo,
		    JSAMPIMAGE input_buf, JDIMENSION *in_row_group_ctr,
		    JDIMENSION in_row_groups_avail,
		    JSAMPARRAY output_buf, JDIMENSION *out_row_ctr,
		    JDIMENSION out_rows_avail)
/* 2:1 vertical sampling case: may need a spare row. */
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;
  JSAMPROW work_ptrs[2];
  JDIMENSION num_rows;		/* number of rows returned to caller */

  if (upsample->spare_full) {
    /* If we have a spare row saved from a previous cycle, just return it. */
    JDIMENSION size = upsample->out_row_width;
#ifdef ANDROID_RGB
    if (cinfo->out_color_space == JCS_RGB_565)
      size = cinfo->output_width * 2;
#endif
    jcopy_sample_rows(& upsample->spare_row, 0, output_buf + *out_row_ctr, 0,
		      1, size);

    num_rows = 1;
    upsample->spare_full = FALSE;
  } else {
    /* Figure number of rows to return to caller. */
    num_rows = 2;
    /* Not more than the distance to the end of the image. */
    if (num_rows > upsample->rows_to_go)
      num_rows = upsample->rows_to_go;
    /* And not more than what the client can accept: */
    out_rows_avail -= *out_row_ctr;
    if (num_rows > out_rows_avail)
      num_rows = out_rows_avail;
    /* Create output pointer array for upsampler. */
    work_ptrs[0] = output_buf[*out_row_ctr];
    if (num_rows > 1) {
      work_ptrs[1] = output_buf[*out_row_ctr + 1];
    } else {
      work_ptrs[1] = upsample->spare_row;
      upsample->spare_full = TRUE;
    }
    /* Now do the upsampling. */
    (*upsample->upmethod) (cinfo, input_buf, *in_row_group_ctr, work_ptrs);
  }

  /* Adjust counts */
  *out_row_ctr += num_rows;
  upsample->rows_to_go -= num_rows;
  /* When the buffer is emptied, declare this input row group consumed */
  if (! upsample->spare_full)
    (*in_row_group_ctr)++;
}


METHODDEF(void)
merged_1v_upsample (j_decompress_ptr cinfo,
		    JSAMPIMAGE input_buf, JDIMENSION *in_row_group_ctr,
		    JDIMENSION in_row_groups_avail,
		    JSAMPARRAY output_buf, JDIMENSION *out_row_ctr,
		    JDIMENSION out_rows_avail)
/* 1:1 vertical sampling case: much easier, never need a spare row. */
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;

  /* Just do the upsampling. */
  (*upsample->upmethod) (cinfo, input_buf, *in_row_group_ctr,
			 output_buf + *out_row_ctr);
  /* Adjust counts */
  (*out_row_ctr)++;
  (*in_row_group_ctr)++;
}


/*
 * These are the routines invoked by the control routines to do
 * the actual upsampling/conversion.  One row group is processed per call.
 *
 * Note: since we may be writing directly into application-supplied buffers,
 * we have to be honest about the output width; we can't assume the buffer
 * has been rounded up to an even width.
 */


/*
 * Upsample and color convert for the case of 2:1 horizontal and 1:1 vertical.
 */

#define H2V1_Proc h2v1_merged_upsample
#define H2V1_Table TRUE

METHODDEF(void)
h2v1_merged_upsample (j_decompress_ptr cinfo,
		      JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
		      JSAMPARRAY output_buf)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;
  register int y, cred, cgreen, cblue;
  int cb, cr;
  register JSAMPROW outptr;
  JSAMPROW inptr0, inptr1, inptr2;
  JDIMENSION col;
  /* copy these pointers into registers if possible */
  register JSAMPLE * range_limit = cinfo->sample_range_limit;
  int * Crrtab = upsample->Cr_r_tab;
  int * Cbbtab = upsample->Cb_b_tab;
  INT32 * Crgtab = upsample->Cr_g_tab;
  INT32 * Cbgtab = upsample->Cb_g_tab;
  SHIFT_TEMPS

  inptr0 = input_buf[0][in_row_group_ctr];
  inptr1 = input_buf[1][in_row_group_ctr];
  inptr2 = input_buf[2][in_row_group_ctr];
  outptr = output_buf[0];
  /* Loop for each pair of output pixels */
  for (col = cinfo->output_width >> 1; col > 0; col--) {
    /* Do the chroma part of the calculation */
    cb = GETJSAMPLE(*inptr1++);
    cr = GETJSAMPLE(*inptr2++);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    /* Fetch 2 Y values and emit 2 pixels */
    y  = GETJSAMPLE(*inptr0++);
    outptr[RGB_RED] = range_limit[y + cred];
    outptr[RGB_GREEN] = range_limit[y + cgreen];
    outptr[RGB_BLUE] = range_limit[y + cblue];
    outptr += RGB_PIXELSIZE;
    y  = GETJSAMPLE(*inptr0++);
    outptr[RGB_RED] = range_limit[y + cred];
    outptr[RGB_GREEN] = range_limit[y + cgreen];
    outptr[RGB_BLUE] = range_limit[y + cblue];
    outptr += RGB_PIXELSIZE;
  }
  /* If image width is odd, do the last output column separately */
  if (cinfo->output_width & 1) {
    cb = GETJSAMPLE(*inptr1);
    cr = GETJSAMPLE(*inptr2);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    y  = GETJSAMPLE(*inptr0);
    outptr[RGB_RED] = range_limit[y + cred];
    outptr[RGB_GREEN] = range_limit[y + cgreen];
    outptr[RGB_BLUE] = range_limit[y + cblue];
  }
}

// Use NEON optimized version?
#if defined(ENABLE_NEON_H2V1)
#undef H2V1_Proc
#undef H2V1_Table
#define H2V1_Proc h2v1_merged_upsample_neon
#define H2V1_Table FALSE

METHODDEF(void)
h2v1_merged_upsample_neon (j_decompress_ptr cinfo,
		      JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
		      JSAMPARRAY output_buf)
{
  JDIMENSION num_cols = cinfo->output_width;

  // Fallback to non NEON method for small conversions
  if (num_cols < 8)
  {
    my_upsample_ptr upsample = (my_upsample_ptr)cinfo->upsample;

    if (upsample->Cr_r_tab == NULL) {
      build_ycc_rgb_table(cinfo);
    }
    h2v1_merged_upsample(cinfo, input_buf, in_row_group_ctr, output_buf);
    return;
  }

  asm volatile (
                // Setup constants
                "ldr            r1, [%[input_buf], #0]          \n\t"   // Setup y input pointer from input_buf[0]
                "vmov.i8        d19, #128                       \n\t"   // Set center sample constant to CENTERJSAMPLE
                "vmov.i32       q8, #32768                      \n\t"   // Set rounding constant to 32768 (0.5)
                "ldr            r0, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldr            r1, [r1, %[in_row], lsl #2]     \n\t"   // input_buf[0][in_row_group_ctr]
                "vdup.32        q0, r0                          \n\t"   //
                "vld1.8         {d31}, [r1]                     \n\t"   // Load eight y values
                "ldr            r0, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [%[input_buf], #4]          \n\t"   // Setup cb input pointer from input_buf[1][in_row_group_ctr]
                "vdup.32        q1, r0                          \n\t"   //
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [r2, %[in_row], lsl #2]     \n\t"   // input_buf[1][in_row_group_ctr]
                "vdup.32        q2, r0                          \n\t"   //
                "vld1.32        {d30[0]}, [r2]                  \n\t"   // Load four cb values
                "ldr            r0, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "ldr            r3, [%[input_buf], #8]          \n\t"   // Setup cr input pointer from input_buf[2][in_row_group_ctr]
                "vdup.32        q3, r0                          \n\t"   //
                "ldr            r3, [r3, %[in_row], lsl #2]     \n\t"   // input_buf[2][in_row_group_ctr]
                "ands           r0, %[num_cols], #0x6           \n\t"   // Should we do a partial first iteration?
                "vld1.32        {d30[1]}, [r3]                  \n\t"   // Load four cr values
                "moveq          r0, #8                          \n\t"   // ...or a full iteration
                "ldr            %[output_buf], [%[output_buf], #0]\n\t" // Get output pointer
                "and            r7, %[num_cols], #0xFFFFFFFE    \n\t"   // Setup loop counter
                "subs           r7, r7, #8                      \n\t"   // Subtract last iteration
                "beq            4f                              \n\t"
                // loop (columns)
                "3:                                             \n\t"   //
                // Read values, subtract 128 from cb/cr, and expand to 32-bit
                "vsubl.u8       q14, d30, d31                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q10, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q10, q2                    \n\t"   // Calculate green
                "add            r2, r0, lsr #1                  \n\t"   // Increment input pointer for cb
                "vmul.i32       q12, q10, q3                    \n\t"   // Calculate blue
                "vld1.32        {d30[0]}, [r2]                  \n\t"   // Pre-load next four cb values
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "add            r3, r0, lsr #1                  \n\t"   // Increment input pointer for cr
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vld1.32        {d30[1]}, [r3]                  \n\t"   // Pre-load next four cr values
                "vadd.i32       q12, q8                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q8                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q8                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result (Vector Shift Right and Insert)
                "vmovl.u8       q14, d31                        \n\t"   // Expand y to 16-bit
                "add            r1, r0                          \n\t"   // Increment input pointer for y
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                "subs           r7, r7, r0                      \n\t"   // Decrement loop counter
                // Add y (Vector add)
                "vld1.8         {d31}, [r1]                     \n\t"   // Pre-load next eight y values
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d23, q13                        \n\t"   // Convert green
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                "vst3.8         {d22, d23, d24}, [%[output_buf]]\n\t"   // Write result to memory
                "add            %[output_buf], r0, lsl #1       \n\t"   // Increment output buffer pointer
                "add            %[output_buf], r0               \n\t"
                // Increase pointers and counters
                "mov            r0, #8                          \n\t"   // Set next loop iteration length
                "bne            3b                              \n\t"   // If loop counter != 0, loop
                // last iteration (post loop)
                "4:                                             \n\t"   //
                "vsubl.u8       q14, d30, d19                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q10, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q10, q2                    \n\t"   // Calculate green
                "vmul.i32       q12, q10, q3                    \n\t"   // Calculate blue
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vadd.i32       q12, q8                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q8                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q8                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result (Vector Shift Right and Insert)
                "vmovl.u8       q14, d31                        \n\t"   // Expand y to 16-bit
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                // Add y (Vector add)
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d23, q13                        \n\t"   // Convert green
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                "vst3.8         {d22, d23, d24}, [%[output_buf]]!\n\t"  // Write result to memory
                // Handle any last odd pixel
                "tst            %[num_cols], #1                 \n\t"   // Is there a last odd pixel?
                "beq            5f                              \n\t"   // If not, exit
                "ldr            r7, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldrb           r3, [r3, #4]                    \n\t"   // Load cr value
                "ldrb           r2, [r2, #4]                    \n\t"   // Load cb value
                "ldrb           r1, [r1, #8]                    \n\t"   // Load y value
                // Calculate red
                "sub            r3, #128                        \n\t"   // Subtract CENTERJSAMPLE from cr
                "smlawb         r0, r7, r3, r1                  \n\t"   // Calculate red and add y
                "ldr            r7, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "strb           r0, [%[output_buf]]             \n\t"   // Store red
                // Calculate green
                "smlawb         r0, r7, r3, r1                  \n\t"   // Calculate green and add y
                "ldr            r7, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "sub            r2, #128                        \n\t"   // Subtract CENTERJSAMPLE from cb
                "smlawb         r0, r7, r2, r0                  \n\t"   // Calculate second green
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "ldr            r7, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "strb           r0, [%[output_buf], #1]         \n\t"   // Store green
                // Calculate blue
                "smlawb         r0, r7, r2, r1                  \n\t"   // Calculate blue and add y
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "strb           r0, [%[output_buf], #2]         \n\t"   // Store blue
                "5:                                             \n\t"   //
                : [output_buf] "+r" (output_buf)
                : [in_row] "r" (in_row_group_ctr), [input_buf] "r" (input_buf), [num_cols] "r" (num_cols)
                : "cc", "memory", "r0", "r1", "r2", "r3", "r7", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d16", "d17", "d19", "d20", "d21", "d22", "d23", "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31"
                );
}

#endif


#ifdef ANDROID_RGB
#define H2V1_8888_Proc h2v1_merged_upsample_8888
#define H2V1_8888_Table TRUE
#define H2V1_565_Proc h2v1_merged_upsample_565
#define H2V1_565_Table TRUE
#define H2V1_565D_Proc h2v1_merged_upsample_565D
#define H2V1_565D_Table TRUE

METHODDEF(void)
h2v1_merged_upsample_8888 (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;
  register int y, cred, cgreen, cblue;
  int cb, cr;
  register JSAMPROW outptr;
  JSAMPROW inptr0, inptr1, inptr2;
  JDIMENSION col;
  /* copy these pointers into registers if possible */
  register JSAMPLE * range_limit = cinfo->sample_range_limit;
  int * Crrtab = upsample->Cr_r_tab;
  int * Cbbtab = upsample->Cb_b_tab;
  INT32 * Crgtab = upsample->Cr_g_tab;
  INT32 * Cbgtab = upsample->Cb_g_tab;
  SHIFT_TEMPS

  inptr0 = input_buf[0][in_row_group_ctr];
  inptr1 = input_buf[1][in_row_group_ctr];
  inptr2 = input_buf[2][in_row_group_ctr];
  outptr = output_buf[0];
  /* Loop for each pair of output pixels */
  for (col = cinfo->output_width >> 1; col > 0; col--) {
    /* Do the chroma part of the calculation */
    cb = GETJSAMPLE(*inptr1++);
    cr = GETJSAMPLE(*inptr2++);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    /* Fetch 2 Y values and emit 2 pixels */
    y  = GETJSAMPLE(*inptr0++);
    outptr[RGB_RED] = range_limit[y + cred];
    outptr[RGB_GREEN] = range_limit[y + cgreen];
    outptr[RGB_BLUE] = range_limit[y + cblue];
    outptr[RGB_ALPHA] = 0xFF;
    outptr += 4;
    y  = GETJSAMPLE(*inptr0++);
    outptr[RGB_RED] = range_limit[y + cred];
    outptr[RGB_GREEN] = range_limit[y + cgreen];
    outptr[RGB_BLUE] = range_limit[y + cblue];
    outptr[RGB_ALPHA] = 0xFF;
    outptr += 4;
  }
  /* If image width is odd, do the last output column separately */
  if (cinfo->output_width & 1) {
    cb = GETJSAMPLE(*inptr1);
    cr = GETJSAMPLE(*inptr2);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    y  = GETJSAMPLE(*inptr0);
    outptr[RGB_RED] = range_limit[y + cred];
    outptr[RGB_GREEN] = range_limit[y + cgreen];
    outptr[RGB_BLUE] = range_limit[y + cblue];
    outptr[RGB_ALPHA] = 0xFF;
  }
}

#if defined(ENABLE_NEON_H2V1_8888)
#undef H2V1_8888_Proc
#undef H2V1_8888_Table
#define H2V1_8888_Proc h2v1_merged_upsample_8888_neon
#define H2V1_8888_Table FALSE

METHODDEF(void)
h2v1_merged_upsample_8888_neon (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  JDIMENSION num_cols = cinfo->output_width;

  // Fallback to non NEON method for small conversions
  if (num_cols < 8)
  {
    my_upsample_ptr upsample = (my_upsample_ptr)cinfo->upsample;

    if (upsample->Cr_r_tab == NULL) {
      build_ycc_rgb_table(cinfo);
    }
    h2v1_merged_upsample_8888(cinfo, input_buf, in_row_group_ctr, output_buf);
    return;
  }

  asm volatile (
                // Setup constants
                "ldr            r1, [%[input_buf], #0]          \n\t"   // Setup y input pointer from input_buf[0]
                "vmov.i8        d19, #128                       \n\t"   // Set center sample constant to CENTERJSAMPLE
                "vmov.i32       q8, #32768                      \n\t"   // Set rounding constant to 32768 (0.5)
                "ldr            r0, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldr            r1, [r1, %[in_row], lsl #2]     \n\t"   // input_buf[0][in_row_group_ctr]
                "vdup.32        q0, r0                          \n\t"   //
                "vld1.8         {d31}, [r1]                     \n\t"   // Load eight y values
                "ldr            r0, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [%[input_buf], #4]          \n\t"   // Setup cb input pointer from input_buf[1][in_row_group_ctr]
                "vdup.32        q1, r0                          \n\t"   //
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [r2, %[in_row], lsl #2]     \n\t"   //
                "vdup.32        q2, r0                          \n\t"   //
                "vld1.32        {d30[0]}, [r2]                  \n\t"   // Load four cb values
                "ldr            r0, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "ldr            r3, [%[input_buf], #8]          \n\t"   // Setup cr input pointer from input_buf[2][in_row_group_ctr]
                "vdup.32        q3, r0                          \n\t"   //
                "ldr            r3, [r3, %[in_row], lsl #2]     \n\t"   //
                "ands           r0, %[num_cols], #0x6           \n\t"   // Should we do a partial first iteration?
                "vld1.32        {d30[1]}, [r3]                  \n\t"   // Load four cr values
                "moveq          r0, #8                          \n\t"   // ...or a full iteration
                "ldr            %[output_buf], [%[output_buf], #0]\n\t" // Get output pointer
                "and            r7, %[num_cols], #0xFFFFFFFE    \n\t"   // Setup loop counter
                "subs           r7, r7, #8                      \n\t"   // Subtract last iteration
                "beq            4f                              \n\t"
                // loop (columns)
                "3:                                             \n\t"   //
                // Read values, subtract 128 from cb/cr, and expand to 32-bit
                "vsubl.u8       q14, d30, d19                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q10, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q10, q2                    \n\t"   // Calculate green
                "add            r2, r0, lsr #1                  \n\t"   // Increment input pointer for cb
                "vmul.i32       q12, q10, q3                    \n\t"   // Calculate blue
                "vld1.32        {d30[0]}, [r2]                  \n\t"   // Load four cb values
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "add            r3, r0, lsr #1                  \n\t"   // Increment input pointer for cr
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vld1.32        {d30[1]}, [r3]                  \n\t"   // Load four cr values
                "vadd.i32       q12, q8                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q8                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q8                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result (Vector Shift Right and Insert)
                "vmovl.u8       q14, d31                        \n\t"   // Expand y to 16-bit
                "add            r1, r0                          \n\t"   // Increment input pointer for y
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                "subs           r7, r7, r0                      \n\t"   // Decrement loop counter
                // Add y (Vector add)
                "vld1.8         {d31}, [r1]                     \n\t"   // Load eight y values
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d23, q13                        \n\t"   // Convert green
                "vmov.i8        d25, #0xFF                      \n\t"   // Set alpha to 0xFF
                "vst4.8         {d22, d23, d24, d25}, [%[output_buf]]\n\t"// Write result to memory
                "add            %[output_buf], r0, lsl #2       \n\t"   // Increment output buffer pointer
                // Increase pointers and counters
                "mov            r0, #8                          \n\t"   // Set next loop iteration length
                "bne            3b                              \n\t"   // If loop counter != 0, loop
                // last iteration (post loop)
                "4:                                             \n\t"   //
                "vsubl.u8       q14, d30, d19                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q10, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q10, q2                    \n\t"   // Calculate green
                "vmul.i32       q12, q10, q3                    \n\t"   // Calculate blue
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vadd.i32       q12, q8                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q8                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q8                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result (Vector Shift Right and Insert)
                "vmovl.u8       q14, d31                        \n\t"   // Expand y to 16-bit
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                // Add y (Vector add)
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d23, q13                        \n\t"   // Convert green
                "vmov.i8        d25, #0xFF                      \n\t"   // Set alpha to 0xFF
                "vst4.8         {d22, d23, d24, d25}, [%[output_buf]]!\n\t"// Write result to memory
                // Handle any last odd pixel
                "tst            %[num_cols], #1                 \n\t"   // Is there a last odd pixel?
                "beq            5f                              \n\t"   // If not, exit
                "ldr            r7, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldrb           r3, [r3, #4]                    \n\t"   // Load cr value
                "ldrb           r2, [r2, #4]                    \n\t"   // Load cb value
                "ldrb           r1, [r1, #8]                    \n\t"   // Load y value
                // Calculate red
                "sub            r3, #128                        \n\t"   // Subtract CENTERJSAMPLE from cr
                "smlawb         r0, r7, r3, r1                  \n\t"   // Calculate red and add y
                "ldr            r7, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "strb           r0, [%[output_buf]]             \n\t"   // Store red
                // Calculate green
                "smlawb         r0, r7, r3, r1                  \n\t"   // Calculate green and add y
                "ldr            r7, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "sub            r2, #128                        \n\t"   // Subtract CENTERJSAMPLE from cb
                "smlawb         r0, r7, r2, r0                  \n\t"   // Calculate second green
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "ldr            r7, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "strb           r0, [%[output_buf], #1]         \n\t"   // Store green
                // Calculate blue
                "smlawb         r0, r7, r2, r1                  \n\t"   // Calculate blue and add y
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "mov            r7, #0xFF                       \n\t"   // Load alpha 0xFF
                "strb           r0, [%[output_buf], #2]         \n\t"   // Store blue
                "strb           r7, [%[output_buf], #3]         \n\t"   // Store alpha
                "5:                                             \n\t"   //
                : [output_buf] "+r" (output_buf)
                : [in_row] "r" (in_row_group_ctr), [input_buf] "r" (input_buf), [num_cols] "r" (num_cols)
                : "cc", "memory", "r0", "r1", "r2", "r3", "r7", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d16", "d17", "d19", "d20", "d21", "d22", "d23", "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31"
                );
}
#endif


METHODDEF(void)
h2v1_merged_upsample_565 (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;
  register int y, cred, cgreen, cblue;
  int cb, cr;
  register JSAMPROW outptr;
  JSAMPROW inptr0, inptr1, inptr2;
  JDIMENSION col;
  /* copy these pointers into registers if possible */
  register JSAMPLE * range_limit = cinfo->sample_range_limit;
  int * Crrtab = upsample->Cr_r_tab;
  int * Cbbtab = upsample->Cb_b_tab;
  INT32 * Crgtab = upsample->Cr_g_tab;
  INT32 * Cbgtab = upsample->Cb_g_tab;
  unsigned int r, g, b;
  INT32 rgb;
  SHIFT_TEMPS

  inptr0 = input_buf[0][in_row_group_ctr];
  inptr1 = input_buf[1][in_row_group_ctr];
  inptr2 = input_buf[2][in_row_group_ctr];
  outptr = output_buf[0];
  /* Loop for each pair of output pixels */
  for (col = cinfo->output_width >> 1; col > 0; col--) {
    /* Do the chroma part of the calculation */
    cb = GETJSAMPLE(*inptr1++);
    cr = GETJSAMPLE(*inptr2++);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    /* Fetch 2 Y values and emit 2 pixels */
    y  = GETJSAMPLE(*inptr0++);
    r = range_limit[y + cred];
    g = range_limit[y + cgreen];
    b = range_limit[y + cblue];
    rgb = PACK_SHORT_565(r,g,b);
    y  = GETJSAMPLE(*inptr0++);
    r = range_limit[y + cred];
    g = range_limit[y + cgreen];
    b = range_limit[y + cblue];
    rgb = PACK_TWO_PIXELS(rgb, PACK_SHORT_565(r,g,b));
    WRITE_TWO_PIXELS(outptr, rgb);
    outptr += 4;
  }
  /* If image width is odd, do the last output column separately */
  if (cinfo->output_width & 1) {
    cb = GETJSAMPLE(*inptr1);
    cr = GETJSAMPLE(*inptr2);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    y  = GETJSAMPLE(*inptr0);
    r = range_limit[y + cred];
    g = range_limit[y + cgreen];
    b = range_limit[y + cblue];
    rgb = PACK_SHORT_565(r,g,b);
    *(INT16*)outptr = rgb;
  }
}

#if defined(ENABLE_NEON_H2V1_565)
#undef H2V1_565_Proc
#undef H2V1_565_Table
#define H2V1_565_Proc h2v1_merged_upsample_565_neon
#define H2V1_565_Table FALSE

METHODDEF(void)
h2v1_merged_upsample_565_neon (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  JDIMENSION num_cols = cinfo->output_width;

  // Fallback to non NEON method for small conversions
  if (num_cols < 8)
  {
    my_upsample_ptr upsample = (my_upsample_ptr)cinfo->upsample;

    if (upsample->Cr_r_tab == NULL) {
      build_ycc_rgb_table(cinfo);
    }
    h2v1_merged_upsample_565(cinfo, input_buf, in_row_group_ctr, output_buf);
    return;
  }

  asm volatile (
                // Setup constants
                "ldr            r1, [%[input_buf], #0]          \n\t"   // Setup y input pointer from input_buf[0]
                "vmov.i8        d19, #128                       \n\t"   // Set center sample constant to CENTERJSAMPLE
                "vmov.i32       q8, #32768                      \n\t"   // Set rounding constant to 32768 (0.5)
                "ldr            r0, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldr            r1, [r1, %[in_row], lsl #2]     \n\t"   // input_buf[0][in_row_group_ctr]
                "vdup.32        q0, r0                          \n\t"   //
                "vld1.8         {d31}, [r1]                     \n\t"   // Load eight y values
                "ldr            r0, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [%[input_buf], #4]          \n\t"   // Setup cb input pointer from input_buf[1][in_row_group_ctr]
                "vdup.32        q1, r0                          \n\t"   //
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [r2, %[in_row], lsl #2]     \n\t"   //
                "vdup.32        q2, r0                          \n\t"   //
                "vld1.32        {d30[0]}, [r2]                  \n\t"   // Load four cb values
                "ldr            r0, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "ldr            r3, [%[input_buf], #8]          \n\t"   // Setup cr input pointer from input_buf[2][in_row_group_ctr]
                "vdup.32        q3, r0                          \n\t"   //
                "ldr            r3, [r3, %[in_row], lsl #2]     \n\t"   //
                "ands           r0, %[num_cols], #0x6           \n\t"   // Should we do a partial first iteration?
                "vld1.32        {d30[1]}, [r3]                  \n\t"   // Load four cr values
                "moveq          r0, #8                          \n\t"   // ...or a full iteration
                "ldr            %[output_buf], [%[output_buf], #0]\n\t" // Get output pointer
                "and            r7, %[num_cols], #0xFFFFFFFE    \n\t"   // Setup loop counter
                "subs           r7, r7, #8                      \n\t"   // Subtract last iteration
                "beq            4f                              \n\t"
                // loop (columns)
                "3:                                             \n\t"   //
                // Read values, subtract 128 from cb/cr, and expand to 32-bit
                "vsubl.u8       q14, d30, d31                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q10, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q10, q2                    \n\t"   // Calculate green
                "add            r2, r0, lsr #1                  \n\t"   // Increment input pointer for cb
                "vmul.i32       q12, q10, q3                    \n\t"   // Calculate blue
                "vld1.32        {d30[0]}, [r2]                  \n\t"   // Load four cb values
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "add            r3, r0, lsr #1                  \n\t"   // Increment input pointer for cr
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vld1.32        {d30[1]}, [r3]                  \n\t"   // Load four cr values
                "vadd.i32       q12, q8                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q8                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q8                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result (Vector Shift Right and Insert)
                "vmovl.u8       q14, d31                        \n\t"   // Expand y to 16-bit
                "add            r1, r0                          \n\t"   // Increment input pointer for y
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                "subs           r7, r7, r0                      \n\t"   // Decrement loop counter
                // Add y (Vector add)
                "vld1.8         {d31}, [r1]                     \n\t"   // Load eight y values
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d26, q13                        \n\t"   // Convert green
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                // Pack to 565 format
                "vshll.u8       q11, d22, #8                    \n\t"   // Shift red and expand to 16-bit
                "vshll.u8       q13, d26, #8                    \n\t"   // Shift green and expand to 16-bit
                "vshll.u8       q12, d24, #8                    \n\t"   // Shift blue and expand to 16-bit
                "vsri.u16       q11, q13, #5                    \n\t"   // Insert green into red
                "vsri.u16       q11, q12, #11                   \n\t"   // Insert blue into red
                "vst1.16        {q11}, [%[output_buf]]          \n\t"   // Write row to memory
                "add            %[output_buf], r0, lsl #1       \n\t"   // Increment output buffer pointer
                // Increase pointers and counters
                "mov            r0, #8                          \n\t"   // Set next loop iteration length
                "bne            3b                              \n\t"   // If loop counter != 0, loop
                // last iteration (post loop)
                "4:                                             \n\t"   //
                "vsubl.u8       q14, d30, d19                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q10, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q10, q2                    \n\t"   // Calculate green
                "vmul.i32       q12, q10, q3                    \n\t"   // Calculate blue
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vadd.i32       q12, q8                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q8                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q8                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result (Vector Shift Right and Insert)
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                // Add y (Vector add)
                "vmovl.u8       q14, d31                        \n\t"   // Expand y to 16-bit
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d26, q13                        \n\t"   // Convert green
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                // Pack to 565 format
                "vshll.u8       q13, d26, #8                    \n\t"   // Shift green and expand to 16-bit
                "vshll.u8       q11, d22, #8                    \n\t"   // Shift red and expand to 16-bit
                "vshll.u8       q12, d24, #8                    \n\t"   // Shift blue and expand to 16-bit
                "vsri.u16       q11, q13, #5                    \n\t"   // Insert green into red
                "vsri.u16       q11, q12, #11                   \n\t"   // Insert blue into red
                "vst1.16        {q11}, [%[output_buf]]!         \n\t"   // Write row to memory
                // Handle any last odd pixel
                "tst            %[num_cols], #1                 \n\t"   // Is there a last odd pixel?
                "beq            5f                              \n\t"   // If not, exit
                "ldr            r7, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldrb           r3, [r3, #4]                    \n\t"   // Load cr value
                "ldrb           r2, [r2, #4]                    \n\t"   // Load cb value
                "ldrb           r1, [r1, #8]                    \n\t"   // Load y value
                // Calculate red
                "sub            r3, #128                        \n\t"   // Subtract CENTERJSAMPLE from cr
                "smlawb         r0, r7, r3, r1                  \n\t"   // Calculate red and add y
                "ldr            r7, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "usat           r0, #5, r0, asr #3              \n\t"   // Saturate to 0-31
                // Calculate green
                "smlawb         r3, r7, r3, r1                  \n\t"   // Calculate green and add y
                "ldr            r7, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "sub            r2, #128                        \n\t"   // Subtract CENTERJSAMPLE from cb
                "smlawb         r3, r7, r2, r3                  \n\t"   // Calculate second green
                "usat           r3, #6, r3, asr #2              \n\t"   // Saturate to 0-63
                // Calculate blue
                "ldr            r7, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "smlawb         r2, r7, r2, r1                  \n\t"   // Calculate blue and add y
                "usat           r2, #5, r2, asr #3              \n\t"   // Saturate to 0-31
                // Store last pixel
                "orr            r2, r3, lsl #5                  \n\t"   // Insert green into blue
                "orr            r2, r0, lsl #11                 \n\t"   // Insert red into blue
                "strh           r2, [%[output_buf]]             \n\t"   // Store RGB
                "5:                                             \n\t"   //
                : [output_buf] "+r" (output_buf)
                : [in_row] "r" (in_row_group_ctr), [input_buf] "r" (input_buf), [num_cols] "r" (num_cols)
                : "cc", "memory", "r0", "r1", "r2", "r3", "r7", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d16", "d17", "d19", "d20", "d21", "d22", "d23", "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31"
                );
}
#endif


METHODDEF(void)
h2v1_merged_upsample_565D (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;
  register int y, cred, cgreen, cblue;
  int cb, cr;
  register JSAMPROW outptr;
  JSAMPROW inptr0, inptr1, inptr2;
  JDIMENSION col;
  /* copy these pointers into registers if possible */
  register JSAMPLE * range_limit = cinfo->sample_range_limit;
  int * Crrtab = upsample->Cr_r_tab;
  int * Cbbtab = upsample->Cb_b_tab;
  INT32 * Crgtab = upsample->Cr_g_tab;
  INT32 * Cbgtab = upsample->Cb_g_tab;
  JDIMENSION col_index = 0;
  INT32 d0 = dither_matrix[cinfo->output_scanline & DITHER_MASK];
  unsigned int r, g, b;
  INT32 rgb;
  SHIFT_TEMPS

  inptr0 = input_buf[0][in_row_group_ctr];
  inptr1 = input_buf[1][in_row_group_ctr];
  inptr2 = input_buf[2][in_row_group_ctr];
  outptr = output_buf[0];
  /* Loop for each pair of output pixels */
  for (col = cinfo->output_width >> 1; col > 0; col--) {
    /* Do the chroma part of the calculation */
    cb = GETJSAMPLE(*inptr1++);
    cr = GETJSAMPLE(*inptr2++);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    /* Fetch 2 Y values and emit 2 pixels */
    y  = GETJSAMPLE(*inptr0++);
    r = range_limit[DITHER_565_R(y + cred, d0)];
    g = range_limit[DITHER_565_G(y + cgreen, d0)];
    b = range_limit[DITHER_565_B(y + cblue, d0)];
    d0 = DITHER_ROTATE(d0);
    rgb = PACK_SHORT_565(r,g,b);
    y  = GETJSAMPLE(*inptr0++);
    r = range_limit[DITHER_565_R(y + cred, d0)];
    g = range_limit[DITHER_565_G(y + cgreen, d0)];
    b = range_limit[DITHER_565_B(y + cblue, d0)];
    d0 = DITHER_ROTATE(d0);
    rgb = PACK_TWO_PIXELS(rgb, PACK_SHORT_565(r,g,b));
    WRITE_TWO_PIXELS(outptr, rgb);
    outptr += 4;
  }
  /* If image width is odd, do the last output column separately */
  if (cinfo->output_width & 1) {
    cb = GETJSAMPLE(*inptr1);
    cr = GETJSAMPLE(*inptr2);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    y  = GETJSAMPLE(*inptr0);
    r = range_limit[DITHER_565_R(y + cred, d0)];
    g = range_limit[DITHER_565_G(y + cgreen, d0)];
    b = range_limit[DITHER_565_B(y + cblue, d0)];
    rgb = PACK_SHORT_565(r,g,b);
    *(INT16*)outptr = rgb;
  }
}


#if defined(ENABLE_NEON_H2V1_565D)
#undef H2V1_565D_Proc
#undef H2V1_565D_Table
#define H2V1_565D_Proc h2v1_merged_upsample_565D_neon
#define H2V1_565D_Table FALSE

METHODDEF(void)
h2v1_merged_upsample_565D_neon (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  JDIMENSION num_cols = cinfo->output_width;
  uint8_t* matrix = (uint8_t*)dither_matrix_neon + ((cinfo->output_scanline & DITHER_MASK) * 12);

  // Fallback to non NEON method for small conversions
  if (num_cols < 8)
  {
    my_upsample_ptr upsample = (my_upsample_ptr)cinfo->upsample;

    if (upsample->Cr_r_tab == NULL) {
      build_ycc_rgb_table(cinfo);
    }
    h2v1_merged_upsample_565D(cinfo, input_buf, in_row_group_ctr, output_buf);
    return;
  }

  asm volatile (
                // Setup constants
                "ldr            r1, [%[input_buf], #0]          \n\t"   // Setup y input pointer from input_buf[0]
                "vmov.i8        d17, #128                       \n\t"   // Set center sample constant to CENTERJSAMPLE
                "vmov.i32       q10, #32768                      \n\t"   // Set rounding constant to 32768 (0.5)
                "ldr            r0, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldr            r1, [r1, %[in_row], lsl #2]     \n\t"   // input_buf[0][in_row_group_ctr]
                "vdup.32        q0, r0                          \n\t"   //
                "vld1.8         {d31}, [r1]                     \n\t"   // Load eight y values
                "ldr            r0, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [%[input_buf], #4]          \n\t"   // Setup cb input pointer from input_buf[1][in_row_group_ctr]
                "vdup.32        q1, r0                          \n\t"   //
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [r2, %[in_row], lsl #2]     \n\t"   //
                "vdup.32        q2, r0                          \n\t"   //
                "vld1.32        {d30[0]}, [r2]                  \n\t"   // Load four cb values
                "ldr            r0, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "ldr            r3, [%[input_buf], #8]          \n\t"   // Setup cr input pointer from input_buf[2][in_row_group_ctr]
                "vdup.32        q3, r0                          \n\t"   //
                "ldr            r3, [r3, %[in_row], lsl #2]     \n\t"   //
                "ands           r0, %[num_cols], #0x6           \n\t"   // Should we do a partial first iteration?
                "vld1.32        {d30[1]}, [r3]                  \n\t"   // Load four cr values
                "vld1.8         {d20}, [%[matrix]]              \n\t"   //
                "beq            1f                              \n\t"   //
                "and            r7, r0, #0x3                    \n\t"   // If columns are not even eight, calculate offset in matrix array
                "add            %[matrix], r7                   \n\t"   //
                "vld1.8         {d21}, [%[matrix]]              \n\t"   // ...and load iteration 2+ dither values for row 1
                "b              2f                              \n\t"   //
                "1:                                             \n\t"   //
                "vmov           d21, d20                        \n\t"   // If columns are even eight, use the same dither matrix for all iterations
                "mov            r0, #8                          \n\t"   // Do full iteration
                "2:                                             \n\t"   //
                "ldr            %[output_buf], [%[output_buf], #0]\n\t" // Get output pointer
                "and            r7, %[num_cols], #0xFFFFFFFE    \n\t"   // Setup loop counter
                "subs           r7, r7, #8                      \n\t"   // Subtract last iteration
                "beq            4f                              \n\t"
                // loop (columns)
                "3:                                             \n\t"   //
                // Read values, subtract 128 from cb/cr, and expand to 32-bit
                "vsubl.u8       q14, d30, d31                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q10, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q10, q2                    \n\t"   // Calculate green
                "add            r2, r0, lsr #1                  \n\t"   // Increment input pointer for cb
                "vmul.i32       q12, q12, q3                    \n\t"   // Calculate blue
                "vld1.32        {d30[0]}, [r2]                  \n\t"   // Load four cb values
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "add            r3, r0, lsr #1                  \n\t"   // Increment input pointer for cr
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vld1.32        {d30[1]}, [r3]                  \n\t"   // Load four cr values
                "vadd.i32       q12, q10                        \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q10                        \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q10                        \n\t"   // Add 0.5 to red
                // Duplicate RGB result (Vector Shift Right and Insert)
                "vmovl.u8       q14, d31                        \n\t"   // Expand y to 16-bit
                "add            r1, r0                          \n\t"   // Increment input pointer for y
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                "subs           r7, r7, r0                      \n\t"   // Decrement loop counter
                // Add y (Vector add)
                "vld1.8         {d31}, [r1]                     \n\t"   // Load eight y values
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                // Do the dither
                "vaddw.s8       q12, d20                        \n\t"   // Add dither to blue
                "vaddw.s8       q11, d20                        \n\t"   // Add dither to red
                "vshr.s8        d20, #1                         \n\t"   // Shift green dither by one, since green will use 6 bits
                "vaddw.s8       q13, d20                        \n\t"   // Add dither to green
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d26, q13                        \n\t"   // Convert green
                // Pack to 565 format
                "vshll.u8       q11, d22, #8                    \n\t"   // Shift red and expand to 16-bit
                "vshll.u8       q13, d26, #8                    \n\t"   // Shift green and expand to 16-bit
                "vshll.u8       q12, d24, #8                    \n\t"   // Shift blue and expand to 16-bit
                "vsri.u16       q11, q13, #5                    \n\t"   // Insert green into red
                "vsri.u16       q11, q12, #11                   \n\t"   // Insert blue into red
                "vst1.16        {q11}, [%[output_buf]]          \n\t"   // Write row to memory
                "add            %[output_buf], r0, lsl #1       \n\t"   // Increment output buffer pointer
                // Increase pointers and counters
                "vmov.i8        d18, d19                        \n\t"   // Set dither matrix to iteration 2+ values
                "mov            r0, #8                          \n\t"   // Set next loop iteration length
                "bne            3b                              \n\t"   // If loop counter != 0, loop
                // last iteration (post loop)
                "4:                                             \n\t"   //
                "vsubl.u8       q14, d30, d17                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q12, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q12, q2                    \n\t"   // Calculate green
                "vmul.i32       q12, q12, q3                    \n\t"   // Calculate blue
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vadd.i32       q12, q10                        \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q10                        \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q10                        \n\t"   // Add 0.5 to red
                // Duplicate RGB result (Vector Shift Right and Insert)
                "vmovl.u8       q14, d31                        \n\t"   // Expand y to 16-bit
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                // Add y (Vector add)
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                // Do the dither
                "vaddw.s8       q12, d19                        \n\t"   // Add dither to blue
                "vaddw.s8       q11, d19                        \n\t"   // Add dither to red
                "vshr.s8        d19, #1                         \n\t"   // Shift green dither by one, since green will use 6 bits
                "vaddw.s8       q13, d19                        \n\t"   // Add dither to green
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d26, q13                        \n\t"   // Convert green
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                // Pack to 565 format
                "vshll.u8       q11, d22, #8                    \n\t"   // Shift red and expand to 16-bit
                "vshll.u8       q13, d26, #8                    \n\t"   // Shift green and expand to 16-bit
                "vshll.u8       q12, d24, #8                    \n\t"   // Shift blue and expand to 16-bit
                "vsri.u16       q11, q13, #5                    \n\t"   // Insert green into red
                "vsri.u16       q11, q12, #11                   \n\t"   // Insert blue into red
                "vst1.16        {q11}, [%[output_buf]]!         \n\t"   // Write row to memory
                // Handle any last odd pixel
                "tst            %[num_cols], #1                 \n\t"   // Is there a last odd pixel?
                "beq            5f                              \n\t"   // If not, exit
                "ldr            r7, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldrb           r3, [r3, #4]                    \n\t"   // Load cr value
                "ldrb           r2, [r2, #4]                    \n\t"   // Load cb value
                "ldrb           r1, [r1, #8]                    \n\t"   // Load y value
                "ldrb           %[matrix], [%[matrix]]          \n\t"   // Load dither matrix value
                // Calculate red
                "sub            r3, #128                        \n\t"   // Subtract CENTERJSAMPLE from cr
                "smlawb         r0, r7, r3, r1                  \n\t"   // Calculate red and add y
                "add            r0, %[matrix]                   \n\t"   // Add dither to red
                "ldr            r7, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "usat           r0, #5, r0, asr #3              \n\t"   // Saturate red to 0-31
                // Calculate green
                "smlawb         r3, r7, r3, r1                  \n\t"   // Calculate green and add y
                "ldr            r7, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "sub            r2, #128                        \n\t"   // Subtract CENTERJSAMPLE from cb
                "smlawb         r3, r7, r2, r3                  \n\t"   // Calculate second green
                "add            r3, %[matrix], asr #1           \n\t"   // Add dither to green
                "usat           r3, #6, r3, asr #2              \n\t"   // Saturate green to 0-63
                // Calculate blue
                "ldr            r7, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "smlawb         r2, r7, r2, r1                  \n\t"   // Calculate blue and add y
                "add            r2, %[matrix]                   \n\t"   // Add dither to blue
                "usat           r2, #5, r2, asr #3              \n\t"   // Saturate blue to 0-31
                // Store last pixel
                "orr            r2, r3, lsl #5                  \n\t"   // Insert green into blue
                "orr            r2, r0, lsl #11                 \n\t"   // Insert red into blue
                "strh           r2, [%[output_buf]]             \n\t"   // Store RGB
                "5:                                             \n\t"   //
                : [matrix] "+r" (matrix), [output_buf] "+r" (output_buf)
                : [in_row] "r" (in_row_group_ctr), [input_buf] "r" (input_buf), [num_cols] "r" (num_cols)
                : "cc", "memory", "r0", "r1", "r2", "r3", "r7", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23", "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31"
                );
}
#endif

#endif

/*
 * Upsample and color convert for the case of 2:1 horizontal and 2:1 vertical.
 */

#define H2V2_Proc h2v2_merged_upsample
#define H2V2_Table TRUE

METHODDEF(void)
h2v2_merged_upsample (j_decompress_ptr cinfo,
		      JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
		      JSAMPARRAY output_buf)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;
  register int y, cred, cgreen, cblue;
  int cb, cr;
  register JSAMPROW outptr0, outptr1;
  JSAMPROW inptr00, inptr01, inptr1, inptr2;
  JDIMENSION col;
  /* copy these pointers into registers if possible */
  register JSAMPLE * range_limit = cinfo->sample_range_limit;
  int * Crrtab = upsample->Cr_r_tab;
  int * Cbbtab = upsample->Cb_b_tab;
  INT32 * Crgtab = upsample->Cr_g_tab;
  INT32 * Cbgtab = upsample->Cb_g_tab;
  SHIFT_TEMPS

  inptr00 = input_buf[0][in_row_group_ctr*2];
  inptr01 = input_buf[0][in_row_group_ctr*2 + 1];
  inptr1 = input_buf[1][in_row_group_ctr];
  inptr2 = input_buf[2][in_row_group_ctr];
  outptr0 = output_buf[0];
  outptr1 = output_buf[1];
  /* Loop for each group of output pixels */
  for (col = cinfo->output_width >> 1; col > 0; col--) {
    /* Do the chroma part of the calculation */
    cb = GETJSAMPLE(*inptr1++);
    cr = GETJSAMPLE(*inptr2++);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    /* Fetch 4 Y values and emit 4 pixels */
    y  = GETJSAMPLE(*inptr00++);
    outptr0[RGB_RED] = range_limit[y + cred];
    outptr0[RGB_GREEN] = range_limit[y + cgreen];
    outptr0[RGB_BLUE] = range_limit[y + cblue];
    outptr0 += RGB_PIXELSIZE;
    y  = GETJSAMPLE(*inptr00++);
    outptr0[RGB_RED] = range_limit[y + cred];
    outptr0[RGB_GREEN] = range_limit[y + cgreen];
    outptr0[RGB_BLUE] = range_limit[y + cblue];
    outptr0 += RGB_PIXELSIZE;
    y  = GETJSAMPLE(*inptr01++);
    outptr1[RGB_RED] = range_limit[y + cred];
    outptr1[RGB_GREEN] = range_limit[y + cgreen];
    outptr1[RGB_BLUE] = range_limit[y + cblue];
    outptr1 += RGB_PIXELSIZE;
    y  = GETJSAMPLE(*inptr01++);
    outptr1[RGB_RED] = range_limit[y + cred];
    outptr1[RGB_GREEN] = range_limit[y + cgreen];
    outptr1[RGB_BLUE] = range_limit[y + cblue];
    outptr1 += RGB_PIXELSIZE;
  }
  /* If image width is odd, do the last output column separately */
  if (cinfo->output_width & 1) {
    cb = GETJSAMPLE(*inptr1);
    cr = GETJSAMPLE(*inptr2);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    y  = GETJSAMPLE(*inptr00);
    outptr0[RGB_RED] = range_limit[y + cred];
    outptr0[RGB_GREEN] = range_limit[y + cgreen];
    outptr0[RGB_BLUE] = range_limit[y + cblue];
    y  = GETJSAMPLE(*inptr01);
    outptr1[RGB_RED] = range_limit[y + cred];
    outptr1[RGB_GREEN] = range_limit[y + cgreen];
    outptr1[RGB_BLUE] = range_limit[y + cblue];
  }
}

// Use NEON optimized version?
#if defined(ENABLE_NEON_H2V2)
#undef H2V2_Proc
#undef H2V2_Table
#define H2V2_Proc h2v2_merged_upsample_neon
#define H2V2_Table FALSE

METHODDEF(void)
h2v2_merged_upsample_neon (j_decompress_ptr cinfo,
		      JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
		      JSAMPARRAY output_buf)
{
  JDIMENSION num_cols = cinfo->output_width;

  // Fallback to non NEON method for small conversions
  if (num_cols < 8)
  {
    my_upsample_ptr upsample = (my_upsample_ptr)cinfo->upsample;

    if (upsample->Cr_r_tab == NULL) {
      build_ycc_rgb_table(cinfo);
    }
    h2v2_merged_upsample(cinfo, input_buf, in_row_group_ctr, output_buf);
    return;
  }

  asm volatile (
                // Setup constants
                "ldr            r2, [%[input_buf], #0]          \n\t"   // Setup y input pointer from input_buf[0]
                "vmov.i8        d18, #128                       \n\t"   // Set center sample constant to CENTERJSAMPLE
                "vmov.i32       q8, #32768                      \n\t"   // Set rounding constant to 32768 (0.5)
                "ldr            r0, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "add            r2, r2, %[in_row], lsl #3       \n\t"   //
                "ldr            r1, [r2]                        \n\t"   // input_buf[0][in_row_group_ctr * 2] and
                "vdup.32        q0, r0                          \n\t"   //
                "pld            [r1]                            \n\t"   //
                "ldr            r0, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [r2, #4]                    \n\t"   // input_buf[0][in_row_group_ctr * 2 + 1]
                "vdup.32        q1, r0                          \n\t"   //
                "pld            [r2]                            \n\t"   //
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "ldr            r3, [%[input_buf], #4]          \n\t"   // Setup cb input pointer from input_buf[1][in_row_group_ctr]
                "vdup.32        q2, r0                          \n\t"   //
                "ldr            r0, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "ldr            r3, [r3, %[in_row], lsl #2]     \n\t"   //
                "vdup.32        q3, r0                          \n\t"   //
                "vld1.32        {d30[0]}, [r3]                  \n\t"   // Load four cb values
                "ldr            r4, [%[input_buf], #8]          \n\t"   // Setup cr input pointer from input_buf[2][in_row_group_ctr]
                "ldr            r4, [r4, %[in_row], lsl #2]     \n\t"   //
                "ands           r0, %[num_cols], #0x6           \n\t"   // Should we do a partial first iteration?
                "vld1.32        {d30[1]}, [r4]                  \n\t"   // Load four cr values
                "moveq          r0, #8                          \n\t"   // ...or a full iteration
                "ldr            r5, [%[output_buf], #0]         \n\t"   // Get output pointer for row 1
                "and            r7, %[num_cols], #0xFFFFFFFE    \n\t"   // Setup loop counter
                "ldr            %[output_buf], [%[output_buf], #4]\n\t" // Get output pointer for row 2
                "subs           r7, r7, #8                      \n\t"   // Subtract last iteration
                "beq            4f                              \n\t"
                // loop (columns)
                "3:                                             \n\t"   //
                // Read values, subtract 128 from cb/cr, and expand to 32-bit
                "vsubl.u8       q14, d30, d18                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vld1.8         {d30}, [r1]                     \n\t"   // Load eight y values for row 1
                "vmovl.s16      q10, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q12, q2                    \n\t"   // Calculate green
                "add            r3, r0, lsr #1                  \n\t"   // Increment input pointer for cb
                "vmul.i32       q12, q12, q3                    \n\t"   // Calculate blue
                "vld1.8         {d31}, [r2]                     \n\t"   // Load eight y values for row 2
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "add            r4, r0, lsr #1                  \n\t"   // Increment input pointer for cr
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "pld            [r3]                            \n\t"   //
                "vadd.i32       q12, q8                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q8                         \n\t"   // Add 0.5 to green
                "pld            [r4]                            \n\t"   //
                "vadd.i32       q11, q8                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result and clone for each row (Vector Shift Right and Insert)
                "add            r1, r0                          \n\t"   // Increment input pointer for y
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                // Row 1...
                // Add y (Vector add)
                "vmovl.u8       q14, d30                        \n\t"   // Expand y to 16-bit
                "subs           r7, r7, r0                      \n\t"   // Decrement loop counter
                "vld1.32        {d30[0]}, [r3]                  \n\t"   // Pre-load next four cb values
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vld1.32        {d30[1]}, [r4]                  \n\t"   // Pre-load next four cr values
                // Pre-calculate row 2 y delta
                "vmovl.u8       q10, d31                        \n\t"   // Expand row 2 y to 16-bit
                "vsub.i16       q14, q10, q14                   \n\t"   // Calculate row 2 y delta
                "pld            [r1]                            \n\t"   //
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d19, q11                        \n\t"   // Convert red
                "vqmovun.s16    d20, q13                        \n\t"   // Convert green
                "vqmovun.s16    d21, q12                        \n\t"   // Convert blue
                "add            r2, r0                          \n\t"   // Increment input pointer for y
                "vst3.8         {d19, d20, d21}, [r5]           \n\t"   // Write row 1 result to memory
                // Row 2...
                // Add y delta (Vector add)
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "pld            [r2]                            \n\t"   //
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "add            r5, r0, lsl #1                  \n\t"   // Increment output buffer pointer for row 1
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d23, q13                        \n\t"   // Convert green
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                "add            r5, r0                          \n\t"   // Increment output buffer pointer for row 1
                "vst3.8         {d22, d23, d24}, [%[output_buf]]\n\t"   // Write row 2 result to memory
                // Increase pointers and counters
                "add            %[output_buf], r0, lsl #1       \n\t"   // Increment output buffer pointer for row 2
                "add            %[output_buf], r0               \n\t"   //
                "mov            r0, #8                          \n\t"   // Set next loop iteration length
                "bne            3b                              \n\t"   // If loop counter != 0, loop
                // last iteration (post loop)
                "4:                                             \n\t"   //
                "vsubl.u8       q14, d30, d18                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q12, d28                        \n\t"   // Expand cb values to 32-bit
                "vld1.8         {d30}, [r1]                     \n\t"   // Load eight y values for row 1
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                 // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q12, q2                    \n\t"   // Calculate green
                "vld1.8         {d31}, [r2]                     \n\t"   // Load eight y values for row 2
                "vmul.i32       q12, q12, q3                    \n\t"   // Calculate blue
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vadd.i32       q12, q8                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q8                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q8                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result and clone for each row (Vector Shift Right and Insert)
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                // Row 1...
                // Add y (Vector add)
                "vmovl.u8       q14, d30                        \n\t"   // Expand y to 16-bit
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                // Pre-calculate row 2 y delta
                "vmovl.u8       q10, d31                        \n\t"   // Expand row 2 y to 16-bit
                "vsub.i16       q14, q10, q14                   \n\t"   // Calculate row 2 y delta
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d19, q11                        \n\t"   // Convert red
                "vqmovun.s16    d20, q13                        \n\t"   // Convert green
                "vqmovun.s16    d21, q12                        \n\t"   // Convert blue
                "vst3.8         {d19, d20, d21}, [r5]!          \n\t"   // Write row 1 result to memory
                // Row 2...
                // Add y delta (Vector add)
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d23, q13                        \n\t"   // Convert green
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                "vst3.8         {d22, d23, d24}, [%[output_buf]]!\n\t"  // Write row 2 result to memory
                // Handle any last odd pixel
                "tst            %[num_cols], #1                 \n\t"   // Is there a last odd pixel?
                "beq            5f                              \n\t"   // If not, exit
                "ldr            r7, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldrb           r4, [r4, #4]                    \n\t"   // Load cr value
                "ldrb           r3, [r3, #4]                    \n\t"   // Load cb value
                "ldrb           r1, [r1, #8]                    \n\t"   // Load y value for row 1
                "ldrb           r2, [r2, #8]                    \n\t"   // Load y value for row 2
                // Calculate red
                "sub            r4, #128                        \n\t"   // Subtract CENTERJSAMPLE from cr
                "smulwb         r0, r7, r4                      \n\t"   // Calculate red
                "add            r7, r0, r1                      \n\t"   // Add y for row 1
                "usat           r7, #8, r7                      \n\t"   // Saturate to 0-255
                "strb           r7, [r5]                        \n\t"   // Store red for row 1
                "add            r0, r0, r1                      \n\t"   // Add y for row 2
                "ldr            r7, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "strb           r0, [%[output_buf]]             \n\t"   // Store red for row 2
                // Calculate green
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "smulwb         r4, r7, r4                      \n\t"   // Calculate green
                "sub            r3, #128                        \n\t"   // Subtract CENTERJSAMPLE from cb
                "smlawb         r4, r0, r3, r4                  \n\t"   // Calculate second green
                "ldr            r7, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "add            r0, r4, r1                      \n\t"   // Add y for row 1
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "strb           r0, [r5, #1]                    \n\t"   // Store green for row 1
                "add            r4, r4, r2                      \n\t"   // Add y for row 2
                "usat           r4, #8, r4                      \n\t"   // Saturate to 0-255
                "strb           r4, [%[output_buf], #1]         \n\t"   // Store green for row 2
                // Calculate blue
                "smulwb         r0, r7, r3                      \n\t"   // Calculate blue
                "add            r4, r0, r1                      \n\t"   // Add y for row 1
                "usat           r4, #8, r4                      \n\t"   // Saturate to 0-255
                "strb           r4, [r5, #2]                    \n\t"   // Store blue for row 1
                "add            r0, r0, r2                      \n\t"   // Add y for row 2
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "strb           r0, [%[output_buf], #2]         \n\t"   // Store blue for row 2
                "5:                                             \n\t"   //
                : [output_buf] "+r" (output_buf)
                : [in_row] "r" (in_row_group_ctr), [input_buf] "r" (input_buf), [num_cols] "r" (num_cols)
                : "cc", "memory", "r0", "r1", "r2", "r3", "r4", "r5", "r7", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23", "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31"
                );
}

#endif


#ifdef ANDROID_RGB
#define H2V2_8888_Proc h2v2_merged_upsample_8888
#define H2V2_8888_Table TRUE
#define H2V2_565_Proc h2v2_merged_upsample_565
#define H2V2_565_Table TRUE
#define H2V2_565D_Proc h2v2_merged_upsample_565D
#define H2V2_565D_Table TRUE

METHODDEF(void)
h2v2_merged_upsample_8888 (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;
  register int y, cred, cgreen, cblue;
  int cb, cr;
  register JSAMPROW outptr0, outptr1;
  JSAMPROW inptr00, inptr01, inptr1, inptr2;
  JDIMENSION col;
  /* copy these pointers into registers if possible */
  register JSAMPLE * range_limit = cinfo->sample_range_limit;
  int * Crrtab = upsample->Cr_r_tab;
  int * Cbbtab = upsample->Cb_b_tab;
  INT32 * Crgtab = upsample->Cr_g_tab;
  INT32 * Cbgtab = upsample->Cb_g_tab;
  SHIFT_TEMPS

  inptr00 = input_buf[0][in_row_group_ctr*2];
  inptr01 = input_buf[0][in_row_group_ctr*2 + 1];
  inptr1 = input_buf[1][in_row_group_ctr];
  inptr2 = input_buf[2][in_row_group_ctr];
  outptr0 = output_buf[0];
  outptr1 = output_buf[1];
  /* Loop for each group of output pixels */
  for (col = cinfo->output_width >> 1; col > 0; col--) {
    /* Do the chroma part of the calculation */
    cb = GETJSAMPLE(*inptr1++);
    cr = GETJSAMPLE(*inptr2++);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    /* Fetch 4 Y values and emit 4 pixels */
    y  = GETJSAMPLE(*inptr00++);
    outptr0[RGB_RED] = range_limit[y + cred];
    outptr0[RGB_GREEN] = range_limit[y + cgreen];
    outptr0[RGB_BLUE] = range_limit[y + cblue];
    outptr0[RGB_ALPHA] = 0xFF;
    outptr0 += 4;
    y  = GETJSAMPLE(*inptr00++);
    outptr0[RGB_RED] = range_limit[y + cred];
    outptr0[RGB_GREEN] = range_limit[y + cgreen];
    outptr0[RGB_BLUE] = range_limit[y + cblue];
    outptr0[RGB_ALPHA] = 0xFF;
    outptr0 += 4;
    y  = GETJSAMPLE(*inptr01++);
    outptr1[RGB_RED] = range_limit[y + cred];
    outptr1[RGB_GREEN] = range_limit[y + cgreen];
    outptr1[RGB_BLUE] = range_limit[y + cblue];
    outptr1[RGB_ALPHA] = 0xFF;
    outptr1 += 4;
    y  = GETJSAMPLE(*inptr01++);
    outptr1[RGB_RED] = range_limit[y + cred];
    outptr1[RGB_GREEN] = range_limit[y + cgreen];
    outptr1[RGB_BLUE] = range_limit[y + cblue];
    outptr1[RGB_ALPHA] = 0xFF;
    outptr1 += 4;
  }
  /* If image width is odd, do the last output column separately */
  if (cinfo->output_width & 1) {
    cb = GETJSAMPLE(*inptr1);
    cr = GETJSAMPLE(*inptr2);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    y  = GETJSAMPLE(*inptr00);
    outptr0[RGB_RED] = range_limit[y + cred];
    outptr0[RGB_GREEN] = range_limit[y + cgreen];
    outptr0[RGB_BLUE] = range_limit[y + cblue];
    outptr0[RGB_ALPHA] = 0xFF;
    y  = GETJSAMPLE(*inptr01);
    outptr1[RGB_RED] = range_limit[y + cred];
    outptr1[RGB_GREEN] = range_limit[y + cgreen];
    outptr1[RGB_BLUE] = range_limit[y + cblue];
    outptr1[RGB_ALPHA] = 0xFF;
  }
}


// Use NEON optimized version?
#if defined(ENABLE_NEON_H2V2_8888)
#undef H2V2_8888_Proc
#undef H2V2_8888_Table
#define H2V2_8888_Proc h2v2_merged_upsample_8888_neon
#define H2V2_8888_Table FALSE

METHODDEF(void)
h2v2_merged_upsample_8888_neon (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  JDIMENSION num_cols = cinfo->output_width;

  // Fallback to non NEON method for small conversions
  if (num_cols < 8)
  {
    my_upsample_ptr upsample = (my_upsample_ptr)cinfo->upsample;

    if (upsample->Cr_r_tab == NULL) {
      build_ycc_rgb_table(cinfo);
    }
    h2v2_merged_upsample_8888(cinfo, input_buf, in_row_group_ctr, output_buf);
    return;
  }

  asm volatile (
                // Setup constants
                "ldr            r2, [%[input_buf], #0]          \n\t"   // Setup y input pointer from input_buf[0]
                "vmov.i8        d15, #128                       \n\t"   // Set center sample constant to CENTERJSAMPLE
                "vmov.i32       q8, #32768                      \n\t"   // Set rounding constant to 32768 (0.5)
                "ldr            r0, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "add            r2, r2, %[in_row], lsl #3       \n\t"   //
                "ldr            r1, [r2]                        \n\t"   // input_buf[0][in_row_group_ctr * 2] and
                "vdup.32        q0, r0                          \n\t"   //
                "pld            [r3]                            \n\t"   //
                "ldr            r0, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [r2, #4]                    \n\t"   // input_buf[0][in_row_group_ctr * 2 + 1]
                "vdup.32        q1, r0                          \n\t"   //
                "pld            [r4]                            \n\t"   //
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "ldr            r3, [%[input_buf], #4]          \n\t"   // Setup cb input pointer from input_buf[1][in_row_group_ctr]
                "vdup.32        q2, r0                          \n\t"   //
                "ldr            r0, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "ldr            r3, [r3, %[in_row], lsl #2]     \n\t"   //
                "vdup.32        q3, r0                          \n\t"   //
                "vld1.32        {d30[0]}, [r3]                  \n\t"   // Load four cb values
                "ldr            r4, [%[input_buf], #8]          \n\t"   // Setup cr input pointer from input_buf[2][in_row_group_ctr]
                "ldr            r4, [r4, %[in_row], lsl #2]     \n\t"   //
                "ands           r0, %[num_cols], #0x6           \n\t"   // Should we do a partial first iteration?
                "vld1.32        {d30[1]}, [r4]                  \n\t"   // Load four cr values
                "moveq          r0, #8                          \n\t"   // ...or a full iteration
                "ldr            r5, [%[output_buf], #0]         \n\t"   // Get output pointer for row 1
                "and            r7, %[num_cols], #0xFFFFFFFE    \n\t"   // Setup loop counter
                "ldr            %[output_buf], [%[output_buf], #4]\n\t"   // Get output pointer for row 2
                "subs           r7, r7, #8                      \n\t"   // Subtract last iteration
                "beq            4f                              \n\t"
                // loop (columns)
                "3:                                             \n\t"   //
                // Read values, subtract 128 from cb/cr, and expand to 32-bit
                "vsubl.u8       q14, d30, d15                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q12, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q10, q2                    \n\t"   // Calculate green
                "add            r3, r0, lsr #1                  \n\t"   // Increment input pointer for cb
                "vmul.i32       q12, q12, q3                    \n\t"   // Calculate blue
                "vld1.8         {d30}, [r1]                     \n\t"   // Load eight y values for row 1
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "add            r4, r0, lsr #1                  \n\t"   // Increment input pointer for cr
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vld1.8         {d31}, [r2]                     \n\t"   // Load eight y values for row 2
                "vadd.i32       q12, q8                         \n\t"   // Add 0.5 to blue
                "pld            [r3]                            \n\t"   //
                "vadd.i32       q13, q8                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q8                         \n\t"   // Add 0.5 to red
                "pld            [r4]                            \n\t"   //
                // Duplicate RGB result and clone for each row (Vector Shift Right and Insert)
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                "subs           r7, r7, r0                      \n\t"   // Decrement loop counter
                // Row 1...
                // Add y (Vector add)
                "vmovl.u8       q14, d30                        \n\t"   // Expand y to 16-bit
                "add            r1, r0                          \n\t"   // Increment input pointer for y
                "vld1.32        {d30[0]}, [r3]                  \n\t"   // Pre-load next four cb values
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                // Pre-calculate row 2 y delta
                "vmovl.u8       q10, d31                        \n\t"   // Expand row 2 y to 16-bit
                "vsub.i16       q14, q10, q14                   \n\t"   // Calculate row 2 y delta
                "vld1.32        {d30[1]}, [r4]                  \n\t"   // Pre-load next four cr values
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d18, q11                        \n\t"   // Convert red
                "pld            [r1]                            \n\t"   //
                "vqmovun.s16    d19, q13                        \n\t"   // Convert green
                "vqmovun.s16    d20, q12                        \n\t"   // Convert blue
                "vmov.i8        d21, #0xFF                      \n\t"   // Set alpha to 0xFF
                "add            r2, r0                          \n\t"   // Increment input pointer for y
                "vst4.8         {d18, d19, d20, d21}, [r5]      \n\t"   // Write row 1 result to memory
                // Row 2...
                // Add y delta (Vector add)
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "pld            [r2]                            \n\t"   //
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d23, q13                        \n\t"   // Convert green
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                "vmov.i8        d25, #0xFF                      \n\t"   // Set alpha to 0xFF
                "add            r5, r0, lsl #2                  \n\t"   // Increment output buffer pointer for row 1
                "vst4.8         {d22, d23, d24, d25}, [%[output_buf]]\n\t"// Write row 2 result to memory
                // Increase pointers and counters
                "add            %[output_buf], r0, lsl #2       \n\t"   // Increment output buffer pointer for row 2
                "mov            r0, #8                          \n\t"   // Set next loop iteration length
                "bne            3b                              \n\t"   // If loop counter != 0, loop
                // last iteration (post loop)
                "4:                                             \n\t"   //
                "vsubl.u8       q14, d30, d15                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q12, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                 // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q12, q2                    \n\t"   // Calculate green
                "vmul.i32       q12, q12, q3                    \n\t"   // Calculate blue
                "vld1.8         {d30}, [r2]                     \n\t"   // Load eight y values for row 2
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vld1.8         {d31}, [r2]                     \n\t"   // Load eight y values for row 2
                "vadd.i32       q12, q8                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q8                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q8                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result and clone for each row (Vector Shift Right and Insert)
                "vmovl.u8       q14, d30                        \n\t"   // Expand y to 16-bit
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                // Row 2...
                // Add y (Vector add)
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vadd.i16       q10, q14                        \n\t"   // Add y to green
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                // Pre-calculate row 2 y delta
                "vmovl.u8       q10, d31                        \n\t"   // Expand row 2 y to 16-bit
                "vsub.i16       q14, q10, q14                   \n\t"   // Calculate row 2 y delta
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d18, q11                        \n\t"   // Convert red
                "vqmovun.s16    d17, q10                        \n\t"   // Convert green
                "vqmovun.s16    d20, q12                        \n\t"   // Convert blue
                "vmov.i8        d21, #0xFF                      \n\t"   // Set alpha to 0xFF
                "vst4.8         {d18, d19, d20, d21}, [r5]!     \n\t"   // Write row 1 result to memory
                // Row 2...
                // Add y delta (Vector add)
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d23, q13                        \n\t"   // Convert green
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                "vmov.i8        d25, #0xFF                      \n\t"   // Set alpha to 0xFF
                "vst4.8         {d22, d23, d24, d25}, [%[output_buf]]!\n\t"// Write row 2 result to memory
                // Handle any last odd pixel
                "tst            %[num_cols], #1                 \n\t"   // Is there a last odd pixel?
                "beq            5f                              \n\t"   // If not, exit
                "ldr            r7, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldrb           r4, [r4, #4]                    \n\t"   // Load cr value
                "ldrb           r3, [r3, #4]                    \n\t"   // Load cb value
                "ldrb           r1, [r1, #8]                    \n\t"   // Load y value for row 1
                "ldrb           r2, [r2, #8]                    \n\t"   // Load y value for row 2
                // Calculate red
                "sub            r4, #128                        \n\t"   // Subtract CENTERJSAMPLE from cr
                "smulwb         r0, r7, r4                      \n\t"   // Calculate red
                "add            r7, r0, r1                      \n\t"   // Add y for row 1
                "usat           r7, #8, r7                      \n\t"   // Saturate to 0-255
                "strb           r7, [r5]                        \n\t"   // Store red for row 1
                "add            r0, r0, r1                      \n\t"   // Add y for row 2
                "ldr            r7, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "strb           r0, [%[output_buf]]             \n\t"   // Store red for row 2
                // Calculate green
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "smulwb         r4, r7, r4                      \n\t"   // Calculate green
                "sub            r3, #128                        \n\t"   // Subtract CENTERJSAMPLE from cb
                "smlawb         r4, r0, r3, r4                  \n\t"   // Calculate second green
                "ldr            r7, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "add            r0, r4, r1                      \n\t"   // Add y for row 1
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "strb           r0, [r5, #1]                    \n\t"   // Store green for row 1
                "add            r4, r4, r2                      \n\t"   // Add y for row 2
                "usat           r4, #8, r4                      \n\t"   // Saturate to 0-255
                "strb           r4, [%[output_buf], #1]         \n\t"   // Store green for row 2
                // Calculate blue
                "smulwb         r0, r7, r3                      \n\t"   // Calculate blue
                "add            r4, r0, r1                      \n\t"   // Add y for row 1
                "mov            r7, #0xFF                       \n\t"   // Load alpha 0xFF
                "usat           r4, #8, r4                      \n\t"   // Saturate to 0-255
                "strb           r4, [r5, #2]                    \n\t"   // Store blue for row 1
                "add            r0, r0, r2                      \n\t"   // Add y for row 2
                "strb           r7, [r5, #3]                    \n\t"   // Store alpha for row 1
                "usat           r0, #8, r0                      \n\t"   // Saturate to 0-255
                "strb           r0, [%[output_buf], #2]         \n\t"   // Store blue for row 2
                "strb           r7, [%[output_buf], #3]         \n\t"   // Store alpha for row 2
                "5:                                             \n\t"   //
                : [output_buf] "+r" (output_buf)
                : [in_row] "r" (in_row_group_ctr), [input_buf] "r" (input_buf), [num_cols] "r" (num_cols)
                : "cc", "memory", "r0", "r1", "r2", "r3", "r4", "r5", "r7", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d15", "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23", "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31"
                );
}
#endif


METHODDEF(void)
h2v2_merged_upsample_565 (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;
  register int y, cred, cgreen, cblue;
  int cb, cr;
  register JSAMPROW outptr0, outptr1;
  JSAMPROW inptr00, inptr01, inptr1, inptr2;
  JDIMENSION col;
  /* copy these pointers into registers if possible */
  register JSAMPLE * range_limit = cinfo->sample_range_limit;
  int * Crrtab = upsample->Cr_r_tab;
  int * Cbbtab = upsample->Cb_b_tab;
  INT32 * Crgtab = upsample->Cr_g_tab;
  INT32 * Cbgtab = upsample->Cb_g_tab;
  unsigned int r, g, b;
  INT32 rgb;
  SHIFT_TEMPS

  inptr00 = input_buf[0][in_row_group_ctr*2];
  inptr01 = input_buf[0][in_row_group_ctr*2 + 1];
  inptr1 = input_buf[1][in_row_group_ctr];
  inptr2 = input_buf[2][in_row_group_ctr];
  outptr0 = output_buf[0];
  outptr1 = output_buf[1];
  /* Loop for each group of output pixels */
  for (col = cinfo->output_width >> 1; col > 0; col--) {
    /* Do the chroma part of the calculation */
    cb = GETJSAMPLE(*inptr1++);
    cr = GETJSAMPLE(*inptr2++);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    /* Fetch 4 Y values and emit 4 pixels */
    y  = GETJSAMPLE(*inptr00++);
    r = range_limit[y + cred];
    g = range_limit[y + cgreen];
    b = range_limit[y + cblue];
    rgb = PACK_SHORT_565(r,g,b);
    y  = GETJSAMPLE(*inptr00++);
    r = range_limit[y + cred];
    g = range_limit[y + cgreen];
    b = range_limit[y + cblue];
    rgb = PACK_TWO_PIXELS(rgb, PACK_SHORT_565(r,g,b));
    WRITE_TWO_PIXELS(outptr0, rgb);
    outptr0 += 4;
    y  = GETJSAMPLE(*inptr01++);
    r = range_limit[y + cred];
    g = range_limit[y + cgreen];
    b = range_limit[y + cblue];
    rgb = PACK_SHORT_565(r,g,b);
    y  = GETJSAMPLE(*inptr01++);
    r = range_limit[y + cred];
    g = range_limit[y + cgreen];
    b = range_limit[y + cblue];
    rgb = PACK_TWO_PIXELS(rgb, PACK_SHORT_565(r,g,b));
    WRITE_TWO_PIXELS(outptr1, rgb);
    outptr1 += 4;
  }
  /* If image width is odd, do the last output column separately */
  if (cinfo->output_width & 1) {
    cb = GETJSAMPLE(*inptr1);
    cr = GETJSAMPLE(*inptr2);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    y  = GETJSAMPLE(*inptr00);
    r = range_limit[y + cred];
    g = range_limit[y + cgreen];
    b = range_limit[y + cblue];
    rgb = PACK_SHORT_565(r,g,b);
    *(INT16*)outptr0 = rgb;
    y  = GETJSAMPLE(*inptr01);
    r = range_limit[y + cred];
    g = range_limit[y + cgreen];
    b = range_limit[y + cblue];
    rgb = PACK_SHORT_565(r,g,b);
    *(INT16*)outptr1 = rgb;
  }
}


// Use NEON optimized version?
#if defined(ENABLE_NEON_H2V2_565)
#undef H2V2_565_Proc
#undef H2V2_565_Table
#define H2V2_565_Proc h2v2_merged_upsample_565_neon
#define H2V2_565_Table FALSE

METHODDEF(void)
h2v2_merged_upsample_565_neon (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  JDIMENSION num_cols = cinfo->output_width;

  // Fallback to non NEON method for small conversions
  if (num_cols < 8)
  {
    my_upsample_ptr upsample = (my_upsample_ptr)cinfo->upsample;

    if (upsample->Cr_r_tab == NULL) {
      build_ycc_rgb_table(cinfo);
    }
    h2v2_merged_upsample_565(cinfo, input_buf, in_row_group_ctr, output_buf);
    return;
  }

  asm volatile (
                // Setup constants
                "ldr            r2, [%[input_buf], #0]          \n\t"   // Setup y input pointer from input_buf[0]
                "vmov.i8        d16, #128                       \n\t"   // Set center sample constant to CENTERJSAMPLE
                "vmov.i32       q7, #32768                      \n\t"   // Set rounding constant to 32768 (0.5)
                "ldr            r0, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "add            r2, r2, %[in_row], lsl #3       \n\t"   //
                "ldr            r1, [r2]                        \n\t"   // input_buf[0][in_row_group_ctr * 2] and
                "vdup.32        q0, r0                          \n\t"   //
                "pld            [r1]                            \n\t"   //
                "ldr            r0, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [r2, #4]                    \n\t"   // input_buf[0][in_row_group_ctr * 2 + 1]
                "vdup.32        q1, r0                          \n\t"   //
                "pld            [r2]                            \n\t"   //
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "ldr            r3, [%[input_buf], #4]          \n\t"   // Setup cb input pointer from input_buf[1][in_row_group_ctr]
                "vdup.32        q2, r0                          \n\t"   //
                "ldr            r0, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "ldr            r3, [r3, %[in_row], lsl #2]     \n\t"   //
                "vdup.32        q3, r0                          \n\t"   //
                "vld1.32        {d30[0]}, [r3]                  \n\t"   // Load four cb values
                "ldr            r4, [%[input_buf], #8]          \n\t"   // Setup cr input pointer from input_buf[2][in_row_group_ctr]
                "ldr            r4, [r4, %[in_row], lsl #2]     \n\t"   //
                "ands           r0, %[num_cols], #0x6           \n\t"   // Should we do a partial first iteration?
                "vld1.32        {d30[1]}, [r4]                  \n\t"   // Load four cr values
                "moveq          r0, #8                          \n\t"   // ...or a full iteration
                "ldr            r5, [%[output_buf], #0]         \n\t"   // Get output pointer for row 1
                "and            r7, %[num_cols], #0xFFFFFFFE    \n\t"   // Setup loop counter
                "ldr            %[output_buf], [%[output_buf], #4]\n\t" // Get output pointer for row 2
                "subs           r7, r7, #8                      \n\t"   // Subtract last iteration
                "beq            4f                              \n\t"
                // loop (columns)
                "3:                                             \n\t"   //
                // Read values, subtract 128 from cb/cr, and expand to 32-bit
                "vsubl.u8       q14, d30, d31                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "add            r3, r0, lsr #1                  \n\t"   // Increment input pointer for cb
                "vld1.8         {d30}, [r1]                     \n\t"   // Load eight y values for row 1
                "vmovl.s16      q10, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                "add            r4, r0, lsr #1                  \n\t"   // Increment input pointer for cr
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q10, q2                    \n\t"   // Calculate green
                "vmul.i32       q12, q12, q3                    \n\t"   // Calculate blue
                "vld1.8         {d31}, [r2]                     \n\t"   // Load eight y values for row 2
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vadd.i32       q12, q7                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q7                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q7                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result and clone for each row (Vector Shift Right and Insert)
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                "subs           r7, r7, r0                      \n\t"   // Decrement loop counter
                // Row 1...
                // Add y (Vector add)
                "vmovl.u8       q14, d30                        \n\t"   // Expand y to 16-bit
                "add            r1, r0                          \n\t"   // Increment input pointer for y
                "vld1.32        {d30[0]}, [r3]                  \n\t"   // Pre-load next four cb values
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vld1.32        {d30[1]}, [r4]                  \n\t"   // Pre-load next four cr values
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d26, q13                        \n\t"   // Convert green
                "vqmovun.s16    d18, q11                        \n\t"   // Convert red
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                // Pack to 565 format
                "vshll.u8       q13, d26, #8                    \n\t"   // Shift green and expand to 16-bit
                "vshll.u8       q9, d18, #8                     \n\t"   // Shift red and expand to 16-bit
                "vsri.u16       q9, q10, #5                     \n\t"   // Insert green into red
                "vshll.u8       q10, d17, #8                    \n\t"   // Shift blue and expand to 16-bit
                "vsri.u16       q9, q10, #11                    \n\t"   // Insert blue into red
                "add            r2, r0                          \n\t"   // Increment input pointer for y
                "vst1.16        {q9}, [r5]                      \n\t"   // Write row 1 result to memory
                // Row 2...
                // Add y (Vector add)
                "vmovl.u8       q10, d31                        \n\t"   // Expand y to 16-bit
                "add            r5, r0, lsl #1                  \n\t"   // Increment output buffer pointer for row 1
                "vsub.i16       q14, q10, q14                   \n\t"   // Calculate y delta
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d26, q13                        \n\t"   // Convert green
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                // Pack to 565 format
                "vshll.u8       q11, d22, #8                    \n\t"   // Shift red and expand to 16-bit
                "vshll.u8       q13, d26, #8                    \n\t"   // Shift green and expand to 16-bit
                "vshll.u8       q12, d24, #8                    \n\t"   // Shift blue and expand to 16-bit
                "vsri.u16       q11, q13, #5                    \n\t"   // Insert green into red
                "vsri.u16       q11, q12, #11                   \n\t"   // Insert blue into red
                "vst1.16        {q11}, [%[output_buf]]          \n\t"   // Write row 2 result to memory
                // Increase pointers and counters
                "add            %[output_buf], r0, lsl #1       \n\t"   // Increment output buffer pointer for row 2
                "mov            r0, #8                          \n\t"   // Set next loop iteration length
                "bne            3b                              \n\t"   // If loop counter != 0, loop
                // last iteration (post loop)
                "4:                                             \n\t"   //
                "vsubl.u8       q14, d30, d16                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vld1.8         {d30}, [r1]                     \n\t"   // Load eight y values for row 1
                "vmovl.s16      q12, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                 // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q12, q2                    \n\t"   // Calculate green
                "vld1.8         {d31}, [r2]                     \n\t"   // Load eight y values for row 2
                "vmul.i32       q12, q12, q3                    \n\t"   // Calculate blue
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vadd.i32       q12, q7                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q7                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q7                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result and clone for each row (Vector Shift Right and Insert)
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                // Row 2...
                // Add y (Vector add)
                "vmovl.u8       q14, d30                        \n\t"   // Expand y to 16-bit
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                "vadd.i16       q10, q14                        \n\t"   // Add y to green
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d20, q10                        \n\t"   // Convert green
                "vqmovun.s16    d18, q11                        \n\t"   // Convert red
                "vqmovun.s16    d17, q12                        \n\t"   // Convert blue
                // Pack to 565 format
                "vshll.u8       q10, d20, #8                    \n\t"   // Shift green and expand to 16-bit
                "vshll.u8       q8, d16, #8                     \n\t"   // Shift red and expand to 16-bit
                "vsri.u16       q8, q10, #5                     \n\t"   // Insert green into red
                "vshll.u8       q10, d17, #8                    \n\t"   // Shift blue and expand to 16-bit
                "vsri.u16       q9, q10, #11                    \n\t"   // Insert blue into red
                "vst1.16        {q9}, [r5]!                     \n\t"   // Write row 1 result to memory
                // Row 2...
                // Add y delta (Vector add)
                "vmovl.u8       q10, d31                        \n\t"   // Expand row 2 y to 16-bit
                "vsub.i16       q14, q10, q14                   \n\t"   // Calculate row 2 y delta
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d26, q13                        \n\t"   // Convert green
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                // Pack to 565 format
                "vshll.u8       q11, d22, #8                    \n\t"   // Shift red and expand to 16-bit
                "vshll.u8       q13, d26, #8                    \n\t"   // Shift green and expand to 16-bit
                "vshll.u8       q12, d24, #8                    \n\t"   // Shift blue and expand to 16-bit
                "vsri.u16       q11, q13, #5                    \n\t"   // Insert green into red
                "vsri.u16       q11, q12, #11                   \n\t"   // Insert blue into red
                "vst1.16        {q11}, [%[output_buf]]!         \n\t"   // Write row 2 result to memory
                // Handle any last odd pixel
                "tst            %[num_cols], #1                 \n\t"   // Is there a last odd pixel?
                "beq            5f                              \n\t"   // If not, exit
                "ldr            r7, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldrb           r4, [r4, #4]                    \n\t"   // Load cr value
                "ldrb           r3, [r3, #4]                    \n\t"   // Load cb value
                "ldrb           r1, [r1, #8]                    \n\t"   // Load y value for row 1
                "ldrb           %[in_row], [r2, #8]             \n\t"   // Load y value for row 2
                // Calculate red
                "sub            r4, #128                        \n\t"   // Subtract CENTERJSAMPLE from cr
                "smulwb         r0, r7, r4                      \n\t"   // Calculate red
                // Calculate green
                "ldr            r7, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "smulwb         r4, r7, r4                      \n\t"   // Calculate green
                "ldr            r7, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "sub            r3, #128                        \n\t"   // Subtract CENTERJSAMPLE from cb
                "smlawb         r4, r7, r3, r4                  \n\t"   // Calculate second green
                // Calculate blue
                "ldr            r7, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "smulwb         r3, r7, r3                      \n\t"   // Calculate blue
                // Calculate row 1 RGB
                "add            r2, r0, r1                      \n\t"   // Add y to red
                "usat           r2, #5, r2, asr #3              \n\t"   // Saturate to 0-31
                "add            r7, r3, r1                      \n\t"   // Add y to blue
                "usat           r7, #5, r7, asr #3              \n\t"   // Saturate to 0-31
                "orr            r7, r2, lsl #11                 \n\t"   // Insert red into blue
                "add            r2, r4, r1                      \n\t"   // Add y to green
                "usat           r2, #6, r2, asr #2              \n\t"   // Saturate to 0-63
                "orr            r7, r2, lsl #5                  \n\t"   // Insert green into blue
                "strh           r7, [r5]                        \n\t"   // Store RGB
                // Calculate row 2 RGB
                "add            r2, r0, %[in_row]               \n\t"   // Add y to red
                "usat           r2, #5, r2, asr #3              \n\t"   // Saturate to 0-31
                "add            r7, r3, %[in_row]               \n\t"   // Add y to blue
                "usat           r7, #5, r7, asr #3              \n\t"   // Saturate to 0-31
                "orr            r7, r2, lsl #11                 \n\t"   // Insert red into blue
                "add            r2, r4, %[in_row]               \n\t"   // Add y to green
                "usat           r2, #6, r2, asr #2              \n\t"   // Saturate to 0-63
                "orr            r7, r2, lsl #5                  \n\t"   // Insert green into blue
                "strh           r7, [%[output_buf]]             \n\t"   // Store RGB
                "5:                                             \n\t"   //
                : [in_row] "+r" (in_row_group_ctr), [output_buf] "+r" (output_buf)
                : [input_buf] "r" (input_buf), [num_cols] "r" (num_cols)
                : "cc", "memory", "r0", "r1", "r2", "r3", "r4", "r5", "r7", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d14", "d15", "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23", "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31"
                );
}
#endif


METHODDEF(void)
h2v2_merged_upsample_565D (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  my_upsample_ptr upsample = (my_upsample_ptr) cinfo->upsample;
  register int y, cred, cgreen, cblue;
  int cb, cr;
  register JSAMPROW outptr0, outptr1;
  JSAMPROW inptr00, inptr01, inptr1, inptr2;
  JDIMENSION col;
  /* copy these pointers into registers if possible */
  register JSAMPLE * range_limit = cinfo->sample_range_limit;
  int * Crrtab = upsample->Cr_r_tab;
  int * Cbbtab = upsample->Cb_b_tab;
  INT32 * Crgtab = upsample->Cr_g_tab;
  INT32 * Cbgtab = upsample->Cb_g_tab;
  JDIMENSION col_index = 0;
  INT32 d0 = dither_matrix[cinfo->output_scanline & DITHER_MASK];
  INT32 d1 = dither_matrix[(cinfo->output_scanline+1) & DITHER_MASK];
  unsigned int r, g, b;
  INT32 rgb;
  SHIFT_TEMPS

  inptr00 = input_buf[0][in_row_group_ctr*2];
  inptr01 = input_buf[0][in_row_group_ctr*2 + 1];
  inptr1 = input_buf[1][in_row_group_ctr];
  inptr2 = input_buf[2][in_row_group_ctr];
  outptr0 = output_buf[0];
  outptr1 = output_buf[1];
  /* Loop for each group of output pixels */
  for (col = cinfo->output_width >> 1; col > 0; col--) {
    
    /* Do the chroma part of the calculation */
    cb = GETJSAMPLE(*inptr1++);
    cr = GETJSAMPLE(*inptr2++);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    /* Fetch 4 Y values and emit 4 pixels */    
    y  = GETJSAMPLE(*inptr00++);
    r = range_limit[DITHER_565_R(y + cred, d0)];
    g = range_limit[DITHER_565_G(y + cgreen, d0)];
    b = range_limit[DITHER_565_B(y + cblue, d0)];
    d0 = DITHER_ROTATE(d0);
    rgb = PACK_SHORT_565(r,g,b);
    y  = GETJSAMPLE(*inptr00++);
    r = range_limit[DITHER_565_R(y + cred, d1)];
    g = range_limit[DITHER_565_G(y + cgreen, d1)];
    b = range_limit[DITHER_565_B(y + cblue, d1)];
    d1 = DITHER_ROTATE(d1);
    rgb = PACK_TWO_PIXELS(rgb, PACK_SHORT_565(r,g,b));
    WRITE_TWO_PIXELS(outptr0, rgb);
    outptr0 += 4;
    y  = GETJSAMPLE(*inptr01++);
    r = range_limit[DITHER_565_R(y + cred, d0)];
    g = range_limit[DITHER_565_G(y + cgreen, d0)];
    b = range_limit[DITHER_565_B(y + cblue, d0)];
    d0 = DITHER_ROTATE(d0);
    rgb = PACK_SHORT_565(r,g,b);
    y  = GETJSAMPLE(*inptr01++);
    r = range_limit[DITHER_565_R(y + cred, d1)];
    g = range_limit[DITHER_565_G(y + cgreen, d1)];
    b = range_limit[DITHER_565_B(y + cblue, d1)];
    d1 = DITHER_ROTATE(d1);
    rgb = PACK_TWO_PIXELS(rgb, PACK_SHORT_565(r,g,b));
    WRITE_TWO_PIXELS(outptr1, rgb);
    outptr1 += 4;
  }
  /* If image width is odd, do the last output column separately */
  if (cinfo->output_width & 1) {
    cb = GETJSAMPLE(*inptr1);
    cr = GETJSAMPLE(*inptr2);
    cred = Crrtab[cr];
    cgreen = (int) RIGHT_SHIFT(Cbgtab[cb] + Crgtab[cr], SCALEBITS);
    cblue = Cbbtab[cb];
    y  = GETJSAMPLE(*inptr00);
    r = range_limit[DITHER_565_R(y + cred, d0)];
    g = range_limit[DITHER_565_G(y + cgreen, d0)];
    b = range_limit[DITHER_565_B(y + cblue, d0)];
    rgb = PACK_SHORT_565(r,g,b);
    *(INT16*)outptr0 = rgb;
    y  = GETJSAMPLE(*inptr01);
    r = range_limit[DITHER_565_R(y + cred, d1)];
    g = range_limit[DITHER_565_G(y + cgreen, d1)];
    b = range_limit[DITHER_565_B(y + cblue, d1)];
    rgb = PACK_SHORT_565(r,g,b);
    *(INT16*)outptr1 = rgb;
  }
}


// Use NEON optimized version?
#if defined(ENABLE_NEON_H2V2_565D)
#undef H2V2_565D_Proc
#undef H2V2_565D_Table
#define H2V2_565D_Proc h2v2_merged_upsample_565D_neon
#define H2V2_565D_Table FALSE

METHODDEF(void)
h2v2_merged_upsample_565D_neon (j_decompress_ptr cinfo,
              JSAMPIMAGE input_buf, JDIMENSION in_row_group_ctr,
              JSAMPARRAY output_buf)
{
  JDIMENSION num_cols = cinfo->output_width;
  uint8_t* matrix[2];

  // Fallback to non NEON method for small conversions
  if (num_cols < 8)
  {
    my_upsample_ptr upsample = (my_upsample_ptr)cinfo->upsample;

    if (upsample->Cr_r_tab == NULL) {
      build_ycc_rgb_table(cinfo);
    }
    h2v2_merged_upsample_565D(cinfo, input_buf, in_row_group_ctr, output_buf);
    return;
  }

  matrix[0] = (uint8_t*)dither_matrix_neon + ((cinfo->output_scanline & DITHER_MASK) * 12);
  matrix[1] = (uint8_t*)dither_matrix_neon + (((cinfo->output_scanline + 1) & DITHER_MASK) * 12);

  asm volatile (
                // Setup constants
                "ldr            r2, [%[input_buf], #0]          \n\t"   // Setup y input pointer from input_buf[0]
                "vmov.i8        d31, #128                       \n\t"   // Set center sample constant to CENTERJSAMPLE
                "vmov.i32       q5, #32768                      \n\t"   // Set rounding constant to 32768 (0.5)
                "ldr            r0, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "add            r2, r2, %[in_row], lsl #3       \n\t"   //
                "ldr            r1, [r2]                        \n\t"   // input_buf[0][in_row_group_ctr * 2] and
                "vdup.32        q0, r0                          \n\t"   //
                "pld            [r1]                            \n\t"   //
                "ldr            r0, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "ldr            r2, [r2, #4]                    \n\t"   // input_buf[0][in_row_group_ctr * 2 + 1]
                "vdup.32        q1, r0                          \n\t"   //
                "pld            [r2]                            \n\t"   //
                "ldr            r0, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "ldr            r3, [%[input_buf], #4]          \n\t"   // Setup cb input pointer from input_buf[1][in_row_group_ctr]
                "vdup.32        q2, r0                          \n\t"   //
                "ldr            r0, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "ldr            r3, [r3, %[in_row], lsl #2]     \n\t"   //
                "vdup.32        q3, r0                          \n\t"   //
                "pld            [r3]                            \n\t"   //
                "ldr            r4, [%[input_buf], #8]          \n\t"   // Setup cr input pointer from input_buf[2][in_row_group_ctr]
                "ldr            r4, [r4, %[in_row], lsl #2]     \n\t"   //
                "ands           r0, %[num_cols], #0x6           \n\t"   // Calculate first iteration increment
                "pld            [r4]                            \n\t"   //
                "ldr            r5, [%[matrix], #0]             \n\t"   // Load dither values for row 1
                "vld1.8         {d12}, [r5]                     \n\t"   //
                "ldr            r6, [%[matrix], #4]             \n\t"   // Load dither values for row 2
                "vld1.8         {d13}, [r6]                     \n\t"   //
                "beq            1f                              \n\t"   //
                "and            r7, r0, #0x3                    \n\t"   // If columns are not even eight, calculate offset in matrix array
                "add            r5, r7                          \n\t"   //
                "vld1.8         {d14}, [r5]                     \n\t"   // ...and load iteration 2+ dither values for row 1
                "add            r6, r7                          \n\t"   //
                "vld1.8         {d15}, [r6]                     \n\t"   // ...and load iteration 2+ dither values for row 2
                "b              2f                              \n\t"   //
                "1:                                             \n\t"   //
                "vmov           q7, q6                          \n\t"   // If columns are even eight, use the same dither matrix for all iterations
                "mov            r0, #8                          \n\t"   // Do full iteration
                "2:                                             \n\t"   //
                "ldr            r5, [%[output_buf], #0]         \n\t"   // Get output pointer for row 1
                "and            r7, %[num_cols], #0xFFFFFFFE    \n\t"   // Setup loop counter
                "ldr            %[output_buf], [%[output_buf], #4]\n\t" // Get output pointer for row 2
                // loop (columns)
                "3:                                             \n\t"   //
                // Read values, subtract 128 from cb/cr, and expand to 32-bit
                "vld1.32        {d30[0]}, [r3]                  \n\t"   // Load four cb values
                "vld1.32        {d30[1]}, [r4]                  \n\t"   // Load four cr values
                "add            r3, r0, lsr #1                  \n\t"   // Increment input pointer for cb
                "add            r4, r0, lsr #1                  \n\t"   // Increment input pointer for cr
                "pld            [r3]                            \n\t"   //
                "pld            [r4]                            \n\t"   //
                "vsubl.u8       q14, d30, d31                   \n\t"   // Subtract CENTERJSAMPLE from cb/cr and expand to 16-bit
                "vmovl.s16      q10, d28                        \n\t"   // Expand cb values to 32-bit
                "vmovl.s16      q11, d29                        \n\t"   // Expand cr values to 32-bit
                // Multiply with the constants. Split into RGB (Vector multiply)
                "vmul.i32       q13, q10, q2                    \n\t"   // Calculate green
                "vmul.i32       q12, q10, q3                    \n\t"   // Calculate blue
                "vld1.8         {d30}, [r1]                     \n\t"   // Load eight y values for row 1
                "vmla.i32       q13, q11, q1                    \n\t"   //
                "vmul.i32       q11, q11, q0                    \n\t"   // Calculate red
                "vadd.i32       q12, q5                         \n\t"   // Add 0.5 to blue
                "vadd.i32       q13, q5                         \n\t"   // Add 0.5 to green
                "vadd.i32       q11, q5                         \n\t"   // Add 0.5 to red
                // Duplicate RGB result and clone for each row (Vector Shift Right and Insert)
                "add            r1, r0                          \n\t"   // Increment input pointer for y
                "vsri.32        q12, q12, #16                   \n\t"   // Duplicate blue
                "pld            [r1]                            \n\t"   //
                "vsri.32        q13, q13, #16                   \n\t"   // Duplicate green to eight 16-bit values
                "vmov.i32       q9, q12                         \n\t"   // Backup blue for row 2
                "vsri.32        q11, q11, #16                   \n\t"   // Duplicate red
                "vmov.i32       q10, q13                        \n\t"   // Backup green for row 2
                "vmov.i32       q8, q11                         \n\t"   // Backup red for row 2
                // Row 1...
                // Add y (Vector add)
                "vmovl.u8       q14, d30                        \n\t"   // Expand y to 16-bit
                "vld1.8         {d30}, [r2]                     \n\t"   // Load eight y values for row 2
                "vadd.i16       q11, q14                        \n\t"   // Add y to red
                "vadd.i16       q13, q14                        \n\t"   // Add y to green
                "vadd.i16       q12, q14                        \n\t"   // Add y to blue
                // Do the dither
                "vaddw.s8       q11, d12                        \n\t"   // Add dither to red
                "subs           r7, r7, r0                      \n\t"   // Decrement loop counter
                "vaddw.s8       q12, d12                        \n\t"   // Add dither to blue
                "vshr.s8        d12, #1                         \n\t"   // Shift green dither by one, since green will use 6 bits
                "vaddw.s8       q13, d12                        \n\t"   // Add dither to green
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d22, q11                        \n\t"   // Convert red
                "vqmovun.s16    d26, q13                        \n\t"   // Convert green
                "vqmovun.s16    d24, q12                        \n\t"   // Convert blue
                // Pack to 565 format
                "vshll.u8       q11, d22, #8                    \n\t"   // Shift red and expand to 16-bit
                "vshll.u8       q13, d26, #8                    \n\t"   // Shift green and expand to 16-bit
                "vshll.u8       q12, d24, #8                    \n\t"   // Shift blue and expand to 16-bit
                "vsri.u16       q11, q13, #5                    \n\t"   // Insert green into red
                "vsri.u16       q11, q12, #11                   \n\t"   // Insert blue into red
                "vst1.16        {q11}, [r5]                     \n\t"   // Write row 1 result to memory
                "add            r2, r0                          \n\t"   // Increment input pointer for y
                // Row 2...
                // Add y (Vector add)
                "vmovl.u8       q14, d30                        \n\t"   // Expand y to 16-bit
                "pld            [r2]                            \n\t"   //
                "vadd.i16       q9, q14                         \n\t"   // Add y to blue
                "vadd.i16       q8, q14                         \n\t"   // Add y to red
                "vadd.i16       q10, q14                        \n\t"   // Add y to green
                // Do the dither
                "vaddw.s8       q9, d13                         \n\t"   // Add dither to blue
                "vaddw.s8       q8, d13                         \n\t"   // Add dither to red
                "vshr.s8        d13, #1                         \n\t"   // Shift green dither by one, since green will use 6 bits
                "add            r5, r0, lsl #1                  \n\t"   // Increment output buffer pointer for row 1
                "vaddw.s8       q10, d13                        \n\t"   // Add dither to green
                // Convert result from signed 16-bit to unsigned 8-bit with range limitation.
                // Range-limiting is essential due to noise introduced by DCT losses.
                // (Vector Saturating Move and Narrow, signed operand with Unsigned result)
                "vqmovun.s16    d16, q8                         \n\t"   // Convert red
                "vqmovun.s16    d20, q10                        \n\t"   // Convert green
                "vqmovun.s16    d18, q9                         \n\t"   // Convert blue
                // Pack to 565 format
                "vshll.u8       q8, d16, #8                     \n\t"   // Shift red and expand to 16-bit
                "vshll.u8       q10, d20, #8                    \n\t"   // Shift green and expand to 16-bit
                "vshll.u8       q9, d18, #8                     \n\t"   // Shift blue and expand to 16-bit
                "vsri.u16       q8, q10, #5                     \n\t"   // Insert green into red
                "vsri.u16       q8, q9, #11                     \n\t"   // Insert blue into red
                "vst1.16        {q8}, [%[output_buf]]           \n\t"   // Write row 2 result to memory
                // Increase pointers and counters
                "add            %[output_buf], r0, lsl #1       \n\t"   // Increment output buffer pointer for row 2
                "vmov.i8        q6, q7                          \n\t"   // Set dither matrix to iteration 2+ values
                "mov            r0, #8                          \n\t"   // Set next loop iteration length
                "bne            3b                              \n\t"   // If loop counter != 0, loop
                // Handle any last odd pixel
                "tst            %[num_cols], #1                 \n\t"   // Is there a last odd pixel?
                "beq            4f                              \n\t"   // If not, exit
                "ldr            r7, ="CR_R_CONST"               \n\t"   // Load Cr_r constant for red part of YCrCb to RGB conversion
                "ldrb           r4, [r4]                        \n\t"   // Load cr value
                "ldrb           r3, [r3]                        \n\t"   // Load cb value
                "ldrb           r1, [r1]                        \n\t"   // Load y value for row 1
                "ldrb           %[in_row], [r2]                 \n\t"   // Load y value for row 2
                // Calculate red
                "sub            r4, #128                        \n\t"   // Subtract CENTERJSAMPLE from cr
                "smulwb         r0, r7, r4                      \n\t"   // Calculate red
                // Calculate green
                "ldr            r7, ="CR_G_CONST"               \n\t"   // Load Cr_g constant for green part of YCrCb to RGB conversion
                "smulwb         r4, r7, r4                      \n\t"   // Calculate green
                "ldr            r7, ="CB_G_CONST"               \n\t"   // Load Cb_g constant for green part of YCrCb to RGB conversion
                "sub            r3, #128                        \n\t"   // Subtract CENTERJSAMPLE from cb
                "smlawb         r4, r7, r3, r4                  \n\t"   // Calculate second green
                // Calculate blue
                "ldr            r7, ="CB_B_CONST"               \n\t"   // Load Cb_b constant for blue part of YCrCb to RGB conversion
                "smulwb         r3, r7, r3                      \n\t"   // Calculate blue
                // Calculate row 1 RGB
                "ldr            r7, [%[matrix], #0]             \n\t"   // Load dither table address
                "and            r2, %[num_cols], #0x2           \n\t"   // Calculate offset in matrix array
                "ldrb           r6, [r7, r2]                    \n\t"   // Load dither value for row 1
                "add            r2, r4, r1                      \n\t"   // Add y to green
                "add            r2, r6, asr #1                  \n\t"   // Add dither to green
                "usat           r2, #6, r2, asr #2              \n\t"   // Saturate to 0-63
                "add            r1, r6                          \n\t"   // Add dither to y
                "add            r7, r3, r1                      \n\t"   // Add y to blue
                "usat           r7, #5, r7, asr #3              \n\t"   // Saturate to 0-31
                "orr            r7, r2, lsl #5                  \n\t"   // Insert green into blue
                "add            r2, r0, r1                      \n\t"   // Add y to red
                "usat           r2, #5, r2, asr #3              \n\t"   // Saturate to 0-31
                "orr            r7, r2, lsl #11                 \n\t"   // Insert red into blue
                "strh           r7, [r5]                        \n\t"   // Store RGB
                // Calculate row 2 RGB
                "ldr            r7, [%[matrix], #4]             \n\t"   // Load dither table address
                "and            r2, %[num_cols], #0x2           \n\t"   // Calculate offset in matrix array
                "ldrb           r6, [r7, r2]                    \n\t"   // Load dither value for row 2
                "add            r2, r4, %[in_row]               \n\t"   // Add y to green
                "add            r2, r6, asr #1                  \n\t"   // Add dither to green
                "usat           r2, #6, r2, asr #2              \n\t"   // Saturate to 0-63
                "add            %[in_row], r6                   \n\t"   // Add dither to y
                "add            r7, r3, %[in_row]               \n\t"   // Add y to blue
                "usat           r7, #5, r7, asr #3              \n\t"   // Saturate to 0-31
                "orr            r7, r2, lsl #5                  \n\t"   // Insert green into blue
                "add            r2, r0, %[in_row]               \n\t"   // Add y to red
                "usat           r2, #5, r2, asr #3              \n\t"   // Saturate to 0-31
                "orr            r7, r2, lsl #11                 \n\t"   // Insert red into blue
                "strh           r7, [%[output_buf]]             \n\t"   // Store RGB
                "4:                                             \n\t"   //
                : [in_row] "+r" (in_row_group_ctr), [output_buf] "+r" (output_buf)
                : [input_buf] "r" (input_buf), [num_cols] "r" (num_cols), [matrix] "r" (matrix)
                : "cc", "memory", "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d10", "d11", "d12", "d13", "d14", "d15", "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23", "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31"
                );
}
#endif

#endif

/*
 * Module initialization routine for merged upsampling/color conversion.
 *
 * NB: this is called under the conditions determined by use_merged_upsample()
 * in jdmaster.c.  That routine MUST correspond to the actual capabilities
 * of this module; no safety checks are made here.
 */

GLOBAL(void)
jinit_merged_upsampler (j_decompress_ptr cinfo)
{
  my_upsample_ptr upsample;
  int table = TRUE;

  upsample = (my_upsample_ptr)
    (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE,
				SIZEOF(my_upsampler));
  cinfo->upsample = (struct jpeg_upsampler *) upsample;
  upsample->pub.start_pass = start_pass_merged_upsample;
  upsample->pub.need_context_rows = FALSE;

  upsample->out_row_width = cinfo->output_width * cinfo->out_color_components;
  
  if (cinfo->max_v_samp_factor == 2) {
    upsample->pub.upsample = merged_2v_upsample;
    upsample->upmethod = H2V2_Proc;
    table = H2V2_Table;
#ifdef ANDROID_RGB
    if (cinfo->out_color_space == JCS_RGBA_8888) {
        upsample->upmethod = H2V2_8888_Proc;
        table = H2V2_8888_Table;
    } else if (cinfo->out_color_space == JCS_RGB_565) {
        if (cinfo->dither_mode == JDITHER_NONE) {
            upsample->upmethod = H2V2_565_Proc;
            table = H2V2_565_Table;
        } else {
            upsample->upmethod = H2V2_565D_Proc;
            table = H2V2_565D_Table;
        }
    }
#endif
    /* Allocate a spare row buffer */
    upsample->spare_row = (JSAMPROW)
      (*cinfo->mem->alloc_large) ((j_common_ptr) cinfo, JPOOL_IMAGE,
		(size_t) (upsample->out_row_width * SIZEOF(JSAMPLE)));
  } else {
    upsample->pub.upsample = merged_1v_upsample;
    upsample->upmethod = H2V1_Proc;
    table = H2V1_Table;
#ifdef ANDROID_RGB
    if (cinfo->out_color_space == JCS_RGBA_8888) {
        upsample->upmethod = H2V1_8888_Proc;
        table = H2V1_8888_Table;
    } else if (cinfo->out_color_space == JCS_RGB_565) {
        if (cinfo->dither_mode == JDITHER_NONE) {
            upsample->upmethod = H2V1_565_Proc;
            table = H2V1_565_Table;
        } else {
            upsample->upmethod = H2V1_565D_Proc;
            table = H2V1_565D_Table;
        }
    }
#endif
    /* No spare row needed */
    upsample->spare_row = NULL;
  }

#if defined(ENABLE_NEON_H2V2_565D) || defined(ENABLE_NEON_H2V1_565D) || \
    defined(ENABLE_NEON_H2V2_565) || defined(ENABLE_NEON_H2V1_565) || \
    defined(ENABLE_NEON_H2V2_8888) || defined(ENABLE_NEON_H2V1_8888) || \
    defined(ENABLE_NEON_H2V2) || defined(ENABLE_NEON_H2V1)
  if (!table) {
    clear_ycc_rgb_table(cinfo);
  } else {
    build_ycc_rgb_table(cinfo);
  }
#else
  build_ycc_rgb_table(cinfo);
#endif
}

#endif /* UPSAMPLE_MERGING_SUPPORTED */
