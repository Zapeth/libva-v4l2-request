/*
 * Copyright (C) 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "mpeg2.h"
#include "context.h"
#include "request.h"
#include "surface.h"

#include <assert.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/videodev2.h>

#include "v4l2.h"

int mpeg2_set_controls(struct request_data *driver_data,
		       struct object_context *context_object,
		       struct object_surface *surface_object)
{
	VAPictureParameterBufferMPEG2 *VAPicture =
		&surface_object->params.mpeg2.picture;
	VAIQMatrixBufferMPEG2 *iqmatrix =
		&surface_object->params.mpeg2.iqmatrix;
	bool iqmatrix_set = surface_object->params.mpeg2.iqmatrix_set;
	struct v4l2_ctrl_mpeg2_sequence sequence = { 0 };
	struct v4l2_ctrl_mpeg2_picture picture = { 0 };
	struct v4l2_ctrl_mpeg2_quantisation quantisation = { 0 };
	struct object_surface *forward_reference_surface;
	struct object_surface *backward_reference_surface;
	uint64_t timestamp;
	unsigned int i;
	int rc;

	sequence.horizontal_size = VAPicture->horizontal_size;
	sequence.vertical_size = VAPicture->vertical_size;
	sequence.vbv_buffer_size = SOURCE_SIZE_MAX;

	sequence.profile_and_level_indication = 0;
	sequence.flags = 0;
	sequence.chroma_format = 1; // 4:2:0

	rc = v4l2_set_control(driver_data->video_fd, surface_object->request_fd,
			      V4L2_CID_STATELESS_MPEG2_SEQUENCE,
			      &sequence, sizeof(sequence));
	if (rc < 0)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	picture.picture_coding_type = VAPicture->picture_coding_type;
	picture.f_code[0][0] = (VAPicture->f_code >> 12) & 0x0f;
	picture.f_code[0][1] = (VAPicture->f_code >> 8) & 0x0f;
	picture.f_code[1][0] = (VAPicture->f_code >> 4) & 0x0f;
	picture.f_code[1][1] = (VAPicture->f_code >> 0) & 0x0f;

	picture.intra_dc_precision =
		VAPicture->picture_coding_extension.bits.intra_dc_precision;
	picture.picture_structure =
		VAPicture->picture_coding_extension.bits.picture_structure;

	if (VAPicture->picture_coding_extension.bits.top_field_first)
		picture.flags |= V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST;

	if (VAPicture->picture_coding_extension.bits.frame_pred_frame_dct)
		picture.flags |= V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT;

	if (VAPicture->picture_coding_extension.bits.concealment_motion_vectors)
		picture.flags |= V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV;

	if (VAPicture->picture_coding_extension.bits.q_scale_type)
		picture.flags |= V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE;

	if (VAPicture->picture_coding_extension.bits.intra_vlc_format)
		picture.flags |= V4L2_MPEG2_PIC_FLAG_INTRA_VLC;

	if (VAPicture->picture_coding_extension.bits.alternate_scan)
		picture.flags |= V4L2_MPEG2_PIC_FLAG_ALT_SCAN;

	if (VAPicture->picture_coding_extension.bits.repeat_first_field)
		picture.flags |= V4L2_MPEG2_PIC_FLAG_REPEAT_FIRST;

	if (VAPicture->picture_coding_extension.bits.progressive_frame)
		picture.flags |= V4L2_MPEG2_PIC_FLAG_PROGRESSIVE;

	forward_reference_surface =
		SURFACE(driver_data, VAPicture->forward_reference_picture);
	if (forward_reference_surface == NULL)
		forward_reference_surface = surface_object;

	timestamp = v4l2_timeval_to_ns(&forward_reference_surface->timestamp);
	picture.forward_ref_ts = timestamp;

	backward_reference_surface =
		SURFACE(driver_data, VAPicture->backward_reference_picture);
	if (backward_reference_surface == NULL)
		backward_reference_surface = surface_object;

	timestamp = v4l2_timeval_to_ns(&backward_reference_surface->timestamp);
	picture.backward_ref_ts = timestamp;

	rc = v4l2_set_control(driver_data->video_fd, surface_object->request_fd,
			      V4L2_CID_STATELESS_MPEG2_PICTURE,
			      &picture, sizeof(picture));
	if (rc < 0)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	if (iqmatrix_set) {
		for (i = 0; i < 64; i++) {
			quantisation.intra_quantiser_matrix[i] =
				iqmatrix->intra_quantiser_matrix[i];
			quantisation.non_intra_quantiser_matrix[i] =
				iqmatrix->non_intra_quantiser_matrix[i];
			quantisation.chroma_intra_quantiser_matrix[i] =
				iqmatrix->chroma_intra_quantiser_matrix[i];
			quantisation.chroma_non_intra_quantiser_matrix[i] =
				iqmatrix->chroma_non_intra_quantiser_matrix[i];
		}

		rc = v4l2_set_control(driver_data->video_fd,
				      surface_object->request_fd,
				      V4L2_CID_STATELESS_MPEG2_QUANTISATION,
				      &quantisation, sizeof(quantisation));
	}

	return 0;
}
