/*
 * VC-1 and WMV3 parser
 * Copyright (c) 2006-2007 Konstantin Shishkov
 * Partly based on vc9.c (c) 2005 Anonymous, Alex Beregszaszi, Michael Niedermayer
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * VC-1 and WMV3 parser
 */

#include "libavutil/attributes.h"
#include "libavutil/mem.h"
#include "parser.h"
#include "vc1.h"
#include "get_bits.h"
#include "vc1dsp.h"

typedef struct VC1ParseContext {
    ParseContext pc;
    VC1Context v;
} VC1ParseContext;

static void vc1_extract_headers(AVCodecParserContext *s, AVCodecContext *avctx,
                                const uint8_t *buf, int buf_size)
{
    VC1ParseContext *vpc = s->priv_data;
    GetBitContext gb;
    const uint8_t *start, *end, *next;
    uint8_t *buf2 = av_mallocz(buf_size + AV_INPUT_BUFFER_PADDING_SIZE);

    vpc->v.s.avctx = avctx;
    vpc->v.parse_only = 1;
    vpc->v.first_pic_header_flag = 1;
    next = buf;
    s->repeat_pict = 0;

    for(start = buf, end = buf + buf_size; next < end; start = next){
        int buf2_size, size;
        int ret;

        next = find_next_marker(start + 4, end);
        size = next - start - 4;
        buf2_size = vc1_unescape_buffer(start + 4, size, buf2);
        init_get_bits8(&gb, buf2, buf2_size);
        if(size <= 0) continue;
        switch(AV_RB32(start)){
        case VC1_CODE_SEQHDR:
            ff_vc1_decode_sequence_header(avctx, &vpc->v, &gb);
            break;
        case VC1_CODE_ENTRYPOINT:
            ff_vc1_decode_entry_point(avctx, &vpc->v, &gb);
            break;
        case VC1_CODE_FRAME:
            if(vpc->v.profile < PROFILE_ADVANCED)
                ret = ff_vc1_parse_frame_header    (&vpc->v, &gb);
            else
                ret = ff_vc1_parse_frame_header_adv(&vpc->v, &gb);

            if (ret < 0)
                break;

            /* keep AV_PICTURE_TYPE_BI internal to VC1 */
            if (vpc->v.s.pict_type == AV_PICTURE_TYPE_BI)
                s->pict_type = AV_PICTURE_TYPE_B;
            else
                s->pict_type = vpc->v.s.pict_type;

            if (avctx->ticks_per_frame > 1){
                // process pulldown flags
                s->repeat_pict = 1;
                // Pulldown flags are only valid when 'broadcast' has been set.
                // So ticks_per_frame will be 2
                if (vpc->v.rff){
                    // repeat field
                    s->repeat_pict = 2;
                }else if (vpc->v.rptfrm){
                    // repeat frames
                    s->repeat_pict = vpc->v.rptfrm * 2 + 1;
                }
            }

            if (vpc->v.broadcast && vpc->v.interlace && !vpc->v.psf)
                s->field_order = vpc->v.tff ? AV_FIELD_TT : AV_FIELD_BB;
            else
                s->field_order = AV_FIELD_PROGRESSIVE;

            break;
        }
        if (avctx->framerate.num)
            avctx->time_base = av_inv_q(av_mul_q(avctx->framerate, (AVRational){avctx->ticks_per_frame, 1}));
        s->format = vpc->v.chromaformat == 1 ? AV_PIX_FMT_YUV420P
                                             : AV_PIX_FMT_NONE;
        if (avctx->width && avctx->height) {
            s->width        = avctx->width;
            s->height       = avctx->height;
            s->coded_width  = FFALIGN(avctx->coded_width,  16);
            s->coded_height = FFALIGN(avctx->coded_height, 16);
        }
    }

    av_free(buf2);
}

/**
 * Find the end of the current frame in the bitstream.
 * @return the position of the first byte of the next frame, or -1
 */
static int vc1_find_frame_end(ParseContext *pc, const uint8_t *buf,
                               int buf_size) {
    int pic_found, i;
    uint32_t state;

    pic_found= pc->frame_start_found;
    state= pc->state;

    i=0;
    if(!pic_found){
        for(i=0; i<buf_size; i++){
            state= (state<<8) | buf[i];
            if(state == VC1_CODE_FRAME || state == VC1_CODE_FIELD){
                i++;
                pic_found=1;
                break;
            }
        }
    }

    if(pic_found){
        /* EOF considered as end of frame */
        if (buf_size == 0)
            return 0;
        for(; i<buf_size; i++){
            state= (state<<8) | buf[i];
            if(IS_MARKER(state) && state != VC1_CODE_FIELD && state != VC1_CODE_SLICE){
                pc->frame_start_found=0;
                pc->state=-1;
                return i-3;
            }
        }
    }
    pc->frame_start_found= pic_found;
    pc->state= state;
    return END_NOT_FOUND;
}

static int vc1_parse(AVCodecParserContext *s,
                           AVCodecContext *avctx,
                           const uint8_t **poutbuf, int *poutbuf_size,
                           const uint8_t *buf, int buf_size)
{
    VC1ParseContext *vpc = s->priv_data;
    int next;

    if(s->flags & PARSER_FLAG_COMPLETE_FRAMES){
        next= buf_size;
    }else{
        next= vc1_find_frame_end(&vpc->pc, buf, buf_size);

        if (ff_combine_frame(&vpc->pc, next, &buf, &buf_size) < 0) {
            *poutbuf = NULL;
            *poutbuf_size = 0;
            return buf_size;
        }
    }

    vc1_extract_headers(s, avctx, buf, buf_size);

    *poutbuf = buf;
    *poutbuf_size = buf_size;
    return next;
}

static av_cold int vc1_parse_init(AVCodecParserContext *s)
{
    VC1ParseContext *vpc = s->priv_data;
    vpc->v.s.slice_context_count = 1;
    ff_vc1dsp_init(&vpc->v.vc1dsp); /* startcode_find_candidate */
    return 0;
}

const AVCodecParser ff_vc1_parser = {
    .codec_ids      = { AV_CODEC_ID_VC1 },
    .priv_data_size = sizeof(VC1ParseContext),
    .parser_init    = vc1_parse_init,
    .parser_parse   = vc1_parse,
    .parser_close   = ff_parse_close,
};
