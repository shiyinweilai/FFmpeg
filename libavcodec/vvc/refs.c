/*
 * VVC reference management
 *
 * Copyright (C) 2023 Nuo Mi
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

#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>
#include "libavutil/mem.h"
#include "libavutil/thread.h"
#include "libavutil/refstruct.h"
#include "libavutil/video_enc_params.h"   /* PlayerX 定制：VVC 块级导出 */
#include "libavutil/codec_block_info.h"   /* PlayerX 定制：逐 CU 编码信息 */
#include "libavcodec/vvc/ctu.h"           /* PlayerX 定制：CodingUnit 定义 */
#include "libavcodec/thread.h"
#include "libavcodec/decode.h"

#include "refs.h"


typedef struct FrameProgress {
    atomic_int progress[VVC_PROGRESS_LAST];
    VVCProgressListener *listener[VVC_PROGRESS_LAST];
    AVMutex lock;
    AVCond  cond;
    uint8_t has_lock;
    uint8_t has_cond;
} FrameProgress;

void ff_vvc_unref_frame(VVCFrameContext *fc, VVCFrame *frame, int flags)
{
    /* frame->frame can be NULL if context init failed */
    if (!frame->frame || !frame->frame->buf[0])
        return;

    frame->flags &= ~flags;
    if (!(frame->flags & ~VVC_FRAME_FLAG_CORRUPT))
        frame->flags = 0;
    if (!frame->flags) {
        av_frame_unref(frame->frame);

        if (frame->needs_fg) {
            av_frame_unref(frame->frame_grain);
            frame->needs_fg = 0;
        }

        av_refstruct_unref(&frame->sps);
        av_refstruct_unref(&frame->pps);
        av_refstruct_unref(&frame->progress);

        av_refstruct_unref(&frame->tab_dmvr_mvf);

        av_refstruct_unref(&frame->rpl);
        frame->nb_rpl_elems = 0;
        av_refstruct_unref(&frame->rpl_tab);

        /* PlayerX: release partition snapshot */
        av_freep(&frame->cu_snap);
        frame->nb_cu_snap   = 0;
        frame->cu_snap_cap  = 0;

        frame->collocated_ref = NULL;
        av_refstruct_unref(&frame->hwaccel_picture_private);
    }
}

const RefPicList *ff_vvc_get_ref_list(const VVCFrameContext *fc, const VVCFrame *ref, int x0, int y0)
{
    const int x_cb         = x0 >> fc->ps.sps->ctb_log2_size_y;
    const int y_cb         = y0 >> fc->ps.sps->ctb_log2_size_y;
    const int pic_width_cb = fc->ps.pps->ctb_width;
    const int ctb_addr_rs  = y_cb * pic_width_cb + x_cb;

    return (const RefPicList *)ref->rpl_tab[ctb_addr_rs];
}

void ff_vvc_clear_refs(VVCFrameContext *fc)
{
    for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++)
        ff_vvc_unref_frame(fc, &fc->DPB[i],
            VVC_FRAME_FLAG_SHORT_REF | VVC_FRAME_FLAG_LONG_REF);
}

void ff_vvc_flush_dpb(VVCFrameContext *fc)
{
    for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++)
        ff_vvc_unref_frame(fc, &fc->DPB[i], ~0);
}

static void free_progress(AVRefStructOpaque unused, void *obj)
{
    FrameProgress *p = (FrameProgress *)obj;

    if (p->has_cond)
        ff_cond_destroy(&p->cond);
    if (p->has_lock)
        ff_mutex_destroy(&p->lock);
}

static FrameProgress *alloc_progress(void)
{
    FrameProgress *p = av_refstruct_alloc_ext(sizeof(*p), 0, NULL, free_progress);

    if (p) {
        p->has_lock = !ff_mutex_init(&p->lock, NULL);
        p->has_cond = !ff_cond_init(&p->cond, NULL);
        if (!p->has_lock || !p->has_cond)
            av_refstruct_unref(&p);
    }
    return p;
}

static VVCFrame *alloc_frame(VVCContext *s, VVCFrameContext *fc)
{
    const VVCSPS *sps = fc->ps.sps;
    const VVCPPS *pps = fc->ps.pps;
    for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++) {
        int ret;
        VVCFrame *frame = &fc->DPB[i];
        VVCWindow *win = &frame->scaling_win;
        if (frame->frame->buf[0])
            continue;

        frame->sps = av_refstruct_ref_c(fc->ps.sps);
        frame->pps = av_refstruct_ref_c(fc->ps.pps);

        // Add LCEVC SEI metadata here, as it's needed in get_buffer()
        if (fc->sei.common.itut_t35.lcevc) {
            ret = ff_frame_new_side_data_from_buf(s->avctx, frame->frame,
                                                  AV_FRAME_DATA_LCEVC, &fc->sei.common.itut_t35.lcevc);
            if (ret < 0)
                goto fail;
        }

        ret = ff_thread_get_buffer(s->avctx, frame->frame, AV_GET_BUFFER_FLAG_REF);
        if (ret < 0)
            return NULL;

        frame->rpl = av_refstruct_allocz(s->current_frame.nb_units * sizeof(RefPicListTab));
        if (!frame->rpl)
            goto fail;
        frame->nb_rpl_elems = s->current_frame.nb_units;

        frame->tab_dmvr_mvf = av_refstruct_pool_get(fc->tab_dmvr_mvf_pool);
        if (!frame->tab_dmvr_mvf)
            goto fail;

        frame->rpl_tab = av_refstruct_pool_get(fc->rpl_tab_pool);
        if (!frame->rpl_tab)
            goto fail;
        frame->ctb_count = pps->ctb_width * pps->ctb_height;
        for (int j = 0; j < frame->ctb_count; j++)
            frame->rpl_tab[j] = frame->rpl;

        win->left_offset   = pps->r->pps_scaling_win_left_offset   * (1 << sps->hshift[CHROMA]);
        win->right_offset  = pps->r->pps_scaling_win_right_offset  * (1 << sps->hshift[CHROMA]);
        win->top_offset    = pps->r->pps_scaling_win_top_offset    * (1 << sps->vshift[CHROMA]);
        win->bottom_offset = pps->r->pps_scaling_win_bottom_offset * (1 << sps->vshift[CHROMA]);
        frame->ref_width   = pps->r->pps_pic_width_in_luma_samples  - win->left_offset   - win->right_offset;
        frame->ref_height  = pps->r->pps_pic_height_in_luma_samples - win->bottom_offset - win->top_offset;

        if (fc->sei.frame_field_info.present) {
            if (fc->sei.frame_field_info.picture_struct == AV_PICTURE_STRUCTURE_TOP_FIELD)
                frame->frame->flags |= AV_FRAME_FLAG_TOP_FIELD_FIRST;
            if (fc->sei.frame_field_info.picture_struct == AV_PICTURE_STRUCTURE_TOP_FIELD ||
                fc->sei.frame_field_info.picture_struct == AV_PICTURE_STRUCTURE_BOTTOM_FIELD)
                frame->frame->flags |= AV_FRAME_FLAG_INTERLACED;
        }

        frame->progress = alloc_progress();
        if (!frame->progress)
            goto fail;

        ret = ff_hwaccel_frame_priv_alloc(s->avctx, &frame->hwaccel_picture_private);
        if (ret < 0)
            goto fail;

        return frame;
fail:
        ff_vvc_unref_frame(fc, frame, ~0);
        return NULL;
    }
    av_log(s->avctx, AV_LOG_ERROR, "Error allocating frame, DPB full.\n");
    return NULL;
}

static void set_pict_type(AVFrame *frame, const VVCContext *s, const VVCFrameContext *fc)
{
    bool has_b = false, has_inter = false;

    if (IS_IRAP(s)) {
        frame->pict_type = AV_PICTURE_TYPE_I;
        frame->flags |= AV_FRAME_FLAG_KEY;
        return;
    }

    if (fc->ps.ph.r->ph_inter_slice_allowed_flag) {
        // At this point, fc->slices is not fully initialized; we need to inspect the CBS directly.
        const CodedBitstreamFragment *current = &s->current_frame;
        for (int i = 0; i < current->nb_units && !has_b; i++) {
            const CodedBitstreamUnit *unit = current->units + i;
            if (unit->content_ref && unit->type <= VVC_RSV_IRAP_11) {
                const H266RawSliceHeader *rsh = unit->content_ref;
                has_inter |= !IS_I(rsh);
                has_b     |= IS_B(rsh);
            }
        }
    }
    if (!has_inter)
        frame->pict_type = AV_PICTURE_TYPE_I;
    else if (has_b)
        frame->pict_type = AV_PICTURE_TYPE_B;
    else
        frame->pict_type = AV_PICTURE_TYPE_P;
}

int ff_vvc_set_new_ref(VVCContext *s, VVCFrameContext *fc, AVFrame **frame)
{
    const VVCPH *ph= &fc->ps.ph;
    const int poc = ph->poc;
    VVCFrame *ref;

    /* check that this POC doesn't already exist */
    for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++) {
        VVCFrame *frame = &fc->DPB[i];

        if (frame->frame->buf[0] && frame->sequence == s->seq_decode &&
            frame->poc == poc) {
            av_log(s->avctx, AV_LOG_ERROR, "Duplicate POC in a sequence: %d.\n", poc);
            return AVERROR_INVALIDDATA;
        }
    }

    ref = alloc_frame(s, fc);
    if (!ref)
        return AVERROR(ENOMEM);

    set_pict_type(ref->frame, s, fc);
    *frame = ref->frame;
    fc->ref = ref;

    /* PlayerX: 不在此处清零 nb_cu_snap。
     * 该槽位只有走过 ff_vvc_unref_frame（buf[0] 已 NULL）才会被 alloc_frame
     * 复用，而 unref 时已由 av_freep(&cu_snap) 把 nb_cu_snap 归零。若在此处
     * 额外清零，会在“数据缓冲仍在、帧尚未真正释放”的窗口把计数抹成 0，
     * 导致该 POC 被输出时导出端按 nb_cu_snap>0 查不到快照（表现为个别帧无
     * CU 网格）。计数的生命周期须与 cu_snap 缓冲严格一致，仅在 av_freep 时归零。 */
    if (s->no_output_before_recovery_flag && (IS_RASL(s) || !GDR_IS_RECOVERED(s)))
        ref->flags = VVC_FRAME_FLAG_SHORT_REF;
    else if (ph->r->ph_pic_output_flag)
        ref->flags = VVC_FRAME_FLAG_OUTPUT | VVC_FRAME_FLAG_SHORT_REF;

    if (!ph->r->ph_non_ref_pic_flag)
        ref->flags |= VVC_FRAME_FLAG_SHORT_REF;

    ref->poc      = poc;
    ref->sequence = s->seq_decode;
    ref->frame->crop_left   = fc->ps.pps->r->pps_conf_win_left_offset << fc->ps.sps->hshift[CHROMA];
    ref->frame->crop_right  = fc->ps.pps->r->pps_conf_win_right_offset << fc->ps.sps->hshift[CHROMA];
    ref->frame->crop_top    = fc->ps.pps->r->pps_conf_win_top_offset << fc->ps.sps->vshift[CHROMA];
    ref->frame->crop_bottom = fc->ps.pps->r->pps_conf_win_bottom_offset << fc->ps.sps->vshift[CHROMA];

    return 0;
}

/*
 * PlayerX 定制：导出 VVC（H.266）真实 CU 划分 + QP。
 *
 * VVC 的编码单元是 CTU（CtbSizeY，通常 64 或 128），内部可用
 * 四叉树 + 多类型树（MTT）递归划分为各种矩形 CU，尺寸远丰富于 H.264
 * （如 64x64 / 32x16 / 8x128 / 4x4 等，含非方形）。
 *
 * 数据来源（出帧时 fc->tab 仍有效）：
 *   fc->tab.cus[rs]  —— 每个 CTU 的 CodingUnit 链表（cu->next 串联）
 *   cu->x0 / y0      —— 亮度像素坐标（CU 左上角）
 *   cu->cb_width / cb_height —— CU 尺寸
 *   fc->tab.qp[LUMA] —— 按 min_cb（4x4）为单位索引的亮度 QP
 *
 * 复用与 H.264 相同的 AV_FRAME_DATA_VIDEO_ENC_PARAMS 管线，
 * 上层 RBBlockAnalyzer 无需改逻辑即可显示（已尺寸无关）。
 * 注意：AVVideoEncParamsType 目前无 VVC 枚举，暂用 H264 容器，
 * 上层仅读取块坐标/尺寸/delta_qp，不区分 type。
 */
void ff_vvc_export_enc_params(VVCContext *s, const VVCFrameContext *fc,
                              const VVCFrame *vf, AVFrame *out)
{
    const VVCSPS *sps = fc->ps.sps;
    const VVCPPS *pps = fc->ps.pps;
    unsigned int nb_blocks = 0;
    AVVideoEncParams *par;
    unsigned int block_idx = 0;
    const int ctu_count    = fc->tab.sz.ctu_count;
    /* 亮度 QP 按 min_cb（4x4）网格索引 */
    const int min_cb_log2  = sps ? sps->min_cb_log2_size_y : 2;
    const int min_cb_width = pps ? pps->min_cb_width : 0;
    int frame_qp           = 0;
    AVFrame  *dst = out ? out : (vf ? vf->frame : NULL);

    if (!sps || !pps || !fc->tab.cus || !fc->tab.qp[LUMA] || ctu_count <= 0)
        return;
    if (!dst)
        return;

    /* PlayerX: 本函数现由 frame_end() 调用，vf == fc->ref、out == fc->ref->frame，
     * cu_snap 此刻已在 vf 上定稿。快照与承载帧一一对应，无需跨 frame context
     * 查找（旧的按 POC 跨 fc 借用方案已废弃，那既有 use-after-free 风险，命中率
     * 又不满，导致非参考帧丢网格）。 */
    if (vf && vf->nb_cu_snap > 0 && vf->cu_snap) {
        nb_blocks = vf->nb_cu_snap;
        if (!nb_blocks)
            return;

        par = av_video_enc_params_create_side_data(
                  dst, AV_VIDEO_ENC_PARAMS_H264, nb_blocks);
        if (!par)
            return;

        frame_qp = 0;
        par->qp  = frame_qp;

        /* PlayerX: 与 enc_params 块一一对应的完整编码信息 */
        /* PlayerX: 与 enc_params 块一一对应的完整编码信息。
         * 注意 av_frame_new_side_data() 返回的是 AVFrameSideData*，
         * 真正的缓冲区在 ->data，不可直接当数据指针使用。 */
        AVFrameSideData *cbsd = av_frame_new_side_data(
            dst, AV_FRAME_DATA_CODEC_BLOCK_INFO,
            nb_blocks * (int)sizeof(AVCodecBlockInfo));
        AVCodecBlockInfo *cb = cbsd
            ? (AVCodecBlockInfo *)cbsd->data : NULL;
        if (cb)
            memset(cb, 0, nb_blocks * sizeof(AVCodecBlockInfo));

        for (int i = 0; i < nb_blocks; i++) {
            const struct VVCCUInfo *ci = &vf->cu_snap[i];
            AVVideoBlockParams *b = av_video_enc_params_block(par, block_idx++);
            b->src_x    = ci->x;
            b->src_y    = ci->y;
            b->w        = ci->w;
            b->h        = ci->h;
            b->delta_qp = ci->qp;

            if (cb) {
                AVCodecBlockInfo *o = &cb[i];
                o->x          = ci->x;
                o->y          = ci->y;
                o->w          = ci->w;
                o->h          = ci->h;
                o->qp         = ci->qp;
                o->pred_mode  = ci->pred_mode;
                o->pred_flag  = ci->pred_flag;
                o->skip_flag  = ci->skip_flag;
                o->qt_depth   = ci->depth;
                o->tree_type  = ci->tree_type;
                o->ref_idx[0] = ci->ref_idx[0];
                o->ref_idx[1] = ci->ref_idx[1];
                o->mv[0][0]   = ci->mv[0][0];
                o->mv[0][1]   = ci->mv[0][1];
                o->mv[1][0]   = ci->mv[1][0];
                o->mv[1][1]   = ci->mv[1][1];
                /* MV 已在 ff_vvc_cu_snap_fill() 里从 pu->mi 正确回填，
                 * 此处不再从 fc->tab.mvf 重读——frame_end 阶段 tab.mvf
                 * 可能已属于下一帧，重读会污染运动信息。 */
            }
        }
        return;
    } else {
        const int min_cb_w  = pps->min_cb_width;
        const int min_cb_h  = pps->min_cb_height;
        const int unit      = 1 << min_cb_log2;   /* 通常 4 */

        for (int ycb = 0; ycb < min_cb_h; ycb++) {
            for (int xcb = 0; xcb < min_cb_w; xcb++) {
                const int w = SAMPLE_CTB(fc->tab.cb_width[LUMA],  xcb, ycb);
                const int h = SAMPLE_CTB(fc->tab.cb_height[LUMA], xcb, ycb);
                if (w > 0 && h > 0 &&
                    (xcb * unit) % w == 0 && (ycb * unit) % h == 0)
                    nb_blocks++;
            }
        }
        if (!nb_blocks)
            return;

        par = av_video_enc_params_create_side_data(
                  dst, AV_VIDEO_ENC_PARAMS_H264, nb_blocks);
        if (!par)
            return;

        frame_qp = fc->tab.qp[LUMA][0];
        par->qp  = frame_qp;

        for (int ycb = 0; ycb < min_cb_h && block_idx < nb_blocks; ycb++) {
            for (int xcb = 0; xcb < min_cb_w && block_idx < nb_blocks; xcb++) {
                const int w = SAMPLE_CTB(fc->tab.cb_width[LUMA],  xcb, ycb);
                const int h = SAMPLE_CTB(fc->tab.cb_height[LUMA], xcb, ycb);
                if (w > 0 && h > 0 &&
                    (xcb * unit) % w == 0 && (ycb * unit) % h == 0) {
                    AVVideoBlockParams *b = av_video_enc_params_block(par, block_idx++);
                    const int q = fc->tab.qp[LUMA][xcb + ycb * min_cb_w];
                    b->src_x    = xcb * unit;
                    b->src_y    = ycb * unit;
                    b->w        = w;
                    b->h        = h;
                    b->delta_qp = q - frame_qp;
                }
            }
        }
    }
}

int ff_vvc_output_frame(VVCContext *s, VVCFrameContext *fc, AVFrame *out, const int no_output_of_prior_pics_flag, int flush)
{
    const VVCSPS *sps = fc->ps.sps;
    /* PlayerX: 导出 CU 划分时，当前 fc 正在解码的帧（fc->ref）此刻 CU 快照
     * 尚未捕获（frame_end 才定稿并挂 side_data）。若按显示序提前 bump 输出，
     * side_data 永远赶不上。故导出模式下只推迟这一帧的输出，直到 frame_end。
     * 对照 HEVC ff_hevc_output_frames 只跳 cur_frame 的 defer_export 机制。
     * flush 阶段无条件输出，避免残留帧卡住。 */
    const int defer_export =
        s->avctx && !flush &&
        (s->avctx->export_side_data & AV_CODEC_EXPORT_DATA_VIDEO_ENC_PARAMS);
    do {
        int nb_output = 0;
        int min_poc   = INT_MAX;
        int min_idx, ret;

        if (no_output_of_prior_pics_flag) {
            for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++) {
                VVCFrame *frame = &fc->DPB[i];
                if (!(frame->flags & VVC_FRAME_FLAG_BUMPING) && frame->poc != fc->ps.ph.poc &&
                        frame->sequence == s->seq_output) {
                    ff_vvc_unref_frame(fc, frame, VVC_FRAME_FLAG_OUTPUT);
                }
            }
        }

        for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++) {
            VVCFrame *frame = &fc->DPB[i];
            /* 只推迟当前 fc 正在解码、快照未捕获的那一帧 */
            if (defer_export && frame == fc->ref && frame->nb_cu_snap == 0)
                continue;
            if ((frame->flags & VVC_FRAME_FLAG_OUTPUT) &&
                frame->sequence == s->seq_output) {
                nb_output++;
                if (frame->poc < min_poc || nb_output == 1) {
                    min_poc = frame->poc;
                    min_idx = i;
                }
            }
        }

        /* wait for more frames before output */
        if (!flush && s->seq_output == s->seq_decode && sps &&
            nb_output <= sps->r->sps_dpb_params.dpb_max_num_reorder_pics[sps->r->sps_max_sublayers_minus1])
            return 0;

        if (nb_output) {
            VVCFrame *frame = &fc->DPB[min_idx];

            if (frame->flags & VVC_FRAME_FLAG_CORRUPT)
                frame->frame->flags |= AV_FRAME_FLAG_CORRUPT;

            /* PlayerX: 块编码信息已在 frame_end() 阶段导出到 frame->frame 的
             * side_data（那时 cu_snap 定稿、载体 AVFrame 稳定）。此处无需再次
             * 导出——下面的 av_frame_ref() 会把已挂好的 side_data 一并带进
             * 输出帧。旧的“输出时导出 + 跨 fc 按 POC 查找”方案已废弃。 */
            ret = av_frame_ref(out, frame->needs_fg ? frame->frame_grain : frame->frame);

            if (!ret && !(s->avctx->export_side_data & AV_CODEC_EXPORT_DATA_FILM_GRAIN))
                av_frame_remove_side_data(out, AV_FRAME_DATA_FILM_GRAIN_PARAMS);

            if (frame->flags & VVC_FRAME_FLAG_BUMPING)
                ff_vvc_unref_frame(fc, frame, VVC_FRAME_FLAG_OUTPUT | VVC_FRAME_FLAG_BUMPING);
            else
                ff_vvc_unref_frame(fc, frame, VVC_FRAME_FLAG_OUTPUT);

            if (ret < 0)
                return ret;

            av_log(s->avctx, AV_LOG_DEBUG,
                   "Output frame with POC %d.\n", frame->poc);
            return 1;
        }

        if (s->seq_output != s->seq_decode)
            s->seq_output = (s->seq_output + 1) & 0xff;
        else
            break;
    } while (1);
    return 0;
}

void ff_vvc_bump_frame(VVCContext *s, VVCFrameContext *fc)
{
    const VVCSPS *sps = fc->ps.sps;
    const int poc = fc->ps.ph.poc;
    int dpb = 0;
    int min_poc = INT_MAX;

    for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++) {
        VVCFrame *frame = &fc->DPB[i];
        if ((frame->flags) &&
            frame->sequence == s->seq_output &&
            frame->poc != poc) {
            dpb++;
        }
    }

    if (sps && dpb >= sps->r->sps_dpb_params.dpb_max_dec_pic_buffering_minus1[sps->r->sps_max_sublayers_minus1] + 1) {
        for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++) {
            VVCFrame *frame = &fc->DPB[i];
            if ((frame->flags) &&
                frame->sequence == s->seq_output &&
                frame->poc != poc) {
                if (frame->flags == VVC_FRAME_FLAG_OUTPUT && frame->poc < min_poc) {
                    min_poc = frame->poc;
                }
            }
        }

        for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++) {
            VVCFrame *frame = &fc->DPB[i];
            if (frame->flags & VVC_FRAME_FLAG_OUTPUT &&
                frame->sequence == s->seq_output &&
                frame->poc <= min_poc) {
                frame->flags |= VVC_FRAME_FLAG_BUMPING;
            }
        }

        dpb--;
    }
}

static VVCFrame *find_ref_idx(VVCContext *s, VVCFrameContext *fc, int poc, uint8_t use_msb)
{
    const unsigned mask = use_msb ? ~0 : fc->ps.sps->max_pic_order_cnt_lsb - 1;

    for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++) {
        VVCFrame *ref = &fc->DPB[i];

        // In the specification, the current frame is not considered part of
        // the DPB at this stage, therefore we treat it as if it's not there.
        if (ref == fc->ref)
            continue;

        if (ref->frame->buf[0] && ref->sequence == s->seq_decode) {
            if ((ref->poc & mask) == poc)
                return ref;
        }
    }
    return NULL;
}

static void mark_ref(VVCFrame *frame, int flag)
{
    frame->flags &= ~(VVC_FRAME_FLAG_LONG_REF | VVC_FRAME_FLAG_SHORT_REF);
    frame->flags |= flag;
}

static VVCFrame *generate_missing_ref(VVCContext *s, VVCFrameContext *fc, int poc)
{
    const VVCSPS *sps = fc->ps.sps;
    const VVCPPS *pps = fc->ps.pps;
    VVCFrame *frame;

    frame = alloc_frame(s, fc);
    if (!frame)
        return NULL;

    if (!s->avctx->hwaccel) {
        if (!sps->pixel_shift) {
            for (int i = 0; frame->frame->buf[i]; i++)
                memset(frame->frame->buf[i]->data, 1 << (sps->bit_depth - 1),
                       frame->frame->buf[i]->size);
        } else {
            for (int i = 0; frame->frame->data[i]; i++)
                for (int y = 0; y < (pps->height >> sps->vshift[i]); y++) {
                    uint8_t *dst = frame->frame->data[i] + y * frame->frame->linesize[i];
                    AV_WN16(dst, 1 << (sps->bit_depth - 1));
                    av_memcpy_backptr(dst + 2, 2, 2*(pps->width >> sps->hshift[i]) - 2);
                }
        }
    }

    frame->poc      = poc;
    frame->sequence = s->seq_decode;
    frame->flags    = VVC_FRAME_FLAG_CORRUPT;

    /* 缺失参考帧：无快照，传 NULL 跳过块信息导出 */
    ff_vvc_report_frame_finished(frame);

    return frame;
}

#define CHECK_MAX(d) (frame->ref_##d * frame->sps->r->sps_pic_##d##_max_in_luma_samples >= ref->ref_##d * (frame->pps->r->pps_pic_##d##_in_luma_samples - max))
#define CHECK_SAMPLES(d) (frame->pps->r->pps_pic_##d##_in_luma_samples == ref->pps->r->pps_pic_##d##_in_luma_samples)
static int check_candidate_ref(const VVCFrame *frame, const VVCRefPic *refp)
{
    const VVCFrame *ref = refp->ref;

    if (refp->is_scaled) {
        const int max = FFMAX(8, frame->sps->min_cb_size_y);
        return frame->ref_width * 2 >= ref->ref_width &&
            frame->ref_height * 2 >= ref->ref_height &&
            frame->ref_width <= ref->ref_width * 8 &&
            frame->ref_height <= ref->ref_height * 8 &&
            CHECK_MAX(width) && CHECK_MAX(height);
    }
    return CHECK_SAMPLES(width) && CHECK_SAMPLES(height);
}

#define RPR_SCALE(f) (((ref->f << 14) + (fc->ref->f >> 1)) / fc->ref->f)
/* add a reference with the given poc to the list and mark it as used in DPB */
static int add_candidate_ref(VVCContext *s, VVCFrameContext *fc, RefPicList *list,
                             int poc, int ref_flag, uint8_t use_msb)
{
    VVCRefPic *refp = &list->refs[list->nb_refs];
    VVCFrame *ref;

    if (use_msb && poc == fc->ref->poc || list->nb_refs >= VVC_MAX_REF_ENTRIES)
        return AVERROR_INVALIDDATA;

    ref = find_ref_idx(s, fc, poc, use_msb);

    if (!IS_CVSS(s)) {
        const bool ref_corrupt = !ref || (ref->flags & VVC_FRAME_FLAG_CORRUPT);
        const bool recovering  = s->no_output_before_recovery_flag &&
            (IS_RASL(s) || !GDR_IS_RECOVERED(s));

        if (ref_corrupt && !recovering) {
            if (!(s->avctx->flags & AV_CODEC_FLAG_OUTPUT_CORRUPT) &&
                !(s->avctx->flags2 & AV_CODEC_FLAG2_SHOW_ALL))
                return AVERROR_INVALIDDATA;

            fc->ref->flags |= VVC_FRAME_FLAG_CORRUPT;
        }
    }

    if (!ref) {
        ref = generate_missing_ref(s, fc, poc);
        if (!ref)
            return AVERROR(ENOMEM);
    }

    refp->poc = poc;
    refp->ref = ref;
    refp->is_lt = ref_flag & VVC_FRAME_FLAG_LONG_REF;
    refp->is_scaled = ref->sps->r->sps_num_subpics_minus1 != fc->ref->sps->r->sps_num_subpics_minus1||
        memcmp(&ref->scaling_win, &fc->ref->scaling_win, sizeof(ref->scaling_win)) ||
        ref->pps->r->pps_pic_width_in_luma_samples != fc->ref->pps->r->pps_pic_width_in_luma_samples ||
        ref->pps->r->pps_pic_height_in_luma_samples != fc->ref->pps->r->pps_pic_height_in_luma_samples;

    if (!check_candidate_ref(fc->ref, refp))
        return AVERROR_INVALIDDATA;

    if (refp->is_scaled) {
        refp->scale[0] = RPR_SCALE(ref_width);
        refp->scale[1] = RPR_SCALE(ref_height);
    }
    list->nb_refs++;

    mark_ref(ref, ref_flag);
    return 0;
}

static int init_slice_rpl(const VVCFrameContext *fc, SliceContext *sc)
{
    VVCFrame *frame = fc->ref;
    const VVCSH *sh = &sc->sh;

    if (sc->slice_idx >= frame->nb_rpl_elems)
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < sh->num_ctus_in_curr_slice; i++) {
        const int rs = sh->ctb_addr_in_curr_slice[i];
        frame->rpl_tab[rs] = frame->rpl + sc->slice_idx;
    }

    sc->rpl = frame->rpl_tab[sh->ctb_addr_in_curr_slice[0]]->refPicList;

    return 0;
}

static int delta_poc_st(const H266RefPicListStruct *rpls,
    const int lx, const int i, const VVCSPS *sps)
{
    int abs_delta_poc_st = rpls->abs_delta_poc_st[i];
    if (!((sps->r->sps_weighted_pred_flag ||
        sps->r->sps_weighted_bipred_flag) && i != 0))
        abs_delta_poc_st++;
    return (1 - 2 * rpls->strp_entry_sign_flag[i]) * abs_delta_poc_st;
}

static int poc_lt(int *prev_delta_poc_msb, const int poc, const H266RefPicLists *ref_lists,
    const int lx, const int j, const int max_poc_lsb)
{
    const H266RefPicListStruct *rpls = ref_lists->rpl_ref_list + lx;
    int lt_poc =  rpls->ltrp_in_header_flag ? ref_lists->poc_lsb_lt[lx][j] : rpls->rpls_poc_lsb_lt[j];

    if (ref_lists->delta_poc_msb_cycle_present_flag[lx][j]) {
        const uint32_t delta = ref_lists->delta_poc_msb_cycle_lt[lx][j] + *prev_delta_poc_msb;
        lt_poc += poc - delta * max_poc_lsb - (poc & (max_poc_lsb - 1));
        *prev_delta_poc_msb = delta;
    }
    return lt_poc;
}

int ff_vvc_slice_rpl(VVCContext *s, VVCFrameContext *fc, SliceContext *sc)
{
    const VVCSPS *sps                = fc->ps.sps;
    const H266RawPPS *pps            = fc->ps.pps->r;
    const VVCPH *ph                  = &fc->ps.ph;
    const H266RawSliceHeader *rsh    = sc->sh.r;
    const int max_poc_lsb            = sps->max_pic_order_cnt_lsb;
    const H266RefPicLists *ref_lists =
        pps->pps_rpl_info_in_ph_flag ?  &ph->r->ph_ref_pic_lists : &rsh->sh_ref_pic_lists;
    int ret = 0;

    ret = init_slice_rpl(fc, sc);
    if (ret < 0)
        return ret;

    for (int lx = L0; lx <= L1; lx++) {
        const H266RefPicListStruct *rpls = ref_lists->rpl_ref_list + lx;
        RefPicList *rpl = sc->rpl + lx;
        int poc_base = ph->poc;
        int prev_delta_poc_msb = 0;

        rpl->nb_refs = 0;
        for (int i = 0, j = 0; i < rpls->num_ref_entries; i++) {
            int poc;
            if (!rpls->inter_layer_ref_pic_flag[i]) {
                int use_msb = 1;
                int ref_flag;
                if (rpls->st_ref_pic_flag[i]) {
                    poc = poc_base + delta_poc_st(rpls, lx, i, sps);
                    poc_base = poc;
                    ref_flag = VVC_FRAME_FLAG_SHORT_REF;
                } else {
                    use_msb = ref_lists->delta_poc_msb_cycle_present_flag[lx][j];
                    poc = poc_lt(&prev_delta_poc_msb, ph->poc, ref_lists, lx, j, max_poc_lsb);
                    ref_flag = VVC_FRAME_FLAG_LONG_REF;
                    j++;
                }
                ret = add_candidate_ref(s, fc, rpl, poc, ref_flag, use_msb);
                if (ret < 0)
                    return ret;
            } else {
                // OPI_B_3.bit and VPS_A_3.bit should cover this
                avpriv_report_missing_feature(fc->log_ctx, "Inter layer ref");
                ret = AVERROR_PATCHWELCOME;
                return ret;
            }
        }
        if (ph->r->ph_temporal_mvp_enabled_flag &&
            (!rsh->sh_collocated_from_l0_flag) == lx &&
            rsh->sh_collocated_ref_idx < rpl->nb_refs) {
            const VVCRefPic *refp = rpl->refs + rsh->sh_collocated_ref_idx;
            if (refp->is_scaled || refp->ref->sps->ctb_log2_size_y != sps->ctb_log2_size_y)
                return AVERROR_INVALIDDATA;
            fc->ref->collocated_ref = refp->ref;
        }
    }
    return 0;
}

int ff_vvc_frame_rpl(VVCContext *s, VVCFrameContext *fc, SliceContext *sc)
{
    int ret = 0;

    /* clear the reference flags on all frames except the current one */
    for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++) {
        VVCFrame *frame = &fc->DPB[i];

        if (frame == fc->ref)
            continue;

        mark_ref(frame, 0);
    }

    if ((ret = ff_vvc_slice_rpl(s, fc, sc)) < 0)
        goto fail;

fail:
    /* release any frames that are now unused */
    for (int i = 0; i < FF_ARRAY_ELEMS(fc->DPB); i++)
        ff_vvc_unref_frame(fc, &fc->DPB[i], 0);
    return ret;
}

void ff_vvc_report_frame_finished(VVCFrame *frame)
{
    ff_vvc_report_progress(frame, VVC_PROGRESS_MV, INT_MAX);
    ff_vvc_report_progress(frame, VVC_PROGRESS_PIXEL, INT_MAX);
}

static int is_progress_done(const FrameProgress *p, const VVCProgressListener *l)
{
    return p->progress[l->vp] > l->y;
}

static void add_listener(VVCProgressListener **prev, VVCProgressListener *l)
{
    l->next = *prev;
    *prev   = l;
}

static VVCProgressListener* remove_listener(VVCProgressListener **prev, VVCProgressListener *l)
{
    *prev  = l->next;
    l->next = NULL;
    return l;
}

static VVCProgressListener* get_done_listener(FrameProgress *p, const VVCProgress vp)
{
    VVCProgressListener *list = NULL;
    VVCProgressListener **prev = &p->listener[vp];

    while (*prev) {
        if (is_progress_done(p, *prev)) {
            VVCProgressListener *l = remove_listener(prev, *prev);
            add_listener(&list, l);
        } else {
            prev = &(*prev)->next;
        }
    }
    return list;
}

void ff_vvc_report_progress(VVCFrame *frame, const VVCProgress vp, const int y)
{
    FrameProgress *p = frame->progress;
    VVCProgressListener *l = NULL;

    ff_mutex_lock(&p->lock);
    if (p->progress[vp] < y) {
        // Due to the nature of thread scheduling, later progress may reach this point before earlier progress.
        // Therefore, we only update the progress when p->progress[vp] < y.
        p->progress[vp] = y;
        l = get_done_listener(p, vp);
        ff_cond_signal(&p->cond);
    }
    ff_mutex_unlock(&p->lock);

    while (l) {
        l->progress_done(l);
        l = l->next;
    }
}

void ff_vvc_add_progress_listener(VVCFrame *frame, VVCProgressListener *l)
{
    FrameProgress *p = frame->progress;

    ff_mutex_lock(&p->lock);

    if (is_progress_done(p, l)) {
        ff_mutex_unlock(&p->lock);
        l->progress_done(l);
    } else {
        add_listener(p->listener + l->vp, l);
        ff_mutex_unlock(&p->lock);
    }
}
