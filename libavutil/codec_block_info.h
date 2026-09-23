/*
 * PlayerX 定制：逐编码单元（CU）真实编码信息
 *
 * 背景：AVVideoBlockParams（见 libavutil/video_enc_params.h）只有
 *   src_x / src_y / w / h / delta_qp 五个字段，
 * 装不下预测模式、运动矢量、参考索引等码流分析必需的信息。
 * 因此另开一条 side data：AV_FRAME_DATA_CODEC_BLOCK_INFO，
 * 数据即本结构体的紧凑定长数组，与 enc_params 的块一一对应、同序。
 *
 * 元素个数 = AVFrameSideData.size / sizeof(AVCodecBlockInfo)。
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef AVUTIL_CODEC_BLOCK_INFO_H
#define AVUTIL_CODEC_BLOCK_INFO_H

#include <stdint.h>

/**
 * 预测模式，与 VVC 内部 enum PredMode 对齐。
 * 上层据此区分帧内 / 帧间 / Skip / IBC。
 */
typedef enum AVCodecBlockPredMode {
    AV_CB_PRED_INTER = 0,   ///< 帧间预测
    AV_CB_PRED_INTRA = 1,   ///< 帧内预测
    AV_CB_PRED_SKIP  = 2,   ///< skip（merge，无残差）
    AV_CB_PRED_PLT   = 3,   ///< palette
    AV_CB_PRED_IBC   = 4,   ///< 帧内块复制
} AVCodecBlockPredMode;

/**
 * 参考方向位标志，与 VVC 内部 PredFlag 对齐。
 */
typedef enum AVCodecBlockPredFlag {
    AV_CB_PF_INTRA = 0x0,
    AV_CB_PF_L0    = 0x1,   ///< 使用前向参考 L0
    AV_CB_PF_L1    = 0x2,   ///< 使用后向参考 L1
    AV_CB_PF_BI    = 0x3,   ///< 双向 L0+L1
    AV_CB_PF_IBC   = 0x5,   ///< IBC（PF_L0 | 0x4）
} AVCodecBlockPredFlag;

/**
 * 单个编码单元的完整编码信息（24 字节，紧凑定长，便于批量拷贝）。
 */
typedef struct AVCodecBlockInfo {
    int16_t x;              ///< CU 左上角 x（亮度像素坐标）
    int16_t y;              ///< CU 左上角 y
    int16_t w;              ///< CU 宽度
    int16_t h;              ///< CU 高度

    int8_t  qp;             ///< 该 CU 的最终亮度 QP（Qp''Y，已含 slice_qp + delta）
    int8_t  pred_mode;      ///< AVCodecBlockPredMode
    int8_t  ref_idx[2];     ///< refIdxL0 / refIdxL1，未使用时为 -1
    uint8_t pred_flag;      ///< AVCodecBlockPredFlag
    uint8_t skip_flag;      ///< cu_skip_flag：1 表示 skip（无残差）
    int8_t  qt_depth;       ///< cqt_depth（四叉树深度）
    int8_t  tree_type;      ///< 0=SINGLE_TREE 1=DUAL_TREE_LUMA 2=DUAL_TREE_CHROMA

    int16_t mv[2][2];       ///< mv[LX][0=x,1=y]，LX: 0=L0, 1=L1；单位 1/16 像素
} AVCodecBlockInfo;

#endif /* AVUTIL_CODEC_BLOCK_INFO_H */
