/* -*-c++-*- SemiGlobalMatching - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/SemiGlobalMatching
* Describe	: header of sgm_types
*/

#pragma once

#include <cstdint>
#include <limits>

/** \brief float无效值 */

namespace censusSGM
{
	constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();
}


/** \brief 基础类型别名 */
typedef int8_t			sint8;		// 有符号8位整数
typedef uint8_t			uint8;		// 无符号8位整数
typedef int16_t			sint16;		// 有符号16位整数
typedef uint16_t		uint16;		// 无符号16位整数
typedef int32_t			sint32;		// 有符号32位整数
typedef uint32_t		uint32;		// 无符号32位整数
typedef int64_t			sint64;		// 有符号64位整数
typedef uint64_t		uint64;		// 无符号64位整数
typedef float			float32;	// 单精度浮点
typedef double			float64;	// 双精度浮点

/** \brief Census窗口尺寸类型 */
enum CensusSize {
	Census5x5 = 0,
	Census9x7
};

/** \brief SGM参数结构体 */
struct SGMOption {
	uint8	num_paths;			// 聚合路径数 4 and 8
	sint32  min_disparity;		// 最小视差
	sint32	max_disparity;		// 最大视差

	CensusSize census_size;		// census窗口尺寸

	bool	is_check_unique;	// 是否检查唯一性
	float32	uniqueness_ratio;	// 唯一性约束阈值 （最小代价-次最小代价)/最小代价 > 阈值 为有效像素

	bool	is_check_lr;		// 是否检查左右一致性
	float32	lrcheck_thres;		// 左右一致性约束阈值

	bool	is_remove_speckles;	// 是否移除小的连通区
	int		min_speckle_aera;	// 最小的连通区面积（像素数）

	bool	is_fill_holes;		// 是否填充视差空洞

	// P1,P2 
	// P2 = P2_init / (Ip-Iq)
	sint32  p1;				// 惩罚项参数P1
	sint32  p2_init;		// 惩罚项参数P2

	SGMOption() : num_paths(8), min_disparity(0), max_disparity(64), census_size(Census5x5),
		is_check_unique(true), uniqueness_ratio(0.95f),
		is_check_lr(true), lrcheck_thres(1.0f),
		is_remove_speckles(true), min_speckle_aera(20),
		is_fill_holes(true),
		p1(10), p2_init(150) { }
};
