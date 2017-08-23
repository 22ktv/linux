// SPDX-License-Identifier: GPL-2.0+
/* Copyright (C) 2015-2018 Broadcom */

#if !defined(_VC5_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _VC5_TRACE_H_

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM vc5
#define TRACE_INCLUDE_FILE vc5_trace

TRACE_EVENT(vc5_wait_for_seqno_begin,
	    TP_PROTO(struct drm_device *dev, uint64_t seqno, uint64_t timeout),
	    TP_ARGS(dev, seqno, timeout),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, seqno)
			     __field(u64, timeout)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->seqno = seqno;
			   __entry->timeout = timeout;
			   ),

	    TP_printk("dev=%u, seqno=%llu, timeout=%llu",
		      __entry->dev, __entry->seqno, __entry->timeout)
);

TRACE_EVENT(vc5_wait_for_seqno_end,
	    TP_PROTO(struct drm_device *dev, uint64_t seqno),
	    TP_ARGS(dev, seqno),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, seqno)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->seqno = seqno;
			   ),

	    TP_printk("dev=%u, seqno=%llu",
		      __entry->dev, __entry->seqno)
);

TRACE_EVENT(vc5_hangcheck,
	    TP_PROTO(struct drm_device *dev,
		     uint64_t bin_seqno, uint64_t render_seqno,
		     uint32_t ct0ca, uint32_t ct1ca,
		     uint32_t last_ct0ca, uint32_t last_ct1ca),
	    TP_ARGS(dev,
		    bin_seqno, render_seqno,
		    ct0ca, ct1ca,
		    last_ct0ca, last_ct1ca),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, bin_seqno)
			     __field(u64, render_seqno)
			     __field(u32, ct0ca)
			     __field(u32, ct1ca)
			     __field(u32, last_ct0ca)
			     __field(u32, last_ct1ca)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->bin_seqno = bin_seqno;
			   __entry->render_seqno = render_seqno;
			   __entry->ct0ca = ct0ca;
			   __entry->ct1ca = ct1ca;
			   __entry->last_ct0ca = last_ct0ca;
			   __entry->last_ct1ca = last_ct1ca;
			   ),

	    TP_printk("dev=%u, bin_seqno=%llu, render_seqno=%llu, "
		      "ct0ca=0x%08x vs last 0x%08x, "
		      "ct1ca=0x%08x vs last 0x%08x",
		      __entry->dev, __entry->bin_seqno, __entry->render_seqno,
		      __entry->ct0ca, __entry->last_ct0ca,
		      __entry->ct1ca, __entry->last_ct1ca)
);

TRACE_EVENT(vc5_submit_cl,
	    TP_PROTO(struct drm_device *dev, bool is_render,
		     uint64_t seqno,
		     uint32_t ctnqba, uint32_t ctnqea),
	    TP_ARGS(dev, is_render, seqno, ctnqba, ctnqea),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(bool, is_render)
			     __field(u64, seqno)
			     __field(u32, ctnqba)
			     __field(u32, ctnqea)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->is_render = is_render;
			   __entry->seqno = seqno;
			   __entry->ctnqba = ctnqba;
			   __entry->ctnqea = ctnqea;
			   ),

	    TP_printk("dev=%u, %s, seqno=%llu, 0x%08x..0x%08x",
		      __entry->dev,
		      __entry->is_render ? "RCL" : "BCL",
		      __entry->seqno,
		      __entry->ctnqba,
		      __entry->ctnqea)
);

TRACE_EVENT(vc5_finish_bin_job,
	    TP_PROTO(struct drm_device *dev, uint64_t seqno),
	    TP_ARGS(dev, seqno),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, seqno)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->seqno = seqno;
			   ),

	    TP_printk("dev=%u, seqno=%llu",
		      __entry->dev,
		      __entry->seqno)
);

TRACE_EVENT(vc5_cancel_bin_job,
	    TP_PROTO(struct drm_device *dev, uint64_t seqno),
	    TP_ARGS(dev, seqno),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, seqno)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->seqno = seqno;
			   ),

	    TP_printk("dev=%u, seqno=%llu",
		      __entry->dev,
		      __entry->seqno)
);

TRACE_EVENT(vc5_finish_render_job,
	    TP_PROTO(struct drm_device *dev, uint64_t seqno),
	    TP_ARGS(dev, seqno),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, seqno)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->seqno = seqno;
			   ),

	    TP_printk("dev=%u, seqno=%llu",
		      __entry->dev,
		      __entry->seqno)
);

TRACE_EVENT(vc5_reset_begin,
	    TP_PROTO(struct drm_device *dev),
	    TP_ARGS(dev),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   ),

	    TP_printk("dev=%u",
		      __entry->dev)
);

TRACE_EVENT(vc5_reset_end,
	    TP_PROTO(struct drm_device *dev),
	    TP_ARGS(dev),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   ),

	    TP_printk("dev=%u",
		      __entry->dev)
);

#endif /* _VC5_TRACE_H_ */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#include <trace/define_trace.h>
