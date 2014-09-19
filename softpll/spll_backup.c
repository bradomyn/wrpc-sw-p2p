/*
 * This work is part of the White Rabbit project
 *
 * Copyright (C) 2010 - 2014 CERN (www.cern.ch)
 * Author: Maciej Lipinski <maciej.lipinski@cern.ch> 
 *        Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
 *
 * Released according to the GNU GPL, version 2 or any later version.
 */

/* spll_backup.c - Implementation of the backup channel for DDMTD PLL. */

#include "spll_backup.h"
#include "spll_debug.h"
#include <pp-printf.h>
#include "trace.h"

#define MPLL_TAG_WRAPAROUND 100000000

#define MATCH_NEXT_TAG 0
#define MATCH_WAIT_REF 1
#define MATCH_WAIT_OUT 2

#undef WITH_SEQUENCING

void bpll_init(struct spll_backup_state *s, int id_ref, int id_out)
{
	s->delock_count = 0;
	s->enabled = 0;

	/* Freqency branch lock detection */
	s->ld.threshold = 1200;
	s->ld.lock_samples = 1000;
	s->ld.delock_samples = 100;
	s->id_ref = id_ref;
	s->id_out = id_out;
	s->dac_index = id_out - spll_n_chan_ref;

	TRACE_DEV("[bpll] ref %d out %d idx %x", s->id_ref, s->id_out, s->dac_index);
}

void bpll_start(struct spll_backup_state *s)
{
	TRACE_DEV("[bpll] Start backup channel %d\n", s->id_ref);

	s->adder_ref = s->adder_out = 0;
	s->tag_ref = -1;
	s->tag_out = -1;
	s->tag_ref_d = -1;
	s->tag_out_d = -1;
	s->seq_ref = 0;
	s->seq_out = 0;
	s->err_d = 0;
	s->match_state = MATCH_NEXT_TAG;

	s->phase_shift_target = 0;
	s->phase_shift_current = 0;
	s->sample_n = 0;
	s->enabled = 1;

	spll_enable_tagger(s->id_ref, 1);
// 	spll_enable_tagger(s->id_out, 1);
}

void bpll_stop(struct spll_backup_state *s)
{
	spll_enable_tagger(s->id_ref, 0);
	s->enabled = 0;
}

int bpll_update(struct spll_backup_state *s, int tag, int source)
{
	if(!s->enabled)
	    return SPLL_LOCKED;

	int err = 0;

	if (source == s->id_ref)
		s->tag_ref = tag;

	if (source == s->id_out)
		s->tag_out = tag;

	if (s->tag_ref >= 0) {
		if(s->tag_ref_d >= 0 && s->tag_ref_d > s->tag_ref)
			s->adder_ref += (1 << TAG_BITS);

		s->tag_ref_d = s->tag_ref;
	}


	if (s->tag_out >= 0) {
		if(s->tag_out_d >= 0 && s->tag_out_d > s->tag_out)
			s->adder_out += (1 << TAG_BITS);

		s->tag_out_d = s->tag_out;
	}

	if (s->tag_ref >= 0 && s->tag_out >= 0) {
		err = s->adder_ref + s->tag_ref - s->adder_out - s->tag_out;

#ifndef WITH_SEQUENCING

		/* Hack: the PLL is locked, so the tags are close to
		   each other. But when we start phase shifting, after
		   reaching full clock period, one of the reference
		   tags will flip before the other, causing a suddent
		   2**HPLL_N jump in the error.  So, once the PLL is
		   locked, we just mask out everything above
		   2**HPLL_N.

		   Proper solution: tag sequence numbers */
		if (s->ld.locked) {
			err &= (1 << HPLL_N) - 1;
			if (err & (1 << (HPLL_N - 1)))
				err |= ~((1 << HPLL_N) - 1);
		}

#endif
		if(err!=0 && s->err_d == 0 && s->phase_shift_current == 0 && s->adder_ref == 0)
		{
		    s->phase_shift_target = -err;
		    s->phase_shift_current= -err;
		    s->adder_ref          = -err;
		    TRACE_DEV("[bpll] initial set of setpoint %d\n", s->phase_shift_target );
		}
		else if(err!=0)
		{
// 		    TRACE_DEV("[bpll] adjust setpoint %d by %d\n", s->phase_shift_target,(err-s->err_d) );
		    s->phase_shift_target =- (err-s->err_d);
// 		    s->phase_shift_current=- (err-s->err_d);
// 		    s->adder_ref          =- (err-s->err_d);
		    
		}
		
		
		s->err_d = err;
		s->tag_out = -1;
		s->tag_ref = -1;

		if (s->adder_ref > 2 * MPLL_TAG_WRAPAROUND
		    && s->adder_out > 2 * MPLL_TAG_WRAPAROUND) {
			s->adder_ref -= MPLL_TAG_WRAPAROUND;
			s->adder_out -= MPLL_TAG_WRAPAROUND;
		}
		
// 		if (s->ld.locked) {
			if (s->phase_shift_current < s->phase_shift_target) {
				s->phase_shift_current++;
				s->adder_ref++;
			} else if (s->phase_shift_current >
				   s->phase_shift_target) {
				s->phase_shift_current--;
				s->adder_ref--;
			}
// 		}
// 		if (ld_update((spll_lock_det_t *)&s->ld, err))
			return SPLL_LOCKED;
	}

	return SPLL_LOCKING;
}

#ifdef CONFIG_PPSI /* use __div64_32 from ppsi library to save libgcc memory */
static int32_t from_picos(int32_t ps)
{
	extern uint32_t __div64_32(uint64_t *n, uint32_t base);
	uint64_t ups = ps;

	if (ps >= 0) {
		ups *= 1 << HPLL_N;
		__div64_32(&ups, CLOCK_PERIOD_PICOSECONDS);
		return ups;
	}
	ups = -ps * (1 << HPLL_N);
	__div64_32(&ups, CLOCK_PERIOD_PICOSECONDS);
	return -ups;
}
#else /* previous implementation: ptp-noposix has no __div64_32 available */
static int32_t from_picos(int32_t ps)
{
	return (int32_t) ((int64_t) ps * (int64_t) (1 << HPLL_N) /
			  (int64_t) CLOCK_PERIOD_PICOSECONDS);
}
#endif

int bpll_set_phase_shift(struct spll_backup_state *s, int desired_shift_ps)
{
	int div = (DIVIDE_DMTD_CLOCKS_BY_2 ? 2 : 1);
	s->phase_shift_target = from_picos(desired_shift_ps) / div;
	TRACE_DEV("[bpll] set target phaseshift %d\n", s->phase_shift_target);
	return 0;
}

int bpll_shifter_busy(struct spll_backup_state *s)
{
	return s->phase_shift_target != s->phase_shift_current;
}
