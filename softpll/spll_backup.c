/*
 * This work is part of the White Rabbit project
 *
 * Copyright (C) 2010 - 2014 CERN (www.cern.ch)
 * Author: Maciej Lipinski <maciej.lipinski@cern.ch> 
 *        Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
 *
 * Released according to the GNU GPL, version 2 or any later version.
 */

/* spll_backup.c - Implementation of the backup channel for DDMTD PLL. 
 * 
 * ------------------------------------------------------------------------------------------
 * Intro: my understanding of SoftPLL and its components
 * ------------------------------------------------------------------------------------------
 * The DDMTD clock (125+MHz or 62.5+MHz) is used to tag:
 * 1) feedback clock - the one that we control, which is local and which we use to encode
 *                     data on all the port
 * 2) active ref clk - the clock that is used as the reference for the feedback
 * 3) backup ref clks- the clocks that are backup
 * 
 * three usages of the tags:
 * 1) helper PLL uses consecutive tags of the feedback clock to check whether it's frequency
 *    is "perfect", i.e. subtraction of consecutive tags should give perfect period
 *    (of the offset frequency, the one that results from mixing DDMTD clock with the
 *    other clock)
 * 2) main PLL uses tags for two things:
 *    - control of frequency by checking whether the time advances in both clocks at the same
 *      pace, in other words the rising edge of both clocks, relative to the DDMTD counter
 *      (i.e. tag value), is tried to be maintained at the same value. In this way, the
 *      feedback clock follows the ref clock. 
 *    - once the feedback and ref are syntonized, the phase is adjusted:
 *      -> the adjustment is kept in the adder_ref incremented by 1 each SPLL updated,
 *         the adder does not hold the value of the tags, just the difference in phase and
 *         it is used to handle the overflow of tags
 *      -> the phase is adjusted by manipulating the frequency (DAC->VXCO), if we change the
 *         frequency, the clocks will advance differently and a phase difference will 
 *         be created. This phase difference is maintained thanks to the adder
 * 3) phase tracker uses the tags to measure the real phase offset 
 *    - it can measure phase offsets between the feedback clock and any other clock (rx ref, 
 *      aux)
 *    - in WRPTP synchronization the phase measurement between feedback and rx ref is used
 *    - in nodes, the phase measurement between feedback and oux channels is used
 *    - in the backup switchover, the phase measurement between feedback and backup rx clock(s)
 *      is used
 *    - the phase measurement reflects averaged phase offset, the average is over n_avg
 *      previous samples every n_avg samples (no updates in between, it's not a moving
 *      average window, thus delayed updates of the value phase_val are expected)
 *
 *-------------------------------------------------------------------------------------------
 * Changes to the WR code-base (where/what)
 *-------------------------------------------------------------------------------------------
 *
 * The changes spam: PPSi, wrsw_hal and wrpc/softpll
 * PPSi (ppsi git repo, branch: ml-140906-switchover):
 * - added new port config: backup
 * - made the per-port priority atribute work - it is set to "1" if a port is "backup"
 * - modifed BMC to use prio/backup information
 * - enabled servo per port (servo-related info is now in a table)
 * - enabled currentDS per port (currentDS info is now in a table)
 * - modifed wr-servo.c to enable handling backup (many) servo, i.e. it uses prio to tell
 *   active from backup and uses the local static structure only for the active
 *   TODO: this is hackish - probably needs better way (this was already a mess...see comments)
 * - modified msg.c : this is a hack to prevent PTP messages received on 
 *   the backup port from being discarded, again, prio value used to tell active from backup
 *   TODO: good question how to make it nice...need some protocol hack (it seems)
 * - enabled adjust_phase() per port (was global) in wrs-time.c and servo.c
 * - added quite some debug
 * - TODO: change some hard-coded-size tables (currentDS, servos) to use global defines
 *         (candidate: PP_MAX_LINKS)
 * 
 * wrsw_hal (wr-switch-sw git repo, branch: ml-140906-switchover):
 * - enabled to pass priority value from PPSi to SoftPLL, when locking
 * - interpre the priority value to avoid reseting/etc SoftPLL when locking and link down
 * - remember and recognize backup channel/port number to use dedicated softPLL functions
 *   where needed
 * - added IPC call to SoftPLL: rts_backup_channel()
 * - enabled adjust_phase per port
 * - extended wr_phytool to enable setpoint adjustment
 * - added some debugs
 *
 * wrpc/softpll
 * - added tones of debugs which are nasty but help to get an idea what is happening
 * - added IPC communication with wrsw_hal to handle backup
 * - added bakcup pll to handle backup port
 * - added functions to handle IPC calls to init/start/stop backp port
 * - added functions to switchover
 * 
 * 
 *-------------------------------------------------------------------------------------------
 * Backup switchover
 *-------------------------------------------------------------------------------------------
 * The switch over needs to mess up with the following parts of SoftPLL
 * 1) helper PLL - the source of the DDMTD frequency needs to be changed
 * 2) "main PLL" - what was done: a "backup PLL" was added which is a stripped down version
 *                 of the "main PLL". the aim of the backup PLL is to "fake" PLL functionalities
 *                 for the WRPTP, that includes: lock and timestamps adjustment. This enables
 *                 to obtain "operational data" that would be true if the backup PLL was actualy
 *                 working as the main PLL. The operational data includes 
 *                 1) in softPLL: setpoint and phase value/offset measurement in the SoftPLL 
 *                 2) in WRPTP  : link delay and offset from master
 *                 The later (2) is possible since we run WRPTP exchange and "fake" WRPTP
 *                 synchronization on the backup port.
 *                 The fake operational data is needed to be able to take over the role of
 *                 the main PLL "at the full speed" (it's like changing a car driver while
 *                 driving 100km/h on the highway.
 * 
 * The above are described in details below
 * ------------------------------------------------------------------------------------------
 * Helper PLL switchover
 * ------------------------------------------------------------------------------------------
 * At the moment a "slow" switchover of helper PLL is implemented, it might not be sufficient.
 * How it works:
 * - the PI controller of the PLL works based on tags provided, i.e. it is tag-driven, updated
 *   each time new tag is received from HDL via FIFO+irq
 * - when the link goes down, there is no tag updates and the last DAC control word is 
 *   maintained. This is some kind of simple holdover. The last control world might be 
 *   somehow corrupted since the link never goes down instantly. So here some simple average
 *   might be required if the performance is not good enough
 * - the link down is detected in the wrsw_hal which polls link state and manages all ports
 * - when wrsw_hal detects link down the link set to be backup, it commands SoftPLL to 
 *   switchover:
 *   wrsw_hal/hal_ports.c:handle_link_down()->rts_backup_channel(p->hw_index, RTS_BACKUP_CH_ACTIVATE);
 * - the switchover of helper PLL is done by (softpll/spll_helper.c:helper_switch_reference()) 
 *   1) switching off the tagger on the active rx clk, 
 *   2) "clearing the current tag-based measurement (setting p_adder=0, tag_d0-1 forces that)
 *   3) switching on the tagger on the port defined to be backup (active from now on)..
 *      TODO: hmm, this seems not necessary
 *   4) changing the ref_src value 
 * - since the (previously) active rx ref clock and the (previously) backup rx ref clock are
 *   (supposed to be) the same and the frequency should not drift too much during the process,
 *   this should work.
 * - TODO: if it does not, two things can be done:
 *   1) implement primitive holdover or outlier elimination to discard the wrong tag while 
 *      disconnecting cable -> some kind of intelligence will need to be added here later
 *      probably, since the cable disconnection is a very theoretical failure use case
 *   2) provide information about active rx ref clock failure directly from HDL and activate
 *      the function based on that info (i.e. irq)
 *
 * ------------------------------------------------------------------------------------------
 * Backup PLL 
 * ------------------------------------------------------------------------------------------
 * It provides the facility to measure/track the phase shift between the feedback clock and 
 * the backup rx ref clock. Simirarily as main PLL, it allows to calculate the error between
 * the two clock, taking into account the setpoint, e.g.:
 * 
 * setpoint ~= phase measurement +/-jitter (due to frequency error)
 * 
 * it is represented by a special backup PLL structure: 
 * struct spll_backup_state bpll
 * which is derived from the main PLL. both sit in the softpll_state "global" structue
 * TODO: to enable more backup ports, bpll must be a table
 * 
 * In wrpc/softpll/softpll_ng.c I added added a bunch of spll_*_backup_* functions which 
 * "mirror" the spll_* functions but refer (update/read) to softpll->bpll rather than 
 * softpll->mpll
 * TODO: probably needs more beautiful solution later
 * 
 * -
 *-------------------------------------------------------------------------------------------
 */

#include "spll_backup.h"
#include "spll_debug.h"
#include <pp-printf.h>
#include "trace.h"

#define MPLL_TAG_WRAPAROUND 100000000

#define MATCH_NEXT_TAG 0
#define MATCH_WAIT_REF 1
#define MATCH_WAIT_OUT 2

#undef WITH_SEQUENCING

/* initialization of pll "configuration" (unlike runtime data as in bpll_start())
 * copied from mpll except, just missing:
 * - the initialization of PI controller - no need, we don't control anything
 * - lock checkup - no need, we are not really locked 
 *   TODO: later, we might want to implement something like ld() but checking whether the
 *         active channel is ok with respect to backup(s), even voting logic (brrr) 
 */
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
	s->dac_index = id_out - spll_n_chan_ref; //TODO:probably not needed, T
	TRACE_DEV("[bpll] ref %d out %d idx %x", s->id_ref, s->id_out, s->dac_index);
}

/*
 * this is to start bpll, it is alsomainly copy of mpll except:
 * - enabling of tagging on the feedback channel (id_out) as it is already in place 
 * - initializing PI/LD
 */
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
}

void bpll_stop(struct spll_backup_state *s)
{
	spll_enable_tagger(s->id_ref, 0);
	s->enabled = 0;
}

/*
 * the main bulk of work is here. it is again taken from mpll, except:
 * - running the PI controller and then triving DAC
 * - verying whether we are locked on this channel - we don't check whether we are locked 
 *   on backup because:
 *   * in theory we do not need
 *   * in practice, at the beginning, the error (in the current state) is huge and it 
 *     indicates unlocked while we are really locked
 *   TODO: later, this function (ld_update) could be actually used to check whether the
 *         two cloks (active and backup(s)) do not wander with respect to each other
 * 
 * some additional magic is considered here (see the code below)
 * 
 */
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
		/*
		 * In theory, the phase of the feedback clock (when synchronized/syntonized
		 * with the active rx clk clock) is in the very right place. This means that
		 * the phase measurement shows what the setpoint should be, and the same
		 * applies to the error: it shows the "intended" setpoint. it should be in
		 * fact close to zero since we are perfectly synchonized with the second
		 * port, So, when first called, the phase_shift (so the adder_ref) is set
		 * with the value of the error
		 * TODO: question is whether the first value measured is somehow correct
		 * 
		 */
		if(err!=0 && s->err_d == 0 && s->phase_shift_current == 0 && s->adder_ref == 0)
		{
		    s->phase_shift_target = -err;
		    s->phase_shift_current= -err;
		    s->adder_ref          = -err;
		    TRACE_DEV("[bpll] initial set of setpoint %d\n", s->phase_shift_target );
		}
		/*
		 * THe idea is that the change in the error should affect the change  of the 
		 * phase shift (so setpoint), since the setpoint compensate the phase shift.
		 * Meybe, this should be done by the WRPTP...
		 * TODO: verify whether this is the way to go, I think not
		 */
		else if(err!=0)
		{
		    s->phase_shift_target =- (err-s->err_d);	    
		}
		
		
		s->err_d = err;
		s->tag_out = -1;
		s->tag_ref = -1;

		if (s->adder_ref > 2 * MPLL_TAG_WRAPAROUND
		    && s->adder_out > 2 * MPLL_TAG_WRAPAROUND) {
			s->adder_ref -= MPLL_TAG_WRAPAROUND;
			s->adder_out -= MPLL_TAG_WRAPAROUND;
		}
		
// 		if (s->ld.locked) { // we ignore it, it initially shows unlocked due to the error
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

/*
 * all the functions below are copied from mpll, 
 * TODO: put it to spll_common.c ? or some shared place 
 */
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
