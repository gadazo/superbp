/* 046267 Computer Architecture - Spring 2016 - HW #2 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include "math.h"
#include "stdio.h"

// struct
typedef enum {
	SNT = 0, WNT, WT, ST
} state;

typedef struct {
	bool is_valid;
	uint32_t tag;
	uint32_t target;
	uint8_t* history;
	state* state_mach;
} btb_line;

//Global
int btb_size;
int history_size;
int tag_size;
bool is_global_hist;
bool is_global_table;
int shared_type;
btb_line* btb_table;
uint8_t global_history;
state* global_sm;
SIM_stats stats;

#define BIT16 65,536  //TODO: check if true!!!!!

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize,
		bool isGlobalHist, bool isGlobalTable, int Shared) {
	//intializing the global variables and the btb table
	btb_size = btbSize;
	history_size = historySize;
	tag_size = tagSize;
	is_global_hist = isGlobalHist;
	is_global_table = isGlobalTable;
	shared_type = Shared;

	stats.flush_num = 0;
	stats.br_num = 0;
	stats.size = 0;


	int sm_size = pow(2, history_size);

	btb_table = new btb_line[btb_size];

	global_history = 0;

	//if is global table (state machine)
	if (is_global_table) {
		global_sm = new state[sm_size];

		for (int i = 0; i < sm_size ; i++) {
			global_sm[i] = WNT;
		}
	}


	// to initialize the table (0) in all the fields and is_valid = false
	for (int i = 0; i < btb_size; i++) {
		btb_table[i].is_valid = false;

		if (is_global_hist) {
			btb_table[i].history = &global_history;
		} else {
			// if not global history need to add history for each btb_line
			btb_table[i].history = new uint8_t;
			*btb_table[i].history = 0;
		}

		if (is_global_table) {
			btb_table[i].state_mach = global_sm;
		} else {
			//if not global table need to add state table size of pow(2,history_size)
			btb_table[i].state_mach = new state[sm_size];
			for (int j = 0; j < sm_size; j++) {
				btb_table[i].state_mach[j] = WNT;
			}
		}
	}

	return 0;
}

int get_sm_index(uint8_t history, uint32_t pc) {
	//calculate the index in the state machine btb_table
	uint32_t pc_masked = 0;
	int sm_size = (int) pow(2, history_size);
	if (shared_type == 0) {
		//not using share
		return history;
	} else if (shared_type == 1) {
		//lsb shared type
		pc_masked = pc / 4;
		pc_masked = pc_masked & (sm_size - 1);
		return (pc_masked) ^ (history);
	} else if (shared_type == 2) {
		//mid shared
		pc_masked = pc / (int)(pow(2,16));
		pc_masked = pc_masked & (sm_size - 1);
		return (pc_masked) ^ (history);
	}
	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst) {
	// check if in the btb by direct mapping
	// index is calculated by : (pc/4) & (btb_size-1)
	int btb_index = (pc / 4) & (btb_size - 1);
	if (btb_table[btb_index].is_valid) {
		if (btb_table[btb_index].tag
				== ((pc / 4) & ((int) (pow(2, tag_size)) - 1))) {
			int sm_index = get_sm_index(*btb_table[btb_index].history, pc);
			state cur_state = btb_table[btb_index].state_mach[sm_index];

			if ((cur_state == ST) || (cur_state == WT)) {
				*dst = (btb_table[btb_index].target) * 4;
				return true;
			}
		}
	}
	*dst = pc + 4;
	return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
	//find the right line in the btb (same as predict)

	int btb_index = (pc / 4) & (btb_size - 1);
	btb_line* btb_ent = &btb_table[btb_index];
	bool tag_compare = (btb_table[btb_index].tag
			!= ((pc / 4) & ((int) (pow(2, tag_size)) - 1)));
	if ((btb_ent->is_valid && tag_compare) || !btb_ent->is_valid) {
		btb_ent->tag = ((pc / 4) & ((int) (pow(2, tag_size)) - 1));
		if (!is_global_hist) {
			*btb_ent->history = 0;
		}
		if (!is_global_table) {
			for (int j = 0; j < pow(2, history_size); j++) {
				btb_ent->state_mach[j] = WNT;
			}
		}
		if (!btb_ent->is_valid) {
			btb_ent->is_valid = true;
			// increasing the stats size
		}
	}

	btb_table[btb_index].target = targetPc;
	int sm_index = get_sm_index(*btb_table[btb_index].history, pc);
	state* cur_state = &btb_table[btb_index].state_mach[sm_index];
	if (taken) {
		switch (*cur_state) {
		case SNT:
			*cur_state = WNT;
			break;
		case WNT:
			*cur_state = WT;
			break;
		case WT:
		case ST:
			*cur_state = ST;
			break;
		}
	} else {
		switch (*cur_state) {
		case ST:
			*cur_state = WT;
			break;
		case WT:
			*cur_state = WNT;
			break;
		case WNT:
		case SNT:
			*cur_state = SNT;
			break;
		}
	}

	uint8_t prev_history = *btb_table[btb_index].history;
	*btb_table[btb_index].history = (prev_history * 2) & ((int) (pow(2, history_size)) - 1);
	*btb_table[btb_index].history += (taken) ? 1 : 0;

	//update the stats
	stats.br_num += 1;

	if (taken) {
		if (targetPc != pred_dst)
			stats.flush_num += 1;
	} else {
		if ((pc + 4) != pred_dst)
			stats.flush_num += 1;
	}


	return;
}

void BP_GetStats(SIM_stats *curStats) {
	*curStats = stats;
	return;
}
