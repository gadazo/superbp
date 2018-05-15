/* 046267 Computer Architecture - Spring 2016 - HW #2 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include "math.h"
#include "stdio.h"

// ********************** structs ***********************************

// state enum
typedef enum {
	SNT = 0, WNT, WT, ST
} state;

// BTB line struct
typedef struct {
	bool is_valid;
	uint32_t tag;
	uint32_t target;
	uint8_t* history;
	state* state_mach;
} btb_line;

//*********************** Global Variables ***************************
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

// ********************* Functions ***********************************


/*
Function Name : BP_init
Purpose :       to initialize the Branch table ,History table (global/local)
				and the predictor (state machine)
Input :         btbSize (unsigned int) - the size of the branch table
				historySize (unsigned int) - the size of the history table
				tagSize (unsigned int) - how many bits are used to as TAG
				isGlobalHist (bool) - is the history is used globally
				isGlobalTable (bool) - if the predictor os used globally
Output :        int - 0 if intialized sussefully
 */
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
	stats.size = btb_size * (tag_size + 30);


	int sm_size = pow(2, history_size);

	btb_table = new btb_line[btb_size];

	if (is_global_hist) {
		global_history = 0;
		stats.size += history_size;
	}
	else {
		stats.size += history_size * btb_size;
	}


	//if is global table (state machine)
	if (is_global_table) {
		global_sm = new state[sm_size];

		for (int i = 0; i < sm_size; i++) {
			global_sm[i] = WNT;
		}

		stats.size += 2 * sm_size;
	}
	else {
		stats.size += 2 * sm_size * btb_size;
	}

	// to initialize the table (0) in all the fields and is_valid = false
	for (int i = 0; i < btb_size; i++) {
		btb_table[i].is_valid = false;

		if (is_global_hist) {
			btb_table[i].history = &global_history;
		}
		else {
			// if not global history need to add history for each btb_line
			btb_table[i].history = new uint8_t;
			*btb_table[i].history = 0;
		}

		if (is_global_table) {
			btb_table[i].state_mach = global_sm;
		}
		else {
			//if not global table need to add state table size of pow(2,history_size)
			btb_table[i].state_mach = new state[sm_size];
			for (int j = 0; j < sm_size; j++) {
				btb_table[i].state_mach[j] = WNT;
			}
		}
	}

	return 0;
}


/*
  Function Name : get_sm_index
  Purpose :       calculate the index of the state machine in the BTB
  Input :         history (unsigned 8bit int) - the wanted history (global/local)
				  pc (unsigned 8bit int) - the pc (for Lshare/Gshare)
  Output :        int - the index
*/
int get_sm_index(uint8_t history, uint32_t pc) {
	uint32_t pc_masked = 0;
	int sm_size = (int)pow(2, history_size);
	if (shared_type == 0) {
		//not using share - the index is the history itself
		return history;
	}
	else if (shared_type == 1) {
		//lsb shared type - bit wise xor between the history and pc from the third bit
		pc_masked = pc / 4;
		pc_masked = pc_masked & (sm_size - 1);
		return (pc_masked) ^ (history);
	}
	else if (shared_type == 2) {
		//mid shared - bit wise xor between the history and the pc from the 16th bit
		pc_masked = pc / (int)(pow(2, 16));
		pc_masked = pc_masked & (sm_size - 1);
		return (pc_masked) ^ (history);
	}
	return 0;
}


/*
  Function Name : BP_predict
  Purpose :       to make a prediction based on the history (local/global) of the branches
				  using BTB
  Input :         pc (unsigned 32bit int) - the current pc used identify the branch (TAG/SET)
				  dst (pointer - unsigned 32 int) - returned value = PC+4/ Taken dst
  Output :        bool - true -Taken , false - Not Taken
*/
bool BP_predict(uint32_t pc, uint32_t *dst) {
	// check if the branch is in the btb using direct mapping
	int btb_index = (pc / 4) & (btb_size - 1);
	if (btb_table[btb_index].is_valid) {
		// check if the Tag is matched
		if (btb_table[btb_index].tag
			== ((pc / 4) & ((int)(pow(2, tag_size)) - 1))) {
			int sm_index = get_sm_index(*btb_table[btb_index].history, pc);
			state cur_state = btb_table[btb_index].state_mach[sm_index];

			if ((cur_state == ST) || (cur_state == WT)) {
				*dst = btb_table[btb_index].target;
				return true;
			}
		}
	}
	//if the set is not valid / tag is not matched / Not Taken state - PC+4 and false
	*dst = pc + 4;
	return false;
}

/*
  Function Name : BP_update
  Purpose :       updates the BTB ,history and state machine after exeucution
				  "flush" if needed
  Input :         pc (unsigned 32bit int) - the current pc
				  targetPc (unsigned 32bit int) - the target pc
				  taken (bool) - jump should occured
				  pred_dst (unsigned 32 int) - the predicted destination
  Output :        none
*/
void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
	//find the right line in the btb (same as predict
	int btb_index = (pc / 4) & (btb_size - 1);
	btb_line* btb_ent = &btb_table[btb_index];

	bool prediction = false;
	if (btb_ent->is_valid) {
		// check if the Tag is matched
		if (btb_ent->tag == ((pc / 4) & ((int)(pow(2, tag_size)) - 1))) {
			int sm_index = get_sm_index(*btb_table[btb_index].history, pc);
			state cur_state = btb_table[btb_index].state_mach[sm_index];
			if ((cur_state == ST) || (cur_state == WT)) {
				prediction = true;
			}
		}
	}

	bool tag_compare = (btb_table[btb_index].tag
		!= ((pc / 4) & ((int)(pow(2, tag_size)) - 1)));
	if ((btb_ent->is_valid && tag_compare) || !btb_ent->is_valid) {
		btb_ent->tag = ((pc / 4) & ((int)(pow(2, tag_size)) - 1));
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
		}
	}

	btb_table[btb_index].target = targetPc;
	// finds the next state
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
	}
	else {
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

	//updates the history
	uint8_t prev_history = *btb_table[btb_index].history;
	*btb_table[btb_index].history = (prev_history * 2) & ((int)(pow(2, history_size)) - 1);
	*btb_table[btb_index].history += (taken) ? 1 : 0;

	//update the stats
	stats.br_num += 1;
	//check if flushed
	if (prediction != taken){
		stats.flush_num += 1;
	}
	else if (taken) {
		if (targetPc != pred_dst)
			stats.flush_num += 1;
	}
	else {
		if ((pc + 4) != pred_dst)
			stats.flush_num += 1;
	}

	return;
}

/*
  Function Name : BP_GetStats
  Purpose :       return the current stats
  Input :         curStats (pointer - SIM_stats) - the stats in the end of the program
  Output :        none
*/
void BP_GetStats(SIM_stats *curStats) {
	*curStats = stats;
	return;
}
