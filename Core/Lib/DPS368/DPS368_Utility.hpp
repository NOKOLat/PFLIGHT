/*
 * DPS368_Utility.hpp
 *
 *  Created on: Aug 17, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_DPS368_UTILITY_HPP_
#define INC_DPS368_UTILITY_HPP_


enum class DPS368_Error{

	NO_ERROR = 0,
	NOT_FOUND = 1,

};

enum class MEAS_RATE{

	_001pr_sec = 0,
	_002pr_sec,
	_004pr_sec,
	_008pr_sec,
	_016pr_sec,
	_032pr_sec,
	_064pr_sec,
	_128pr_sec,
};

enum class MEAS_SAMPLING{

	_001_times = 0,
    _002_times,
	_004_times,
	_008_times,
	_016_times,
	_032_times,
	_064_times,
	_128_times,
};


#endif /* INC_DPS368_UTILITY_HPP_ */
