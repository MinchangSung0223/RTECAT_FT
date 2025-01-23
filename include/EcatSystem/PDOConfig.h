/*! 
 *  @file PDOConfig.h
 *  @brief PDOConfig
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Oct. 26. 2023
 *  @Comm
 */

#ifndef PDOCONFIG_H_
#define PDOCONFIG_H_

#pragma once 

// Ethercat Master---- 
#include "ecrt.h" 

// Vendor ID & Product Code 
#define iServo_VendorID  0x00000000
#define iServo_ProductCode 0x00020001

#define Elmo_VendorID  0x0000009a
#define Elmo_ProductCode 0x00030924

#define Robotus_VendorID  0x000008ee
#define Robotus_ProductCode 0x00000003

extern ec_pdo_entry_info_t 	iServo_pdo_entries[];
extern ec_pdo_info_t		iServo_pdos[];
extern ec_sync_info_t		iServo_syncs[5];

extern ec_pdo_entry_info_t 	Elmo_pdo_entries[];
extern ec_pdo_info_t		Elmo_pdos[];
extern ec_sync_info_t		Elmo_syncs[5];

extern ec_pdo_entry_info_t 	RobotusFT_pdo_entries[];
extern ec_pdo_info_t		RobotusFT_pdos[];
extern ec_sync_info_t		RobotusFT_syncs[5];






#endif /* PDOCONFIG_H_ */