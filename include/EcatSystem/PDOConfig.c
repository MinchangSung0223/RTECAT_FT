#include "PDOConfig.h"

// Index, Subindex, DataType
ec_pdo_entry_info_t iServo_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* controlword */
    {0x607a, 0x00, 32}, /* target_position */
    {0x60ff, 0x00, 32}, /* target_velocity */
    {0x6071, 0x00, 16}, /* target_torque */
    {0x6060, 0x00, 8}, /* modes_of_operation */

    {0x6041, 0x00, 16}, /* statusword */
    {0x6064, 0x00, 32}, /* position_actual_value */
    {0x606c, 0x00, 32}, /* velocity_actual_value */
    {0x6077, 0x00, 16}, /* torque_actual_value */
    {0x6061, 0x00, 8}, /* modes_of_operation_display */
};

ec_pdo_info_t iServo_pdos[] = {
    {0x1600, 5, iServo_pdo_entries + 0}, /* Drive RxPDO */
    {0x1a00, 5, iServo_pdo_entries + 5}, /* Drive TxPDO */
};

ec_sync_info_t iServo_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, iServo_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, iServo_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
ec_pdo_entry_info_t RobotusFT_pdo_entries[] =
{
    {0x7000, 0x01, 16},
    {0x6000, 0x01, 32},
    {0x6000, 0x02, 32},
    {0x6000, 0x03, 32},
    {0x6000, 0x04, 32},
    {0x6000, 0x05, 32},
    {0x6000, 0x06, 32},
    {0x6000, 0x07, 32},
    {0x6000, 0x08, 32},
};

ec_pdo_info_t RobotusFT_pdos[] = {
		{0x1600, 1, 	RobotusFT_pdo_entries + 0}, //OUT_GENERIC process data mapping
		{0x1a00, 8, 	RobotusFT_pdo_entries + 1}, //IN_GENERIC process data mapping
};

ec_sync_info_t RobotusFT_syncs[5] = {
		{0, EC_DIR_OUTPUT, 	0, NULL, 				EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 	0, NULL, 				EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 	1, RobotusFT_pdos + 0, 	EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 	1, RobotusFT_pdos + 1, 	EC_WD_DISABLE},
		{0xff}
};

// Index, Subindex, DataType
ec_pdo_entry_info_t Elmo_pdo_entries[] = {
    {0x607a, 0x00, 32}, 	/* Target position */
    {0x60ff, 0x00, 32}, 	/* Target velocity */
    {0x6071, 0x00, 16}, 	/* Target torque */
    {0x6072, 0x00, 16}, 	/* Maximal torque */
    {0x6040, 0x00, 16}, 	/* Controlword */
    {0x6060, 0x00, 8}, 		/* Modes of operation */
    {0x6064, 0x00, 32}, 	/* Position actual value */
    {0x6077, 0x00, 16}, 	/* Torque value */
    {0x6041, 0x00, 16}, 	/* Statusword */
    {0x6061, 0x00, 8}, 		/* Modes of operation display */
	{0x6069, 0x00, 32},		/* Velocity actual value*/
};


ec_pdo_info_t Elmo_pdos[] = {
    {0x1605, 6, Elmo_pdo_entries + 0}, /* RPDO6 Mapping */
    {0x1a02, 4, Elmo_pdo_entries + 6}, /* TPDO3 Mapping */
	{0x1A0f, 1, Elmo_pdo_entries + 10},
};


ec_sync_info_t Elmo_syncs[5] = { 
    {0, EC_DIR_OUTPUT, 	0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 	0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 	1, Elmo_pdos + 0, EC_WD_ENABLE},
	{3, EC_DIR_INPUT, 	2, Elmo_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
