#ifndef ECATSYSTEM_ECAT_FT_H_
#define ECATSYSTEM_ECAT_FT_H_

#include "Ecat_Slave.h"
#include "PDOConfig.h"

class Ecat_FT : public Slave {
public:
    Ecat_FT() : Slave(Robotus_VendorID, Robotus_ProductCode) {}
    virtual ~Ecat_FT() {}

    /**
     * Configure FT sensor settings.
     * @param config Configuration parameter for the sensor.
     */
    void writeFTconfig(uint32_t config) {
        FT_configparam_ = config;
    }

    /**
     * Process PDO data (read/write operations).
     * Handles domain data for FT sensor.
     * @param index Current domain index.
     * @param domain_address Pointer to domain data.
     */
    virtual void processData(size_t index, uint8_t* domain_address) {
        if (domain_address == nullptr) {
            rt_printf("Error: Null domain_address encountered at index %d\n", index);
            return;
        }

        switch (index) {
            case 0:
                EC_WRITE_S16(domain_address, iLed_);
                // rt_printf("iLed written: %d\n", iLed_);
                break;
            case 1:
                FT_Raw_F[0] = EC_READ_S32(domain_address);
                // rt_printf("Fx: %d\n", FT_Raw_F[0]);
                break;
            case 2:
                FT_Raw_F[1] = EC_READ_S32(domain_address);
                // rt_printf("Fy: %d\n", FT_Raw_F[1]);
                break;
            case 3:
                FT_Raw_F[2] = EC_READ_S32(domain_address);
                // rt_printf("Fz: %d\n", FT_Raw_F[2]);
                break;
            case 4:
                FT_Raw_T[0] = EC_READ_S32(domain_address);
                break;
            case 5:
                FT_Raw_T[1] = EC_READ_S32(domain_address);
                break;
            case 6:
                FT_Raw_T[2] = EC_READ_S32(domain_address);
                break;
            case 7:
                FTS_Status = EC_READ_U32(domain_address);
                break;
            case 8:
                Temperature = EC_READ_S32(domain_address);
                break;
            default:
                rt_printf("Warning: Index %d not mapped to any PDO data.\n", index);
                break;
        }

        // State change check for the last domain entry
        if (index == 7 && last_iStatus_ != -1) {
            if (iStatus_ != last_iStatus_) {
                rt_printf("RobotusFT_POS: %d, RobotusFT_STATE: %d\n", slave_position, iStatus_);
            }
            last_iStatus_ = iStatus_;
        }
        int32_t raw_data = EC_READ_S32(domain_address);
        // rt_printf("Raw data at index %d: %d\n", index, raw_data);
        // rt_printf("Index: %d, Domain address: %p\n", index, domain_address);


    }

    // Synchronization info
    virtual const ec_sync_info_t* syncs() { return &RobotusFT_syncs[0]; }
    virtual size_t syncSize() { return sizeof(RobotusFT_syncs) / sizeof(ec_sync_info_t); }

    // PDO entry info
    virtual const ec_pdo_entry_info_t* channels() { return RobotusFT_pdo_entries; }

    // Domain mapping
    virtual void domains(DomainMap& domains) const {
        domains = domains_;
    }

    // Member variables
    int16_t iLed_ = 0;                // Write
    int32_t FT_configparam_ = 0;    // Write
    uint8_t iStatus_ = 0;            // Read
    int32_t FT_Raw_F[3] = {0};       // Read (Fx, Fy, Fz)
    int32_t FT_Raw_T[3] = {0};       // Read (Tx, Ty, Tz)
    uint32_t FTS_Status = 0;
    int32_t Temperature = 0;

private:
    DomainMap domains_ = {
        {0, {0, 1, 2, 3, 4, 5, 6, 7, 8}}
    };

    int last_iStatus_ = -1; // Last recorded status
};

#endif /* ECATSYSTEM_ECAT_FT_H_ */
