#ifndef DEF_H
#define DEF_H

#include <QObject>
#include <eigen/Eigen/Dense>

typedef struct {
    double signal_rate; // MCPS
    double ambient_rate; // MCPS
    double sigma_mm; // mm
    double eff_spad_count;
    int range_mm; // mm
    int timeEsp_ms; // ms, time on arduino when measurement was conducted
                   // only valid on a system with 32-bit ints, otherwise overflow will occur
    int status; //
    int timePc_ms; // ms, since start of system timer
} RangingData_t;

typedef struct {
    double z;
    double z_dot;
    int timeEsp_ms;
    int timePc_ms;
    Eigen::Matrix<double, 2, 2> P;
    bool valid;

} AltState_t;

typedef struct {
    double z;
    double z_dot;
} AltTarget_t;

// Range status defines copied from ST VL53L1X api
#define	 VL53L1_RANGESTATUS_RANGE_VALID				0           /*!<The Range is valid. */
#define	 VL53L1_RANGESTATUS_SIGMA_FAIL				1           /*!<Sigma Fail. */
#define	 VL53L1_RANGESTATUS_SIGNAL_FAIL				2           /*!<Signal fail. */
#define	 VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED	3   /*!<Target is below minimum detection threshold. */
#define	 VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL			4       /*!<Phase out of valid limits -  different to a wrap exit. */
#define	 VL53L1_RANGESTATUS_HARDWARE_FAIL			5           /*!<Hardware fail. */
#define	 VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL	6   /*!<The Range is valid but the wraparound check has not been done. */
#define	VL53L1_RANGESTATUS_WRAP_TARGET_FAIL			7           /*!<Wrapped target - no matching phase in other VCSEL period timing. */
#define	VL53L1_RANGESTATUS_PROCESSING_FAIL			8           /*!<Internal algo underflow or overflow in lite ranging. */
#define	VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL			9       /*!<Specific to lite ranging. */
#define	VL53L1_RANGESTATUS_SYNCRONISATION_INT			10      /*!<1st interrupt when starting ranging in back to back mode. Ignore data. */
#define	VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE		11
/*!<All Range ok but object is result of multiple pulses merging together.
 * Used by RQL for merged pulse detection
 */
#define	VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL	12  /*!<Used  by RQL  as different to phase fail. */
#define	VL53L1_RANGESTATUS_MIN_RANGE_FAIL			13          /*!<User ROI input is not valid e.g. beyond SPAD Array.*/
#define	VL53L1_RANGESTATUS_RANGE_INVALID			14          /*!<lld returned valid range but negative value ! */
#define	 VL53L1_RANGESTATUS_NONE				255             /*!<No Update. */

#endif // DEF_H
