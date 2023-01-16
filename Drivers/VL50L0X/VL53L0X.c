// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include <stdint.h>
#include "stm32g0xx_hal.h" // Change it for your requirements.
#include "string.h"
#include "VL53L0X.h"
#include "main.h"



//---------------------------------------------------------
// Local variables within this file (private)
//---------------------------------------------------------
uint8_t g_i2cAddr = ADDRESS_DEFAULT;
uint16_t g_ioTimeout = 0;  // no timeout
uint8_t g_isTimeout = 0;
uint16_t g_timeoutStartMs;
uint8_t g_stopVariable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t g_measTimBudUs;

#define I2C_TIMEOUT 100 // I2C timeout in ms
#define I2C_READ 1
#define I2C_WRITE 0
I2C_HandleTypeDef VL53L0X_I2C_Handler; // I2C handler

I2C_HandleTypeDef VL53L0X_I2C_Handler_l;
I2C_HandleTypeDef VL53L0X_I2C_Handler_r;

uint8_t msgBuffer[4];
HAL_StatusTypeDef i2cStat;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

//---------------------------------------------------------
// Locally used functions (private)
//---------------------------------------------------------
bool getSpadInfo(uint8_t *count, bool *type_is_aperture,uint8_t tof_chosen);
void getSequenceStepEnables(SequenceStepEnables * enables,uint8_t tof_chosen);
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts,uint8_t tof_chosen);
bool performSingleRefCalibration(uint8_t vhv_init_byte,uint8_t tof_chosen);
static uint16_t decodeTimeout(uint16_t value);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

//---------------------------------------------------------
// I2C communication Functions
//---------------------------------------------------------
// Write an 8-bit register
void writeReg(uint8_t reg, uint8_t value,uint8_t tof_chosen) {

	msgBuffer[0] = value; // Assign the value to the buffer.
	if (tof_chosen==TOF_RIGHT)
		i2cStat = HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler_r, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 1, I2C_TIMEOUT);
	else
		i2cStat = HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler_l, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 1, I2C_TIMEOUT);
}

// Write a 16-bit register
void writeReg16Bit(uint8_t reg, uint16_t value,uint8_t tof_chosen){

	memcpy(msgBuffer, &value, 2); // Assign the value to the buffer.
	if (tof_chosen==TOF_RIGHT)
		i2cStat = HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler_r, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 2, I2C_TIMEOUT);
	else
		i2cStat = HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler_l, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 2, I2C_TIMEOUT);
}

// Write a 32-bit register
void writeReg32Bit(uint8_t reg, uint32_t value,uint8_t tof_chosen){

	memcpy(msgBuffer, &value, 4); // Assign the value to the buffer.
	if (tof_chosen==TOF_RIGHT)
		i2cStat = HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler_r, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 4, I2C_TIMEOUT);
	else
		i2cStat = HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler_l, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 4, I2C_TIMEOUT);
}

// Read an 8-bit register
uint8_t readReg(uint8_t reg,uint8_t tof_chosen) {
	uint8_t value;
	if (tof_chosen==TOF_RIGHT)
		i2cStat = HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler_r, g_i2cAddr | I2C_READ, reg, 1, msgBuffer, 1, I2C_TIMEOUT);
	else
		i2cStat = HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler_l, g_i2cAddr | I2C_READ, reg, 1, msgBuffer, 1, I2C_TIMEOUT);
	value = msgBuffer[0];

	return value;
}

// Read a 16-bit register
uint16_t readReg16Bit(uint8_t reg,uint8_t tof_chosen) {
	uint16_t value;
	if (tof_chosen==TOF_RIGHT)
		i2cStat = HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler_r, g_i2cAddr | I2C_READ, reg, 1, msgBuffer, 2, I2C_TIMEOUT);
	else
		i2cStat = HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler_l, g_i2cAddr | I2C_READ, reg, 1, msgBuffer, 2, I2C_TIMEOUT);
	memcpy(&value, msgBuffer, 2);

	return value;
}

// Read a 32-bit register
uint32_t readReg32Bit(uint8_t reg,uint8_t tof_chosen) {
	uint32_t value;
	if (tof_chosen==TOF_RIGHT)
		i2cStat = HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler_r, g_i2cAddr | I2C_READ, reg, 1, msgBuffer, 4, I2C_TIMEOUT);
	else
		i2cStat = HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler_l, g_i2cAddr | I2C_READ, reg, 1, msgBuffer, 4, I2C_TIMEOUT);
	memcpy(&value, msgBuffer, 4);

	return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void writeMulti(uint8_t reg, uint8_t const *src, uint8_t count,uint8_t tof_chosen){

	memcpy(msgBuffer, src, 4);
	if (tof_chosen==TOF_RIGHT)
		i2cStat = HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler_r, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, count, I2C_TIMEOUT);
	else
		i2cStat = HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler_l, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, count, I2C_TIMEOUT);
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void readMulti(uint8_t reg, uint8_t * dst, uint8_t count,uint8_t tof_chosen) {
	if (tof_chosen==TOF_RIGHT)
		i2cStat = HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler_r, g_i2cAddr | I2C_READ, reg, 1, dst, count, I2C_TIMEOUT);
	else
		i2cStat = HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler_l, g_i2cAddr | I2C_READ, reg, 1, dst, count, I2C_TIMEOUT);
}


// Public Methods //////////////////////////////////////////////////////////////

void setAddress_VL53L0X(uint8_t new_addr,uint8_t tof_chosen) {
	writeReg( I2C_SLAVE_DEVICE_ADDRESS, (new_addr>>1) & 0x7F ,tof_chosen);
	g_i2cAddr = new_addr;
}

uint8_t getAddress_VL53L0X() {
	return g_i2cAddr;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool initVL53L0X(bool io_2v8, I2C_HandleTypeDef *handler,uint8_t tof_chosen){
	// VL53L0X_DataInit() begin

	// Handler
	if (tof_chosen==TOF_RIGHT)
		memcpy(&VL53L0X_I2C_Handler_r, handler, sizeof(*handler));
	else
		memcpy(&VL53L0X_I2C_Handler_l, handler, sizeof(*handler));

	// Reset the message buffer.
	msgBuffer[0] = 0;
	msgBuffer[1] = 0;
	msgBuffer[2] = 0;
	msgBuffer[3] = 0;

	// sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	if (io_2v8)
	{
		writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
				readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,tof_chosen) | 0x01,tof_chosen); // set bit 0
	}

	// "Set I2C standard mode"
	writeReg(0x88, 0x00,tof_chosen);

	writeReg(0x80, 0x01,tof_chosen);
	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x00, 0x00,tof_chosen);
	g_stopVariable = readReg(0x91,tof_chosen);
	writeReg(0x00, 0x01,tof_chosen);
	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x80, 0x00,tof_chosen);

	// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL,tof_chosen) | 0x12,tof_chosen);

	// set final range signal rate limit to 0.25 MCPS (million counts per second)
	setSignalRateLimit(0.25,tof_chosen);

	writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF,tof_chosen);

	// VL53L0X_DataInit() end

	// VL53L0X_StaticInit() begin

	uint8_t spad_count;
	bool spad_type_is_aperture;
	if (!getSpadInfo(&spad_count, &spad_type_is_aperture,tof_chosen)) { return false; }

	// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	// the API, but the same data seems to be more easily readable from
	// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	uint8_t ref_spad_map[6];
	readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6,tof_chosen);

	// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00,tof_chosen);
	writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C,tof_chosen);
	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4,tof_chosen);

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++)
	{
		if (i < first_spad_to_enable || spads_enabled == spad_count)
		{
			// This bit is lower than the first one that should be enabled, or
			// (reference_spad_count) bits have already been enabled, so zero this bit
			ref_spad_map[i / 8] &= ~(1 << (i % 8));
		}
		else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
		{
			spads_enabled++;
		}
	}

	writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6,tof_chosen);

	// -- VL53L0X_set_reference_spads() end

	// -- VL53L0X_load_tuning_settings() begin
	// DefaultTuningSettings from vl53l0x_tuning.h

	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x00, 0x00,tof_chosen);

	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x09, 0x00,tof_chosen);
	writeReg(0x10, 0x00,tof_chosen);
	writeReg(0x11, 0x00,tof_chosen);

	writeReg(0x24, 0x01,tof_chosen);
	writeReg(0x25, 0xFF,tof_chosen);
	writeReg(0x75, 0x00,tof_chosen);

	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x4E, 0x2C,tof_chosen);
	writeReg(0x48, 0x00,tof_chosen);
	writeReg(0x30, 0x20,tof_chosen);

	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x30, 0x09,tof_chosen);
	writeReg(0x54, 0x00,tof_chosen);
	writeReg(0x31, 0x04,tof_chosen);
	writeReg(0x32, 0x03,tof_chosen);
	writeReg(0x40, 0x83,tof_chosen);
	writeReg(0x46, 0x25,tof_chosen);
	writeReg(0x60, 0x00,tof_chosen);
	writeReg(0x27, 0x00,tof_chosen);
	writeReg(0x50, 0x06,tof_chosen);
	writeReg(0x51, 0x00,tof_chosen);
	writeReg(0x52, 0x96,tof_chosen);
	writeReg(0x56, 0x08,tof_chosen);
	writeReg(0x57, 0x30,tof_chosen);
	writeReg(0x61, 0x00,tof_chosen);
	writeReg(0x62, 0x00,tof_chosen);
	writeReg(0x64, 0x00,tof_chosen);
	writeReg(0x65, 0x00,tof_chosen);
	writeReg(0x66, 0xA0,tof_chosen);

	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x22, 0x32,tof_chosen);
	writeReg(0x47, 0x14,tof_chosen);
	writeReg(0x49, 0xFF,tof_chosen);
	writeReg(0x4A, 0x00,tof_chosen);

	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x7A, 0x0A,tof_chosen);
	writeReg(0x7B, 0x00,tof_chosen);
	writeReg(0x78, 0x21,tof_chosen);

	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x23, 0x34,tof_chosen);
	writeReg(0x42, 0x00,tof_chosen);
	writeReg(0x44, 0xFF,tof_chosen);
	writeReg(0x45, 0x26,tof_chosen);
	writeReg(0x46, 0x05,tof_chosen);
	writeReg(0x40, 0x40,tof_chosen);
	writeReg(0x0E, 0x06,tof_chosen);
	writeReg(0x20, 0x1A,tof_chosen);
	writeReg(0x43, 0x40,tof_chosen);

	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x34, 0x03,tof_chosen);
	writeReg(0x35, 0x44,tof_chosen);

	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x31, 0x04,tof_chosen);
	writeReg(0x4B, 0x09,tof_chosen);
	writeReg(0x4C, 0x05,tof_chosen);
	writeReg(0x4D, 0x04,tof_chosen);

	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x44, 0x00,tof_chosen);
	writeReg(0x45, 0x20,tof_chosen);
	writeReg(0x47, 0x08,tof_chosen);
	writeReg(0x48, 0x28,tof_chosen);
	writeReg(0x67, 0x00,tof_chosen);
	writeReg(0x70, 0x04,tof_chosen);
	writeReg(0x71, 0x01,tof_chosen);
	writeReg(0x72, 0xFE,tof_chosen);
	writeReg(0x76, 0x00,tof_chosen);
	writeReg(0x77, 0x00,tof_chosen);

	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x0D, 0x01,tof_chosen);

	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x80, 0x01,tof_chosen);
	writeReg(0x01, 0xF8,tof_chosen);

	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x8E, 0x01,tof_chosen);
	writeReg(0x00, 0x01,tof_chosen);
	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x80, 0x00,tof_chosen);

	// -- VL53L0X_load_tuning_settings() end

	// "Set interrupt config to new sample ready"
	// -- VL53L0X_SetGpioConfig() begin

	writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04,tof_chosen);
	writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH,tof_chosen) & ~0x10,tof_chosen); // active low
	writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01,tof_chosen);

	// -- VL53L0X_SetGpioConfig() end

	g_measTimBudUs = getMeasurementTimingBudget(tof_chosen);

	// "Disable MSRC and TCC by default"
	// MSRC = Minimum Signal Rate Check
	// TCC = Target CentreCheck
	// -- VL53L0X_SetSequenceStepEnable() begin

	writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8,tof_chosen);

	// -- VL53L0X_SetSequenceStepEnable() end

	// "Recalculate timing budget"
	setMeasurementTimingBudget(g_measTimBudUs,tof_chosen);

	// VL53L0X_StaticInit() end

	// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

	// -- VL53L0X_perform_vhv_calibration() begin

	writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01,tof_chosen);
	if (!performSingleRefCalibration(0x40,tof_chosen)) { return false; }

	// -- VL53L0X_perform_vhv_calibration() end

	// -- VL53L0X_perform_phase_calibration() begin

	writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02,tof_chosen);
	if (!performSingleRefCalibration(0x00,tof_chosen)) { return false; }

	// -- VL53L0X_perform_phase_calibration() end

	// "restore the previous Sequence Config"
	writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8,tof_chosen);

	// VL53L0X_PerformRefCalibration() end

	return true;
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool setSignalRateLimit(float limit_Mcps,uint8_t tof_chosen)
{
	if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

	// Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7),tof_chosen);
	return true;
}

// Get the return signal rate limit check value in MCPS
float getSignalRateLimit(uint8_t tof_chosen)
{
	return (float)readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,tof_chosen) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool setMeasurementTimingBudget(uint32_t budget_us,uint8_t tof_chosen)
{
	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
	uint16_t const EndOverhead        = 960;
	uint16_t const MsrcOverhead       = 660;
	uint16_t const TccOverhead        = 590;
	uint16_t const DssOverhead        = 690;
	uint16_t const PreRangeOverhead   = 660;
	uint16_t const FinalRangeOverhead = 550;

	uint32_t const MinTimingBudget = 20000;

	if (budget_us < MinTimingBudget) { return false; }

	uint32_t used_budget_us = StartOverhead + EndOverhead;

	getSequenceStepEnables(&enables,tof_chosen);
	getSequenceStepTimeouts(&enables, &timeouts,tof_chosen);

	if (enables.tcc)
	{
		used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss)
	{
		used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}
	else if (enables.msrc)
	{
		used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range)
	{
		used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range)
	{
		used_budget_us += FinalRangeOverhead;

		// "Note that the final range timeout is determined by the timing
		// budget and the sum of all other timeouts within the sequence.
		// If there is no room for the final range timeout, then an error
		// will be set. Otherwise the remaining time will be applied to
		// the final range."

		if (used_budget_us > budget_us)
		{
			// "Requested timeout too big."
			return false;
		}

		uint32_t final_range_timeout_us = budget_us - used_budget_us;

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."

		uint16_t final_range_timeout_mclks =
				timeoutMicrosecondsToMclks(final_range_timeout_us,
						timeouts.final_range_vcsel_period_pclks);

		if (enables.pre_range)
		{
			final_range_timeout_mclks += timeouts.pre_range_mclks;
		}

		writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
				encodeTimeout(final_range_timeout_mclks),tof_chosen);

		// set_sequence_step_timeout() end

		g_measTimBudUs = budget_us; // store for internal reuse
	}
	return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t getMeasurementTimingBudget(uint8_t tof_chosen)
{
	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
	uint16_t const EndOverhead        = 960;
	uint16_t const MsrcOverhead       = 660;
	uint16_t const TccOverhead        = 590;
	uint16_t const DssOverhead        = 690;
	uint16_t const PreRangeOverhead   = 660;
	uint16_t const FinalRangeOverhead = 550;

	// "Start and end overhead times always present"
	uint32_t budget_us = StartOverhead + EndOverhead;

	getSequenceStepEnables(&enables,tof_chosen);
	getSequenceStepTimeouts(&enables, &timeouts,tof_chosen);

	if (enables.tcc)
	{
		budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss)
	{
		budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}
	else if (enables.msrc)
	{
		budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range)
	{
		budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range)
	{
		budget_us += (timeouts.final_range_us + FinalRangeOverhead);
	}

	g_measTimBudUs = budget_us; // store for internal reuse
	return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks,uint8_t tof_chosen)
{
	uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	getSequenceStepEnables(&enables,tof_chosen);
	getSequenceStepTimeouts(&enables, &timeouts,tof_chosen);

	// "Apply specific settings for the requested clock period"
	// "Re-calculate and apply timeouts, in macro periods"

	// "When the VCSEL period for the pre or final range is changed,
	// the corresponding timeout must be read from the device using
	// the current VCSEL period, then the new VCSEL period can be
	// applied. The timeout then must be written back to the device
	// using the new VCSEL period.
	//
	// For the MSRC timeout, the same applies - this timeout being
	// dependant on the pre-range vcsel period."


	if (type == VcselPeriodPreRange)
	{
		// "Set phase check limits"
		switch (period_pclks)
		{
		case 12:
			writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18,tof_chosen);
			break;

		case 14:
			writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30,tof_chosen);
			break;

		case 16:
			writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40,tof_chosen);
			break;

		case 18:
			writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50,tof_chosen);
			break;

		default:
			// invalid period
			return false;
		}
		writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08,tof_chosen);

		// apply new VCSEL period
		writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg,tof_chosen);

		// update timeouts

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

		uint16_t new_pre_range_timeout_mclks =
				timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

		writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
				encodeTimeout(new_pre_range_timeout_mclks),tof_chosen);

		// set_sequence_step_timeout() end

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

		uint16_t new_msrc_timeout_mclks =
				timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

		writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
				(new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1),tof_chosen);

		// set_sequence_step_timeout() end
	}
	else if (type == VcselPeriodFinalRange)
	{
		switch (period_pclks)
		{
		case 8:
			writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10,tof_chosen);
			writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08,tof_chosen);
			writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02,tof_chosen);
			writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C,tof_chosen);
			writeReg(0xFF, 0x01,tof_chosen);
			writeReg(ALGO_PHASECAL_LIM, 0x30,tof_chosen);
			writeReg(0xFF, 0x00,tof_chosen);
			break;

		case 10:
			writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28,tof_chosen);
			writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08,tof_chosen);
			writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03,tof_chosen);
			writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09,tof_chosen);
			writeReg(0xFF, 0x01,tof_chosen);
			writeReg(ALGO_PHASECAL_LIM, 0x20,tof_chosen);
			writeReg(0xFF, 0x00,tof_chosen);
			break;

		case 12:
			writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38,tof_chosen);
			writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08,tof_chosen);
			writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03,tof_chosen);
			writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08,tof_chosen);
			writeReg(0xFF, 0x01,tof_chosen);
			writeReg(ALGO_PHASECAL_LIM, 0x20,tof_chosen);
			writeReg(0xFF, 0x00,tof_chosen);
			break;

		case 14:
			writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48,tof_chosen);
			writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08,tof_chosen);
			writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03,tof_chosen);
			writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07,tof_chosen);
			writeReg(0xFF, 0x01,tof_chosen);
			writeReg(ALGO_PHASECAL_LIM, 0x20,tof_chosen);
			writeReg(0xFF, 0x00,tof_chosen);
			break;

		default:
			// invalid period
			return false;
		}

		// apply new VCSEL period
		writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg,tof_chosen);

		// update timeouts

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."

		uint16_t new_final_range_timeout_mclks =
				timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

		if (enables.pre_range)
		{
			new_final_range_timeout_mclks += timeouts.pre_range_mclks;
		}

		writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
				encodeTimeout(new_final_range_timeout_mclks),tof_chosen);

		// set_sequence_step_timeout end
	}
	else
	{
		// invalid type
		return false;
	}

	// "Finally, the timing budget must be re-applied"

	setMeasurementTimingBudget(g_measTimBudUs,tof_chosen);

	// "Perform the phase calibration. This is needed after changing on vcsel period."
	// VL53L0X_perform_phase_calibration() begin

	uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG,tof_chosen);
	writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02,tof_chosen);
	performSingleRefCalibration(0x0,tof_chosen);
	writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config,tof_chosen);

	// VL53L0X_perform_phase_calibration() end

	return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t getVcselPulsePeriod(vcselPeriodType type,uint8_t tof_chosen)
{
	if (type == VcselPeriodPreRange)
	{
		return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD,tof_chosen));
	}
	else if (type == VcselPeriodFinalRange)
	{
		return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD,tof_chosen));
	}
	else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void startContinuous(uint32_t period_ms,uint8_t tof_chosen)
{
	writeReg(0x80, 0x01,tof_chosen);
	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x00, 0x00,tof_chosen);
	writeReg(0x91, g_stopVariable,tof_chosen);
	writeReg(0x00, 0x01,tof_chosen);
	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x80, 0x00,tof_chosen);

	if (period_ms != 0)
	{
		// continuous timed mode

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

		uint16_t osc_calibrate_val = readReg16Bit(OSC_CALIBRATE_VAL,tof_chosen);

		if (osc_calibrate_val != 0)
		{
			period_ms *= osc_calibrate_val;
		}

		writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms,tof_chosen);

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

		writeReg(SYSRANGE_START, 0x04,tof_chosen); // VL53L0X_REG_SYSRANGE_MODE_TIMED
	}
	else
	{
		// continuous back-to-back mode
		writeReg(SYSRANGE_START, 0x02,tof_chosen); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	}
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void stopContinuous(uint8_t tof_chosen)
{
	writeReg(SYSRANGE_START, 0x01,tof_chosen); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x00, 0x00,tof_chosen);
	writeReg(0x91, 0x00,tof_chosen);
	writeReg(0x00, 0x01,tof_chosen);
	writeReg(0xFF, 0x00,tof_chosen);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
// extraStats provides additional info for this measurment. Set to 0 if not needed.
uint16_t readRangeContinuousMillimeters( statInfo_t_VL53L0X *extraStats,uint8_t tof_chosen ) {
	uint8_t tempBuffer[12];
	uint16_t temp;
	startTimeout();
	while ((readReg(RESULT_INTERRUPT_STATUS,tof_chosen) & 0x07) == 0) {
		if (checkTimeoutExpired())
		{
			g_isTimeout = true;
			return 65535;
		}
	}
	if( extraStats == 0 ){
		// assumptions: Linearity Corrective Gain is 1000 (default);
		// fractional ranging is not enabled
		temp = readReg16Bit(RESULT_RANGE_STATUS + 10,tof_chosen);
	} else {
		// Register map starting at 0x14
		//     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
		//    5A 06 BC 04 00 85 00 38 00 19 06 B6 00 00 00 00
		//   0: Ranging status, uint8_t
		//   1: ???
		// 3,2: Effective SPAD return count, uint16_t, fixpoint8.8
		//   4: 0 ?
		//   5: ???
		// 6,7: signal count rate [mcps], uint16_t, fixpoint9.7
		// 9,8: AmbientRateRtnMegaCps  [mcps], uint16_t, fixpoimt9.7
		// A,B: uncorrected distance [mm], uint16_t
		readMulti(0x14, tempBuffer, 12,tof_chosen);
		extraStats->rangeStatus =  tempBuffer[0x00]>>3;
		extraStats->spadCnt     = (tempBuffer[0x02]<<8) | tempBuffer[0x03];
		extraStats->signalCnt   = (tempBuffer[0x06]<<8) | tempBuffer[0x07];
		extraStats->ambientCnt  = (tempBuffer[0x08]<<8) | tempBuffer[0x09];
		temp                    = (tempBuffer[0x0A]<<8) | tempBuffer[0x0B];
		extraStats->rawDistance = temp;
	}
	writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01,tof_chosen);
	return temp;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
// extraStats provides additional info for this measurment. Set to 0 if not needed.
uint16_t readRangeSingleMillimeters( statInfo_t_VL53L0X *extraStats ,uint8_t tof_chosen) {
	writeReg(0x80, 0x01,tof_chosen);
	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x00, 0x00,tof_chosen);
	writeReg(0x91, g_stopVariable,tof_chosen);
	writeReg(0x00, 0x01,tof_chosen);
	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x80, 0x00,tof_chosen);
	writeReg(SYSRANGE_START, 0x01,tof_chosen);
	// "Wait until start bit has been cleared"
	startTimeout();
	while (readReg(SYSRANGE_START,tof_chosen) & 0x01){
		if (checkTimeoutExpired()){
			g_isTimeout = true;
			return 65535;
		}
	}
	return readRangeContinuousMillimeters( extraStats,tof_chosen );
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool timeoutOccurred()
{
	bool tmp = g_isTimeout;
	g_isTimeout = false;
	return tmp;
}

void setTimeout(uint16_t timeout){
	g_ioTimeout = timeout;
}

uint16_t getTimeout(void){
	return g_ioTimeout;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool getSpadInfo(uint8_t * count, bool * type_is_aperture,uint8_t tof_chosen)
{
	uint8_t tmp;

	writeReg(0x80, 0x01,tof_chosen);
	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x00, 0x00,tof_chosen);

	writeReg(0xFF, 0x06,tof_chosen);
	writeReg(0x83, readReg(0x83,tof_chosen) | 0x04,tof_chosen);
	writeReg(0xFF, 0x07,tof_chosen);
	writeReg(0x81, 0x01,tof_chosen);

	writeReg(0x80, 0x01,tof_chosen);

	writeReg(0x94, 0x6b,tof_chosen);
	writeReg(0x83, 0x00,tof_chosen);
	startTimeout();
	while (readReg(0x83,tof_chosen) == 0x00)
	{
		if (checkTimeoutExpired()) { return false; }
	}
	writeReg(0x83, 0x01,tof_chosen);
	tmp = readReg(0x92,tof_chosen);

	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;

	writeReg(0x81, 0x00,tof_chosen);
	writeReg(0xFF, 0x06,tof_chosen);
	writeReg(0x83, readReg(0x83,tof_chosen)  & ~0x04,tof_chosen);
	writeReg(0xFF, 0x01,tof_chosen);
	writeReg(0x00, 0x01,tof_chosen);

	writeReg(0xFF, 0x00,tof_chosen);
	writeReg(0x80, 0x00,tof_chosen);

	return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void getSequenceStepEnables(SequenceStepEnables * enables,uint8_t tof_chosen)
{
	uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG,tof_chosen);

	enables->tcc          = (sequence_config >> 4) & 0x1;
	enables->dss          = (sequence_config >> 3) & 0x1;
	enables->msrc         = (sequence_config >> 2) & 0x1;
	enables->pre_range    = (sequence_config >> 6) & 0x1;
	enables->final_range  = (sequence_config >> 7) & 0x1;
}
uint8_t initVXL_right()
{
	uint8_t tof_chosen=TOF_RIGHT;
	initVL53L0X(1,&hi2c2,tof_chosen);
	setSignalRateLimit(20,tof_chosen);
	setVcselPulsePeriod(VcselPeriodPreRange, 10,tof_chosen);
	setVcselPulsePeriod(VcselPeriodFinalRange, 14,tof_chosen);
	setMeasurementTimingBudget(300 * 1000UL,tof_chosen);
	return 0;

}
uint8_t initVXL_left()
{
	uint8_t tof_chosen=TOF_LEFT;
	initVL53L0X(1,&hi2c1,tof_chosen);// a changer en fonction de l'i2c qu'on a
	setSignalRateLimit(20,tof_chosen);
	setVcselPulsePeriod(VcselPeriodPreRange, 10,tof_chosen);
	setVcselPulsePeriod(VcselPeriodFinalRange, 14,tof_chosen);
	setMeasurementTimingBudget(300 * 1000UL,tof_chosen);
	return 0;


}
uint8_t initVXL_tofs(tofs_t *tofs)
{
	HAL_GPIO_WritePin(TOF1_XSHUT_GPIO_Port, TOF1_XSHUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TOF2_XSHUT_GPIO_Port, TOF2_XSHUT_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(TOF1_GPIO1_GPIO_Port, TOF1_GPIO1_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(TOF2_GPIO1_GPIO_Port, TOF2_GPIO1_Pin, GPIO_PIN_RESET);
	initVXL_left();
	initVXL_right();
	tofs->left.drv_tof.readRangeSingleMillimeters=readRangeSingleMillimeters;
	tofs->right.drv_tof.readRangeSingleMillimeters=readRangeSingleMillimeters;
	return 1;
}



// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts,uint8_t tof_chosen)
{
	timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange,tof_chosen);

	timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP,tof_chosen) + 1;
	timeouts->msrc_dss_tcc_us =
			timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
					timeouts->pre_range_vcsel_period_pclks);

	timeouts->pre_range_mclks =
			decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,tof_chosen));
	timeouts->pre_range_us =
			timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
					timeouts->pre_range_vcsel_period_pclks);

	timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange,tof_chosen);

	timeouts->final_range_mclks =
			decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,tof_chosen));

	if (enables->pre_range)
	{
		timeouts->final_range_mclks -= timeouts->pre_range_mclks;
	}

	timeouts->final_range_us =
			timeoutMclksToMicroseconds(timeouts->final_range_mclks,
					timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t decodeTimeout(uint16_t reg_val)
{
	// format: "(LSByte * 2^MSByte) + 1"
	return (uint16_t)((reg_val & 0x00FF) <<
			(uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t encodeTimeout(uint16_t timeout_mclks)
{
	// format: "(LSByte * 2^MSByte) + 1"

	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_mclks > 0)
	{
		ls_byte = timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0)
		{
			ls_byte >>= 1;
			ms_byte++;
		}

		return (ms_byte << 8) | (ls_byte & 0xFF);
	}
	else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
bool performSingleRefCalibration(uint8_t vhv_init_byte,uint8_t tof_chosen)
{
	writeReg(SYSRANGE_START, 0x01 | vhv_init_byte,tof_chosen); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

	startTimeout();
	while ((readReg(RESULT_INTERRUPT_STATUS,tof_chosen) & 0x07) == 0)
	{
		if (checkTimeoutExpired()) { return false; }
	}

	writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01,tof_chosen);

	writeReg(SYSRANGE_START, 0x00,tof_chosen);

	return true;
}
