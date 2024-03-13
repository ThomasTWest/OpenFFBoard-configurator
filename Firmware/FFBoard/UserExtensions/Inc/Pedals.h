/*
 * ADS111X.h
 *
 *  Created on: 21.04.2022
 *      Author: Yannick
 */

#ifndef USEREXTENSIONS_SRC_PEDALS_H_
#define USEREXTENSIONS_SRC_PEDALS_H_

#include "I2C.h"
#include "CommandHandler.h"
#include "AnalogSource.h"
#include "PersistentStorage.h"
#include "thread.hpp"
#include "cpp_target_config.h"
#include "target_constants.h"
#include "AnalogAxisProcessing.h"
#include "Filters.h"

#include "I2C_BitBang.hpp"
#include "Tle493d.hpp"
#include "SimpleKalmanFilter.h"


#ifdef PEDALS




struct PedalConf{
	int16_t input_min				= 0;
	int16_t input_max				= 32767;
	uint16_t interpolation_interval = 6553;	//0, 6553, 13106, 19659, 26212, 32765
	int16_t normalization_offset	= 0;
	int16_t normalization_factor	= 1;
	int16_t bottom_deadzone			= 0;
	int16_t top_deadzone			= 0;

	uint16_t kalman_meas_error		= 0;	//Remember this must be 100 times bigger than the parameter in the kalman filter (float)
	uint16_t kalman_proc_noise		= 0;//Remember this must be 100 times bigger than the parameter in the kalman filter (float)
};

struct st_PedalData {
	int16_t RawValue = 0;
	int16_t NormalizedValue = 0;
};

typedef enum {Throttle, Brake, Clutch} PEDALS_Name;

// Internal struct for storing Interpolation Points
#define MAX_INTERPOLATIONPOINTS 6

struct LocalPedalsConfig{
	uint8_t pedalsmask = 0xff;
};
#include "Filters.h"
struct InterpolationPair
{
	int32_t x;	//Inputs
	int32_t y;	//Outputs
} ;

// Structure definition
struct table_1d {
    uint8_t x_length;

    float *x_values;
    float *y_values;
};

class PEDALS_AnalogSource : public I2C_BitBang, public AnalogSource, public CommandHandler, public AnalogAxisProcessing, public ErrorHandler, public cpp_freertos::Thread {
	enum class PEDALS_AnalogSource_commands : uint32_t {
		pedalmask, pedals, cmd_rawval, cmd_normval, cmd_pedal_min, cmd_pedal_max, cmd_ipp_x, cmd_ipp_y, cmd_auto_interval, cmd_norm_offset,
		cmd_botdeadzone, cmd_topdeadzone, cmd_filter, cmd_kalmeas, cmd_kalnoise
	};

public:
	PEDALS_AnalogSource(uint8_t instance = 0); //const char* name
	~PEDALS_AnalogSource();
	const virtual ClassIdentifier getInfo();
	static ClassIdentifier info;
	//static bool isCreatable() {return true;};
	std::string getHelpstring(){return "Pedals: Throttle, Brake, Clutch - analog source";}
	const ClassType getClassType() override {return ClassType::Analogsource;};
	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies);

	void Run();

	void saveFlash();
	void restoreFlash();

	std::vector<int32_t>* getAxes();

private:
	const Error saveflashError = Error(ErrorCode::pedalsError, ErrorType::critical, "Configuration NOT saved");
	const Error restoreflashError = Error(ErrorCode::pedalsError, ErrorType::critical, "Configuration NOT restored");

	uint8_t PedalInstance = 0;

	LocalPedalsConfig pconf;

	const uint8_t numPedals = 1;
	//uint8_t pedalenmask = 0xff;
	//int16_t PedalsDataBuffer[3] = {0, 0, 0};
	st_PedalData PedalsDataBuffer;

	void setThrottleInterPolationPoint_X (uint8_t idx, int16_t val);
	void setThrottleInterPolationPoint_Y (uint8_t idx, int16_t val);

	void setupKalmanFilter(void);

	std::vector<InterpolationPair> InterpolationVals;

	PedalConf PedalConfiguration;
	//void setLimits(TMC4671Limits limits);
	//TMC4671Limits getLimits();

	int16_t sampleBuffer;

	I2C_BitBang I2C_Port;
	Tle493d MagSensor = Tle493d(I2C_Port);


	uint8_t brake_chan{6};
	uint8_t error = 0;

protected:
	Biquad filter;  // Create a single Biquad filter instance
	std::vector<Biquad> filters; // Optional filters
	float filterF = 30.0/1000.0 , filterQ = 0.5;
	uint32_t filterSamples = 0;
	const uint32_t waitFilterSamples = 500;


	SimpleKalmanFilter KalmanFilter;


};



class Pedal_Throttle : public PEDALS_AnalogSource {
public:
	Pedal_Throttle()
		: PEDALS_AnalogSource {0}{}

	const ClassIdentifier getInfo() override;
	static ClassIdentifier info;
	static bool isCreatable();
	static bool exists;
};

class Pedal_Brake : public PEDALS_AnalogSource {
public:
	Pedal_Brake()
		: PEDALS_AnalogSource {1}{}

	const ClassIdentifier getInfo() override;
	static ClassIdentifier info;
	static bool isCreatable();
	static bool exists;
};

#endif
#endif /* USEREXTENSIONS_SRC_PEDALS_H_ */

