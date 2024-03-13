/*
 * ADS111X.cpp
 *
 *  Created on: 21.04.2022
 *      Author: Yannick
 */

#include "Pedals.h"

// library containing trunc function
#include <math.h>

#include "printf.h"
#include "SimpleKalmanFilter.h"


// ---------------------------------------------------------------------- //
#ifdef PEDALS


ClassIdentifier Pedal_Throttle::info = {
	.name = "Throttle",
	.id=CLSID_PEDAL_THROTTLE, // 1
};

const ClassIdentifier Pedal_Throttle::getInfo(){
	return info;
}


bool Pedal_Throttle::isCreatable() {
	return true;
}

ClassIdentifier Pedal_Brake::info = {
	.name = "Brake",
	.id=CLSID_PEDAL_BRAKE, // 1
};

const ClassIdentifier Pedal_Brake::getInfo(){
	return info;
}


bool Pedal_Brake::isCreatable() {
	return true;
}


ClassIdentifier PEDALS_AnalogSource::info = {
		 .name = "Pedals" ,
		 .id=CLSID_ANALOG_PEDALS,
 };
const ClassIdentifier PEDALS_AnalogSource::getInfo(){
	return info;
}
//AnalogAxisProcessing(const uint32_t axisAmount,AnalogSource* analogSource,CommandHandler* cmdHandler,bool allowFilters,bool allowAutoscale,bool allowRawValues,bool allowManualScale) : axisAmount(axisAmount),analogSource(analogSource), modes({allowFilters,allowAutoscale,allowRawValues,allowManualScale}){
//PEDALS_AnalogSource::PEDALS_AnalogSource() : AD_PEDALS(i2cport) , CommandHandler("pedals", CLSID_ANALOG_PEDALS, 0),AnalogAxisProcessing(3,this,this, false,false,true,true), Thread("pedals", 64, 25) {
PEDALS_AnalogSource::PEDALS_AnalogSource(uint8_t instance) : CommandHandler("pedals", CLSID_ANALOG_PEDALS, 0) ,AnalogAxisProcessing(1,this,this, false,false,true,false), Thread("pedals", 256, 25) {


	this->PedalInstance = instance;
	setInstance(instance); //Throttle will always have instance '0', Brake = '1', Clutch = '2'

	InterpolationVals.resize(MAX_INTERPOLATIONPOINTS); // Create the memory array for the Interpolation pairs

	restoreFlash();


	// Initialize filters
	this->setupKalmanFilter();
	//Throttle
	//KalmanFilter.setMeasurementError(120.0);
	//KalmanFilter.setEstimateError(120.0);
	//KalmanFilter.setProcessNoise(0.1);

	//KalmanFilter.setMeasurementError(20.0);
	//KalmanFilter.setEstimateError(20.0);
	//KalmanFilter.setProcessNoise(0.3);


	I2C_BitBang::i2c_gpio_init(); //Set I2C pins to GPIOs for SW I2C Bitbang mode

	I2C_BitBang I2C_Port;
	Tle493d MagSensor = Tle493d(I2C_Port);

	MagSensor.Initialize();

	CommandHandler::registerCommands();
	registerCommand("mask", PEDALS_AnalogSource_commands::pedalmask, "Enabled pedals",CMDFLAG_GET|CMDFLAG_SET);
	registerCommand("pedals", PEDALS_AnalogSource_commands::pedals, "Available pedals",CMDFLAG_GET|CMDFLAG_SET);

	registerCommand("prawval", PEDALS_AnalogSource_commands::cmd_rawval, "Pedal sensor value", CMDFLAG_GET);
	registerCommand("normval", PEDALS_AnalogSource_commands::cmd_normval, "Pedal normalized sensor value", CMDFLAG_GET);
	registerCommand("pmin", PEDALS_AnalogSource_commands::cmd_pedal_min, "Min pedal value", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("pmax", PEDALS_AnalogSource_commands::cmd_pedal_max, "Max pedal value", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("norm", PEDALS_AnalogSource_commands::cmd_norm_offset, "Normalize the pedal input range", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("auto", PEDALS_AnalogSource_commands::cmd_auto_interval, "Auto cal interpolation intervals", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("ippx", PEDALS_AnalogSource_commands::cmd_ipp_x, "Interpolation Points Input (X axis)", CMDFLAG_SETADR | CMDFLAG_GETADR | CMDFLAG_GET);
	registerCommand("ippy", PEDALS_AnalogSource_commands::cmd_ipp_y, "Interpolation Points Output (Y axis)", CMDFLAG_SETADR | CMDFLAG_GETADR | CMDFLAG_GET);
	registerCommand("botdz", PEDALS_AnalogSource_commands::cmd_botdeadzone, "Bottom deadzone", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("topdz", PEDALS_AnalogSource_commands::cmd_topdeadzone, "Top deadzone", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("pfilter", PEDALS_AnalogSource_commands::cmd_filter, "Enable lowpass filters", CMDFLAG_GET|CMDFLAG_SET);
	registerCommand("kalmeas", PEDALS_AnalogSource_commands::cmd_kalmeas, "Set Kalman Measurement Error (x*100)", CMDFLAG_GET|CMDFLAG_SET);
	registerCommand("kalnoise", PEDALS_AnalogSource_commands::cmd_kalnoise, "Set Kalman Process Noise (x*100)", CMDFLAG_GET|CMDFLAG_SET);

	this->Start();
}

PEDALS_AnalogSource::~PEDALS_AnalogSource(){

}

void PEDALS_AnalogSource::saveFlash(){

	uint16_t FlashAddress = ADR_PEDAL_1_STADDR;	//

	if (this->PedalInstance == Throttle) {
		FlashAddress = ADR_PEDAL_1_STADDR;
	}
	else if (this->PedalInstance == Brake) {
		FlashAddress = ADR_PEDAL_2_STADDR;
	}
	/*else if (this->PedalInstance == Clutch) {
		FlashAddress = ADR_PEDAL_3_STADDR;
	}*/
	else {
		ErrorHandler::addError(saveflashError);
		return;
	}

	//ADR_PEDAL_x_CONF 0x6?0
	uint16_t processingConf = AnalogAxisProcessing::encodeAnalogProcessingConfToInt(AnalogAxisProcessing::getAnalogProcessingConfig());
	uint16_t conf1 = pconf.pedalsmask | (processingConf << 8);
	Flash_Write(FlashAddress, conf1);

	FlashAddress ++;
	//#define ADR_PEDAL_1_NORM 	  0x601
	Flash_Write(FlashAddress, (uint16_t)(this->PedalConfiguration.normalization_offset + 0x8000));
	printf("\r\nNorm %d", this->PedalConfiguration.normalization_offset);

	FlashAddress ++;
	//	#define ADR_PEDAL_1_PMIN 	  0x602
	// The limit value can be negative. Therefore is 32768 added.
	// Range Limitations: Ensure your signed values fit within the representable range of 16-bit unsigned integers (0 to 65535).
	Flash_Write(FlashAddress, (uint16_t)(this->PedalConfiguration.input_min + 0x8000));

	FlashAddress ++;
	//#define ADR_PEDAL_1_PMAX 	  0x603
	// The limit value can be negative. Therefore is 32768 added.
	// Range Limitations: Ensure your signed values fit within the representable range of 16-bit unsigned integers (0 to 65535).
	Flash_Write(FlashAddress, (uint16_t)(this->PedalConfiguration.input_max + 0x8000));

	//#define ADR_PEDAL_1_IPP_X_0   0x604
	FlashAddress ++;
	for(uint8_t i = 0; i < MAX_INTERPOLATIONPOINTS; i++){
		Flash_Write(FlashAddress, (uint16_t)this->InterpolationVals[i].x);
		FlashAddress ++;
		Flash_Write(FlashAddress, (uint16_t)this->InterpolationVals[i].y);
		FlashAddress ++;
	}
	Flash_Write(FlashAddress, (uint16_t)this->PedalConfiguration.bottom_deadzone);
	printf("\r\nBot DZ 0x%x  %d",FlashAddress, this->PedalConfiguration.bottom_deadzone);
	FlashAddress ++;
	Flash_Write(FlashAddress, (uint16_t)this->PedalConfiguration.top_deadzone);
	//Kalman filter
	FlashAddress ++;
	Flash_Write(FlashAddress, (uint16_t)this->PedalConfiguration.kalman_meas_error);
	FlashAddress ++;
	Flash_Write(FlashAddress, (uint16_t)this->PedalConfiguration.kalman_proc_noise);
}

void PEDALS_AnalogSource::restoreFlash(){
	uint16_t FlashAddress = ADR_PEDAL_1_STADDR;	//
	uint16_t tmp = 0;

	if (this->PedalInstance == Throttle) {
		FlashAddress = ADR_PEDAL_1_STADDR;
	}
	else if (this->PedalInstance == Brake) {
		FlashAddress = ADR_PEDAL_2_STADDR;
	}
	/*else if (this->PedalInstance == Clutch) {
		FlashAddress = ADR_PEDAL_3_STADDR;
	}*/
	else {
		ErrorHandler::addError(restoreflashError);
		return;
	}

	uint16_t pconfint;
	if(Flash_Read(FlashAddress, &pconfint)){
		AnalogAxisProcessing::setAnalogProcessingConfig(AnalogAxisProcessing::decodeAnalogProcessingConfFromInt(pconfint >> 8));
		pconf.pedalsmask = pconfint & 0xff;
	}

	FlashAddress ++;
	//#define ADR_PEDAL_1_NORM 	  0x601
	Flash_Read(FlashAddress, &tmp);
	this->PedalConfiguration.normalization_offset = (tmp - 0x8000);
	printf("\r\nRD Norm %d", this->PedalConfiguration.normalization_offset);

	FlashAddress ++;
	//	#define ADR_PEDAL_1_PMIN 	  0x602
	//Flash_Read(FlashAddress, &this->PedalConfiguration.input_min);
	Flash_Read(FlashAddress, &tmp);
	this->PedalConfiguration.input_min = (tmp - 0x8000);

	FlashAddress ++;
	//#define ADR_PEDAL_1_PMAX 	  0x603
	//Flash_Read(FlashAddress, &this->PedalConfiguration.input_max);
	Flash_Read(FlashAddress, &tmp);
	this->PedalConfiguration.input_max = (tmp - 0x8000);


	//#define ADR_PEDAL_1_IPP_X_0   0x604
	FlashAddress ++;
	for(uint8_t i = 0; i < MAX_INTERPOLATIONPOINTS; i++){
		uint16_t bufX,bufY;
		if(Flash_Read(FlashAddress, &bufX) && Flash_Read(FlashAddress+1, &bufY)){
			this->InterpolationVals[i].x = (int16_t)bufX;
			this->InterpolationVals[i].y = (int16_t)bufY;
			FlashAddress = FlashAddress + 2;
		}
	}

	Flash_Read(FlashAddress, &tmp);
	this->PedalConfiguration.bottom_deadzone = tmp;
	printf("\r\nRD Bot DZ 0x%x  %d",FlashAddress, this->PedalConfiguration.bottom_deadzone);
	FlashAddress ++;
	Flash_Read(FlashAddress, &tmp);
	this->PedalConfiguration.top_deadzone = tmp;
	//Kalman filter
	FlashAddress ++;
	Flash_Read(FlashAddress, &tmp);
	this->PedalConfiguration.kalman_meas_error = tmp;
	printf("\r\nRD Kalman 0x%x  %d",FlashAddress, this->PedalConfiguration.kalman_meas_error);
	FlashAddress ++;
	Flash_Read(FlashAddress, &tmp);
	this->PedalConfiguration.kalman_proc_noise = tmp;
	printf("  %d", this->PedalConfiguration.kalman_proc_noise);

}

//Segment interpolation
// Returns the interpolated y-value.
// Saturates to y0 or y1 if x outside interval [x0, x1].
float interpolate_segment(float x0, float y0, float x1, float y1, float x)
{
    float t;

    if (x <= x0) { return y0; }
    if (x >= x1) { return y1; }

    t =  (x-x0);
    t /= (x1-x0);

    return y0 + t*(y1-y0);
}

/*static struct table_1d throttle_table = {
    5,      // Number of data points
	throttle_x, // Array of x-coordinates
	throttle_y  // Array of y-coordinates
};*/


float interpolate_table_1d(struct table_1d *table, float x)
/* 1D Table lookup with interpolation */
{
    uint8_t segment;

    /* Check input bounds and saturate if out-of-bounds */
    if (x > (table->x_values[table->x_length-1])) {
       /* x-value too large, saturate to max y-value */
        return table->y_values[table->x_length-1];
    }
    else if (x < (table->x_values[0])) {
       /* x-value too small, saturate to min y-value */
        return table->y_values[0];
    }

    /* Find the segment that holds x */
    for (segment = 0; segment<(table->x_length-1); segment++)
    {
        if ((table->x_values[segment]   <= x) &&
            (table->x_values[segment+1] >= x))
        {
            /* Found the correct segment */
            /* Interpolate */
            return interpolate_segment(table->x_values[segment],   /* x0 */
                                       table->y_values[segment],   /* y0 */
                                       table->x_values[segment+1], /* x1 */
                                       table->y_values[segment+1], /* y1 */
                                       x);                         /* x  */
        }
    }

    /* Something with the data was wrong if we get here */
    /* Saturate to the max value */
    return table->y_values[table->x_length-1];
}


int32_t interpolate_table_1d_int32(std::vector<InterpolationPair>& table, int32_t x)
/* 1D Table lookup with interpolation */
{
	uint8_t table_length = 6;
    uint8_t segment;

    /* Check input bounds and saturate if out-of-bounds */
    if (x > table[table_length-1].x) {
       /* x-value too large, saturate to max y-value */
        return table[table_length-1].y;
    }
    else if (x < (table[0].x)) {
       /* x-value too small, saturate to min y-value */
        return table[0].y;
    }

    /* Find the segment that holds x */
    for (segment = 0; segment<(table_length-1); segment++)
    {
        if ((table[segment].x   <= x) &&
            (table[segment+1].x >= x))
        {
            /* Found the correct segment */
            /* Interpolate */
            return interpolate_segment(table[segment].x,   /* x0 */
                                       table[segment].y,   /* y0 */
                                       table[segment+1].x, /* x1 */
                                       table[segment+1].y, /* y1 */
                                       x);                         /* x  */
        }
    }

    /* Something with the data was wrong if we get here */
    /* Saturate to the max value */
    return table[table_length-1].y;
}

int32_t old_tmp = 0;
// Handles starting next transfer for >1 byte transfers
void PEDALS_AnalogSource::Run() {
	uint8_t chans = 0;

	int32_t tmp = 0;
	int32_t filtered = 0;
	float rawval = 0.0;
	float estimated_value = 0.0;
	while(true) {
		//ToDo code....
		if (this->PedalInstance == Throttle) {
			this->error += MagSensor.updateData();
			rawval = this->MagSensor.getAzimuth();
			tmp = trunc(rawval * ((180.0/3.14)+42.0)*100.0);

			// calculate the estimated value with Kalman Filter
			estimated_value = KalmanFilter.updateEstimate(tmp);
			/*
			printf("%.4f", rawval);
			printf(",%.4f", estimated_value);
			printf("\r\n");*/
			tmp = estimated_value;
		}
		else if (this->PedalInstance == Brake) {
			//tmp = 0;
			volatile uint32_t* adcbuf = getAnalogBuffer(&AIN_HADC,&chans);
			//tmp = ((adcbuf[ADC_CHAN_FPIN+1] & 0xFFF) << 4)-0x7fff;
			tmp = adcbuf[ADC_CHAN_FPIN+brake_chan-1];
			if (tmp != old_tmp) {
				old_tmp = tmp;
				//printf("\r\nBrake %d", tmp);
			}
			// calculate the estimated value with Kalman Filter
			estimated_value = KalmanFilter.updateEstimate(tmp);

/*			printf("%.d", tmp);
			printf(",%.4f", estimated_value);
			printf("\r\n");*/
			tmp = estimated_value;

		}

		this->PedalsDataBuffer.RawValue = (int16_t)tmp;


		this->Delay(100);
	}
}

std::vector<int32_t>* PEDALS_AnalogSource::getAxes(){
	this->buf.clear();
	/*
	volatile int16_t* pedalbuf = PedalsDataBuffer;

	for(uint8_t i = 0; i<numPedals; i++){
		int32_t val = ((pedalbuf[i]));
		val = interpolate_table_1d_int32(this->InterpolationVals, val);
		if(!(pconf.pedalsmask & 0x01 << i))
			continue;

		//val =  trunc(interpolate_table_1d(&throttle_table, (float)val));
		this->buf.push_back(val);
	}
	*/
	int32_t val = PedalsDataBuffer.RawValue + this->PedalConfiguration.normalization_offset;
	PedalsDataBuffer.NormalizedValue = (int16_t)val;

	val = interpolate_table_1d_int32(this->InterpolationVals, val);
	this->buf.push_back(val);

	AnalogAxisProcessing::processAxes(buf);
	return &this->buf;
}

void PEDALS_AnalogSource::setupKalmanFilter(){
	float MeasError = float(this->PedalConfiguration.kalman_meas_error) / 100.0;
	float ProcessNoise = float(this->PedalConfiguration.kalman_proc_noise) / 100.0;

	printf("\r\nsetupKalmanFilter ME = %.4f   PN = %.4f", MeasError, ProcessNoise);

	KalmanFilter.setMeasurementError(MeasError);
	KalmanFilter.setEstimateError(MeasError);	// Set to the same as measurement errors, will change over time
	KalmanFilter.setProcessNoise(ProcessNoise);
}



CommandStatus PEDALS_AnalogSource::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){
	uint16_t pedal_range = 0;
	uint32_t tmp_deadzone = 0;
	uint16_t pedal_bottom_range = 0;
	uint16_t pedal_top_range = 0;

	switch(static_cast<PEDALS_AnalogSource_commands>(cmd.cmdId)){


	case PEDALS_AnalogSource_commands::pedalmask:
		printf("\r\nMask %d", this->pconf.pedalsmask);
		return handleGetSet(cmd, replies, this->pconf.pedalsmask);
	break;

	case PEDALS_AnalogSource_commands::pedals:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(numPedals);
		}else{
			return CommandStatus::ERR;
		}
	break;
	case PEDALS_AnalogSource_commands::cmd_rawval:
		return handleGetSet(cmd, replies, this->PedalsDataBuffer.RawValue);
	break;

	case PEDALS_AnalogSource_commands::cmd_normval:
		return handleGetSet(cmd, replies, this->PedalsDataBuffer.NormalizedValue);
	break;


	case PEDALS_AnalogSource_commands::cmd_pedal_min:
		return handleGetSet(cmd, replies, this->PedalConfiguration.input_min);
	break;

	case PEDALS_AnalogSource_commands::cmd_pedal_max:
		return handleGetSet(cmd, replies, this->PedalConfiguration.input_max);
	break;

	case PEDALS_AnalogSource_commands::cmd_norm_offset:
		return handleGetSet(cmd, replies, this->PedalConfiguration.normalization_offset);
	break;

	case PEDALS_AnalogSource_commands::cmd_botdeadzone:
		return handleGetSet(cmd, replies, this->PedalConfiguration.bottom_deadzone);
	break;

	case PEDALS_AnalogSource_commands::cmd_topdeadzone:
		return handleGetSet(cmd, replies, this->PedalConfiguration.top_deadzone);
	break;

	case PEDALS_AnalogSource_commands::cmd_filter:

		if(cmd.type == CMDtype::set){
			this->filterF = float(cmd.val) / 1000.0;
			filter.setFc(this->filterF);
		}
		else if(cmd.type == CMDtype::get){
			replies.emplace_back(trunc(this->filterF * 1000.0));
		}
	break;

	case PEDALS_AnalogSource_commands::cmd_kalmeas:
		if(cmd.type == CMDtype::set){
			this->PedalConfiguration.kalman_meas_error = cmd.val;
			this->setupKalmanFilter();
		}
		else if(cmd.type == CMDtype::get){
			replies.emplace_back(this->PedalConfiguration.kalman_meas_error);
		}
	break;

	case PEDALS_AnalogSource_commands::cmd_kalnoise:
		if(cmd.type == CMDtype::set){
			this->PedalConfiguration.kalman_proc_noise = cmd.val;
			this->setupKalmanFilter();
		}
		else if(cmd.type == CMDtype::get){
			replies.emplace_back(this->PedalConfiguration.kalman_proc_noise);
		}
	break;


	case PEDALS_AnalogSource_commands::cmd_auto_interval:

		printf("\r\n-----");
		printf("\r\nAuto %d  %d  %d",this->PedalConfiguration.input_max, this->PedalConfiguration.input_min, this->PedalConfiguration.normalization_offset);

		pedal_range = ((this->PedalConfiguration.input_max + this->PedalConfiguration.normalization_offset) - (this->PedalConfiguration.input_min + this->PedalConfiguration.normalization_offset));
		printf("\r\nPRange %d",pedal_range);


		tmp_deadzone = ((uint32_t)pedal_range * (uint32_t)this->PedalConfiguration.bottom_deadzone) / 100;
		printf("\r\nBDead %d  %d",tmp_deadzone, this->PedalConfiguration.bottom_deadzone);
		pedal_bottom_range = (this->PedalConfiguration.input_min + this->PedalConfiguration.normalization_offset) + tmp_deadzone;
		printf("\r\nBot %d",pedal_bottom_range);

		tmp_deadzone = ((uint32_t)pedal_range * (uint32_t)this->PedalConfiguration.top_deadzone) / 100;
		printf("\r\nTDead %d  %d",tmp_deadzone, this->PedalConfiguration.top_deadzone);
		pedal_top_range = (this->PedalConfiguration.input_max + this->PedalConfiguration.normalization_offset) - tmp_deadzone;
		printf("\r\nTop %d",pedal_top_range);

		if ((this->PedalConfiguration.input_max+5) > this->PedalConfiguration.input_min) {

			uint16_t tmp_interval = (pedal_top_range - pedal_bottom_range) / 5;
			printf("\r\nInterval %d", tmp_interval);
			//this->InterpolationVals[0].x = (this->PedalConfiguration.input_min + (this->PedalConfiguration.normalization_offset));
			this->InterpolationVals[0].x = pedal_bottom_range;
			printf ("\r\nint[%d] %d",0 ,this->InterpolationVals[0].x);
			for (uint8_t i = 1; i<(MAX_INTERPOLATIONPOINTS); i++) {
				this->InterpolationVals[i].x = this->InterpolationVals[0].x + (i * tmp_interval);
				printf ("\r\nint[%d] %d",i ,this->InterpolationVals[i].x);
			}
			replies.emplace_back(tmp_interval);
		}
		else {
			return CommandStatus::ERR;
		}

	break;

	case PEDALS_AnalogSource_commands::cmd_ipp_x:

		if(cmd.type == CMDtype::setat && cmd.adr < MAX_INTERPOLATIONPOINTS){
			this->InterpolationVals[cmd.adr].x = cmd.val;
		} else if(cmd.type == CMDtype::getat && cmd.adr < MAX_INTERPOLATIONPOINTS){
			replies.emplace_back(this->InterpolationVals[cmd.adr].x);
		} else if(cmd.type == CMDtype::get) {
			for (size_t i=0; i < MAX_INTERPOLATIONPOINTS; i++) {
				replies.emplace_back(this->InterpolationVals[i].x);
			}
		}
		else{
			return CommandStatus::ERR;
		}

	break;

	case PEDALS_AnalogSource_commands::cmd_ipp_y:
		if(cmd.type == CMDtype::setat && cmd.adr < MAX_INTERPOLATIONPOINTS){
			this->InterpolationVals[cmd.adr].y = cmd.val;
		} else if(cmd.type == CMDtype::getat && cmd.adr < MAX_INTERPOLATIONPOINTS){
			replies.emplace_back(this->InterpolationVals[cmd.adr].y);
		} else if(cmd.type == CMDtype::get) {
			for (size_t i=0; i < MAX_INTERPOLATIONPOINTS; i++) {
				replies.emplace_back(this->InterpolationVals[i].y);
			}
		}
		else{
			return CommandStatus::ERR;
		}

	break;

	default:
		return AnalogAxisProcessing::command(cmd, replies); // Try processing command
	}
	return CommandStatus::OK;
}
#endif

