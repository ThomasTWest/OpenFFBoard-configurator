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
PEDALS_AnalogSource::PEDALS_AnalogSource(uint8_t instance) : CommandHandler("pedals", CLSID_ANALOG_PEDALS, 0) ,AnalogAxisProcessing(1,this,this, false,false,true,false), Thread("pedals", 64, 25) {

	printf ("\r\nPedal = %d", instance);
	this->PedalInstance = instance;
	setInstance(instance); //Throttle will always have instance '0', Brake = '1', Clutch = '2'

	InterpolationVals.resize(MAX_INTERPOLATIONPOINTS); // Create the memory array for the Interpolation pairs

	restoreFlash();

	CommandHandler::registerCommands();
	registerCommand("mask", PEDALS_AnalogSource_commands::pedalmask, "Enabled pedals",CMDFLAG_GET|CMDFLAG_SET);
	registerCommand("pedals", PEDALS_AnalogSource_commands::pedals, "Available pedals",CMDFLAG_GET|CMDFLAG_SET);

	registerCommand("throttle", PEDALS_AnalogSource_commands::cmd_throttle, "Throttle value", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("pmin", PEDALS_AnalogSource_commands::cmd_pedal_min, "Min pedal value", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("pmax", PEDALS_AnalogSource_commands::cmd_pedal_max, "Max pedal value", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("auto", PEDALS_AnalogSource_commands::cmd_auto_interval, "Auto cal interpolation intervals", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("ipptx", PEDALS_AnalogSource_commands::cmd_ipp_throttle_x, "Throttle Interpolation Points Input (X axis)", CMDFLAG_SETADR | CMDFLAG_GETADR | CMDFLAG_GET);
	registerCommand("ippty", PEDALS_AnalogSource_commands::cmd_ipp_throttle_y, "Throttle Interpolation Points Output (Y axis)", CMDFLAG_SETADR | CMDFLAG_GETADR | CMDFLAG_GET);

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
	//#define ADR_PEDAL_1_xxx 	  0x601

	FlashAddress ++;
	//	#define ADR_PEDAL_1_PMIN 	  0x602
	Flash_Write(FlashAddress, (uint16_t)this->PedalConfiguration.input_min);

	FlashAddress ++;
	//#define ADR_PEDAL_1_PMAX 	  0x603
	Flash_Write(FlashAddress, (uint16_t)this->PedalConfiguration.input_max);

	//#define ADR_PEDAL_1_IPP_X_0   0x604
	FlashAddress ++;
	for(uint8_t i = 0; i < MAX_INTERPOLATIONPOINTS; i++){
		Flash_Write(FlashAddress, (uint16_t)this->InterpolationVals[i].x);
		printf ("\r\nsaveFlashX [%d] 0x%x %d",i, FlashAddress, this->InterpolationVals[i].x);
		FlashAddress ++;
		Flash_Write(FlashAddress, (uint16_t)this->InterpolationVals[i].y);
		printf ("         Y [%d] 0x%x %d",i, FlashAddress, this->InterpolationVals[i].y);
		FlashAddress ++;
	}

}

void PEDALS_AnalogSource::restoreFlash(){
	printf ("\r\nRestore Flash");

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
		ErrorHandler::addError(restoreflashError);
		return;
	}

	uint16_t pconfint;
	if(Flash_Read(FlashAddress, &pconfint)){
		AnalogAxisProcessing::setAnalogProcessingConfig(AnalogAxisProcessing::decodeAnalogProcessingConfFromInt(pconfint >> 8));
		pconf.pedalsmask = pconfint & 0xff;
	}

	FlashAddress ++;
	//#define ADR_PEDAL_1_xxx 	  0x601

	FlashAddress ++;
	//	#define ADR_PEDAL_1_PMIN 	  0x602
	Flash_Read(FlashAddress, &this->PedalConfiguration.input_min);

	FlashAddress ++;
	//#define ADR_PEDAL_1_PMAX 	  0x603
	Flash_Read(FlashAddress, &this->PedalConfiguration.input_max);

	//#define ADR_PEDAL_1_IPP_X_0   0x604
	FlashAddress ++;
	for(uint8_t i = 0; i < MAX_INTERPOLATIONPOINTS; i++){
		uint16_t bufX,bufY;
		if(Flash_Read(FlashAddress, &bufX) && Flash_Read(FlashAddress+1, &bufY)){
			this->InterpolationVals[i].x = (int16_t)bufX;
			this->InterpolationVals[i].y = (int16_t)bufY;
			printf ("\r\nintX[%d] 0x%x  %d %d",i, FlashAddress, this->InterpolationVals[i].x, this->InterpolationVals[i].y);
			FlashAddress = FlashAddress + 2;
		}
	}

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


// Handles starting next transfer for >1 byte transfers
void PEDALS_AnalogSource::Run() {
	while(true) {
		//ToDo code....
		this->Delay(500);
	}
}

std::vector<int32_t>* PEDALS_AnalogSource::getAxes(){
	uint8_t chans = 0;
	this->buf.clear();
	volatile uint32_t* pedalbuf = PedalsDataBuffer;

	for(uint8_t i = 0; i<numPedals; i++){
		int32_t val = ((pedalbuf[i]));
		val = interpolate_table_1d_int32(this->InterpolationVals, val);
		if(!(pconf.pedalsmask & 0x01 << i))
			continue;

		//val =  trunc(interpolate_table_1d(&throttle_table, (float)val));
		this->buf.push_back(val);
	}
	AnalogAxisProcessing::processAxes(buf);
	return &this->buf;
}


CommandStatus PEDALS_AnalogSource::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){

	switch(static_cast<PEDALS_AnalogSource_commands>(cmd.cmdId)){


	case PEDALS_AnalogSource_commands::pedalmask:
		return handleGetSet(cmd, replies, this->pconf.pedalsmask);
	break;

	case PEDALS_AnalogSource_commands::pedals:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(numPedals);
		}else{
			return CommandStatus::ERR;
		}
	break;
	case PEDALS_AnalogSource_commands::cmd_throttle:
		return handleGetSet(cmd, replies, this->PedalsDataBuffer[0]);
	break;

	case PEDALS_AnalogSource_commands::cmd_pedal_min:
		this->InterpolationVals[0].x = cmd.val;
		return handleGetSet(cmd, replies, this->PedalConfiguration.input_min);
	break;

	case PEDALS_AnalogSource_commands::cmd_pedal_max:
		return handleGetSet(cmd, replies, this->PedalConfiguration.input_max);
	break;


	case PEDALS_AnalogSource_commands::cmd_auto_interval:
		if ((this->PedalConfiguration.input_max+5) > this->PedalConfiguration.input_min) {
			uint16_t tmp_interval = (this->PedalConfiguration.input_max - this->PedalConfiguration.input_min) / 5;
			for (uint8_t i = 1; i<(MAX_INTERPOLATIONPOINTS); i++) {
				this->InterpolationVals[i].x = i * tmp_interval;
				printf ("\r\nint[%d] %d",i ,this->InterpolationVals[i].x);
			}
			replies.emplace_back(tmp_interval);
		}
		else {
			return CommandStatus::ERR;
		}

	break;

	case PEDALS_AnalogSource_commands::cmd_ipp_throttle_x:
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

	case PEDALS_AnalogSource_commands::cmd_ipp_throttle_y:
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

