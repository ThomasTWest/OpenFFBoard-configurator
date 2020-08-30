/*
 * FFBWheel.h
 *
 *  Created on: 31.01.2020
 *      Author: Yannick
 */

#ifndef SRC_FFBWHEEL_H_
#define SRC_FFBWHEEL_H_
#include <CmdParser.h>
#include <FFBoardMain.h>
#include <MotorPWM.h>
#include <ShifterAnalog.h>
#include "usbd_customhid.h"
#include "TMC4671.h"
#include "flash_helpers.h"
#include "ButtonSource.h"
#include "LocalButtons.h"
#include "SPIButtons.h"
#include "EncoderLocal.h"

#include "cppmain.h"
#include "HidFFB.h"
#include "AdcHandler.h"
#include "TimerHandler.h"
#include "ClassChooser.h"
#include "ExtiHandler.h"


struct FFBWheelConfig{
	uint8_t drvtype = 0;
	uint8_t enctype = 0;
	uint8_t axes = 1;

};

enum class AnalogOffset : uint8_t{
	FULL=0,LOWER=1,UPPER=2,NONE
};

struct FFBWheelAnalogConfig{
	uint8_t analogmask = 0xff;
	AnalogOffset offsetmode;
	bool invertX = false;
};


class FFBWheel: public FFBoardMain, AdcHandler, TimerHandler, PersistentStorage,ExtiHandler{
public:
	FFBWheel();
	virtual ~FFBWheel();

	static ClassIdentifier info;
	const ClassIdentifier getInfo();

	void setupTMC4671();
	void setupTMC4671_enc(PhiE enctype);
	bool command(ParsedCommand* cmd,std::string* reply);

	void setDrvType(uint8_t drvtype);
	void setEncType(uint8_t enctype);
	void setBtnTypes(uint16_t btntypes);
	void addBtnType(uint16_t id);
	void clearBtnTypes();
	ButtonSource* getBtnSrc(uint16_t id);

	void SOF();
	void usbInit(); // initialize a composite usb device
	void usbSuspend(); // Called on usb disconnect and suspend
	void usbResume(); // Called on usb resume

	void saveFlash();
	void restoreFlash();

	void update();

	static FFBWheelConfig decodeConfFromInt(uint16_t val);
	static uint16_t encodeConfToInt(FFBWheelConfig conf);
	static FFBWheelAnalogConfig decodeAnalogConfFromInt(uint16_t val);
	static uint16_t encodeAnalogConfToInt(FFBWheelAnalogConfig conf);


	void adcUpd(volatile uint32_t* ADC_BUF);
	void timerElapsed(TIM_HandleTypeDef* htim);
	void exti(uint16_t GPIO_Pin);

	volatile bool usb_update_flag = false;
	volatile bool update_flag = false;

	uint16_t degreesOfRotation = 900; // How many degrees of range for the full gamepad range
	int32_t getEncValue(Encoder* enc,uint16_t degrees);


	void setPower(uint16_t power);
	uint16_t getPower();

private:
	void send_report();
	int16_t updateEndstop();

	// USB Report rate
	uint8_t usb_report_rate = HID_FS_BINTERVAL; // 1 = 1000hz, 2 = 500hz, 3 = 250hz etc...
	uint8_t report_rate_cnt = 0;

	uint8_t endstop_gain_i = 128; // Sets how much extra torque per count above endstop is added. High = stiff endstop. Low = softer
	uint8_t fx_ratio_i = 204; // Reduce effects to a certain ratio of the total power to have a margin for the endstop

	HidFFB* ffb;
	TIM_HandleTypeDef* timer_update;
	int32_t torque = 0; // last torque
	int32_t effectTorque = 0; // last torque
	FFBWheelConfig conf;

	MotorDriver* drv = nullptr;
	Encoder* enc = nullptr;

	std::vector<ButtonSource*> btns;
	FFBWheelAnalogConfig aconf;
	volatile uint16_t adc_buf[ADC_PINS];
	reportHID_t reportHID;

	uint16_t power = 2000;

	int32_t lastScaledEnc = 0;
	int32_t scaledEnc = 0;
	int32_t speed = 0;
	bool tmcFeedForward = false; // Experimental
	uint16_t btnsources = 1; // Default ID1 = local buttons

	volatile bool usb_disabled = true;


//	TMC4671PIDConf tmcpids = TMC4671PIDConf({
//
//		.fluxI		= 2,
//		.fluxP		= 3000,
//		.torqueI	= 1500,
//		.torqueP	= 600,
//		.velocityI	= 0,
//		.velocityP	= 128,
//		.positionI	= 0,
//		.positionP	= 64
//	});
	TMC4671PIDConf tmcpids = TMC4671PIDConf({
		.fluxI		= 50,
		.fluxP		= 500,
		.torqueI	= 500,
		.torqueP	= 300,
		.velocityI	= 0,
		.velocityP	= 128,
		.positionI	= 0,
		.positionP	= 64,
		.sequentialPI = true
	});
	TMC4671Limits tmclimits = TMC4671Limits({
		.pid_torque_flux_ddt	= 32767,
		.pid_uq_ud				= 30000,
		.pid_torque_flux		= 30000,
		.pid_acc_lim			= 2147483647,
		.pid_vel_lim			= 2147483647,
		.pid_pos_low			= -2147483647,
		.pid_pos_high			= 2147483647
	});

	// Lowpass 1KHZ
	TMC4671Biquad fluxbq = TMC4671Biquad({
		.a1 = -134913,
		.a2 = 536735999,
		.b0 = 67457,
		.b1 = 134913,
		.b2 = 67457,
		.enable = true
	});
//	TMC4671Biquad fluxbq = TMC4671Biquad({
//			.a1 = -269793,
//			.a2 = 536601119,
//			.b0 = 134896,
//			.b1 = 269793,
//			.b2 = 134896,
//			.enable = true
//		});

	int32_t torqueFFgain = 50000;
	int32_t torqueFFconst = 0;
	int32_t velocityFFgain = 30000;
	int32_t velocityFFconst = 0;

	ClassChooser<ButtonSource> btn_chooser;
	ClassChooser<MotorDriver> drv_chooser;
	ClassChooser<Encoder> enc_chooser;
};

#endif /* SRC_FFBWHEEL_H_ */