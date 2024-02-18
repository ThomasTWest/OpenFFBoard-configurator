/*
 * FFBPedals.cpp
 *
 *  Created on: 29.01.2024
 *      Author: Thomas West
 */

#include "FFBPedals.h"
#ifdef FFBPEDALS


#include "usb_hid_ffb_desc.h"

// Unique identifier for listing
ClassIdentifier FFBPedals::info = {
		 .name = "FFB Pedals (2 Axis)" ,
		 .id=CLSID_MAIN_FFBPEDAL,
 };

const ClassIdentifier FFBPedals::getInfo(){
	return info;
}


FFBPedals::FFBPedals() :
		FFBHIDMain(2)
{
	FFBHIDMain::setFFBEffectsCalc(ffb, effects_calc);
}

FFBPedals::~FFBPedals() {

}



void FFBPedals::usbInit(){
	this->usbdev = std::make_unique<USBdevice>(&usb_devdesc_ffboard_composite,usb_cdc_hid_conf_2axis,&usb_ffboard_strings_default);
	FFBHIDMain::UsbHidHandler::setHidDesc(hid_2ffb_desc);
	usbdev->registerUsb();
}
#endif
