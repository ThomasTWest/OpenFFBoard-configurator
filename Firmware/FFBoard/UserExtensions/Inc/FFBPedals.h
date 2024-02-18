/*
 * FFBJoystick.h
 *
 *  Created on: 29.03.2022
 *      Author: Yannick
 */

#ifndef USEREXTENSIONS_SRC_FFBPEDALS_H_
#define USEREXTENSIONS_SRC_FFBPEDALS_H_
#include "constants.h"
#ifdef FFBPEDALS

#include "FFBHIDMain.h"

class FFBPedals : public FFBHIDMain {
public:
	FFBPedals();
	virtual ~FFBPedals();

	static ClassIdentifier info;
	const ClassIdentifier getInfo();

	void usbInit() override;

private:
	std::shared_ptr<EffectsCalculator> effects_calc = std::make_shared<EffectsCalculator>();
	std::shared_ptr<EffectsControlItf> ffb = std::make_shared<HidFFB>(effects_calc,2);
};
#endif
#endif /* USEREXTENSIONS_SRC_FFBPEDALS_H_ */
