/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */

import java.util.HashMap;

public class TrajDict {

	public static String[] outsideStartNames = { "LL1ToCS2" };

	public static String[] secondHatchPickupNames = { "CS2ToTurn", "CLEToLoad" };

	public static String[] secondHatchDeliveryNames = { "LoadToCS1", "LoadToCS2", "LoadToCS3", "LoadToFarCenter" };

	// order is Kp, Kd, Ka, Kturn amd Pref tuned and set(1)

	public static HashMap<String, Double[]> gainDict = new HashMap<>();
	static {
		gainDict.put(outsideStartNames[0], new Double[] { .8, .1, 0.1, 1. });
		gainDict.put(secondHatchPickupNames[0], new Double[] { .8, .1, 0.1, 1. });
		gainDict.put(secondHatchPickupNames[1], new Double[] { .8, .1, 0.1, 1. });
		gainDict.put(secondHatchDeliveryNames[0], new Double[] { .8, .1, 0.1, 1. });
		gainDict.put(secondHatchDeliveryNames[1], new Double[] { .8, .1, 0.1, 1. });
		gainDict.put(secondHatchDeliveryNames[2], new Double[] { .8, .1, 0.1, 1. });
		gainDict.put(secondHatchDeliveryNames[3], new Double[] { .8, .1, 0.1, 1. });
	}
	public static HashMap<String, Double[]> gainInvYDict = new HashMap<>();
	static {
		gainInvYDict.put(outsideStartNames[0], new Double[] { .8, .1, 0.1, 1. });
		gainInvYDict.put(secondHatchPickupNames[0], new Double[] { .8, .1, 0.1, 1. });
		gainInvYDict.put(secondHatchPickupNames[1], new Double[] { .8, .1, 0.1, 1. });
		gainInvYDict.put(secondHatchDeliveryNames[0], new Double[] { .8, .1, 0.1, 1. });
		gainInvYDict.put(secondHatchDeliveryNames[1], new Double[] { .8, .1, 0.1, 1. });
		gainInvYDict.put(secondHatchDeliveryNames[2], new Double[] { .8, .1, 0.1, 1. });
		gainInvYDict.put(secondHatchDeliveryNames[3], new Double[] { .8, .1, 0.1, 1. });
	}

	public static double[] getTrajGains(String name) {

		Double[] temp = gainDict.get(name);
		double[] temp1 = new double[4];
		for (int i = 0; i < 4; i++) {
			temp1[i] = temp[i];
		}
		return temp1;
	}
	public static double[] getInvYTrajGains(String name) {

		Double[] temp = gainInvYDict.get(name);
		double[] temp1 = new double[4];
		for (int i = 0; i < 4; i++) {
			temp1[i] = temp[i];
		}
		return temp1;
	}
}