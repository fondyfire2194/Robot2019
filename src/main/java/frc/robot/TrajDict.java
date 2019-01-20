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

	// order is Kp, Kd, Ka and Kturn

	public static HashMap<String, Double[]> gainDict = new HashMap<>();
	static {
		gainDict.put("LSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("RSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("LSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("RSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("LSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("RSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("LSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("RSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("LSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("RSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("LSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("RSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("LSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("RSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("LSW_L", new Double[] { .4, 0., 0.02, 1. });
		// gainDict.put("RSW_L", new Double[] { .4, 0., 0.02, 1. });

	}
	public static Double[] getTrajGains(String name){
	String key = name;
       Double[] temp = gainDict.get(key);
       return temp;
	}
}