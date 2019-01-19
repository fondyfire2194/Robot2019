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

import java.util.Enumeration;
import java.util.HashMap;
import java.util.Vector;

import java.util.Iterator;
import java.util.Map;
import frc.robot.Robot;

public class TrajGains {

	// Center Start
	public static double[] LSW_C = { .4, 0, 0.02, 1 };
	public static double[] LSW_C1 = { .4, 0, 0.02, 1 };
	public static double[] LSW_C1Rev = { .4, 0, 0.02, 1 };

	public static double[] RSW_C = { .4, 0, 0.02, 1 };
	public static double[] RSW_C1 = { .4, .5, .06, .8 };
	public static double[] RSW_C1Rev = { .8, .5, .06, .8 };

	// Left Start
	public static double[] LSW_L = { .4, 0, 0.02, 1 };
	public static double[] LSW_L1Rev = { .4, 1.2, 0, .6 };// reverse
	public static double[] LSW_L2 = { .4, 0, 0, .8 };

	// Right Start
	// order is Kp, Kd, Ka and Kturn
	public static double[] RSW_R = { .4, 0, 0.02, 1 };
	public static double[] RSW_R1Rev = { .8, 0, 0, .1 };// reverse
	public static double[] RSW_R2 = { .8, 0, 0, .1 };

	public static HashMap<String, Double[]> gainDict = new HashMap<>();
	static {
		gainDict.put("LSW_L", new Double[] { .4, 0., 0.02, 1. });
		gainDict.put("RSW_L", new Double[] { .4, 0., 0.02, 1. });
	}
}