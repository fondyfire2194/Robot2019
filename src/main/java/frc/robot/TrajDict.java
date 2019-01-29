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

public static String[] leftStartNames = {"LL1ToCS2","CS2ToTurn"};
public static String[] leftCenterStartNames = {"CLEToLoad"};
public static String[] rightCenterStartNames = {"CLEToLoad"};
public static String[] rightStartNames =   {"LL1ToCS2","CS2ToTurn"};//


public static String[] secondHatchNames = {"LoadToCS1","LoadToCS2","LoadToCS3","LoadToFarCenter"};

	// order is Kp, Kd, Ka, Kturn amd Pref tuned and set(1)

	public static HashMap<String, Double[]> gainDict = new HashMap<>();
	static {
		gainDict.put(leftStartNames[0], new Double[] { .4, 0., 0.02, 1. });
		gainDict.put(leftStartNames[1], new Double[] { .4, 0., 0.02, 1. });
		gainDict.put(leftCenterStartNames[0], new Double[] { .4, 0., 0.02, 1. });
		gainDict.put(rightCenterStartNames[0], new Double[] { .4, 0., 0.02, 1. });
		gainDict.put(rightStartNames[0], new Double[] { .4, 0., 0.02, 1. });
		gainDict.put(rightStartNames[1], new Double[] { .4, 0., 0.02, 1. });

	}
	public static double[] getTrajGains(String name){
	
	   Double[] temp = gainDict.get(name);
	     double[] temp1 = new double[4];
	   for (int i =0;i<4;i++){
		 temp1[i] = temp[i];
	   }
       return temp1;
	}
}