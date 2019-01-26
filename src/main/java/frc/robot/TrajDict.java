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

public static String[] leftStartNames = {"SimpleLeftTurn","SimpleRightTurn","SimpleLeftTurn"};//,"SimpleRightTurn","SimpleLeftTurn","SimpleRightTurn","SimpleLeftTurn"};
public static String[] leftCenterStartNames =  {"SimpleRightTurn","SimpleLeftTurn"};
public static String[] rightCenterStartNames =  {"SimpleRightTurn","SimpleLeftTurn","SimpleRightTurn","SimpleLeftTurn","SimpleRightTurn"};
public static String[] rightStartNames =  {"SimpleRightTurn","SimpleLeftTurn"};
public static String[] secondHatchNames ={"SimpleRightTurn","SimpleLeftTurn","SimpleRightTurn","SimpleLeftTurn","SimpleLeftTurn","SimpleRightTurn"};


	// order is Kp, Kd, Ka, Kturn amd Pref tuned and set(1)

	public static HashMap<String, Double[]> gainDict = new HashMap<>();
	static {
		gainDict.put(leftStartNames[0], new Double[] { .4, 0., 0.02, 1. });
		gainDict.put(leftStartNames[1], new Double[] { .4, 0., 0.02, 1. });

	}
	public static double[] getTrajGains(String name){
	String key = name;
	   Double[] temp = gainDict.get(key);
	     double[] temp1 = new double[4];
	   for (int i =0;i<4;i++){
		 temp1[i] = temp[i];
	   }
       return temp1;
	}
}