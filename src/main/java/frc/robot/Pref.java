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

public class Pref {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Vector<String> v;
  private static Enumeration<String> e;
  private static String tempString;
  private static double tempDouble;

  public static HashMap<String, Double> prefDict = new HashMap<>();

  static {
    prefDict.put("CameraOffset", 0.);// camera offset
    prefDict.put("CargoIntakeAmpsLimit", 9.0);

    prefDict.put("DrivePositionKd", .03);
    prefDict.put("DrivePositionKp", 2.7);
    prefDict.put("DrivePositionRate", 6.);
    prefDict.put("CenterDistance",11.6);//feet
    prefDict.put("CenterLoadDistance",8.8);//feet
    

    prefDict.put("DriveStraightKp", .05);
    prefDict.put("DriveStall", 25.);

    prefDict.put("ElevatorMMKd", 2.0);
    prefDict.put("ElevatorMMKf", .6);
    prefDict.put("ElevatorMMKi", 0.);
    prefDict.put("ElevatorMMKp", .7);

    prefDict.put("JSTwistMaxKp", .4);
    prefDict.put("JSTwistMinKp", .2);

    prefDict.put("PathKa", .0);
    prefDict.put("PathKd", 0.);
    prefDict.put("PathKp", 0.3);
    prefDict.put("PathKt", .4);

    prefDict.put("RotateIzone", 3.);
    prefDict.put("RotateKd", .005);
    prefDict.put("RotateKi", .0);
    prefDict.put("RotateKp", 0.005);

    prefDict.put("VisionKp", .005);
    prefDict.put("VisionPipeline",0.);

  }

  public static void ensureRioPrefs() {
    // init();
    deleteUnused();
    addMissing();
  }

  public static void deleteUnused() {
    v = new Vector<String>();
    v = Robot.prefs.getKeys();
    String[] myArray = v.toArray(new String[v.size()]);

    for (int i = 0; i < v.size(); i++) {
      boolean doNotDelete = false;// myArray[i].equals(".type");

      if (!doNotDelete && !prefDict.containsKey(myArray[i]) && Robot.prefs.containsKey(myArray[i])) {
        Robot.prefs.remove(myArray[i]);
      }
    }

  }

  public static void addMissing() {

    Iterator<Map.Entry<String, Double>> it = prefDict.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<String, Double> pair = it.next();
      tempString = pair.getKey();
      tempDouble = pair.getValue();
      if (!Robot.prefs.containsKey((tempString)))
        Robot.prefs.putDouble(tempString, tempDouble);
    }
  }

  public static double getPref(String key) {
    if (prefDict.containsKey(key))
      return Robot.prefs.getDouble(key, prefDict.get(key));
    else
      return 0;
  }

  public static void deleteAllPrefs() {
    v = new Vector<String>();// creating vector
    v = Robot.prefs.getKeys();
    e = v.elements();
    while (e.hasMoreElements()) {
      Robot.prefs.remove(e.nextElement());
    }
  }

}
