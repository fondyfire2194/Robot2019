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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pref {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
 
  private static Vector<String> v;
  private static Enumeration<String> e;
  private static String tempString;
  private static double tempDouble;

 public static HashMap<String, Double> prefDict = new HashMap<>();

 static
  {
    prefDict.put("DriveStraighKp", .03);
    prefDict.put("DrivePositionKp", .03);
    prefDict.put("RotateKp", 0.2);
    prefDict.put("RotateKi", .0001);
    prefDict.put("RotateKd", .0); 
    prefDict.put("RotateIzone", 3.);
    prefDict.put("PathKp", 0.2);
    prefDict.put("PathKd", 0.);
    prefDict.put("PathKa", .02);
    prefDict.put("PathKt", .2);
    prefDict.put("JSTwistKp",.2);
    prefDict.put("PathKpRev",0.2);
    prefDict.put("PathKdRev",.0);
    prefDict.put("PathKaRev",.02);
    prefDict.put("PathKtRev",.2);
 
  }

  public static void ensureRioPrefs(){
    // init();
        deleteUnused();
         addMissing();
    }

  public static void deleteUnused() {
    v = new Vector<String>();
    v = Robot.prefs.getKeys();
int a=0;
    String[] myArray = v.toArray(new String[v.size()]);
    // SmartDashboard.putNumber("Asz",v.size());
    // SmartDashboard.putString("ASTR1",myArray[1]);

    for (int i = 0; i < v.size(); i++) {
      // SmartDashboard.putString("A"+String.valueOf(i),myArray[i]);
      boolean doNotDelete = myArray[i].equals(".type");
      
      if (!doNotDelete&&!prefDict.containsKey(myArray[i])&&Robot.prefs.containsKey(myArray[1])){
      //   a++;
      //   SmartDashboard.putString("ASTR",myArray[i]);
      //   SmartDashboard.putNumber("AI",i);  
      // SmartDashboard.putNumber("A",a);
        Robot.prefs.remove(myArray[i]);}
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
  // test function
  public static void writePrefs() {
    Robot.prefs.putDouble("Key1", 987.654);
    Robot.prefs.putDouble("Key2", 987.654);
    Robot.prefs.putDouble("Key3", 987.654);
    Robot.prefs.putDouble("Key4", 987.654);
    Robot.prefs.putDouble("Key5", 987.654);
    Robot.prefs.putDouble("Key6", 987.654);
    Robot.prefs.putDouble("Key7", 987.654);
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
