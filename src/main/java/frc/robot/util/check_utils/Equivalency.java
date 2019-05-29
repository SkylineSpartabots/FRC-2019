/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.check_utils;

import java.util.List;

/**
 * 
 */
public class Equivalency {

    private Equivalency(){

    }

    public static boolean equalsInBuffer(double x, double y, double buffer){
        return (x - buffer <= y) && (x + buffer >= y);
    }

    public static boolean allAboutEqualTo(List<Double> list, double value, double buffer){
        boolean equivalency = true;
        for(Double input : list){
            equivalency &= equalsInBuffer(input, value, buffer);
        }
        return equivalency;
    }
}
