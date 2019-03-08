/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Add your docs here.
 */
public class Debouncer {

    private Debouncer.RawInput rawInput;
    private int CLOCK_MAX;
    private Boolean deboucedValue = null;

    private boolean prevRawState, rawState;
    private int clockCounter = 0;
    

  
   /**
    * 
    * @param rawInput Source of the raw "undebounced" input
    * @param CLOCK_MAX Maximum clock count at what point a "debounced" value is recorded
    * @param initDebouncedValue Starting data point for debouncer
    */
    public Debouncer(Debouncer.RawInput rawInput, int CLOCK_MAX){
        this.rawInput = rawInput;
        this.CLOCK_MAX = CLOCK_MAX;
    }

    public boolean compute(){
        
        rawState = rawInput.getBooleanInput();

        if(prevRawState == rawState) {
            clockCounter++;
            if(clockCounter == CLOCK_MAX){
                deboucedValue = rawState;
                clockCounter = 0;
            }
        } else {
            clockCounter = 0;
        }

        prevRawState = rawState;

        //in the first cycles through the debouncer, when a debounced value hasn't
        //been set, it returns the raw value
        if(deboucedValue == null){
            return rawState;
        } else{
            return deboucedValue;
        }
        
    }

    public interface RawInput{
        public boolean getBooleanInput();
    }
}
