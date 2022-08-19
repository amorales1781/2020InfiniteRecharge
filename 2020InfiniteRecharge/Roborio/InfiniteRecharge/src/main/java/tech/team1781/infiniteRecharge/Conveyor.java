/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package tech.team1781.infiniteRecharge;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.playingwithfusion.TimeOfFlight;

/**
 * Add your docs here.
 */
public class Conveyor {
    WPI_TalonSRX conveyor;
    WPI_TalonSRX aligner;
    TimeOfFlight lowerGate;

    double lower = 0.6;
    double upper = 0.6;

    public Conveyor() {
        conveyor = new WPI_TalonSRX(ConfigMap.UPPER_CONVEYOR);
        aligner = new WPI_TalonSRX(ConfigMap.ALIGNER_CONVEYOR);
        lowerGate = new TimeOfFlight(ConfigMap.TOF_LOWER_CANID);
    }

    public void collect() {
        aligner.set(lower);
        if(lowerGate.getRange() < 100)
            conveyor.set(upper);
        else
            conveyor.set(0);
    }

    public void feed()
    {
        conveyor.set(upper);
    }
    
    public void align()
    {
        aligner.set(lower);
    }

    public void outtake() {
        conveyor.set(-lower);
    }

    public void lowerStop() {
        aligner.set(0);
    }
    public void upperStop(){
        conveyor.set(0);
    }
    public void stopMotors(){
        conveyor.set(0);
        aligner.set(0);
    }
    // public void setConveyorSpeed(double _speed) {
    //     if (_speed > 0) {
    //         speed = _speed;
    //     } else if (speed < 0) {
    //         speed = _speed * -1;
    //     } else {
    //         speed = 0;
    //     }
    // }
}
