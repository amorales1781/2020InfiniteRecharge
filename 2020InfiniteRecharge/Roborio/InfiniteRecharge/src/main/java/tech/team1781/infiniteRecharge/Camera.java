/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package tech.team1781.infiniteRecharge;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Camera {
    NetworkTableInstance netInstance;
    NetworkTable table, limeTable;
    NetworkTableEntry xLoc, limeX, limeY, limeTargetArea, camMode, lightMode;

    public Camera()
    {
        limeTable = NetworkTableInstance.getDefault().getTable("limelight");
        limeX = limeTable.getEntry("tx");
        limeY = limeTable.getEntry("ty");
        limeTargetArea = limeTable.getEntry("ta");
        camMode = limeTable.getEntry("camMode");
        lightMode = limeTable.getEntry("ledMode");
    }

    public void setCameraMode(int mode)
    {
        camMode.setNumber(mode);
    }

    public void turnLightsOn()
    {
        lightMode.setNumber(3);
    }

    public void turnLightsOff()
    {
        lightMode.setNumber(1);
    }

    public double getPortX()
    {
        limeX = limeTable.getEntry("tx");
        return limeX.getDouble(-1);
    }
}
