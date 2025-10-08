// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*****Stores common values, non-mutable(can't be changed... at least not in runtime)*/
public class Constants {
    public static final double MaxSpeed = 3; //MPS
    public static final double MaxAngularRate = .925 * Math.PI;// Radians/s   original 0.825

    /********** Elevator heights ********/

    public static final double L1 = -9.54;
    public static final double L2 = -126;
    public static final double L3 = -196.308;
    public static final double L4 = 0;
    public static final double CORAL_INTAKE = -77.04;
    public static final double ELE_ALGAE_L2 = -76.17; // -71.424
    public static final double ELE_ALGAE_L3 = -190.656;


    //********** Suckerfish angles******/
    public static final double CL1 = 80;
    public static final double CL2 = 117;
    public static final double CL3 = 89;
    public static final double CL4 = 0;
    public static final double C_CORAL_INTAKE = 56;
    public static final double C_ALGAE_L2 = 65;
    public static final double C_ALGAE_L3 = 67;

}
