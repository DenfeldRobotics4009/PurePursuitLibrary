// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.odometry;

import frc.robot.odometry.sources.OdometrySource;

/** Add your docs here. */
public class OdometryHandler {

    private static OdometryHandler Instance;

    final OdometrySource[] odometrySources;

    /**
     * This class will provide a method for inputting multiple
     * position tracking systems into one robot, and having each
     * contribute to an assumed most accurate position of the robot.
     * 
     * Odometry sources can be configured to only provide specific data,
     * or to be only regarded in certain conditions,
     * 
     * Constructs or returns odometry source handler
     * 
     * 
     * @param odometrySources
     * @return Instance
     */
    public static OdometryHandler getInstance(OdometrySource... odometrySources) {
        // Singleton
        if (Instance == null) {
            Instance = new OdometryHandler(odometrySources);
        }

        return Instance;
    }

    /**
     * Constructs odometry source handler
     * 
     * @param odometrySources
     */
    private OdometryHandler(OdometrySource... odometrySources) {
        this.odometrySources = odometrySources;
    }
}
