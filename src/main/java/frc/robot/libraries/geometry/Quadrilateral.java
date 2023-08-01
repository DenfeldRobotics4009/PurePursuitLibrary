// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries.geometry;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Quadrilateral {

    /**
     * Points are ordered by how they were inputted
     */
    public Translation2d[] points = new Translation2d[4];

    /**
     * Constructs a quadrilateral from 4 Translation2d
     */
    public Quadrilateral(Translation2d P1, Translation2d P2,Translation2d P3, Translation2d P4) {
        points = new Translation2d[] {P1, P2, P3, P4};
    }

    /**
     * Calculates area of quadrilateral via slicker algorithm
     * @param p Quadrilateral to calculate area of
     * @return Area of quadrilateral
     */
    public static double Area(Quadrilateral p)
    {
        double total = 0;

        for (int i = 0; i < 4; i++) {
            int j = (i + 1) % 4;
            total += (p.points[i].getX() * p.points[j].getY())
                - (p.points[j].getX() * p.points[i].getY());
        }

        return total / 2;
    }

    /**
     * Calculates area of quadrilateral via slicker algorithm
     * @return Area of quadrilateral
     */
    public double Area() {
        
        return Area(this);
    }

    /**
     * Calculates and returns the centroid of the quadrilateral
     * @param p Quadrilateral to calculate centroid of
     * @return Centroid of quadrilateral
     */
    public static Translation2d Centroid(Quadrilateral p) {

        Translation2d total = new Translation2d();

        for (Translation2d point : p.points) {
            total = total.plus(point);
        }

        return total.div(4);
    }

    /**
     * Calculates and returns the centroid of the quadrilateral
     * @return Centroid of quadrilateral
     */
    public Translation2d Centroid() {

        return Centroid(this);
    }
}
