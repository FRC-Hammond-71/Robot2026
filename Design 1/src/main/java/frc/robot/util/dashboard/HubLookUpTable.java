// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.dashboard;

import java.util.TreeMap;

/** 
 * Lookup table for hub shooting parameters based on distance.
 * Supports linear interpolation between data points.
 */
public class HubLookUpTable {
    
    /** Data structure to hold shooting parameters */
    public static class ShootingParameters {
        public final double shooterSpeed;     // RPS (Revolutions Per Second)
        public final double trajectoryAngle;  // Degrees
        public final double timeOfFlight;     // Seconds
        
        public ShootingParameters(double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
            this.shooterSpeed = shooterSpeed;
            this.trajectoryAngle = trajectoryAngle;
            this.timeOfFlight = timeOfFlight;
        }
    }
    
    // TreeMap automatically sorts by distance (key)
    private final TreeMap<Double, ShootingParameters> lookupTable;
    
    public HubLookUpTable() {
        lookupTable = new TreeMap<>();
        initializeLookupTable();
    }
    
    /** Initialize the lookup table with known data points */
    private void initializeLookupTable() {
        // Distance (m), Shooter Speed (RPS), Trajectory Angle (°), Time of Flight (s)
        // KrakenX60 shooting 226g ball - optimized for constant RPS ~75
        // Trajectory angles: 90° = straight up, 45° = maximum distance

        // WE ARE ONLY AT 42 DEG
        // addEntry(1.0,  0, 42, 0.35);  // Close shot - nearly straight up
        // addEntry(1.5,  0, 42, 0.45);
        // addEntry(2.0,  0, 42, 0.55);
        // MIN RANGE IS 2.5
        // NOTE: TIME OF FLIGHT IS INCORRECT AND SHOULDN'T BE CONSIDERED
        addEntry(2.286,  35, 42, 0.65);
        addEntry(2.7432,  40, 42, 0.75);
        addEntry(3.2004,  42.5, 42, 0.85);
        addEntry(3.6576,  45, 42, 0.95);
        addEntry(4.1148,  47.5, 42, 1.05);  // Slight RPS increase for far shots
        addEntry(4.572,  52.5, 42, 1.15);
        addEntry(5.0292,  55, 42, 1.25);  // Max distance - lowest angle
        addEntry(5.4864,  57.5, 42, 1.25);  // Max distance - lowest angle
    }
    
    /** Add an entry to the lookup table */
    public void addEntry(double distance, double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
        lookupTable.put(distance, new ShootingParameters(shooterSpeed, trajectoryAngle, timeOfFlight));
    }
    
    /** 
     * Get interpolated shooting parameters for a given distance 
     * @param distance Distance to target in meters
     * @return Interpolated shooting parameters
     */
    public ShootingParameters getParameters(double distance) {
        // Check if exact match exists
        if (lookupTable.containsKey(distance)) {
            return lookupTable.get(distance);
        }
        
        // Get the surrounding values
        Double lowerKey = lookupTable.floorKey(distance);
        Double upperKey = lookupTable.ceilingKey(distance);
        
        // Handle edge cases
        if (lowerKey == null) {
            return lookupTable.get(upperKey); // Below minimum distance
        }
        if (upperKey == null) {
            return lookupTable.get(lowerKey); // Above maximum distance
        }
        
        // Perform linear interpolation
        ShootingParameters lower = lookupTable.get(lowerKey);
        ShootingParameters upper = lookupTable.get(upperKey);
        
        double ratio = (distance - lowerKey) / (upperKey - lowerKey);
        
        double interpolatedSpeed = lerp(lower.shooterSpeed, upper.shooterSpeed, ratio);
        double interpolatedAngle = lerp(lower.trajectoryAngle, upper.trajectoryAngle, ratio);
        double interpolatedTime = lerp(lower.timeOfFlight, upper.timeOfFlight, ratio);
        
        return new ShootingParameters(interpolatedSpeed, interpolatedAngle, interpolatedTime);
    }
    
    /** Linear interpolation helper */
    private double lerp(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }
    
    /** Get shooter speed for a given distance */
    public double getShooterSpeed(double distance) {
        return getParameters(distance).shooterSpeed;
    }
    
    /** Get trajectory angle for a given distance */
    public double getTrajectoryAngle(double distance) {
        return getParameters(distance).trajectoryAngle;
    }
    
    /** Get time of flight for a given distance */
    public double getTimeOfFlight(double distance) {
        return getParameters(distance).timeOfFlight;
    }
}