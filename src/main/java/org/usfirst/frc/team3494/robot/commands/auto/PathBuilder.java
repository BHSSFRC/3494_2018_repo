package org.usfirst.frc.team3494.robot.commands.auto;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import org.usfirst.frc.team3494.robot.RobotMap;

public class PathBuilder {

    private static final double WHEEL_WIDTH = 1.0D;

    private Trajectory centerToLeftTraj;
    private TankModifier centerToLeftMod;

    private Trajectory centerToRightTraj;
    private TankModifier centerToRightMod;

    private void genCenterToLeft() {
        System.out.println("Generating path, please wait...");
        Waypoint[] centerToLeft = new Waypoint[] {
                new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
                new Waypoint(-2, -2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
                new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
        };

        Trajectory.Config config = new Trajectory.Config(
                Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_HIGH,
                0.05, RobotMap.PATH_MAX_SPEED, 2.0, 60.0
        );
        centerToLeftTraj = Pathfinder.generate(centerToLeft, config);
        centerToLeftMod = new TankModifier(centerToLeftTraj).modify(WHEEL_WIDTH);
        System.out.println("Path generated!");
    }

    private void genCenterToRight() {
        Waypoint[] centerToRight = new Waypoint[]{
                new Waypoint(0, 0, 0),
                new Waypoint(0, 0.5, 0),
                new Waypoint(0, 1, 0)
        };
        Trajectory.Config config = new Trajectory.Config(
                Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_HIGH,
                0.05, RobotMap.PATH_MAX_SPEED, 2.0, 60.0
        );
        centerToRightTraj = Pathfinder.generate(centerToRight, config);
        centerToRightMod = new TankModifier(centerToRightTraj).modify(WHEEL_WIDTH);
    }

    public Trajectory getCenterToLeftTraj() {
        if (centerToLeftTraj == null) {
            this.genCenterToLeft();
        }
        return centerToLeftTraj;
    }

    public Trajectory getCenterToRightTraj() {
        if (centerToRightTraj == null) {
            this.genCenterToRight();
        }
        return centerToRightTraj;
    }

    public TankModifier getCenterToLeftMod() {
        if (centerToLeftMod == null) {
            this.genCenterToLeft();
        }
        return centerToLeftMod;
    }

    public TankModifier getCenterToRightMod() {
        if (centerToRightMod == null) {
            this.genCenterToRight();
        }
        return centerToRightMod;
    }
}
