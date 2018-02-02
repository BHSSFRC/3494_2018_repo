package org.usfirst.frc.team3494.robot.commands.auto;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import org.usfirst.frc.team3494.robot.RobotMap;

/**
 * Class for generating {@link jaci.pathfinder.Pathfinder Pathfinder} trajectories.
 * All distances are in meters. All angles are in radians
 * (conversion methods available with {@link jaci.pathfinder.Pathfinder#d2r(double)}.)
 */
public class PathBuilder {
    /**
     * The distance between the sides of the {@link org.usfirst.frc.team3494.robot.subsystems.Drivetrain drivetrain}.
     */
    private static final double WHEELBASE_WIDTH = 0.83D;

    private Trajectory centerToLeftTraj;
    private TankModifier centerToLeftMod;

    private Trajectory centerToRightTraj;
    private TankModifier centerToRightMod;

    private void genCenterToLeft() {
        System.out.println("Generating path, please wait...");
        Waypoint[] centerToLeft = new Waypoint[]{
                new Waypoint(-4, -1, 0),
                new Waypoint(-2, -2, 0),
                new Waypoint(0, 0, 0)
        };

        Trajectory.Config config = new Trajectory.Config(
                Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_HIGH,
                0.05, RobotMap.PATH_MAX_SPEED, 1.0, 60.0
        );
        centerToLeftTraj = Pathfinder.generate(centerToLeft, config);
        centerToLeftMod = new TankModifier(centerToLeftTraj).modify(WHEELBASE_WIDTH);
        System.out.println("Path generated!");
    }

    private void genCenterToRight() {
        System.out.println("Generating path, please wait...");
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
        centerToRightMod = new TankModifier(centerToRightTraj).modify(WHEELBASE_WIDTH);
        System.out.println("Path generated!");
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
