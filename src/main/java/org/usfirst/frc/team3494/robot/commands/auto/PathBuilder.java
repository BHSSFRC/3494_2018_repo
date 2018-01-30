package org.usfirst.frc.team3494.robot.commands.auto;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class PathBuilder {

    private static final double WHEEL_WIDTH = 1.0D;

    private Trajectory centerToLeftTraj;
    private TankModifier centerToLeftMod;

    private Trajectory centerToRightTraj;
    private TankModifier centerToRightMod;

    public void genCenterToLeft() {
        Waypoint[] centerToLeft = new Waypoint[]{
                new Waypoint(0, 0, 0),
                new Waypoint(-1, 0.5, Pathfinder.d2r(45)),
                new Waypoint(-1, 1, 0)
        };
        Trajectory.Config config = new Trajectory.Config(
                Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_HIGH,
                0.05, 1.7, 2.0, 60.0
        );
        centerToLeftTraj = Pathfinder.generate(centerToLeft, config);
        centerToLeftMod = new TankModifier(centerToLeftTraj).modify(WHEEL_WIDTH);
    }

    public void genCenterToRight() {
        Waypoint[] centerToRight = new Waypoint[]{
                new Waypoint(0, 0, 0),
                new Waypoint(1, 0.5, Pathfinder.d2r(45)),
                new Waypoint(1, 1, 0)
        };
        Trajectory.Config config = new Trajectory.Config(
                Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_HIGH,
                0.05, 1.7, 2.0, 60.0
        );
        centerToRightTraj = Pathfinder.generate(centerToRight, config);
        centerToRightMod = new TankModifier(centerToRightTraj).modify(WHEEL_WIDTH);
    }

    public Trajectory getCenterToLeftTraj() {
        return centerToLeftTraj;
    }

    public Trajectory getCenterToRightTraj() {
        return centerToRightTraj;
    }

    public TankModifier getCenterToLeftMod() {
        return centerToLeftMod;
    }

    public TankModifier getCenterToRightMod() {
        return centerToRightMod;
    }
}
