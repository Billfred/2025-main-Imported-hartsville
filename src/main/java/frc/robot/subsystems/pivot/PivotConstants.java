package frc.robot.subsystems.pivot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class PivotConstants {
    public static final int kLeaderId = 20;
    public static final int kFollowerId = 21;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    public static final boolean kInvert = false;
    public static final boolean kInvertFollower = true;

    public static final double kReduction = 240;

    public static final int kCurrentLimit = 60;

    public static final double kMoveVoltage = 2;
    public static final double kTurboVoltage = 6;
}
