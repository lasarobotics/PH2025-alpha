package frc.robot;

import java.util.Optional;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.child.MAXSwerveModule;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.PIDConstants;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public final class Constants {

  public static class Field {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    public static final AprilTag BLUE_SPEAKER = getTag(7).get();
    public static final AprilTag RED_SPEAKER = getTag(4).get();
    public static final AprilTag BLUE_AMP = getTag(6).get();


    /**
     * Get AprilTag from field
     * @param id Tag ID
     * @return AprilTag matching ID
     */
    public static Optional<AprilTag> getTag(int id) {
      return FIELD_LAYOUT.getTags().stream().filter((tag) -> tag.ID == id).findFirst();
    }
  }

  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;
    public static final Dimensionless CONTROLLER_DEADBAND = Units.Percent.of(10);
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }

  public static class Drive {
    public static final DriveWheel DRIVE_WHEEL = DriveWheel.create(Units.Inches.of(4.0), Units.Value.of(1.3), Units.Value.of(1.2));
    public static final PIDConstants DRIVE_PID = PIDConstants.of(0.3, 0.0, 0.001, 0.0, 0.0);
    public static final FFConstants DRIVE_FF = FFConstants.of(0.2, 0.0, 0.0, 0.0);
    public static final PIDConstants ROTATE_PID = PIDConstants.of(2.0, 0.0, 0.1, 0.0, 0.0);
    public static final FFConstants ROTATE_FF = FFConstants.of(0.2, 0.0, 0.0, 0.0);
    public static final PIDConstants DRIVE_ROTATE_PID = PIDConstants.of(8.0, 0.0, 0.3, 0.0, 0.0);
    public static final PIDConstants DRIVE_AUTO_AIM_PID = PIDConstants.of(12.0, 0.0, 0.1, 0.0, 0.0);
    public static final Dimensionless DRIVE_SLIP_RATIO = Units.Percent.of(3.0);
    public static final Angle DRIVE_TURN_SCALAR = Units.Degrees.of(90.0);
    public static final Time DRIVE_LOOKAHEAD = Units.Seconds.of(0.2);

    public static final Distance DRIVE_WHEELBASE = Units.Meters.of(0.5588);
    public static final Distance DRIVE_TRACK_WIDTH = Units.Meters.of(0.5588);
    public static final Mass MASS = Units.Pounds.of(110.0);
    public static final Time AUTO_LOCK_TIME = Units.Seconds.of(3.0);
    public static final Current DRIVE_CURRENT_LIMIT = Units.Amps.of(90.0);

    public static final ControlCentricity DRIVE_CONTROL_CENTRICITY = ControlCentricity.FIELD_CENTRIC;

    private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000 };
    private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 0.052, 0.207, 0.465, 0.827, 1.293, 1.862, 2.534, 3.310, 4.189, 5.172 };
    private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
    private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.010, 0.050, 0.100, 0.150, 0.200, 0.250, 0.300, 0.400, 0.600, 1.0 };

    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);

    public static final MAXSwerveModule.GearRatio REV_GEAR_RATIO = MAXSwerveModule.GearRatio.L3;

    public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(DRIVE_WHEEL.diameter.div(2), Units.MetersPerSecond.of(5.172), 1.3, DCMotor.getKrakenX60Foc(1), DRIVE_CURRENT_LIMIT, 1);
  }

  public static class Arm {
    public static final Spark.ID ROLLER_MOTOR_ID = new Spark.ID("IntakeHardware/Roller", 51);
    public static final Spark.ID ARM_MOTOR_ID = new Spark.ID("IntakeHardware/Arm", 52);
    public static final Dimensionless ARM_SPEED = Units.Percent.of(5);
    public static final Dimensionless ROLLER_SPEED = Units.Percent.of(2);
    public static final PIDConstants INTAKE_PID = PIDConstants.of(0.05, 0.0, 0.0, 0.0, 0.0);
    public static final double STOW_POS = 0;
    public static final double INTAKE_POS = -9.2;
    public static final Current CURRENT_LIMIT = Units.Amps.of(20);
  }

  public static class Climber {
    public static final Spark.ID CLIMB_MOTOR_ID = new Spark.ID("ClimberHardware/Climb", 61);
    public static final Dimensionless CLIMBER_SPEED = Units.Percent.of(10);
  }

  public static class REVDriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 21);
    public static final Spark.ID LEFT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 22);
    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Drive", 11);
    public static final Spark.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Rotate", 12);
    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Drive", 41);
    public static final Spark.ID LEFT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Rotate", 42);
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Drive", 31);
    public static final Spark.ID RIGHT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Rotate", 32);
  }
}
