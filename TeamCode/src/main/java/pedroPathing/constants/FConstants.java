package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "front_left_drive";
        FollowerConstants.leftRearMotorName = "back_left_drive";
        FollowerConstants.rightFrontMotorName = "front_right_drive";
        FollowerConstants.rightRearMotorName = "back_right_drive";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 12.88202;

        FollowerConstants.xMovement = (79+82+80) / 3.0;
        FollowerConstants.yMovement = 46;

        FollowerConstants.forwardZeroPowerAcceleration = -31;
        FollowerConstants.lateralZeroPowerAcceleration = -63.5;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.05,0,0.001,0.0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.0,0,0.00,0); // Not being used, @see useSecondaryTranslationalPID

        // was 1 and 1.3 for p
        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.05,0); // Not being used, @see useSecondaryHeadingPID



        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.0001,0.6,0.0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.02,0,0.0004,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.95; // was 0.95
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        // does this do anything lol? retune pids
        FollowerConstants.maxPower = 1;

        FollowerConstants.useBrakeModeInTeleOp = true;
    }
}
