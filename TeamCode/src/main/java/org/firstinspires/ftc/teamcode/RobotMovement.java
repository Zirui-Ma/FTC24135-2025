package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/** Button Config for Drive
 * * Joy Right Y                : Drive
 * * Joy Right X                : Strafe
 * * Joy Left X                 : Turn
 * * Left Trigger               : Fine Movement + Joystick
 * * START                      : Field centric / Robot centric toggle
 * * Back                       : Reset Yaw angle --- removed
 * Gamepad 1 override Gamepad 2
 */

@TeleOp
public class RobotMovement extends OpMode {

    private double powerFactor;

    private AprilTag_TEST aprilTag;

    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    private FtcDashboard ftcDashboard;


    public void init() {
        // Initialize IMU from RobotHardware

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());

        aprilTag = new AprilTag_TEST(hardwareMap, ftcDashboard);
        aprilTag.init();
    }


    @SuppressLint("DefaultLocale")
    public void loop() {
        // Toggle control mode

        /** Reset IMU heading using button back and reset odometry
         * * need to remove this feature, as back button reseting imu may interfer with Roadrunner pose estimation.
         * * back button is using for retracting slide after auto phase and initial start of teleops

         if (gamepad_1.getButton(BACK) || gamepad_2.getButton(BACK) && !backPressed) {
         //robot.initIMU();
         //robot.resetDriveEncoders();
         debounceTimer.reset();
         backPressed = true;
         } else if (!gamepad_1.getButton(BACK) || !gamepad_2.getButton(BACK)) {
         backPressed = false;
         }
         */

        double drive = 0.0;
        double strafe = 0.0;
        double rotate = 0.0;

        // Mecanum drive calculations
        setMecanumDrivePower(drive, strafe, rotate, powerFactor);
        // Update telemetry with the latest data
        // empty

        aprilTag.tagProcessing();
        AprilTagDetection tagValue = aprilTag.tagUpdate();

        if (tagValue != null) {
            telemetry.addData("Bearing", tagValue.ftcPose.bearing);
            telemetry.addData("x", tagValue.ftcPose.x);
            telemetry.addData("y", tagValue.ftcPose.y);
            telemetry.addData("z", tagValue.ftcPose.z);
            telemetry.addData("Roll", tagValue.ftcPose.roll);
            telemetry.addData("Pitch", tagValue.ftcPose.pitch);
            telemetry.addData("Yaw", tagValue.ftcPose.yaw);
            telemetry.addData("Range", tagValue.ftcPose.range);
        }

    }// end of driveloop

    double lerp(double current, double target, double accel_Slowness, double decel_Slowness) {
        double alpha = (Math.abs(target) > Math.abs(current)) ? accel_Slowness : decel_Slowness;
        return current + alpha * (target - current);
    }

    private void setMecanumDrivePower(double drive, double strafe, double rotate, double powerFactor) {

        // Mecanum wheel drive formula
        double desiredFrontLeftPower = drive + strafe + rotate;
        double desiredFrontRightPower = drive - strafe - rotate;
        double desiredBackLeftPower = drive - strafe + rotate;
        double desiredBackRightPower = drive + strafe - rotate;

        // Constrain the power within +-1.0
        double maxPower = Math.max(
                Math.max(Math.abs(desiredFrontLeftPower), Math.abs(desiredFrontRightPower)),
                Math.max(Math.abs(desiredBackRightPower), Math.abs(desiredBackLeftPower))
        );

        if (maxPower > 1.0) {
            desiredFrontLeftPower /= maxPower;
            desiredFrontRightPower /= maxPower;
            desiredBackLeftPower /= maxPower;
            desiredBackRightPower /= maxPower;
        }

        double frontLeftPower = lerp(frontLeftMotor.getPower(), desiredFrontLeftPower, 0.25, 0.5);
        double frontRightPower = lerp(frontRightMotor.getPower(), desiredFrontRightPower, 0.25, 0.5);
        double backLeftPower = lerp(backLeftMotor.getPower(), desiredBackLeftPower, 0.25, 0.5);
        double backRightPower = lerp(backRightMotor.getPower(), desiredBackRightPower, 0.25, 0.5);


        // Set motor powers
        frontLeftMotor.setPower(Range.clip(frontLeftPower * powerFactor, -1.0, 1.0));
        frontRightMotor.setPower(Range.clip(frontRightPower * powerFactor, -1.0, 1.0));
        backLeftMotor.setPower(Range.clip(backLeftPower * powerFactor, -1.0, 1.0));
        backRightMotor.setPower(Range.clip(backRightPower * powerFactor, -1.0, 1.0));

    }
}