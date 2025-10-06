package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

    public GoBildaPinpointDriver odo;

    public void init() {
        // Initialize IMU from RobotHardware

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());

        aprilTag = new AprilTag_TEST(hardwareMap, ftcDashboard);
        aprilTag.init();

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setOffsets(-149.225, -165.1);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.METER,0,0,AngleUnit.DEGREES,0));
    }


    @SuppressLint("DefaultLocale")
    public void loop() {
        //Update Odometry
        odo.update();
        //odo.setPosition()

        // Toggle control mode

        /** Reset IMU heading using button back and reset odometry
         * * need to remove this feature, as back button reseting imu may interfer with Roadrunner pose estimation.
         * * back button is using for retracting slide after auto phase and initial start of teleops
         */



        double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double powerFactor = 0.5;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator * powerFactor;
        double backLeftPower = (y - x + rx) / denominator * powerFactor;
        double frontRightPower = (y - x - rx) / denominator * powerFactor;
        double backRightPower = (y + x - rx) / denominator * powerFactor;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        aprilTag.tagProcessing();
        AprilTagDetection tagValue = aprilTag.tagUpdate();

        if (tagValue != null) {
            telemetry.addData("x", roundNumber(tagValue.ftcPose.x));
            telemetry.addData("y", roundNumber(tagValue.ftcPose.y));
            telemetry.addData("z", roundNumber(tagValue.ftcPose.z));
            telemetry.addData("Bearing", roundNumber(tagValue.ftcPose.bearing));
            telemetry.addData("Roll", roundNumber(tagValue.ftcPose.roll));
            telemetry.addData("Pitch", roundNumber(tagValue.ftcPose.pitch));
            telemetry.addData("Yaw", roundNumber(tagValue.ftcPose.yaw));
            telemetry.addData("Range", roundNumber(tagValue.ftcPose.range));
        }

        telemetry.addData("odometry x", roundNumber(odo.getPosX()));
        telemetry.addData("odometry y", roundNumber(odo.getPosY()));
        telemetry.addData("odometry heading", roundNumber(odo.getHeading()));
        telemetry.update();
    }

    private double roundNumber(double number) {
        return (double) Math.round(number * 100) / 100;
    }
}
