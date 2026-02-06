

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@TeleOp(name="Decode Teleop", group="Linear Opmode")
public class MagicSauce extends LinearOpMode {

    GoBildaPinpointDriver odo;

    double oldTime = 0;
    private enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER_HIGH,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
    }

    private RobotState currentState = RobotState.INIT;

    // Arm and wrist target positions for each state
    private static final int ARM_POSITION_INTAKE = 450;
    private static final int ARM_POSITION_WALL_GRAB = 1100;
    private static final int ARM_POSITION_WALL_UNHOOK = 1700;
    private static final int ARM_POSITION_HOVER_HIGH = 2600;
    private static final int ARM_POSITION_CLIP_HIGH = 2100;
    private static final int ARM_POSITION_LOW_BASKET = 2500;

    private static final int WRIST_POSITION_INIT = -490;
    private static final int WRIST_POSITION_SAMPLE = 270;
    private static final int WRIST_POSITION_SPEC = 10;

    private static final int SPINDEXER_TICKS_PER_POSITION = 220;

    private static final double CLAW_POSITION_INIT = 0.9;
    private static final double CLAW_OPEN_POSITION = 0.3;
    private static final double CLAW_CLOSED_POSITION = 0.8;

    private int targetOutake = 0;
    private int targetIntake = 0;
    private int spinpos = 0;
    private double spinoff = 0;
    private int bumptimer = 0;

    private boolean clawOpen = true;

    // --- Helper class for timing
    class Robott {
        long startTime;
        long currentTime;

        void start() {
            startTime = System.currentTimeMillis();
            currentTime = 0;
        }

        void update() {
            currentTime = System.currentTimeMillis() - startTime;
        }

        long getTime() {
            return currentTime;
        }
    }

    @Override
    public void runOpMode() {
        // --- Hardware Mapping ---
        DcMotor Leftbw = hardwareMap.dcMotor.get("Leftbw");
        DcMotor Rightbw = hardwareMap.dcMotor.get("Rightbw");
        DcMotor Rightfw = hardwareMap.dcMotor.get("Rightfw");
        DcMotor Leftfw = hardwareMap.dcMotor.get("Leftfw");
        DcMotor spindexer = hardwareMap.dcMotor.get("spindexer");
        DcMotor Intake = hardwareMap.dcMotor.get("Intake");
        DcMotor Outake = hardwareMap.dcMotor.get("Outake");


        // --- Drive motor setup ---
        Rightfw.setDirection(DcMotorSimple.Direction.REVERSE);
        Rightbw.setDirection(DcMotorSimple.Direction.REVERSE);

        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // --- Arm/Output setup ---
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odowheels");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
            odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
            //odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);


            if (gamepad1.a){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Robott timer = new Robott();
        timer.start();

        

        if (gamepad2.x) {
            spinpos = (spinpos +1) % 6;
        }



            timer.update();

            // --- Manual arm control with d-pad ---
            if (gamepad2.dpad_up) {
                targetOutake = 10;

            } else if (gamepad2.dpad_down) {
                targetOutake = 0;

            } else if (gamepad2.dpad_left) {
                targetIntake = -10;
            } else if (gamepad2.dpad_right) {
                targetIntake = 0;
            }

            // --- Drive control ---
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

            Leftfw.setPower((forward + strafe + turn) / denominator);
            Leftbw.setPower((forward - strafe + turn) / denominator);
            Rightfw.setPower((forward - strafe - turn) / denominator);
            Rightbw.setPower((forward + strafe - turn) / denominator);

            // --- Spindexer Control ---
            spinoff = spinoff + gamepad2.left_stick_x;

            spindexer.setTargetPosition(spinpos * SPINDEXER_TICKS_PER_POSITION + (int) spinoff);

            // --- Arm/Output movement ---
            Outake.setPower(targetOutake);
            Intake.setPower(targetIntake);
            // --- Telemetry ---

            telemetry.addData("Status", "Running V1.0");
            telemetry.addData("spinpos",  spinpos);
            telemetry.addData("Output Target", targetOutake);
            telemetry.addData("Output Pos", Outake.getCurrentPosition());
            telemetry.addData("Intake Target", targetIntake);
            telemetry.addData("Intake Pos", Intake.getCurrentPosition());
            telemetry.addData("System Time (ms)", timer.getTime());
            telemetry.update();
        }
    }
}
