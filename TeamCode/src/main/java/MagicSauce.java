

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@TeleOp(name="BioBuzz Teleop", group="Linear Opmode")
public class  MagicSauce extends LinearOpMode {



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

    // --- set positions for disengager ---
    private static final double DISENGAGER_DISENGAGED_POSITION = 1.0;
    private static final double DISENGAGER_ENGAGED_POSITION = -1.0;


    // --- important variables ---
    private int targetIntake = 0;
    private boolean Inactor = false;
    private double shooterOrder = 0;
    private double Reingager = 0;
    private DigitalChannel touchSensor;




    // --- Helper class for timing
    class Robott {
        long startTime;
        long currentTime;
        double xoff;
        double yoff;
        GoBildaPinpointDriver odo;


        void start(GoBildaPinpointDriver startodo) {

            odo = startodo;
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
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
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

            startTime = System.currentTimeMillis();
            currentTime = 0;
        }

        void update() {
            odo.update();
            currentTime = System.currentTimeMillis() - startTime;
        }

        long getTime() {
            return currentTime;
        }

        String getPosition() {
            Pose2D pos = odo.getPosition();
            return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

        }

        String getVelocity() {
            return String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

        }
    }

    @Override
    public void runOpMode() {
        // --- Hardware Mapping ---
        DcMotor Leftbw = hardwareMap.dcMotor.get("Leftbw");
        DcMotor Rightbw = hardwareMap.dcMotor.get("Rightbw");
        DcMotor Rightfw = hardwareMap.dcMotor.get("Rightfw");
        DcMotor Leftfw = hardwareMap.dcMotor.get("Leftfw");
        DcMotor Intake = hardwareMap.dcMotor.get("Intake");
        DcMotor Spool = hardwareMap.dcMotor.get("Spool");
        Servo Disengager = hardwareMap.get(Servo.class, "Disengager");
        touchSensor =  hardwareMap.get(DigitalChannel.class, "Reingager");

        // --- Drive motor setup ---
        Rightfw.setDirection(DcMotorSimple.Direction.REVERSE);
        Rightbw.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Intake setup ---
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setPower(targetIntake);

        // --- touch sensor setup ---
        touchSensor.setMode(DigitalChannel.Mode.INPUT);


        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();
        Robott timer = new Robott();
        timer.start(hardwareMap.get(GoBildaPinpointDriver.class,"odowheels"));


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            telemetry.addData("Position", timer.getPosition());

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            telemetry.addData("Velocity", timer.getVelocity());





        telemetry.addData("Status", "Initialized");


        waitForStart();
            //--- shooter controls ---
            if (gamepad2.left_bumper && !Inactor) {
                Inactor = true;
                shooterOrder += 1;
            }

            if (!gamepad2.left_bumper && shooterOrder == 0) {
                Inactor = false;
            }

            if (shooterOrder == 1) {
                Disengager.setPosition(DISENGAGER_DISENGAGED_POSITION);
                shooterOrder += 1;
            } else if (shooterOrder == 2) {
                if (0 == Reingager) {
                    Spool.setPower(0.3);
                } else if (Reingager == 1) {
                    Disengager.setPosition(DISENGAGER_ENGAGED_POSITION);
                    Spool.setPower(0);
                    shooterOrder += 1;
                }
            } else if (shooterOrder == 3) {
                shooterOrder = 0;
            }

           // --- intake controls ---
            if (gamepad2.dpad_up) {
                targetIntake = 7;
            } else if (gamepad2.dpad_down) {
                targetIntake = 0;
            }

            // --- Drive control ---
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
     
            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

            Leftfw.setPower((forward + strafe - turn/1.75) / denominator);
            Leftbw.setPower((forward - strafe - turn/1.75) / denominator);
            Rightfw.setPower((forward - strafe + turn/1.75) / denominator);
            Rightbw.setPower((forward + strafe + turn/1.75) / denominator);

            // --- touch sensor settings ---
            if (touchSensor.getState() == false) {
                Reingager = 1;
            } else {
                Reingager = 0;
            }



            // --- Telemetry ---

            telemetry.addData("shooterOrder" ,shooterOrder);
            telemetry.addData("Reingager" , Reingager);
            telemetry.update();
        }
    }
}
