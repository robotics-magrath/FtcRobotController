import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.math.MathContext;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Decode odo", group="Linear Opmode")
public class technotest extends LinearOpMode {



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

    private final RobotState currentState = RobotState.INIT;

    // Arm and wrist target positions for each state
   

    private static final double DISENGAGER_DISENGAGED_POSITION = 1.0;
    private static final double DISENGAGER_ENGAGED_POSITION = -1.0;
    private HuskyLens huskyLens;
    private int targetOutake = 0;
    private int targetIntake = 0;
    private int spinpos = 0;
    private final double spinoff = 0;
    private final int spinWaitTime = 0;
    private double Xcord = 0;
    private double Ycord = 0;
    private double Dcord = 0;
    private double Order = 0;
    private double Xps = 0;
    private double Yps = 0;
    private double Dps = 0;
    private double Distance =0;
    private double Dswipswap = 0;
    private double point = 0;
    private final int READ_PERIOD = 1;
    private boolean Inactor = false;
    private double shooterOrder = 0;
    private double Reingager = 0;

    private double autoShoot = 0;
    private DigitalChannel touchSensor;

    private final boolean spinWait = false;

    // --- Helper class for timing
    static class Robott {
        private enum TechnoState {
            STATE,
            MOVE,
            SHOOT,
            DELAY,
            TURN,
            FLICKER,
            ODOSTOPLINE,
            NEXT,
            SHOOTSPIN,
            SHOOTSPINDOWN,

        }
        private TechnoState myState = TechnoState.NEXT;
        private final Robott.TechnoState currentState = TechnoState.NEXT;
        public double autoForward = 0;
        public double autoTurn = 0;
        public double autoStrafe = 0;
        public int autoOutput = 0;
        public int autoInput = 0;
        public boolean autoFlick = false;
        public int autoSpinBump = 0;

        int instruction = 0;
        long startTime;
        long currentTime;

        long targetTime;
        double targetxoff;
        double targetyoff;
        GoBildaPinpointDriver odo;


        void start(GoBildaPinpointDriver startodo) {
            myState = TechnoState.NEXT;
            odo = startodo;
             /*
        Set the odometry pod positions relative to the point that
         the odometry computer tracks around.
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
            switch(myState) {
                case MOVE:
                    autoForward = 1;
                    if (currentTime > targetTime) {
                        myState = TechnoState.NEXT;
                        autoForward = 0;
                    }
                    break;
                case SHOOT:
                    autoOutput = 1;
                    autoFlick = true;

                    if (currentTime > targetTime) {
                        myState = TechnoState.SHOOTSPIN;
                    }
                    break;
                case SHOOTSPIN:
                    autoSpinBump= 2;
                    myState = TechnoState.SHOOTSPINDOWN;
                    targetTime = currentTime + 500;
                    break;
                case SHOOTSPINDOWN:
                    autoSpinBump =0;
                    if (currentTime > targetTime) {
                        autoOutput = 0;
                        autoFlick = false;
                        myState = TechnoState.NEXT;
                    }
                    break;
                case DELAY:
                    if (currentTime > targetTime) {
                        myState = TechnoState.NEXT;
                    }
                    break;
                case TURN:
                    autoTurn = 1;
                    if (currentTime > targetTime) {
                        myState = TechnoState.NEXT;
                        autoTurn = 0;
                    }
                    break;
                case ODOSTOPLINE:
                    Pose2D pos = odo.getPosition();
                    if (pos.getY(DistanceUnit.MM) >= targetyoff) {
                        autoForward = 0;
                        myState = TechnoState.NEXT;
                    }
                    break;
                case NEXT:

            }
        }

        long getTime() {
            return currentTime;
        }



        String getPositionString() {
            Pose2D pos = odo.getPosition();
            return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.RADIANS));

        }

        String getVelocityString() {
            return String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

        }

        Double getXcoord() {
            Pose2D pos = odo.getPosition();
            return pos.getX(DistanceUnit.CM);
        }
        Double getYcoord() {
            Pose2D pos = odo.getPosition();
            return pos.getY(DistanceUnit.CM);
        }
        Double getOrientation() {
            Pose2D pos = odo.getPosition();
            return pos.getHeading(AngleUnit.RADIANS);
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
        // Husky Hardware Mapping
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // --- Drive motor setup ---
        Rightfw.setDirection(DcMotorSimple.Direction.REVERSE);
        Rightbw.setDirection(DcMotorSimple.Direction.REVERSE);
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);




        /*
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();*/

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();
        Robott timer = new Robott();
        timer.start(hardwareMap.get(GoBildaPinpointDriver.class,"odowheels"));


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */


            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
            //odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);


            /*if (gamepad1.a){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }/*

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double counter = 1;
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            double loaded = 1;
            double Autocenter = 0;
            oldTime = newTime;
            //Huskylens
            if (loaded == 1) {
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                HuskyLens.Block[] blocks = huskyLens.blocks();

                if (blocks.length > 0) {
                    // Grab the first detected tag
                    int tagId = blocks[0].id;
                    int xCenter = blocks[0].x;
                    int width = blocks[0].width;
                    int yCenter = blocks[0].y;

                    telemetry.addData("Tag ID", tagId);
                    telemetry.addData("X Position", xCenter);
                    telemetry.addData("Y Position", yCenter);

                    if (tagId == 1) {
                        Autocenter = 1;
                    } else if (tagId == 2) {
                        Autocenter = 1;
                    } else if (tagId == 3) {
                        Autocenter = 1;
                    } else if (tagId == 4) {
                        Autocenter = 1;
                    } else if (tagId == 5) {
                        Autocenter = 1;
                    } else if (tagId == 6) {
                        Autocenter = 1;
                    } else if (tagId == 7) {
                        Autocenter = 1;
                    } else if (tagId == 8) {
                        Autocenter = 1;
                    } else if (tagId == 9) {
                        Autocenter = 1;
                    } else if (tagId == 10) {
                        Autocenter = 1;
                    } else if (tagId == 11) {
                        Autocenter = 1;
                    } else if (tagId == 12) {
                        Autocenter = 1;
                    } else if (tagId == 13) {
                        Autocenter = 1;
                    } else if (tagId == 14) {
                        Autocenter = 1;
                    } else if (tagId == 15) {
                        Autocenter = 1;
                    } else if (tagId == 16) {
                        Autocenter = 1;
                    }
                    if (Autocenter == 1) {
                        if (xCenter > 180) {
                            timer.autoTurn = -1;
                        } else if (xCenter < 140) {
                            timer.autoTurn = 1;
                        } else {
                            timer.autoTurn = 0;
                        }
                        if (yCenter > 100) {
                            timer.autoForward = 1;
                        } else if (yCenter < 60) {
                            timer.autoForward = -1;
                        } else {
                            timer.autoForward = 0;
                        }
                    }
                }
            } else if (loaded == 0) {
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
            }

            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Xcord = timer.getXcoord();
            Ycord = timer.getYcoord();
            Dcord = timer.getOrientation();


            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            if (autoShoot == 1 && !Inactor) {
                Inactor = true;
                shooterOrder += 1;
                autoShoot = 0;
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



            double Speed = (Distance/10);
            double Ddif = (Dps - Dcord);
            double Xdif = (Xps - Xcord);
            double Ydif = (Yps - Ycord);
            double Dspeed = (Math.abs(Dps - Dcord)/2);

            Distance = (Math.sqrt((Math.pow((Xdif),2)) + (Math.pow((Ydif),2))));

            Dps = Math.atan2(Xdif, Ydif);
            if (Ddif < -0.05) {
                Dswipswap = 1;
            }
            else if (Ddif > 0.05){
                Dswipswap = 0;
            } else {
                Dswipswap = 2;
            }

            if (Dswipswap == 0) {
               timer.autoTurn = Dspeed;
               point = 0;
            } else if (Dswipswap == 1) {
                timer.autoTurn = (Dspeed * -1);
                point = 0;
            } else {
                point = 1;
            }

            if (Distance >= 20 && point == 1) {
                timer.autoForward = Speed;
            } else if (Distance <= -20 && point == 1) {
                timer.autoForward = -Speed;
            } else {
                timer.autoForward = 0;
            }



                if (Xcord == Xps && Ycord == Yps) {
                Order += 1;
                }



            waitForStart();
            Yps = 0;
            Xps = 0;
            if (Order == 1) {
                Xps = 50;
            }

            
            //allows a wait before you can press the button again

            spinpos = (spinpos + timer.autoSpinBump) ;



            timer.update();

            // --- Manual arm control with d-pad ---
            targetOutake = timer.autoOutput;

            targetIntake = timer.autoInput;



            // --- Drive control ---
            double forward = timer.autoForward;
            double strafe = timer.autoStrafe;
            double turn = timer.autoTurn;
            boolean flick = timer.autoFlick;

            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

            Leftfw.setPower((forward + strafe + turn) / denominator);
            Leftbw.setPower((forward - strafe + turn) / denominator);
            Rightfw.setPower((forward - strafe - turn) / denominator);
            Rightbw.setPower((forward + strafe - turn) / denominator);

            telemetry.addData("Velocity", timer.getVelocityString());
            telemetry.addData("Position", timer.getPositionString());
            telemetry.addData("Point" , point);
            telemetry.addData("Dswipswap" , Dswipswap);
            telemetry.update();
        }
    }
}