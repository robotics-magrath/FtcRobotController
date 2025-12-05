package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Decode Teleop", group="Linear Opmode")
public class jhhhhDecodeTeleop extends LinearOpMode {

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

    private static final double CLAW_POSITION_INIT = 0.9;
    private static final double CLAW_OPEN_POSITION = 0.3;
    private static final double CLAW_CLOSED_POSITION = 0.8;

    private int targetOutput = 0;
    private int targetInput = 0;
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
        DcMotor Leftfw = hardwareMap.dcMotor.get("Leftfw");
        DcMotor Rightfw = hardwareMap.dcMotor.get("Rightfw");
        DcMotor Input = hardwareMap.dcMotor.get("Input");
        DcMotor Output = hardwareMap.dcMotor.get("Output");


        // --- Drive motor setup ---
        Rightfw.setDirection(DcMotorSimple.Direction.REVERSE);
        Rightbw.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Arm/Output setup ---
        Input.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Output.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Input.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Output.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Input.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Output.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Robott timer = new Robott();
        timer.start();

        while (opModeIsActive()) {
            timer.update();

            // --- Manual arm control with d-pad ---
            if (gamepad2.dpad_up) {
                targetOutput = 10;

            } else if (gamepad2.dpad_down) {
                targetOutput = 0;

            } else if (gamepad2.dpad_left) {
                targetInput = -10;
            } else if (gamepad2.dpad_right) {
                targetInput = 0;
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

            // --- Arm/Output movement ---
            Output.setPower(targetOutput);
            Input.setPower(targetInput);

            // --- Telemetry ---
            telemetry.addData("Status", "Running");
            telemetry.addData("Output Target", targetOutput);
            telemetry.addData("Output Pos", Output.getCurrentPosition());
            telemetry.addData("Input Target", targetInput);
            telemetry.addData("Input Pos", Input.getCurrentPosition());
            telemetry.addData("System Time (ms)", timer.getTime());
            telemetry.addData("Claw", clawOpen ? "Open" : "Closed");
            telemetry.update();
        }
    }
}
