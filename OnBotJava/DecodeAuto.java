package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Decode Auto", group="Linear Opmode")
public class DecodeAuto extends LinearOpMode {

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

    private int targetOutput = 0;
    private int targetInput = 0;
    private int bumptimer = 0;

    private boolean clawOpen = true;

    // --- Helper class for timing
    class Robott {
        private int delay = 0000;
        long startTime;
        long curentime;

        void start() {
            startTime = System.currentTimeMillis();
            curentime = 0;
        }

        void update() {
            curentime = System.currentTimeMillis() - startTime;
        }

        double autodrive(){
    if ((curentime > delay) && (curentime < delay + 1200)) {
            return 1;
    }
        return  0;
        
    }
    
       
    double autostrafe(){
        
        return  0;
    }
    double autoturn(){
        if ((curentime  > delay + 1200) && (curentime < delay + 2050)){
            return -1;
         }
        return 0;
    }
    double autoshoot()
    {
        if (curentime > delay){
        return 1;
        }
        return 0;
    }
    
    double autoload(){
         if (curentime  > delay + 2050) {
            return 1;
    }
    return 0;
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

        CRServo intake1 = null;
        CRServo intake2 = null;
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Robott bartholemew = new Robott();
        bartholemew.start();
        while (opModeIsActive())
        {
           bartholemew.update();

            // --- Manual arm control with d-pad ---
           double shoot = bartholemew.autoshoot();
           double load = bartholemew.autoload();
            if (shoot == 1){
            targetOutput = -10;
            intake2.setPower(-1);
            }else{
            targetOutput = 0;
            intake2.setPower(0); 
            }
            if (load == 1){
            intake1.setPower(-1);
            }else{
                intake1.setPower(0);
            }
             {
                
           

            // --- Drive control ---
            double forward = bartholemew.autodrive();
            double strafe = bartholemew.autostrafe();
            double turn = bartholemew.autoturn();

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
            telemetry.addData("Claw", clawOpen ? "Open" : "Closed");
            telemetry.update();
        }
    }
}
}
