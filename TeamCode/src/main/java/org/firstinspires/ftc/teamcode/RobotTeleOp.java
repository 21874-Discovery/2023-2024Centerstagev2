/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (sconditions are met:
 *
 * Redistriubject to the limitations in the disclaimer below) provided that
 * the following butions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


@TeleOp(name="RobotTeleOp", group="Linear Opmode")
public class RobotTeleOp extends LinearOpMode {
    private FtcDashboard dashboard;

    // declare OpMode members
    // drivetrain
    private DcMotor         topLeft           = null;     //port 0 - control hub
    private DcMotor         topRight          = null;     //port 0 - expansion hub
    private DcMotor         bottomLeft            = null; //port 1 - control hub
    private DcMotor         bottomRight           = null; //port 1 - expansion hub

    // intake
    private DcMotor         intakeMotor          = null;   //port 2 - control hub

    // arm
    private DcMotor         armLeft            = null;
    private DcMotor         armRight            = null;//port 2 - expansion hub

    private Servo           clawServo           = null;   //port 0 - expansion  hub

    private Servo           dlServo           = null;     //port 0 - control hub

    // power for intake (can be changed later)
//   private double          intakePower         = dashboardData.intakePower
    private double          intakePower        = 1;

    private boolean dlTog=true;

    //dawinching
    //private DcMotor winchMotor = null;


    // toggles so that only one input is registered per button press
    boolean dirToggle=true,clawToggle=true;

    //arm control variables
    private double prevArm = 0.0;
    private double ppr = 4.0;   //ticks per revolution
    private double gr = 72.0;   //gear ratio
    private boolean justStopped=true;

    private boolean manualMode = false;
    private double armSetpoint = 0.0;
    private final double armManualDeadband = 0.03;
    private final int armIntakePosition = 10;
    private final int armScorePosition = -200;
    private final int armShutdownThreshold = 5;

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        telemetry.addData("Status", "newest INIT 1/13/23");
        telemetry.update();

        // init the drive system variables
        topLeft       = hardwareMap.get(DcMotor.class, "topLeft");
        topRight      = hardwareMap.get(DcMotor.class, "topRight");
        bottomLeft        = hardwareMap.get(DcMotor.class, "bottomLeft");
        bottomRight       = hardwareMap.get(DcMotor.class, "bottomRight");

        // init intake motors
        intakeMotor     = hardwareMap.get(DcMotor.class, "intakeMotor");


        // init arm motor
        armLeft        = hardwareMap.get(DcMotor.class, "armLeft");
        armRight        = hardwareMap.get(DcMotor.class, "armRight");

        // init servo
        clawServo       = hardwareMap.get(Servo.class, "clawServo");

        dlServo       = hardwareMap.get(Servo.class, "dlServo");

        //winch motor
        //winchMotor = hardwareMap.get(DcMotor.class, "winchMotor");

        // left is reverse of right
        topLeft.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.FORWARD);
        bottomLeft.setDirection(DcMotor.Direction.REVERSE);
        bottomRight.setDirection(DcMotor.Direction.FORWARD);

        // TODO: may have to flip these
        //inputs by default
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder position

        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeft.setTargetPosition(0);


        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder position

        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armRight.setTargetPosition(0);

        //set default position of servo as 0
        clawServo.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /* READ INPUTS */
            // TODO: SET UP GAMEPAD SPECIFIC INPUTS
            // read current values of joystick
            float lx    = gamepad1.left_stick_x;
            float ly    = -gamepad1.left_stick_y;
            float rx    = gamepad1.right_stick_x;
            float ry    = -gamepad1.right_stick_y;

            // read current value of right bumper
            boolean lBumper = gamepad1.left_bumper;

            //trigger readings
            float rTrigger = gamepad1.right_trigger;
            float lTrigger = gamepad1.left_trigger;



            boolean y = gamepad1.y;

            // read current values of gamepad 2 joystick
            double r2y =gamepad2.right_stick_y;
            // read the face buttons
            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            boolean y2 = gamepad2.y;

            boolean bump2 = gamepad2.left_bumper&&gamepad2.right_bumper;



            /* DO MOTOR STUFF */
            // add the abs of all joystick inputs
            // get the max of ^ and 1.0
            // used to ensure controller inputs are proportionally applied to motor outputs
            float max   = Math.max( Math.abs(lx)+Math.abs(ly)+Math.abs(rx) , 1.0f);

            // normalize each input by either the sum of all 3 joysticks or 1.0f
            float nlx   = lx / max;
            float nly   = ly / max;
            float nrx   = rx / max;

            // apply mechanum drive equations to
            // create commands for each drivetrain motor
            double fl   = nlx   +nly    +nrx;
            double bl   = -nlx  +nly    +nrx;
            double fr   = -nlx  +nly    -nrx;
            double br   = nlx   +nly    -nrx;

            double dtPower = 1.0;


            if(lBumper){
                dtPower = 1.0;
            }

            else{
                dtPower = 0.5;
            }

            // give motor commands
            topLeft.setPower(fl*dtPower);
            bottomLeft.setPower(bl*dtPower);
            topRight.setPower(fr*dtPower);
            bottomRight.setPower(br*dtPower);


            /* DO INTAKE STUFF */
            if (lTrigger>=0.05) {
                intakeMotor.setPower(intakePower);
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else {
                intakeMotor.setPower(0);
            }
            if(rTrigger>=0.05){
                intakeMotor.setPower(intakePower);
                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else{
                intakeMotor.setPower(0);
            }
           // if (lBumper) {
                //only lets the code below run once
                /*if(dirToggle) {
                    //swaps the direction of the motors, therefore reversing the direction
                    if(intakeMotor1.getDirection()==DcMotorSimple.Direction.FORWARD){
                        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                    }
                    else if(intakeMotor1.getDirection()==DcMotorSimple.Direction.REVERSE){
                        intakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
                        intakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    //disables further presses
                    dirToggle = false;
                }
            }
            else{

                //when button is released, let new button presses be registered
                dirToggle=true;
            } */
            if(a2){
                if(clawToggle) {
                    if(clawServo.getPosition()==0||clawServo.getPosition()==-1) {
                        clawServo.setPosition(1);
                    }
                    else{
                        clawServo.setPosition(-1);
                    }
                    clawToggle=false;
                }
            }
            else{
                clawToggle=true;
            }

            //SOURCE: https://docs.revrobotics.com/ftc-kickoff-concepts/centerstage-2023-2024/programming-teleop
            //ARM
//            if (Math.abs(r2y) > 0.05) {
//                if (!manualMode) {
//                    armMotor.setPower(0.0);
//                    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    manualMode = true;
//                }
//                armMotor.setPower(r2y);
//            }
//            else {
//                if (manualMode) {
//                    armMotor.setTargetPosition(armMotor.getCurrentPosition());
//                    armMotor.setPower(1.0);
//                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    manualMode = false;
//                }
//
//                //preset buttons
//
//                else if (b2) {
//                    armMotor.setTargetPosition(armIntakePosition);
//                    armMotor.setPower(1.0);
//                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//                else if (y2) {
//                    armMotor.setTargetPosition(armScorePosition);
//                    armMotor.setPower(1.0);
//                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//            }
//
//            //Re-zero encoder button
//            if (gamepad2.start) {
//                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                armMotor.setPower(0.0);
//                manualMode = false;
//            }
//
//            //Watchdog to shut down motor once the arm reaches the home position
//            if (!manualMode &&
//                    armMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
//                    armMotor.getTargetPosition() <= armShutdownThreshold &&
//                    armMotor.getCurrentPosition() <= armShutdownThreshold
//            ) {
//                armMotor.setPower(0.0);
//                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }


            //Felix, Max and John's Arm code
            // if no joystick, hold position

            if(bump2){
                armLeft.setPower(-r2y);
                armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                armRight.setPower(-r2y);
                armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else{
                /*if(Math.abs(r2y)<=0.05){
                    if(justStopped) {
                        armLeft.setTargetPosition(armLeft.getCurrentPosition());
                        armLeft.setPower(1.0);

                        armRight.setTargetPosition(armRight.getCurrentPosition());
                        armRight.setPower(1.0);
                        justStopped=false;
                    }
                    armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
                else {
                    justStopped=true;
                    armLeft.setPower(r2y*2/3);
                    armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    armRight.setPower(r2y*2/3);
                    armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }*/
                armLeft.setPower(r2y*1/3);
                armRight.setPower(r2y*1/3);
            }


             // if joystick, then command the motor proportionally with the input
             /*if (Math.abs(r2y) < Math.abs(prevArm)) { //take ABS of r2y??? 12/30/2023
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             }
             else {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setTargetPosition(armMotor.getCurrentPosition());
                prevArm = r2y;
             }*/


            //setwinchPower
         /*winchMotor.setPower(0.0);

         if (rTrigger >= 0.05) {
            winchMotor.setPower(rTrigger);
         }
         else if (lTrigger >= 0.05) {
            winchMotor.setPower(-(lTrigger));
         }
         if (rTrigger >= 0.05 && lTrigger >= 0.05) {
            winchMotor.setPower(rTrigger-lTrigger);
         }*/

            /* PRINT STUFF */



            //when button is released, let new button presses be registered
            if(y){
                dlServo.setPosition(0);
            }

         /*
         // print raw joystick readings
         telemetry.addData("lx: ", lx);
         telemetry.addData("ly: ", ly);
         telemetry.addData("rx: ", rx);

         // print the result of the max function
         telemetry.addData("max: ", max);

         // print normalized joystick readings
         telemetry.addData("nlx: ", nlx);
         telemetry.addData("nly: ", nly);
         telemetry.addData("nrx: ", nrx);

         // print motor command values
         telemetry.addData("fl: ", fl);
         telemetry.addData("bl: ", bl);
         telemetry.addData("fr: ", fr);
         //wrote "New" so we can know if it's the newest version
         telemetry.addData("br: New", br);*/
        telemetry.addData("Current Power NEWEST CODE 1/13/24👍", armLeft);
        telemetry.addData("Current Y 👍", r2y);
        telemetry.update();
            // TODO: print time the loop took to execute

            telemetry.update();
        }

        // stop all the motors
        topLeft.setPower(0.0);
        topRight.setPower(0.0);
        bottomLeft.setPower(0.0);
        bottomRight.setPower(0.0);
        armLeft.setPower(0.0);
        armRight.setPower(0.0);
        intakeMotor.setPower(0.0);
    }
}

//code for the arm

