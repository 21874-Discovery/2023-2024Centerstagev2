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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


@TeleOp(name="scoring", group="Linear Opmode")
public class scoring extends LinearOpMode {

    // intake
    private DcMotor         intakeMotor        = null;
    private DcMotor         conveyorMotor        = null;

    // arm
    private DcMotor         armMotor            = null;

    private Servo           clawServo           = null;

    // power for intake (can be changed later)
    private double          intakePower         = 0.5;

    // toggles so that only one input is registered per button press
    boolean dirToggle=true,clawToggle=true;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // init intake motors
        intakeMotor    = hardwareMap.get(DcMotor.class, "intakeMotor");
        conveyorMotor    = hardwareMap.get(DcMotor.class, "conveyorMotor");


        // init arm motor
        //armMotor        = hardwareMap.get(DcMotor.class, "armMotor");

        // init servo
        clawServo       = hardwareMap.get(Servo.class, "clawServo");

        // TODO: may have to flip these
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        conveyorMotor.setDirection(DcMotor.Direction.FORWARD);

        //arm direction

        //set default position of servo as 0
        clawServo.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /* READ INPUTS */

            // read current value of right bumper
            boolean rBumper = gamepad1.right_bumper;
            boolean lBumper = gamepad1.left_bumper;

            // read the face buttons
            boolean a = gamepad1.a;


            /* DO INTAKE STUFF */
            if (rBumper) {
                intakeMotor.setPower(intakePower);
                conveyorMotor.setPower(intakePower);
            }
            else {
                intakeMotor.setPower(0);
                conveyorMotor.setPower(0);
            }
            if (lBumper) {
                //only lets the code below run once
                if(dirToggle) {
                    //swaps the direction of the motors, therefore reversing the direction
                    if(intakeMotor.getDirection()==DcMotor.Direction.FORWARD){
                        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                        conveyorMotor.setDirection(DcMotor.Direction.REVERSE);
                    }
                    else if(intakeMotor.getDirection()==DcMotor.Direction.REVERSE){
                        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                        conveyorMotor.setDirection(DcMotor.Direction.FORWARD);
                    }
                    //disables further presses
                    dirToggle = false;
                }
            }
            else{

                //when button is released, let new button presses be registered
                dirToggle=true;
            }
            if(a){
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



            // TODO: print time the loop took to execute

            telemetry.update();
        }

        //armMotor.setPower(0.0);
        intakeMotor.setPower(0.0);
    }
}

//code for the arm

