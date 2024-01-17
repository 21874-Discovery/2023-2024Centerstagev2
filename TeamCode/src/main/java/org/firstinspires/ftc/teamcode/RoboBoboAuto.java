/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="RoboBobo", group="Robot")
public class RoboBoboAuto extends LinearOpMode {

    // declare OpMode members
    private FtcDashboard dashboard;

    private int step=1;

    private DcMotor topLeft,topRight,bottomLeft,bottomRight;
    private DcMotor intakeMotor;

    private double topLeftCount,topRightCount,bottomLeftCount,bottomRightCount;


    private final double ppr=537.7;
    private final double     dgr    = 1.0 ;     // No External Gearing.
    private final double     wd   = 3.77953 ;     // For figuring circumference
    private final double     tpi         = (ppr * dgr)/(wd * Math.PI);

    private double intakePower=dashboardData.intakeP;


    boolean toggle=false;

    private DistanceSensor distanceSensor = null;




    public boolean distanceCalc(double list[]){
        double sum=0;

        //get the sum
        for(int x=0;x<list.length;x+=1) {

            sum+=list[x];
        }

        //get the average
        double average=sum/list.length;
        double stdev=0;

        //standard deviation step 3
        for(int x=0;x<list.length;x+=1) {
            stdev+=Math.pow(list[x]-average,2);
        }

        //divide and square root
        stdev=Math.sqrt(stdev/list.length);

        //if average larger than 315, not detecting
        if(stdev<=1&&average>=315){
            return false;
        }

        /*
        if standard deviation is larger than 20, so dataset very spread apart,
        then nothing is detected
         */
        else if(stdev>=20&&average>=100){
            return false;
        }
        else if(stdev>=50){
            return false;
        }
        else{
            return true;
        }


    }

    public boolean distanceDetector(){
        double list[]=new double[10];

        for(int x=0;x<list.length;x+=1) {
            list[x]=distanceSensor.getDistance(DistanceUnit.INCH);
        }
        return distanceCalc(list);
    }

    public void DirectForward(){
        topLeft.setDirection(DcMotor.Direction.FORWARD);
        topRight.setDirection(DcMotor.Direction.REVERSE);
        bottomLeft.setDirection(DcMotor.Direction.FORWARD);
        bottomRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void DirectRight(){
        /*
                we're trying to strafe to the right, but because the robot
                is backwards, we strafe to the left instead
                the topleft is backwards, topRight is forwards, bottomleft
                is forwards, and bottomRight is backwards
                However, the motors on the left side need to be reversed in
                direction. So, topleft becomes forwards and bottomleft
                become backwards.
                 */

        topLeft.setDirection(DcMotor.Direction.FORWARD);
        topRight.setDirection(DcMotor.Direction.FORWARD);
        bottomLeft.setDirection(DcMotor.Direction.REVERSE);
        bottomRight.setDirection(DcMotor.Direction.REVERSE);

    }

    public void TurnClockwise(){
        /*Were trying to rotate clockwise but becuase the robot is backwards
        * the left side of the robot has to reverse. In the mecanum drive
        * directions bottom right and top right are forward while top left and
        * bottom left are reverse but they are forward becuase the left side
        * needs to be reverse*/

        topLeft.setDirection(DcMotor.Direction.FORWARD);
        topRight.setDirection(DcMotor.Direction.FORWARD);
        bottomLeft.setDirection(DcMotor.Direction.FORWARD);
        bottomRight.setDirection(DcMotor.Direction.FORWARD);
    }


    public void stopDirectandRun(){
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DirectForward();

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    public void endState(double distance){
        //setting target position for end state
        topLeft.setTargetPosition((int) (distance * tpi));
        topRight.setTargetPosition((int) (distance * tpi));
        bottomLeft.setTargetPosition((int) (distance * tpi));
        bottomRight.setTargetPosition((int) (distance * tpi));

        //stopping and running fuction
        stopDirectandRun();

    }





    @Override
    public void runOpMode() {

        topLeft=hardwareMap.get(DcMotor.class,"topLeft");
        topRight=hardwareMap.get(DcMotor.class,"topRight");
        bottomLeft=hardwareMap.get(DcMotor.class,"bottomLeft");
        bottomRight=hardwareMap.get(DcMotor.class,"bottomRight");

        intakeMotor=hardwareMap.get(DcMotor.class,"intakeMotor");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //zero power behav

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set position of wanted inches

        DirectForward();

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // init intake motors
        // init servo

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //STEP 1
        if(step==1) {
            //starting step for team prop autonomous

            topLeft.setPower(dashboardData.drivePower);
            topRight.setPower(dashboardData.drivePower);
            bottomLeft.setPower(dashboardData.drivePower);
            bottomRight.setPower(dashboardData.drivePower);

            //center detected
            if (!distanceDetector()) {
                step=3;

//TODO: it may be hitting the ground

                DirectRight();

                topLeft.setTargetPosition((int) (12 * tpi));
                topRight.setTargetPosition((int) (12 * tpi));
                bottomLeft.setTargetPosition((int) (12 * tpi));
                bottomRight.setTargetPosition((int) (12 * tpi));
                //TODO: may need to change right strafe values



            }
            //center not detected
            else {
                step=2;

                //running endstate function with distance of 29 inches to reach
                //the middle spikemark
                endState(29);



            }

            topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        while (opModeIsActive()) {

            if(step==2){
                //end state for team prop section of autonomous
                if (!topLeft.isBusy()) {
                    //if the robot finishes moving
                    if(!toggle) {
                        //turn on intake to outake pixel onto spikemark
                        intakeMotor.setPower(intakePower);


                        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                    }
                    else{
                        //stop the program
                        intakeMotor.setPower(0);

                        break;
                    }
                }
            }
            else if(step==3){
                //step that detects if team prop is on the spikemark farther from
                //the centerstage
                if (!topLeft.isBusy()) {
                    //keep moving robot until 4 revolutions

                    //if not detecting anything, get ready to go to step 4
                    if(!distanceDetector()) {
                        step=4;


                        topLeft.setTargetPosition((int)(30*tpi));
                        topRight.setTargetPosition((int)(30*tpi));
                        bottomLeft.setTargetPosition((int)(30*tpi));
                        bottomRight.setTargetPosition((int)(30*tpi));




                    }
                    else{


                        step=2;

                        endState(20);
                        //move to the spikemark farthest from the truss
                        //TODO: ADJUST THE MOVING VALUE

                    }


                    stopDirectandRun();



                }
            }
            else if(step==4){
                //getting ready to move to the spikemark closest to the truss
                if(!topLeft.isBusy()) {
                    if(!toggle) {
                        //rotate 90 clockwise
                        topLeft.setTargetPosition((int) (17.28 * tpi));
                        topRight.setTargetPosition((int) (17.28 * tpi));
                        bottomLeft.setTargetPosition((int) (17.28 * tpi));
                        bottomRight.setTargetPosition((int) (17.28 * tpi));


                        stopDirectandRun();

                        TurnClockwise();

                        toggle=true;
                    }
                    else{
                        step=2;
                        endState(16);
                        //move 16 inches to spikemark closest to truss
                        //TODO: NEED TO CHANGE VALUES LATER
                        toggle=false;
                    }

                }
            }

            telemetry.addData("topLeft: ", topLeft.getCurrentPosition());
            telemetry.addData("topRight: ", topRight.getCurrentPosition());
            telemetry.addData("bottomLeft: ", bottomLeft.getCurrentPosition());
            telemetry.addData("bottomRightt: ", bottomRight.getCurrentPosition());
            telemetry.addData("distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("detecting?: ", distanceDetector());
            telemetry.update();

        }
        topLeft.setPower(0.0);
        topRight.setPower(0.0);
        bottomLeft.setPower(0.0);
        bottomRight.setPower(0.0);
        intakeMotor.setPower(0.0);

    }
}