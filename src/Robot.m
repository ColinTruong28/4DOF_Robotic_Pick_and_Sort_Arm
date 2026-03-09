% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot(dummy)
            arguments
                dummy (1,1) logical = false
            end

            self = self@OM_X_arm(dummy);

            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            if checkSafe(goals)
                goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
                self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
            end
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            disp("time")
            disp(time_ms)
            disp("acc time")
            disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end
    
        %% LAB 1-----------------------------------------------------------
        % ALL code for lab 1 goes in this section ONLY

        function servo_jp(self, q)
        %SERVO_JP Send robot to a joint configuration
        % Inputs:
        %    q: a [1x4] vector containing the target joint positions
            
            % YOUR CODE HERE
            % Hint: look at lab1_base.m to see how to move your robot
            % Hint 2: don't set the travel time to 0, as that causes 
            %         ~unintended behaviors~; do something small like
            %         0.001
            self.writeTime(0.001);
            self.writeMotorState(true);
            self.writeJoints(q);

        end
        
        function interpolate_jp(self, q, time)
        %INTERPOLATE_JP Send robot to a joint configuration over a period of time
        % Inputs:
        %    q: a [1x4] vector containing the target joint positions
        %    t: a scalar that tells how long to take to travel to the new position
        %       in milliseconds

            % YOUR CODE HERE
        self.writeTime(time);
        self.writeMotorState(true);
        self.writeJoints(q);
        end

        function q_curr = measure_js(robot, GETPOS, GETVEL)
        %MEASURED_JS Get the current position and velocity of the robot
        % Inputs:
        %    GETPOS: a boolean indicating whether or not to retrieve joint
        %            positions
        %    GETVEL: a boolean indicating whether or not to retrieve joint
        %            velocities
        % Outputs:
        %    q_curr: a [2x4] matrix whose top row contains joint positions (0s if
        %            GETPOS is false), and whose bottom row contains joint 
        %            velocities
        
            % This line gets the current positions of the joints
            % (robot.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            
            % This line gets the current joint velocities
            % (robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL);
            
            % YOUR CODE HERE
            % Hint: Only collect the data you need; serial read operations
            %       are expensive!
            Temp = zeros(2, 4);
            if (GETPOS) 
                Temp(1, :) = ((robot.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG);
            end
            if (GETVEL)
                Temp(2, :) = (robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL);
            end
            q_curr = Temp;

        end
        %% END LAB 1 CODE -------------------------------------------------
        %% BEGIN LAB 2 CODE -----------------------------------------------
        function ht = dh2mat(self, dh_row)
        %DH2MAT Gives the transformation matrix for a given row of a DH table
        % Inputs:
        %    dh_row: a [1x4] vector containing representing a single row of a DH
        %            table
        % Outputs: 
        %    ht: a [4x4] matrix representing the transformation defined by the DH
        %        row

            % YOUR CODE HERE
            % Hint: Use the degree version of trig functions

            % These variables are extremely self explanatory
            theta_z = dh_row(1);
            displacement_z = dh_row(2);
            displacement_x = dh_row(3);
            theta_x = dh_row(4);

            % Homogenous transformation matrix to handle a rotation about z,
            % displacement about z, displacement about x, and then rotation
            % about x in accordance with the DH formulation
            ht = [cosd(theta_z) (-sind(theta_z)*cosd(theta_x)) (sind(theta_z)*sind(theta_x)) (displacement_x*cosd(theta_z));
                    sind(theta_z) (cosd(theta_z)*cosd(theta_x)) (-cosd(theta_z)*sind(theta_x)) (displacement_x*sind(theta_z));
                    0 sind(theta_x) cosd(theta_x) displacement_z;
                    0 0 0 1]
        end

        function ht = dh2fk(self, dh_tab)
        %DH2MAT_R Calculates FK from a DH table
        % Inputs:
        %    dh_tab: a [nx4] matrix representing a DH table
        % Outputs: 
        %    ht: a [4x4] matrix representing the transformation defined by the DH
        %        table

            % YOUR CODE HERE
            % Hint: Recall from lecture that the full FK of a manipulator can
            % be obtained by post multiplying all the HT matrices from each row
            % of the DH table
    
            % Hint 2: You just wrote a function to calculate the HT matrix
            % corresponding to one row of the DH table
            temp = dh2mat(self, dh_tab(1, :)); % first dh homogenous transformation matrix
            
            if size(dh_tab,1) > 1 % if there is more than one row
                for r = 2:height(dh_tab) % for each row in the table
                    temp2 = dh2mat(self, dh_tab(r, :)); % take the next row, make it into transformation matrix
                    temp = temp * temp2; % and multiply it with the previous, ensuring they stack until we've gone through all rows
                end
            end
            ht = temp;
        end

        % YOUR CODE HERE
        % Write a function fk3001 that takes in an array of four joint
        % values and uses your matlabFunction'd FK to generate the FK of
        % your robot
        function ht = fk3001(self, joints)

            % joints = joints(:).';
            load('fk3001_function.mat') %loads the concrete fk function generated from matlabFunction
            ht = fkFunc(joints(1), joints(2), joints(3), joints(4)); % feed the function joint values to get full transformation matrix from base to EE
        end

        % Hint: All non-static methods (which this is) should take "self" as
        % their first argument

        %% END LAB 2 CODE
        %% BEGIN LAB 3 CODE
        function qs = ik3001(self, pos)
            %IK3001_R Calculates IK for the OMX arm 
            % Inputs: 
            % pos: a [nx4] matrix [x, y, z, alpha] representing a target pose in 
            % task space 
            % Outputs: 
            % qs: a [1x4] matrix representing the joint values needed to reach the 
            % target position

            x = pos(1);
            y = pos(2);
            z = pos(3);
            gamma = pos(4);
        
            L1 = 96.326;         
            L2 = sqrt(128^2 + 24^2);      
            L3 = 124;
            L4 = 133.4;
        
            % YOUR CODE HERE 
            % Hint: MATLAB debugger is your very best friend in the 
            % entire world right now 

            extended_arm = L2 + L3 + L4 + 1e-2; % added a .01mm tolerance
            
            if (z < 0) || (sqrt(x^2 + y^2 + (z - L1)^2) > extended_arm) 
                error("Invalid target: Outside of Robot Bounds");
            end
        
            theta1 = atan2d(y, x);
            x_prime = x - cosd(theta1) * L4 * cosd(gamma);
            y_prime = y - sind(theta1) * L4 * cosd(gamma);
            z_prime = z - L4 * sind(gamma);
        
            % Reduce to planar r-z problem
            A = x_prime*cosd(theta1) + y_prime*sind(theta1);
            B = z_prime - L1;
        
            C = hypot(A, B);
 
 
            % clamp for numerical safety before sqrt/atan2
            cos_elbow = (C^2 - L2^2 - L3^2) / (2*L2*L3);
            cos_elbow = max(-1, min(1, cos_elbow));
            
            sin_elbow = [1, -1] * sqrt(max(0, 1 - cos_elbow^2));
        

            % Angle from shoulder to wrist
            phi = atan2d(B, A);
        
            wrap180 = @(a) mod(a+180,360)-180;   % [-180,180)

            % penalty for violating +/-90 (0 if inside)
            viol = @(t) max(0, abs(t) - 90);
            
            % variables for the tie breaker 
            best_cost = inf;
            bestTheta2 = 0;
            bestTheta3 = 0;
            bestTheta4 = 0;
            
            for candidate = sin_elbow
                a = phi - atan2d(L3*candidate, L2 + L3*cos_elbow);
                theta2 = 79.38 - a;
            
                elbow = atan2d(candidate, cos_elbow);
                theta3 = -(elbow + 79.38);
            
                theta4 = -gamma - theta2 - theta3;
            
                % wrap to compare consistently
                t2 = wrap180(theta2);
                t3 = wrap180(theta3);
                t4 = wrap180(theta4);
            
                % soft-limit penalty (huge weight so it only matters when needed)
                limit_pen = 1e6 * (viol(t2)^2 + viol(t3)^2 + viol(t4)^2);
            
                % tie-breaker if both are within limits (or equally violating):
                % prefer "less extreme" angles overall
                mag_pen = (t2^2 + t3^2 + t4^2);
            
                cost = limit_pen + mag_pen;
            
                if cost < best_cost
                    best_cost = cost;
                    bestTheta2 = t2;
                    bestTheta3 = t3;
                    bestTheta4 = t4;
                end
            end
            
            qs = [theta1, bestTheta2, bestTheta3, bestTheta4];

        end

        %% END LAB 3 CODE
        %% BEGIN LAB 4 CODE
        function j = jacob3001(self, qpos) 
            %JACOB3001 Calculates the jacobian of the OMX arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            % Outputs:
            %   j: a [6x4] jacobian matrix of the robot in the given pos
            load("jacobian_mat.mat");

            j = jacobian(qpos(1), qpos(2), qpos(3), qpos(4));
        end

        function vs = dk3001(self, qpos, qvel)
            %DK3001 Calculates the forward velocity kinematics of the OMX
            %arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            %   qvel: a [1x4] matrix composed of joint angular velocities
            % Outputs:
            %   vs: a [6x1] matrix representing the linear and angular 
            %       velocity of the end effector in the base frame of the 
            %       robot
            
            % YOUR CODE HERE

            J = self.jacob3001(qpos);
            qvel = reshape(qvel, 4, 1);

            vs = J * qvel;

        end
            
        % YOUR CODE HERE: Write a function atSingularity as described in
        % the lab document

        function as = atSingularity(self, qpos, threshold)
            J = self.jacob3001(qpos);
            Jp = J(1:3, 1:4);
            temp = det(Jp * Jp.');

            as = (temp < threshold);

        end    

    %% END LAB 4 CODE
    %% BEGIN LAB 5 CODE
    function blocking_js_move(self, qpos, nvargs)
        arguments
            self Robot;
            qpos double;
            nvargs.time double = 2;
        end
        %BLOCKING_JS_MOVE moves the robot to a position in joint space
        %before exiting
        % Inputs:
        %   qpos: a [1x4] matrix of joint positions
        %   time: (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        
        % YOUR CODE HERE
        % NOTE: this funciton should not exit until the robot has stopped
        % moving
        interpolate_jp(self, qpos, nvargs.time)
        while (abs([0 0 0 0; 0 0 0 0] - measure_js(self, false, true)) < ones([2,4]) * 0.01)
            pause(.1);
        end
    end

    function blocking_ts_move(self, pos, nvargs)
        arguments
            self Robot;
            pos double;
            nvargs.time double = 2;
            nvargs.mode string = "quintic"
        end
        tg = TrajGenerator();
        %BLOCKING_TS_MOVE moves the robot in a straight line in task space 
        %to the target position before exiting
        % Inputs:
        %   pos: a [1x4] matrix representing a target x, y, z, alpha
        %   time (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        %   mode (optional): a string "cubic" or "quintic" indicating what 
        %                    type of trajectory to utilize

        % YOUR CODE HERE
        % NOTE: this funciton should not exit until the robot has stopped
        % moving
        % Generate a trajectory that will move your robot in a straight line from
        % your current (X, Y, Z) to the point (X, -Y, Z)
        v0 = 0; vf = 0;
        acc0 = 0; accf = 0;
        tf = nvargs.time;
    
        temp = measure_js(self, true, false);
        %ikTemp = [temp(1, 1) temp(1, 2) temp(1, 3) (temp(1, 2) + temp(1, 3) + temp(1, 4))];
        fkTemp = fk3001(self, temp(1, :));
        currPose(:, 1) = fkTemp(1:3, 4);

        A = zeros(4, 4);
        %Ay = zeros(1, 4);
        %Az = zeros(1, 4);
        %Ag = zeros(1, 4);
        if (nvargs.mode == "cubic")
            A = zeros(4, 6);
            %Ay = zeros(1, 6);
            %Az = zeros(1, 6);
            %Ag = zeros(1, 6);

            A(1,:) = tg.quinitic_traj(tg, [0 tf currPose(1, 1) pos(1, 1) v0 vf acc0 accf]);
            A(2,:) = tg.quinitic_traj(tg, [0 tf currPose(2, 1) pos(1, 2) v0 vf acc0 accf]);
            A(2,:) = tg.quinitic_traj(tg, [0 tf currPose(3, 1) pos(1, 3) v0 vf acc0 accf]);
            A(4,:) = tg.quinitic_traj(tg, [0 tf (-temp(1, 2) - temp(1, 3) - temp(1, 4)) pos(1, 4) v0 vf acc0 accf]);
        else
            A(1, :) = tg.cubic_traj(tg, [0 tf currPose(1, 1) pos(1, 1) v0 vf]);
            A(2, :) = tg.cubic_traj(tg, [0 tf currPose(2, 1) pos(1, 2) v0 vf]);
            A(3, :) = tg.cubic_traj(tg, [0 tf currPose(3, 1) pos(1, 3) v0 vf]);
            A(4, :) = tg.cubic_traj(tg, [0 tf (-temp(1, 2) - temp(1, 3) - temp(1, 4)) pos(1, 4) v0 vf]); %joint 4 may be 90 degrees offset, I'm not positive, we may be telling it to drive further, when it's already at it's point therefore making it go out of joint limitations for no reason
        end

        %% Execute the trajectory
        travelTime = tf;
        tic;
        while toc < travelTime
        % Evaluate your trajectory, and send your robot to its next setpoint
  
        % Record the joint position of your robot
            t = toc;
            g = tg.eval_traj(A(:, :), t);
            %y = tg.eval_traj(Ay(1, :), t);
            %z = tg.eval_traj(Az(1, :), t); 
            %g = tg.eval_traj(Ag(1, :), t);
            qcmd = ik3001(self, g);
            if (atSingularity(self, qcmd, 0.0001))
                disp('ERROR!!!! AT SINGULARITY!');
                break;
            else
                self.servo_jp(qcmd);
                pause(0.01)
            end
        end
    end

    % @@@@@@@@@@@@@@
    % YOUR CODE HERE
    % @@@@@@@@@@@@@@
    % Write a function pick_up_ball that satisfies the following
    % requirements:
    
    function PICK_UP_BALL(self, pos, color, nvargs)
        arguments
            self Robot;
            pos double;
            color string;
            nvargs.z_offset double = 40;
            nvargs.z_ball double = 5;
        end
    %PICK_UP_BALL picks up a ball and deposits it in the correct color bin
    % Inputs:
    %   pos: a [1x2] matrix representing the position of the ball in the XY
    %        frame of the robot
    %   color: a string indicating what color bin the ball should be placed
    %          in
    %   z_offset (optional): the z-position at which to begin the straight
    %                        vertical trajectory 
    %   z_ball (optional): the z-posiiton at which to stop the vertical 
    %                      trajectory and grasp the ball
        blocking_ts_move(self, [pos(1, 1) pos(1, 2) nvargs.z_offset -90], time = 2);
        writeGripper(self, true);
        blocking_ts_move(self, [pos(1, 1) pos(1, 2) nvargs.z_ball -90], time = 0.5);
        writeGripper(self, false);
        blocking_ts_move(self, [pos(1, 1) pos(1, 2) nvargs.z_offset -90], time = 0.5);
        switch (color)
            case "red"
                blocking_ts_move(self, [200 -210 150 0], time = 1.5);
            case "orange"
                blocking_ts_move(self, [135 -210 150 0], time = 1.5);
            case "yellow"
                blocking_ts_move(self, [70 -210 150 0], time = 1.5);
            case "green"
                blocking_ts_move(self, [0 -210 150 0], time = 1.5);
        end
        writeGripper(self, true);
    end
    end % end methods

end % end class 
