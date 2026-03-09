classdef TrajGenerator < handle  
    methods(Static)
        % TODO: Fill in the arguments for these methods
        function coeffs = cubic_traj(self, vars)
        %CUBIC_TRAJ Generates coefficients for a cubic trajectory
        % Inputs:
        %   TODO: Describe input args
        % Outputs:
        %   coeffs: a [1x4] matrix of cubic trajectory coefficients

            % YOUR CODE HERE
            % Hint: to solve the linear system of equations b = Ax,
            %       use x = A \ b
            t0 = vars(1);
            tf = vars(2);
            q0 = vars(3);
            qf = vars(4);
            v0 = vars(5);
            vf = vars(6);

            M = [1 t0 t0^2 t0^3;
                 0 1 2*t0 3*t0^2;
                 1 tf tf^2 tf^3;
                 0 1 2*tf 3*tf^2];
            b = [q0; v0; qf; vf];

            coeffs = (M \ b);
        end

        % TODO: Fill in the arguments for these methods
        function coeffs = quinitic_traj(self, vars)
        %CUBIC_TRAJ Generates coefficients for a quintic trajectory
        % Inputs:
        %   TODO: Describe input args
        % Outputs:
        %   coeffs: a [1x6] matrix of cubic trajectory coefficients
            t0 = vars(1);
            tf = vars(2);
            q0 = vars(3);
            qf = vars(4);
            v0 = vars(5);
            vf = vars(6);
            acc0 = vars(7);
            accf = vars(8);

            M = [1 t0 t0^2 t0^3 t0^4 t0^5;
                 0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
                 0 0 2 6*t0 12*t0^2 20*t0^3;
                 1 tf tf^2 tf^3 tf^4 tf^5;
                 0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
                 0 0 2 6*tf 12*tf^2 20*tf^3];

            b = [q0; v0; acc0; qf; vf; accf];
            coeffs = (M \ b);
        end

        function state = eval_traj(coeff_mat, t) 
        %EVAL_TRAJ Evaluates multiple trajectories
        % Inputs:
        %   coeff_mat: a [nx4] or [nx6] matrix where each row is a set of
        %              cubic or quintic trajectory coefficients
        %   t: a time in seconds at which to evaluate the trajectories
        % Outputs:
        %   state: a [nx1] column vector containing the results of 
        %          evaluating the input trajectories at time t
            k = size(coeff_mat,2);
            T = t.^(0:k-1).';
            state = coeff_mat * T;
        end
    end
end