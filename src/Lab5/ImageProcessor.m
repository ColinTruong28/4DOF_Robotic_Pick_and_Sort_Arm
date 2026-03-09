classdef ImageProcessor < handle
    properties 
        camera;  % The Camera object for this image processor
        % @@@@@@@@@@@@@@
        % YOUR CODE HERE
        % @@@@@@@@@@@@@@
        % What other attributes does an ImageProcessor need?
        colors;
    end

    methods
        % @@@@@@
        % README: nvargs
        % @@@@@@
        % Throughout this file and the new functions in Robot.m, you will
        % see "nvargs" a lot. This stands for Name Value ARGuments. These
        % are much like kwargs in Python.

        % To pass a name-value argument to a function, run
        % function_name(position_arg1, position_arg2, name=value). Position 
        % arguments (like all the arguments you've used so far in this
        % course) always go before name value args. 

        % Name value arguments should always have a default value. This is
        % defined in the "arguments" field. If you don't pass a certain
        % name value argument to a function, it gets set as the default
        % value.

        % You can access the value of a name value argument via
        % "nvargs.argument_name"
        function self = ImageProcessor(nvargs)
            arguments
                nvargs.debug logical = false;
            end
            self.camera = Camera();  % Instantiate a Camera object
            % @@@@@@@@@@@@@@
            % YOUR CODE HERE
            % @@@@@@@@@@@@@@
            % What else does image processor need to do when it is
            % instantiated? Are there any values you want to calculate once
            % to reuse again later?
        end

        function mask = generate_static_mask(self, nvargs)
            arguments
                self ImageProcessor
                nvargs.margin double = 40;   % pixels; tune 60–200
            end
        
            img = self.camera.getImage();    % adjust if your camera handle name differs
            [H,W,~] = size(img);
        
            % Use grayscale for more reliable detection
            I = img;
            if size(I,3) == 3
                I = rgb2gray(I);
            end
        
            % Try a few preprocessing variants; pick the one with most detected corners
            cands = {
                I
                imadjust(I)
                adapthisteq(I)
                imadjust(medfilt2(I,[3 3]))
            };
        
            bestPts = [];
            bestBoardSize = [0 0];
        
            for i = 1:numel(cands)
                try
                    [pts, bsz] = detectCheckerboardPoints(cands{i}, ...
                        'PartialDetections', true, ...
                        'HighDistortion', true);
        
                    if size(pts,1) > size(bestPts,1)
                        bestPts = pts;
                        bestBoardSize = bsz;
                    end
                catch
                end
            end
        
            imagePoints = bestPts;
            boardSize = bestBoardSize; %#ok<NASGU>
        
            % If detection is weak, don't break the pipeline
            if isempty(imagePoints) || size(imagePoints,1) < 15
                mask = true(H,W);
                if isprop(self,"debug") && self.debug
                    figure; imshow(img);
                    title(sprintf("Checkerboard not reliably found (%d pts) -> full mask", size(imagePoints,1)));
                end
                return;
            end
        
            % Build a tight polygon around detected corners
            k = convhull(imagePoints(:,1), imagePoints(:,2));
            poly = imagePoints(k,:);
        
            mask_tight = poly2mask(poly(:,1), poly(:,2), H, W);
        
            % Expand ROI by margin (pixels)
            r = round(nvargs.margin);
            mask = imdilate(mask_tight, strel('disk', r));
            mask = imfill(mask, 'holes');
        
            % Debug preview
            if isprop(self,"debug") && self.debug
                figure; imshow(img); hold on;
                plot(imagePoints(:,1), imagePoints(:,2), 'r.', 'MarkerSize', 12);
                title(sprintf("Detected checkerboard corners: %d", size(imagePoints,1)));
        
                masked = img;
                for c = 1:size(masked,3)
                    ch = masked(:,:,c);
                    ch(~mask) = 0;
                    masked(:,:,c) = ch;
                end
                figure; imshow(masked);
                title(sprintf("Masked image (margin=%d px)", r));
            end
        end

        function p_robot = image_to_robot(self, uvpos)
            %IMAGE_TO_ROBOT transforms a point on the image to the
            %corresponding point in the frame of the robot
            % Inputs:
            %   uvpos: a [1x2] matrix representing an image (u, v) 
            %          coordinate
            % Outputs:
            %   p_robot: a [1x2] matrix representing the transformation of
            %            the input uvpos into the robot's base frame

            % YOUR CODE HERE
            checker = pointsToWorld(self.camera.cam_IS, self.camera.cam_R, self.camera.cam_T, uvpos);
            p_robot = [(checker(2) + 80) (checker(1) - 108)];
        end

        function uv_centroids = detect_centroids(self, image, nvargs)
            arguments
                self ImageProcessor;
                image
                nvargs.min_size double = 80;  % chooose a value
            end
            %DETECT_CENTROIDS detects the centroids of binary blobs of
            %large enough size
            % Iputs: 
            %   Image: an image of the environment that has already been
            %       
            % 
            % masked for the environment and to isolate a single 
            %          color
            %   min_size (optional): the minimum size of a blob to consider
            %                         a ball
            % Outputs: 
            %   colors: a [1xn] matrix of strings indicating the color of
            %           the ball at each detected centroid
            %   uv_centroids: a [nx2] matrix of coordinates of valid
            %                 centroids in image coordinates

            % If self.debug == true, this function should display the image
            % that was passed to it with colored circles on it marking the
            % balls

            % If self.debug == true, this function should display the image
            % that was passed to it with each color mask applied
            % individually (4 figures total for this part).

            % YOUR CODE HERE

            % Accumulators
            all_uv = zeros(0,2);
            all_colors = strings(0,1);
        
            colorNames = ["red","orange","yellow","green"];

            for c = 1:4

                if ndims(image) == 3
                    switch c
                        case 1, BW = red(image);
                        case 2, BW = orange(image);
                        case 3, BW = yellow(image);
                        case 4, BW = green(image);
                    end
                else
                    BW = image > 0;              % already binary-ish
                end
                BW = logical(BW);                % ensure logical
            
                % Clean up noise
                BW = bwareaopen(BW, 10);
                BW = imclose(BW, strel('disk', 3));
                BW = imfill(BW, 'holes');
            
                % Label blobs
                [L, n] = bwlabel(BW);
                if n == 0
                    uv_centroids = zeros(0,2);
                    return;
                end
            
                % Centroids + area filter
                stats = regionprops(L, 'Centroid', 'Area');
                keep = [stats.Area] >= round(nvargs.min_size);
                stats = stats(keep);
            
                if isempty(stats)
                    uv_centroids = zeros(0,2);
                    return;
                end
            
                uv = reshape([stats.Centroid], 2, []).';   % Nx2 [u v]
                
                all_uv = [all_uv; uv];
                all_colors = [all_colors; repmat(colorNames(c), size(uv,1), 1)];
    
            end
            % figure; imshow(BW); hold on;
            % plot(uv_centroids(:,1), uv_centroids(:,2), 'gx', 'MarkerSize', 12, 'LineWidth', 2);
            % title(sprintf("Green centroids: %d (min\\_size=%d px)", size(uv_centroids,1), round(nvargs.min_size)));
            % 
            
            uv_centroids = all_uv;
            self.colors = all_colors;
                

            % Hint: bwlabel will be very helpful in this function
            % https://www.mathworks.com/help/images/ref/bwlabel.html

            % Hint: regionprops will also be very helpful
            % https://www.mathworks.com/help/images/ref/regionprops.html

        end

        function ts_centroids = correct_centroids(self, centroids, nvargs) 
            arguments 
                self ImageProcessor; 
                centroids double; 
                nvargs.ball_z = 40; % TODO: Pick a good value end 
                %CORRECT_CENTROIDS transforms image coordinate centroids into 
                %task-space coordinates for the ball 
                % Inputs: 
                % centroids: a [nx2] array of centroids in image coordinates 
                % ball_z (optional): how high the center of the ball is in 
                % millimeters % YOUR CODE HERE % Hint: similar triangles 
                % Hint2: imageToWorld will be helpful here 
            end    
            ts_centroids = centroids; 
            for coord = 1:size(centroids, 1) 
                coords = pointsToWorld(self.camera.cam_IS, self.camera.cam_R, self.camera.cam_T, centroids(coord, :)); 
                centroids(coord, :) = coords;
                x = coords(1) - 103;
                y = 255 - coords(2); 
                h = 170; 
                ballRadius = 9; % in mm 
                R = sqrt(x^2 + y^2); 
                theta2 = atan2d(x, y); 
                
                theta1 = atan2d(h, R); 
                j = ballRadius / tand(theta1); 
                deltax = sind(theta2) * j; 
                deltay = cosd(theta2) * j; 
                ts_centroids(coord, 1) = centroids(coord, 1) + deltax; 
                ts_centroids(coord, 2) = centroids(coord, 2) + deltay; 
            end 
        end

        % @@@@@@@@@@@@@@
        % YOUR CODE HERE
        % @@@@@@@@@@@@@@
        % Write a function that acquires an image, returns the
        % coordinates of the balls in the task space of the robot. Satisfy
        % the following requiremetns.
        
        %DETECT_BALLS finds the task space coordinates of all balls on the
        %checkerboard
        % Outputs:
        %   ts_coords: the task space coordinates of all balls in the
        %              workspace
    end

end
