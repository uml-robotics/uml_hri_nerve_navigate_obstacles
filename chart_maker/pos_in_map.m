% Information copied from the map.yaml file
% NOTE: These numbers will change on a case-by-case basis!
resolution = 0.10000;
origin = [-20.000000, -20.000000, 0.000000];

% Read the image - assumes the image file is in your home directory, change path as necessary
ifile = 'nerve2.pgm';   % Image file name
I=imread(ifile);
 
% Span of image in meters
dx = size(I,1)*resolution;
dy = size(I,2)*resolution;
 
% Set the size scaling
xWorldLimits = [0 dx];%+origin(1);
yWorldLimits = [0 dy];%+origin(2);

% Reference to world coordinates (meters)
RI = imref2d(size(I),xWorldLimits,yWorldLimits)

% Plot
figure(1);
clf()
imshow(flipud(I),RI)
set(gca,'YDir','normal')
 
% Optionally, put a marker at the origin
hold on;
plot(0, 0,'r*')

%select ros topic from bag and create a timeseries object
% bag = rosbag('2020-08-06-07-33-31.bag')
% %bag.AvailableTopics
% bagselect = select(bag, 'Topic', '/base_pose_ground_truth')
% ts = timeseries(bagselect, 'Pose.Pose.Position.X','Pose.Pose.Position.Y')
% % 
% %     'Twist.Twist.Linear.X','Twist.Twist.Angular.Z',...
% %     'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X',...
% %     'Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z')
% 
% 
% 
% plot(ts.data(:,1), ts.data(:,2));



