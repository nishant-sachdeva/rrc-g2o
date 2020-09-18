% get the matrix from 4.txt

system('echo "" > results/4_ego_17-43.txt');
system('echo "" > results/4_2_17-43.txt');



ego_car_matrix = readmatrix('4.txt');

% read the matrix for the car poses

car_matrix = readmatrix('MobiliTraj_4_2_17-43.txt');

% so, now we have all these matrices
% we will separate out the rotation and the translation matrices

ego_transform = [];

frame_transform = [];

% go through the two matrices and separate out the rotations and
% translations

for i = 1:length(ego_car_matrix(18:44, :))
    % here we take the row, and reshape it
    ego_row_rot =  ego_car_matrix(17+i, 1:9);
    ego_trans = ego_car_matrix(17+i, 10:12);
    ego_rot = reshape(ego_row_rot, [3,3]);
    ego_rot = [ego_rot ; [0,0,0]];
    ego_trans = [ego_trans, 1];
    
    ego_rot = ego_rot';
    ego_rot = [ego_rot ; ego_trans]';
    
    ego_transform = cat(3, ego_transform, ego_rot);
end

for i = 1:length(car_matrix)
    % similarly deal with the frame car matrix
    car_row_rot = car_matrix(i, 1:9);
    car_trans = car_matrix(i, 10:12);
    car_rot = reshape(car_row_rot, [3,3]);
    
    car_rot = [car_rot ; [0,0,0]];
    car_trans = [car_trans, 1];
    
    car_rot = car_rot';
    car_rot = [car_rot; car_trans]';
    
    frame_transform = cat(3, frame_transform, car_rot);
    % this looks like my matrices are ready
end

% now I am gonna start working on finding the rotations part,
% but atm from here, I am gonna take detour and read the pdf of lie Groups


% equation to be followed in the next portion

% mat(x, 17) = inverse(mat(17, ground))*mat(x, ground)
% and then we store mat(x,17) for the next stages and also store it in the
% file

temp_mat = inv(ego_transform(:,:,1));

ego_ground_transform_mat = [];

for i = 1:length(ego_transform)
    tempa = ego_transform(:,:,i);
    mat = temp_mat*tempa;
    
    ego_ground_transform_mat = cat(3, ego_ground_transform_mat, mat);
    % this mat is frame x with respect to ground
end

% first we save this matrix in a file somewhere

% enter code for that in place of this line
%%%%%%%%%%%%%%%%
diary("results/4_ego_17-43.txt");

ego_motion_trans = [];

for i = 1:length(ego_ground_transform_mat)
    % here we start printing
    temp = ego_ground_transform_mat(:,:,i);
    rot = temp(1:3, 1:3);
    trans = temp(1:3,4);
    % now we print this
    ego_motion_trans = [ego_motion_trans ; trans'];
    
    print_string = sprintf("%f %f ", rot, trans);
    disp(print_string);
end

diary off


% now we create the frame_ego_transform matrix
% the equation used for this transform is 
% mat(17 ego car, x ego car) * mat( x ego car, x frame car) = mat( 17 ego
% car, x frame  car)

frame_ego_transform_mat = [];

for i = 1:length(ego_ground_transform_mat)
    tempa = ego_ground_transform_mat(:,:,i);
    tempb = frame_transform(:,:,i);
    
    mat = tempa*tempb;
    
    frame_ego_transform_mat = cat(3, frame_ego_transform_mat, mat);
end

% now we print this data in another file

diary("results/4_2_17-43.txt");

frame_motion_trans = [];

for i = 1:length(frame_ego_transform_mat)
    % here we start printing
    temp = frame_ego_transform_mat(:,:,i);
    rot = temp(1:3, 1:3);
    trans = temp(1:3,4);
    % now we print this
    frame_motion_trans = [frame_motion_trans ; trans'];
    
    print_string = sprintf("%f %f ", rot, trans);
    disp(print_string);
end

diary off


% plotting ego_motion_trans, and frame_motion_trans

ego_x = ego_motion_trans(:,1);
ego_z = ego_motion_trans(:,3);

plot(ego_x, ego_z);
hold on;
% generate plot here


frame_x = frame_motion_trans(:,1);
frame_z = frame_motion_trans(:,3);
plot(frame_x, frame_z);

hold off;
% generate plot here


