% get the matrix from 4.txt
ego_car_matrix = readmatrix('4.txt');

size(ego_car_matrix)

% read the matrix for the car poses

car_matrix = readmatrix('MobiliTraj_4_2_17-43.txt');
size(car_matrix)


% so, now we have all these matrices
% we will separate out the rotation and the translation matrices

ego_car_rot = [];
ego_car_trans = [];

frame_car_rot = [];
frame_car_trans = [];

% go through the two matrices and separate out the rotations and
% translations

for i = 1:length(ego_car_matrix)
    % here we take the row, and reshape it
    ego_row_rot =  ego_car_matrix(i, 1:9);
    ego_trans = ego_car_matrix(i, 10:12);
    ego_rot = reshape(ego_row_rot, [3,3]);
    
    ego_car_rot = cat(3, ego_car_rot, ego_rot);
    ego_car_trans = [ego_car_trans ; ego_trans];
end

for i = i:length(car_matrix)
    % similarly deal with the frame car matrix
    car_row_rot = car_matrix(i, 1:9);
    car_trans = car_matrix(i, 10:12);
    car_rot = reshape(car_row_rot, [3,3]);
    
    frame_car_rot = cat(3, frame_car_rot, car_rot);
    frame_car_trans = [frame_car_trans ; car_trans];
    
    % this looks like my matrices are ready
end

% now I am gonna start working on finding the rotations part,
% but atm from here, I am gonna take detour and read the pdf of lie Groups
