
% files to be loaded
gtfull3 = load('~/Research@IIITH/IROS_2019_MultibodySLAM/data/gt_full3.txt');
output  = load('~/Research@IIITH/IROS_2019_MultibodySLAM/code/build/optiTranslations.txt');
shapeIn = load('~/Research@IIITH/IROS_2019_MultibodySLAM/data/shapePose_full3.txt');

numFrames = 38;

gtfull3(:,11:13) = gtfull3(:,11:13)-gtfull3(1,11:13);
for i = 1:numFrames
    
    Tgt = [reshape(gtfull3(i,2:10),3,3)' gtfull3(i,11:13)' ; 0 0 0 1];
    Tv =  [reshape(shapeIn(i,2:10),3,3)' shapeIn(i,11:13)' ; 0 0 0 1];
    Tgtv = Tgt*Tv;
    in_veh_global(i,:) = Tgtv(1:3, 4)';
end
%gt veh translations;
%shapeGT_t = shapeGT(:,4:6);
shapeIn_t = shapeIn(:,11:13);


for i = 1:38
    
    R_opt_ego = reshape(output(1:9,i),3,3)';
    t_opt_ego = output(10:12,i)./output(13,i);
    
    T_opt_ego = 
end

% rows 1-38 trans of ego vehicle
output_ego = output(1:38,:);
output_ego = output_ego ./ output_ego(:,4) % scale
%output_ego = output_ego - output_ego(1,:) % to set the init loc as zero





% rows 39-76 trans for other veh.
output_veh = output(39:76,:);
output_veh = output_veh ./ output_veh(:,4) % scale

% ground truth.
gtfull3_t = gtfull3(:, 11:13);
%gtfull3_t = gtfull3_t - gtfull3_t(1,:); % to set the initial loc as zero


%gt_veh_global = gtfull3_t + shapeGT_t;

%in_veh_global = gtfull3_t + shapeIn_t;


% plot
figure;
plot3(gtfull3_t(:,1),gtfull3_t(:,2), gtfull3_t(:,3), '-sg')
hold on;
plot3(output_ego(:,1), output_ego(:,2),output_ego(:,3), '-ob')
plot3(output_veh(:,1),  output_veh(:,2), output_veh(:,3), '-^k')
%plot(gt_veh_global(:,1),  gt_veh_global(:,3), '-^r')
plot3(in_veh_global(:,1), in_veh_global(:,2),in_veh_global(:,3), '-^m')


% figure;
% plot3(gtfull3_t(:,1), gtfull3_t(:,2), gtfull3_t(:,3), '-sg')
% hold on;
% plot3(output_ego(:,1), output_ego(:,2), output_ego(:,3), '-ob')
% plot3(output_veh(:,1),  output_veh(:,2),output_veh(:,3), '-^k')
% %plot(gt_veh_global(:,1),  gt_veh_global(:,3), '-^r')
% plot3(in_veh_global(:,1),  in_veh_global(:,2), in_veh_global(:,3), '-^m')


legend('gt','orbscaled','veh-gobal-est','veh-gobal-in');
legend('gt','orbscaled','veh-gobal-est','veh-gobal-gt','veh-gobal-in');
