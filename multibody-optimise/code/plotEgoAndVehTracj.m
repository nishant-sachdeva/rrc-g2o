ego = load('../data/gt_orbfull3.txt');
veh = load('../data/shapePose_full3.txt');

frameNos = 38;

% for i = 1:frameNos
%     
%     Tei1 = [ reshape(ego(i,2:10), 3,3)' ego(i, 11:13)'; 0 0 0 1];
%     Tvi1 = [ reshape(veh(i,2:10), 3,3)' veh(i, 11:13)'; 0 0 0 1];           
%     
%     Tv_global = Tei1*Tvi1;
%     vehGlobal(i+1,:) = Tv_global(1:3,4)';            
%     
% end



Tv = [ reshape(veh(1,2:10), 3,3)' veh(1, 11:13)'; 0 0 0 1];
Te = [ reshape(ego(1,2:10), 3,3)' ego(1, 11:13)'; 0 0 0 1];
T = Te*Tv;

%vehGlobal(1,:) = T(1:3, 4)'; 
vehGlobalM{1} = T;
vehGlobal(1,:) = T(1:3, 4)'; 

for i = 1:frameNos-1
    
    Tei1 = [ reshape(ego(i,2:10), 3,3)' ego(i, 11:13)'; 0 0 0 1];
    Tvi1 = [ reshape(veh(i,2:10), 3,3)' veh(i, 11:13)'; 0 0 0 1];
    
    Tei2 = [ reshape(ego(i+1,2:10), 3,3)' ego(i+1, 11:13)'; 0 0 0 1];
    Tvi2 = [ reshape(veh(i+1,2:10), 3,3)' veh(i+1, 11:13)'; 0 0 0 1];
    
    
    Tv12  = inv(Tvi1) * inv(Tei1)*Tei2 * Tvi2;
    
    %vehGlobal(i+1,:) = vehGlobal(i,:) + Tv12(1:3, 4)';    
    vehGlobalM{i+1} = vehGlobalM{i} * Tv12;
    
    vehGlobal(i+1,:) = vehGlobalM{i+1}(1:3,4)';
    %Tv_global = Te*Tv;
    
end

figure;

plot3(ego(:,11), ego(:,12), ego(:,13), '-ob');
hold on;
plot3(vehGlobal(:,1), vehGlobal(:,2), vehGlobal(:,3), '-^k');

plot3(ego(1,11), ego(1,12), ego(1,13), '-sb','MarkerSize',20);
hold on;
plot3(vehGlobal(1,1), vehGlobal(1,2), vehGlobal(1,3), '-dk','MarkerSize',20);



xlabel('X');
ylabel('Y');
zlabel('Z');


