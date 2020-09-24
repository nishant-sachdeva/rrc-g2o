function [ mat ] = rowVecToMat(v,rotFormat)
% converts 1x12 row vector into 4x4 matrix. 
% rotFormat = 'c' then it assumes the R to be stored as column format
% rotFormat = 'r' then it assumes the R to be stored as row format
    
    mat = eye(4,4);
    
    if(rotFormat == 'r')
        mat = [reshape(v(1:9), 3, 3)' v(10:12)'; 0 0 0 1];
    end
    
    if(rotFormat == 'c')
        mat = [reshape(v(1:9), 3, 3)  v(10:12)'; 0 0 0 1];
    end
end


% Tv2v = rowVecToMat(shapePose0(1,:),'r');
% for i = 1:154
%     Tm = rowVecToMat(v2v_(i,:),'r');
%     Tv2v = Tv2v * Tm;
%     shapePose_m{i} = Tv2v;
% end
% for i = 1:154
%     plot(shapePose_m{i}(1,4), shapePose_m{i}(3,4), 'sr');
%     hold on;
% end
% axis equal
% gt_m = {};
% for i = 1:154
%     Tgt = rowVecToMat(gt0(i,:),'c');
%     Tp = rowVecToMat(shapePose0(i,:),'r');
%     gt_m{i} = Tgt*Tp;
% end
% for i = 1:154
%     plot(gt_m{i}(1,4), gt_m{i}(3,4), 'sg');
%     hold on;
% end