clear
clc

tolerance = 1e-10;

R_w_c1 = rotz(90);
t_w_c1 = [10, 0, 0];
tform_w_c1 = rigidtform3d(R_w_c1, t_w_c1);

R_w_c2 = rotz(120);
t_w_c2 = [10, 0.05, 0];
tform_w_c2 = rigidtform3d(R_w_c2, t_w_c2);

R_c1_c2 = rotz(30);
t_c1_c2 = [0.05, 0, 0];
t_c1_c2_m = [0 -t_c1_c2(3) t_c1_c2(2); t_c1_c2(3) 0 -t_c1_c2(1); -t_c1_c2(2) t_c1_c2(1) 0];
tform_c1_c2 = rigidtform3d(R_c1_c2, t_c1_c2);

tf_c2_c1 = inv(tform_c1_c2.A);

R_c2_c1 =  tf_c2_c1(1:3, 1:3);
t_c2_c1 = tf_c2_c1(1:3, 4);
t_c2_c1_m = [0 -t_c2_c1(3) t_c2_c1(2); t_c2_c1(3) 0 -t_c2_c1(1); -t_c2_c1(2) t_c2_c1(1) 0];


check_rotation = max(abs(R_w_c2 - R_c1_c2 * R_w_c1), [], "all")  < tolerance;
check_tf = max(abs(tform_w_c2.A - tform_w_c1.A * tform_c1_c2.A), [], "all") < tolerance;

fx = 500;
fy = 500;
W = 800;
H = 600;
cx = 10 + W / 2;
cy = 20 + H / 2;

K = [fx 0, cx, 0; 0, fy, cy, 0; 0, 0, 1, 0];

P_x = 16;
P_y_min = -2;
P_y_max = 2;
P_len = 8000;

% P_in_w = [P_x * ones(1, P_len); P_y_min + (P_y_max - P_y_min) * randn(2, P_len); ones(1, P_len)];
P_in_w = [P_x * randn(1, P_len); P_y_min + (P_y_max - P_y_min) * randn(2, P_len); ones(1, P_len)];

P_in_c1 = K * inv(tform_w_c1.A) * P_in_w;
P_in_c1 = P_in_c1./ P_in_c1(3, :);
P_c1_valid = P_in_c1(1, :) > 0 & P_in_c1(2, :) > 0 & P_in_c1(1, :) < W & P_in_c1(2, :) < H;
count_c1_valid = sum(P_c1_valid);

P_in_c2 = K * inv(tform_w_c2.A) * P_in_w;
P_in_c2 = P_in_c2./ P_in_c2(3, :);
P_c2_valid = P_in_c2(1, :) > 0 & P_in_c2(2, :) > 0 & P_in_c2(1, :) < W & P_in_c2(2, :) < H;
count_c2_valid = sum(P_c2_valid);

E = t_c2_c1_m * R_c2_c1;
% E = t_c1_c2_m * R_c1_c2;
K_small = K(1:3, 1:3);
F = inv(K_small') * E * inv(K_small);

count = 0;

for i = 1:P_len
    v = abs(P_in_c2(:, i)' * F * P_in_c1(:, i)) > tolerance;
    count = count + v;
end

valid = P_c1_valid & P_c2_valid;
inlierP_c1 = P_in_c1(1:2, valid)';
inlierP_c2 = P_in_c2(1:2, valid)';

[f_est, inliers, status] = estimateFundamentalMatrix(inlierP_c1, inlierP_c2, Method="RANSAC", ...
    NumTrials=2e3, DistanceThreshold=0.001);

count_f_est = 0;
for i = 1:P_len
    v = abs(P_in_c2(:, i)' * f_est * P_in_c1(:, i)) > tolerance;
    count_f_est = count_f_est + v;
end

inliers_ratio = sum(inliers) / sum(valid);

intrinsics = cameraIntrinsics([fx, fy], [cx, cy], [W, H]);

[relPose, validFraction] = estrelpose(f_est, intrinsics, ...
        inlierP_c1, inlierP_c2);

est = relPose.A
gt = tform_c1_c2.A

[U, D, V] = svd(K_small' * F * K_small);
R1 = U * [0 -1 0; 1 0 0; 0 0 1] * V';
R2 = U * [0 -1 0; 1 0 0; 0 0 1]' * V';
C1 = -R1' * U(:, 3)
C2 = -R2' * U(:, 3)