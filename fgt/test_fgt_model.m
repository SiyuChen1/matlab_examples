clc
clear
% 
% d          = 2;
% Nx         = 1000;
% Ny         = 1000;
% x          = 1 * randn(d , Nx); 
% y          = 1 * randn(d , Ny);
% w          = rand(1 , Nx);
% h          = 10;
% 
% e          = 10;
% p          = 8;
% K          = 40;
% 
% % tic
% %     for i=1:10
% %         v1         = dval(x , y , w , h);
% %     end
% % toc
% % 
% % 
% % tic
% %     for i=1:10
% %         [xc , A_k] = fgt_model(x , w , h , e , K , p);
% %         v2         = fgt_predict(y , xc , A_k , h , e);
% %     end
% % toc
% 
% v3         = dval(x , y , w , h);

% [xc , A_k] = fgt_model(x , w , h , e , K , p);
% v4         = fgt_predict(y , xc , A_k , h , e);
% 
% disp(sprintf('error = %5.4f',norm(v3 - v4)));

d          = 5;
Nx         = 1000;
Ny         = 1000;
x          = 10 * randn(d , Nx);
y          = 20 * randn(d , Ny);
w          = 5 * rand(1 , Nx);
h          = 2;

e          = 10;
p          = 6;
K          = 5;

v1         = dval(x , y , w , h);

[xc , A_k] = fgt_model(x , w , h , e , K , p);

v2         = fgt_predict(y , xc , A_k , h , e);
max(abs(v1 - v2))

disp(sprintf('error = %5.4f',norm(v1 - v2)));