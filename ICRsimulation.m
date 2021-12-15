% 条件
L = 0.1;
R = 1;
Kt = 0.2;
Ke = 0.2;
J = 10;
D = 0.1;

% リカッチ方程式の係数行列
A = [-R/L 0 -Ke/L; 0 0 1; Kt/J 0 -D/J];
b = [1/L; 0; 0];
c = [0 1 0];
Q = c' * c;

% 入力に対する重み（大きいほど収束が遅くなる）
r = 1;

% 状態空間モデル
sys = ss(A, b, c, []);

% K: フィードバックゲイン
% P: リカッチ方程式の解
[K, P] = lqr(sys, Q, r)


disp("=====初期値応答=====")

% シミュレーション時間
Tfinal = 140;

% フィードバックゲインの条件
k3 = 1 / 2;
k4 = 2;

% 初期値
x0 = [1;1;0];

% (1) フィードバックなし
initial(sys, x0, Tfinal);

hold on

% (2) 最適制御のフィードバックを系に印加
A2 = A + b * (-K);

sys2 = ss(A2, [], c, [])
initial(sys2, x0, Tfinal);

hold on

% (3) フィードバックu=-k3・Kxを系に印加
A3 = A + b * (-K * k3);

sys3 = ss(A3, [], c, [])
initial(sys3, x0, Tfinal);

hold on

% (4) フィードバックu=-k4・Kxを系に印加
A4 = A + b * (-K * k4);

sys4 = ss(A4, [], c, [])
initial(sys4, x0, Tfinal);

legend("(1) FBなし", "(2) FB u=-Kx (最適制御)", "(3) FB u=-" + k3 + "Kx", "(4) FB u=-" + k4 + "Kx", ...
       "Location","east")
grid

saveas(gcf, 'icr.png');
