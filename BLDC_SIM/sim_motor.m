function p = sim_motor(p,ts)

% vu,vv,vwを座標変換してva,vbを計算
p.vab = p.Cuvw * p.vuvw;

% ab座標からdq座標への変換行列Cdqの設定
c = cos(p.th);
s = sin(p.th);
Cdq = [c s;-s c];

% vabを座標変換してvdqを計算
vdq = Cdq * p.vab;

% iabを座標変換してidqを計算
idq = Cdq * p.iab;

% dq軸電圧方程式から，idqの微分値を計算
idq_dot = [p.Ld 0;0 p.Lq]\(vdq - p.R * idq - p.w * [0 -p.Lq;p.Ld 0] * idq - p.w * [0;p.phi]);

% idqの微分値をオイラー法で積分
p.idq = idq + ts * idq_dot;
id = p.idq(1);
iq = p.idq(2);

% モータトルクTmの計算
p.Tm = p.p * (p.phi + (p.Ld - p.Lq) * id) * iq;

% 負荷トルクTLの計算
if abs(p.w) > 5 * 2 * pi %速度が大きくなると
    TL = sign(p.w) * p.TL;
else
    TL = p.w/(5 * 2 * pi) * p.TL;
end
Tall = p.Tm - TL;

% モータ速度wの計算
p.w = p.w + ts * (1.0 / p.Jm) * Tall;

% モータ角度θの計算
p.th = p.th + ts * p.w;

% モータ角度θの値域を0 - 4piに
if p.th > 4 * pi
    p.th = p.th - 4 * pi;
end
if p.th < 0
    p.th = p.th + 4 * pi;
end

% dq座標からab座標への変換行列Cdq_invの設定
c = cos(p.th);
s = sin(p.th);
Cdq_inv = [c -s;s c];

% idqを座標変換してiabを計算
p.iab = Cdq_inv * p.idq;

% ab座標からUVW座標への変換行列Cuvw_inv = Cuvw'
p.iuvw = p.Cuvw' * p.iab;




