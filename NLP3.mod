param BasicParameters{i in {1..13}};
param BV{i in {1..14}};
param Nfe == BasicParameters[1];
param vmax == BasicParameters[2];
param amax == BasicParameters[3];
param phymax == BasicParameters[4];
param wmax == BasicParameters[5];
param L_wheelbase == BasicParameters[6];
param Lrc == BasicParameters[7];
param Lfc == BasicParameters[8];
param w_penalty == BasicParameters[9];

var tf >= 0;
var hi = tf / (Nfe - 1);

var x{i in {1..Nfe}};
var y{i in {1..Nfe}};
var theta{i in {1..Nfe}};
var v{i in {1..Nfe}};
var a{i in {1..Nfe}};
var phy{i in {1..Nfe}};
var w{i in {1..Nfe}};

minimize obj: 
tf;


s.t. DIFF_dxdt {i in {2..Nfe}}:
x[i] = x[i-1] + hi * v[i-1] * cos(theta[i-1]);

s.t. DIFF_dydt {i in {2..Nfe}}:
y[i] = y[i-1] + hi * v[i-1] * sin(theta[i-1]);

s.t. DIFF_dvdt {i in {2..Nfe}}:
v[i] = v[i-1] + hi * a[i];

s.t. DIFF_dtheta1dt {i in {2..Nfe}}:
theta[i] = theta[i-1] + hi * v[i-1] * tan(phy[i-1]) / L_wheelbase;

s.t. DIFF_dphydt {i in {2..Nfe}}:
phy[i] = phy[i-1] + hi * w[i];

s.t. time_limit:
tf <= Nfe * 0.5;


s.t. EQ_init_x :
x[1] = BV[1];
s.t. EQ_init_y :
y[1] = BV[2];
s.t. EQ_init_theta :
theta[1] = BV[3];
s.t. EQ_init_v :
v[1] = BV[4];
s.t. EQ_init_a :
a[1] = BV[5];
s.t. EQ_init_phy :
phy[1] = BV[6];
s.t. EQ_init_w :
w[1] = BV[7];

s.t. EQ_end_x :
x[Nfe] = BV[8];
s.t. EQ_end_y :
y[Nfe] = BV[9];
s.t. EQ_end_theta :
theta[Nfe] = BV[10];
s.t. EQ_end_v :
v[Nfe] = BV[11];
s.t. EQ_end_a :
a[Nfe] = BV[12];
s.t. EQ_end_phy :
phy[Nfe] = BV[13];
s.t. EQ_end_w :
w[Nfe] = BV[14];

s.t. Bonds_phy {i in {1..Nfe}}:
-phymax <= phy[i] <= phymax;
s.t. Bonds_v {i in {1..Nfe}}:
-vmax <= v[i] <= vmax;
s.t. Bonds_w {i in {1..Nfe}}:
-wmax <= w[i] <= wmax;
s.t. Bonds_a {i in {1..Nfe}}:
-amax <= a[i] <= amax;
s.t. Bonds_theta {i in {1..Nfe}}:
BV[3] - 4.7124 <= theta[i] <= BV[3] + 4.7124;

data;
param: BV := include BV3;
param: BasicParameters := include BasicParameters;