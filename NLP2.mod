param BasicParameters{i in {1..13}};
param BV{i in {1..10}};
param Nfe == BasicParameters[1];
param vmax == BasicParameters[2];
param amax == BasicParameters[3];
param phymax == BasicParameters[4];
param wmax == BasicParameters[5];
param L_wheelbase == BasicParameters[6];
param Lrc == BasicParameters[7];
param Lfc == BasicParameters[8];
param w_penalty == BasicParameters[9];

var tf >= 0.1;
var hi = tf / (Nfe - 1);
param WithinTunnelConstraints{i in {1..2}, j in {1..Nfe}, k in {1..4}};

var x{i in {1..Nfe}};
var y{i in {1..Nfe}};
var theta{i in {1..Nfe}};
var v{i in {1..Nfe}};
var a{i in {1..Nfe}};
var phy{i in {1..Nfe}};
var w{i in {1..Nfe}};
var xf{i in {1..Nfe}};
var yf{i in {1..Nfe}};
var xr{i in {1..Nfe}};
var yr{i in {1..Nfe}};

minimize obj: 
tf + 1000000 * (sum{i in {2..Nfe}}((x[i] - x[i-1] - hi * v[i] * cos(theta[i]))^2 + (y[i] - y[i-1] - hi * v[i] * sin(theta[i]))^2 + (v[i] - v[i-1] - hi * a[i])^2 + (theta[i] - theta[i-1] - hi * tan(phy[i]) * v[i] / L_wheelbase)^2 + (phy[i] - phy[i-1] - hi * w[i])^2 + (xf[i] - x[i] - Lfc * cos(theta[i]))^2 + (yf[i] - y[i] - Lfc * sin(theta[i]))^2 + (xr[i] - x[i] - Lrc * cos(theta[i]))^2 + (yr[i] - y[i] - Lrc * sin(theta[i]))^2) + (sin(theta[Nfe]) - sin(BV[6]))^2 + (cos(theta[Nfe]) - cos(BV[6]))^2);



s.t. time_limit:
tf <= Nfe * 0.5;

s.t. EQ_init_x :
x[1] = BV[1];
s.t. EQ_init_y :
y[1] = BV[2];
s.t. EQ_init_theta :
theta[1] = BV[3];
s.t. EQ_init_v :
v[1] = 0;
s.t. EQ_init_a :
a[1] = 0;
s.t. EQ_init_phy :
phy[1] = 0;
s.t. EQ_init_w :
w[1] = 0;


s.t. EQ_end_x :
x[Nfe] = BV[4];
s.t. EQ_end_y :
y[Nfe] = BV[5];
s.t. EQ_end_v :
v[Nfe] = 0;
s.t. EQ_end_a :
a[Nfe] = 0;
s.t. EQ_end_phy :
phy[Nfe] = 0;
s.t. EQ_end_w :
w[Nfe] = 0;

s.t. Bonds_phy {i in {1..Nfe}}:
-phymax <= phy[i] <= phymax;
s.t. Bonds_v {i in {1..Nfe}}:
-vmax <= v[i] <= vmax;
s.t. Bonds_w {i in {1..Nfe}}:
-wmax <= w[i] <= wmax;
s.t. Bonds_a {i in {1..Nfe}}:
-amax <= a[i] <= amax;


s.t. Box_on_xf1 {i in {1..Nfe}}:
WithinTunnelConstraints[1,i,1] <= xf[i];
s.t. Box_on_xf2 {i in {1..Nfe}}:
xf[i] <= WithinTunnelConstraints[1,i,2];
s.t. Box_on_yf1 {i in {1..Nfe}}:
WithinTunnelConstraints[1,i,3] <= yf[i];
s.t. Box_on_yf2 {i in {1..Nfe}}:
yf[i] <= WithinTunnelConstraints[1,i,4];
s.t. Box_on_xr1 {i in {1..Nfe}}:
WithinTunnelConstraints[2,i,1] <= xr[i];
s.t. Box_on_xr2 {i in {1..Nfe}}:
xr[i] <= WithinTunnelConstraints[2,i,2];
s.t. Box_on_yr1 {i in {1..Nfe}}:
WithinTunnelConstraints[2,i,3] <= yr[i];
s.t. Box_on_yr2 {i in {1..Nfe}}:
yr[i] <= WithinTunnelConstraints[2,i,4];

data;
param: BV := include BV2;
param: WithinTunnelConstraints := include WithinTunnelConstraints;
param: BasicParameters := include BasicParameters;