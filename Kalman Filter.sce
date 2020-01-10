clf

function [X, P]= KalmanFilter(zk, x1, p1, F, B, H, Q, R)
    // Prediction
    xkp = F * x1;
    Pkp = F * p1 * F' + Q;
    
    // Calculate residual and Kalman gain
    res = zk - H * xkp;
    K = Pkp * H' / (H * Pkp * H' + R);
    
    // Update
    //X(1:i) = x1;
    X = xkp + K * res;
    //P(1:i) = p1;
    P = Pkp - K * H * Pkp;
endfunction


//Setup for simulation
// Construction of the sinusoid
w = %pi/4; // angular frequency
T = 0.1; // period
t = 0:T:500;
signal = cos(w*t);

// Sinusoid with noise
v=rand(t,"normal");
y=signal+v;
// Plot the sinusoid with noise
subplot(2, 1, 1);
plot(t,y);
xtitle("Sinusoid With Noise","t");

// System Parameter
n = 2; 
F = [cos(w*T) -sin(w*T); sin(w*T) cos(w*T)];
B = 0;
H = [1 0];
p0 = [1000 0; 0 0];
R = 1;
Q = 0;
x0 = zeros(n, 1);

// Initialize for loop
x1 = zeros(2, length(t));
p1 = zeros(2, 2, length(t));
p1(:,:,1) = [1000 0; 0 0];

// Kalman filter
for i = 1:length(t) - 1
    [x1(:,i+1),p1(:,:,i+1)] = KalmanFilter(y(i),x1(:,i),p1(:,:,i),F,B,H,Q,R);
end

// Plot the results (in red) to compare with the sinusoid (in green)
subplot(2, 1, 2);
plot(t, signal, "color", "green");
plot(t, x1(1,:), "color", "red");
xtitle("Comparison Between Sinusoid (green) and Extraction With Kalman Filter (red)","t");
