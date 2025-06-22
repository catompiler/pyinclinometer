clear;

IDLE_SAMPLES = 200;
DT = 0.01;
ALPHA = 0.9;
accel_cal = [-0.100216722972972, 1.35917060472973, 0.0153553790169492];
gyro_cal = [0.001169915805, -0.056912726, 0.0150643665];

function res_data = calibrate(data, data_cal)
  data_size = size(data);
  rows = data_size(1);
  cols = data_size(2);

  res_data = zeros(rows, cols);

  for i=[1:rows]
    res_data(i, 1) = data(i, 1) - data_cal(1);
    res_data(i, 2) = data(i, 2) - data_cal(2);
    res_data(i, 3) = data(i, 3) - data_cal(3);
  endfor

endfunction


function [res_roll, res_pitch] = calc_accel_roll_pitch(data)
  data_size = size(data);
  rows = data_size(1);
  #cols = data_size(2);

  res_roll = zeros(rows, 1);
  res_pitch = zeros(rows, 1);

  acc_x = 0;
  acc_y = 0;
  acc_z = 0;

  roll = 0;
  pitch = 0;

  for i=[1:rows]
    acc_x = data(i, 1);
    acc_y = data(i, 2);
    acc_z = data(i, 3);

    roll = atan2(-acc_x, acc_y);
    pitch = atan2(-acc_x, -acc_z);

    res_roll(i) = roll;
    res_pitch(i) = pitch;
  endfor
endfunction


function [res_droll, res_dpitch] = calc_gyro_droll_dpitch(data)
  data_size = size(data);
  rows = data_size(1);
  #cols = data_size(2);

  res_droll = zeros(rows, 1);
  res_dpitch = zeros(rows, 1);

  gyro_x = 0;
  gyro_y = 0;
  gyro_z = 0;

  droll = 0;
  dpitch = 0;

  for i=[1:rows]
    gyro_x = data(i, 1);
    gyro_y = data(i, 2);
    gyro_z = data(i, 3);

    droll = -gyro_z;
    dpitch = -gyro_y;

    res_droll(i) = droll;
    res_dpitch(i) = dpitch;
  endfor
endfunction


function [res_roll, res_pitch] = calc_roll_pitch(accel_roll, accel_pitch, gyro_droll, gyro_dpitch, dt, alpha)
  data_size = size(accel_roll);
  rows = data_size(1);
  #cols = data_size(2);

  res_roll = zeros(rows, 1);
  res_pitch = zeros(rows, 1);

  acc_roll = 0;
  acc_pitch = 0;

  gy_droll = 0;
  gy_dpitch = 0;

  gy_roll = 0;
  gy_pitch = 0;

  roll = 0;
  pitch = 0;

  for i=[1:rows]
    acc_roll = accel_roll(i);
    acc_pitch = accel_pitch(i);

    gy_droll = gyro_droll(i);
    gy_dpitch = gyro_dpitch(i);

    gy_roll = roll + gy_droll * dt;
    gy_pitch = pitch + gy_dpitch * dt;

    roll = gy_roll * alpha + (1.0 - alpha) * acc_roll;
    pitch = gy_pitch * alpha + (1.0 - alpha) * acc_pitch;

    res_roll(i) = roll;
    res_pitch(i) = pitch;
  endfor
endfunction


# https://gist.github.com/zasimov/25ba4ca78a3bfe82cedd77b1795d8de8
# https://habr.com/ru/articles/503542/
# https://habr.com/ru/companies/singularis/articles/516798/
# https://habr.com/ru/articles/166693/
# https://habr.com/ru/articles/140274/

function res_kf = kalman_init(X0, P0, F, B, Q, H, R)
  kf = struct(
              "X", X0,
              "Xdash", [0; 0],
              "P", P0,
              "Pdash", [0; 0],
              "F", F,
              "FT", transpose(F),
              "B", B,
              "Q", Q,
              "H", H,
              "HT", transpose(H),
              "R", R,
              "I", eye(2),
              "_", 0
              );
  #
  res_kf = kf;
endfunction

function res_kf = kalman_predict(kf, u)

  kf.Xdash = kf.F * kf.X + kf.B * u;

  kf.Pdash = kf.F * kf.P * kf.FT + kf.Q;

  res_kf = kf;
endfunction

function res_kf = kalman_update(kf, z)

  S = kf.H * kf.Pdash * kf.HT + kf.R;
  invS = inverse(S);

  K = kf.Pdash * kf.HT * invS;

  y = z - kf.H * kf.Xdash;

  kf.X = kf.Xdash + K * y;
  kf.P = (kf.I - K * kf.H) * kf.Pdash;

  res_kf = kf;
endfunction


function res_kf = kalman_calc(kf, u, z)
  kf = kalman_predict(kf, u);
  kf = kalman_update(kf, z);

  res_kf = kf;
endfunction


data = csvread("accel_gyro_data.csv");

SAMPLES = length(data);
t = linspace(0, DT * (SAMPLES-1), SAMPLES);

accel_data = calibrate(data(:, 1:3), accel_cal);
gyro_data = calibrate(data(:, 4:6), gyro_cal);

[accel_roll, accel_pitch] = calc_accel_roll_pitch(accel_data);
[gyro_droll, gyro_dpitch] = calc_gyro_droll_dpitch(gyro_data);

[roll, pitch] = calc_roll_pitch(accel_roll, accel_pitch, gyro_droll, gyro_dpitch, DT, ALPHA);

#kalman_init(X0, P0, F, B, Q, H, R)
X0 = [0;
      0];
P0 = [1, 0;
      0, 1];
F = [1, DT;
     0, 1];
B = [0;
     DT];
Q = [0.001, 0;
     0, 0.01];
H = [1 0];
R = [0.01];
kf_roll = kalman_init(X0, P0, F, B, Q, H, R);
kf_pitch = kalman_init(X0, P0, F, B, Q, H, R);

roll_kf = zeros(SAMPLES, 1);
pitch_kf = zeros(SAMPLES, 1);

for i=[1:SAMPLES]
  kf_roll = kalman_calc(kf_roll, gyro_droll(i), accel_roll(i));
  roll_kf(i) = kf_roll.X(1);

  kf_pitch = kalman_calc(kf_pitch, gyro_dpitch(i), accel_pitch(i));
  pitch_kf(i) = kf_pitch.X(1);
endfor

#x = t;
x = [1:1:SAMPLES];

plot(x, accel_roll, "r", x, accel_pitch, "b",
     x, roll_kf, "m", x, pitch_kf, "c",
     x, roll, "m--", x, pitch, "c--");
#
legend(
       "Accel roll",
       "Accel pitch",
       "Kalman roll",
       "Kalman pitch",
       "AB roll",
       "AB pitch"
       );
#
axis([200 400 -3.5 +3.5]);
grid on;
grid minor on;

