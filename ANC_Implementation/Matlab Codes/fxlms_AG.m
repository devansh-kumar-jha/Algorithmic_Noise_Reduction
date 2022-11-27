%% Creating a Filtered X LMS algorithm for simulated noise reduction
% Simulate a FXLMS algorithm on the random data
clear all; clc; close all;

% Number of time samples
T = 1000;

% We do not know P(z) and S(z) in reality. So we have to make dummy paths
% primary path impulse response
Pw=[0.01 0.25 0.5 1 0.5 0.25 0.01];
% secondary path impulse response
Sw=Pw*0.25;

%% Step 1: Estimate the secondary path


% generate a random signal
% x_iden=randn(1,T);
x_r=randn(1,T);

% creating the output of secondary path (y_r = Sw convolution x_r)
y_r =filter(Sw, 1, x_r);

% Then, start the identification process
% applying the LMS to estimate impulse response of secondary path

M=16; % Length of cancellation filter

% Shx=zeros(1,16);     % the state of Sh(z)
x_temp =zeros(1,M);    % The reference mic signal available sequentially

% Shw=zeros(1,16);     % the weight of Sh(z)
cw = zeros(1,M);       % Impulse response or coefficients of cancellation filter

err_s =zeros(1,T); % error of secondary path estimate for cancellation filter design
% e_iden=zeros(1,T);   % data buffer for the identification error

% and apply least mean square algorithm
mu=0.1;                         % learning rate
y_temp=zeros(1,T);              % declare cancellation filter output
% y_temp2=zeros(1,T);

for k=1:T                      % discrete time k
%     Shx=[x_iden(k) Shx(1:15)];  % update the state
    x_temp = [ x_r(k) x_temp(1:15)]; % update reference input sequentially
    
%     Shy=sum(Shx.*Shw);	        % calculate output of Sh(z)
    y_temp(k) = sum(x_temp.*cw);    % calculate cancellation filter output
%     temp = filter(cw,1,x_r);
%     y_temp2(k)= temp(k);
    
%     err_s(k)=y_r(k)-Shy;    % calculate error         
    err_s(k)= y_r(k)-y_temp(k);      
    
%     Shw=Shw+mu*e_iden(k)*Shx;   % adjust the weight
    cw=cw+mu*err_s(k)*x_temp;   % adjust the weight
end

figure(1)
subplot(2,1,1)
plot(Sw,'-o');
hold on; stem(cw);
xlabel('Samples')
ylabel('Amplitude')
title('Impulse respone of secondary path and its estimated IR')
legend('Sw','Cw')

subplot(2,1,2)
plot(err_s);

xlabel('Samples')
ylabel('Amplitude')
title('Error in estimating the secondary path filter')

%% Step 2: Implementing the FXLMS algorithm using estimated secondary path 
%  filter cw

% Creating a reference microphone signal
x_ref=randn(1,T);

% measuring the noise arriving at error microphone from primary path,
Yp=filter(Pw, 1, x_ref);

% Initiate the system,
% Cx=zeros(1,16);       % the state of C(z)
% length of the adaptive filter
N=16;
temp_x_ref = zeros(1,N);

% Cw=zeros(1,16);       % the weight of C(z)
W=zeros(1,N);

% Sx=zeros(size(Sw));   % the dummy state for the secondary path
temp_x_sec= zeros(size(Sw));   % temporary variable to store input for secondary path

% e_cont=zeros(1,T);    % data buffer for the control error
err = zeros(1,T); % to store error of primary and secondary path

% Xhx=zeros(1,16);      % the state of the filtered x(k)
temp_x_c = zeros(1,M);  % temporary variable to store input for estimated 
                        % secondary path

% FxLMS algorithm
mu=0.1;                            % learning rate
for k=1:T                         % discrete time k
    
%     Cx=[X(k) Cx(1:15)];            % update the controller state    
    temp_x_ref = [x_ref(k) temp_x_ref(1:end-1)];
    
%     Cy=sum(Cx.*W);                % calculate the controller output	
    temp_y=sum(temp_x_ref.*W);
    
%     Sx=[Cy Sx(1:length(Sx)-1)];    % propagate to secondary path
    temp_x_sec = [temp_y temp_x_sec(1:end-1)];

%     e_cont(k)=Yd(k)-sum(Sx.*Sw);   % measure the residue
    err(k) = Yp(k) - sum(temp_x_sec.*Sw);
    
    
%     Shx=[X(k) Shx(1:15)];          % update the state of Sh(z)
    
    
%     Xhx=[sum(Shx.*Shw) Xhx(1:15)]; % calculate the filtered x(k)
   temp_x_c =[sum(temp_x_ref.*cw) temp_x_c(1:end-1)]; % calculate the filtered x(k)
    
    W=W+mu*err(k)*temp_x_c;        % adjust the controller weight
end

figure(2)
subplot(2,1,1)
plot(err)
title('error in LMS')
xlabel('Samples')
ylabel('Amplitude')

subplot(2,1,2)
plot(Yp)
hold on; plot(Yp - err)
title('Noise and anti-noise signal at error microphone')
xlabel('Samples')
ylabel('Amplitude')
legend('Noise','Anti-noise');
