% Software-only end-to-end scenario model for the AuroraArm joint.
% Assumed encoder/ADC/PWM resolutions are explicit and are not hardware specs.
clear; clc;
project_root=fileparts(fileparts(mfilename('fullpath'))); cd(project_root);
R=0.8; L=1.2e-3; Kt=0.085; Ke=0.085; J=4.2e-4; b=1.5e-4;
N=100; Vmax=48; imax=8; Ts=2e-3; arm=0.35;
counts_per_rev=65536; adc_bits=12; pwm_bits=12;
kp=800; ki=400; kd=45; ref_step=0.1; duration=30;
A=[-R/L,-Ke/L,0;Kt/J,-b/J,0;0,1,0];
B=[1/L,0;0,-Kt/J;0,0];
Md=expm([A B;zeros(2,5)]*Ts); Ad=Md(1:3,1:3); Bd=Md(1:3,4:5);
n=round(duration/Ts); loads=[0.20,0.25,0.30];
encoder_step=2*pi/counts_per_rev;
current_step=2*imax/(2^adc_bits-1);
voltage_step=2*Vmax/(2^pwm_bits-1);
phi=exp(-R*Ts/L); gamma_i=(1-phi)/R;
results=zeros(numel(loads)+1,7);

% Four deterministic load cases: 0.20, 0.25, 0.30 N m, plus a one-cycle
% actuator-command delay at nominal load. No random noise is used.
for scenario=1:numel(loads)+1
  if scenario<=numel(loads), tau_load=loads(scenario); actuator_delay=0;
  else, tau_load=0.25; actuator_delay=1; end
  x=zeros(3,1); integ=0; prev_motor_enc=0; ylog=zeros(n,1); vlog=ylog; ilog=ylog;
  delayed_u=zeros(actuator_delay,1);
  for k=1:n
    if actuator_delay>0, u_applied=delayed_u(1); else, u_applied=0; end
    theta_motor_enc=round(x(3)/encoder_step)*encoder_step;
    prev_sensed_motor=prev_motor_enc;
    prev_motor_enc=theta_motor_enc;
    sensed_motor=theta_motor_enc;
    theta_joint=sensed_motor/N;
    current_adc=round(x(1)/current_step)*current_step;
    omega_est=(sensed_motor-prev_sensed_motor)/Ts;
    e=ref_step-theta_joint;
    dtheta=(sensed_motor-prev_sensed_motor)/(Ts*N);
    u0=kp*e+ki*integ-kd*dtheta;
    u=max(-Vmax,min(Vmax,u0));
    % Quantized drive command chosen from levels that satisfy the predicted
    % current constraint. With a delayed command, predict through the queued
    % command that will be applied during the next sample.
    if actuator_delay==0
      i_prediction=current_adc;
    else
      i_prediction=phi*current_adc+gamma_i*(u_applied-Ke*omega_est);
    end
    umax_i=(imax-phi*i_prediction)/gamma_i+Ke*omega_est;
    umin_i=(-imax-phi*i_prediction)/gamma_i+Ke*omega_est;
    lo=max(-Vmax,umin_i); hi=min(Vmax,umax_i);
    lo_code=ceil(lo/voltage_step); hi_code=floor(hi/voltage_step);
    if lo_code<=hi_code
      ucode=min(hi_code,max(lo_code,round(u/voltage_step)));
      u=ucode*voltage_step;
    else
      u=0; % fail-safe fallback if sensor estimates leave no feasible code
    end
    if (abs(u-u0)<1e-5)||(u<u0 && e<0)||(u>u0 && e>0), integ+=Ts*e; end
    if actuator_delay==0
      u_applied=u;
    else
      delayed_u=[delayed_u(2:end);u];
    end
    x=Ad*x+Bd*[u_applied;tau_load];
    ylog(k)=theta_joint; vlog(k)=u_applied; ilog(k)=x(1);
  end
  overshoot=max(0,(max(ylog)-ref_step)/ref_step*100);
  outside=find(abs(ylog-ref_step)>0.02*ref_step,1,'last');
  if isempty(outside), settling=0; else, settling=outside*Ts*1000; end
  results(scenario,:)=[tau_load,actuator_delay*Ts*1000,overshoot,settling,abs(ylog(end)-ref_step),max(abs(vlog)),max(abs(ilog))];
  if scenario==2, nominal_y=ylog; nominal_v=vlog; nominal_i=ilog; end
end

if ~exist('figures','dir'), mkdir('figures'); end
figure(1); t=(0:n-1)'*Ts; plot(t,nominal_y,'LineWidth',1.2); hold on;
plot(t,ref_step+0*t,'--'); grid on; xlim([0,1]);
xlabel('Tempo (s)'); ylabel('Posição da junta (rad)');
title('Ensaio SIL com encoder, ADC e comando PWM quantizados');
print('figures/software_only_step.png','-dpng','-r140');

fid=fopen('tests/software_only_results.txt','w');
fprintf(fid,'SOFTWARE_ONLY_MODEL: sampled motor + joint plant, quantized encoder/ADC/PWM, position PID\n');
fprintf(fid,'ASSUMPTIONS encoder=%d counts/rev; ADC=%d bits over +/-8 A; signed PWM=%d bits over +/-48 V; Ts=%.3g s\n',counts_per_rev,adc_bits,pwm_bits,Ts);
fprintf(fid,'ASSUMPTIONS one-cycle command delay scenario; no random noise, backlash, flexible links, thermal model, brake model, or real-time scheduler\n');
fprintf(fid,'load_Nm,actuator_delay_ms,overshoot_pct,settling_ms,final_error_rad,peak_abs_voltage_V,peak_abs_current_A\n');
for s=1:size(results,1)
 fprintf(fid,'%.3f,%.1f,%.6g,%.6g,%.9g,%.6g,%.6g\n',results(s,:));
end
fprintf(fid,'LIMITS: analytical software scenarios only; not physical safety or target WCET evidence\n');
fclose(fid);
disp(fileread('tests/software_only_results.txt'));
