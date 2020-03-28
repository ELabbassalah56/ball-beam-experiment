%$$$$$$$$$$$problem1.1$$$$$$$$$$$$$$%
prompt = {'Enter mass of object ?','Enter raduis of ball?','Enter lever arm offset?','Enter length of beam? ' };
dlgtitle = 'Input';
dims = [1 35];
definput = {'0.11','0.15','0.03','1.0'};
ans1 = inputdlg(prompt,dlgtitle,dims,definput);
M= str2double(ans1{1}); %mass of the ball in (kg)
R= str2double(ans1{2}); %radius of the ball in (m)
D= str2double(ans1{3}); %lever arm offset  in (m)
g = - 9.8; %gravitational acceleration (m/s^2)
L = str2double(ans1{4}); %length of the beam in (m)
J =  9.99e-6; % ball's moment of inertia (kg.m^2)
%(r)         %ball position coordinate
%(alpha) % beam angle coordinate
%(theta)  %servo gear angle

%%%%%%%design criteria%%%%%%%
%%%%%Settling time < 3 seconds
%%%%Overshoot < 5%

%%%%% breif about how system work %%%%%

%%%%after some mathematical equation,linearization method and laplace transform we obtain the
%%%%transfer function (Theta(s)) to the ball position (R(s)) which
%%%%(Theta(s))is a input and control it by Mc send pulse width modulation
%%%%ex(after each 20 msec servo motor change his theta) the ball position
%%%%(R(s)) is the 0output 

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$%

s=tf('s'); %allowed to use symbolic rather than the nomenator /denomenator

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$%
while true 
   prompt = 'What do I show you ?';
   Stage = menu(prompt,'open_loop transfer function of ball beam model ','space_state control model','AID control model','Root_Locus model ','Exit');
    
   switch Stage
       case  1
           figure('Name','open loop system analysis ','NumberTitle','off');
            subplot(1,13,[1 2 3 4 5 6]);
            title('open loop specifications');
           Gp_ball = -M*g*D/L/(J/R^2+M)/s^2; %%open_loop transfer function of ball beam model  in (m/rad)
           Gp_ball_text=text(0.0,0.8,'Gpball=-M*g*D/L/(J/R^2+M)/s^2');
          Gp_ball_tex.FontSize=80;
                x_num=0.5;
                yA=0.5;
               
                poles = pole(Gp_ball);  %$system is unstable because has two poles at zero                  
                pole_textA=text(0.0,yA,'poles of open loop sys are :');
                pole_textA.FontSize = 28;
                pole_num=text(x_num,yA-0.2,num2str(poles));
                pole_num.FontSize=24;
             
                subplot(1,13,[ 7 8 9] );
                step(Gp_ball);
                title('step responce ');
                
                subplot(1,13,[11 12 13]);
                pzmap(Gp_ball);
                
              
       case 2   
   %%%%after using state_space model which is powerful tool to analyze the
                %%%%sytem (mathimatical model of physical systems set of i/o,state variables relatedto 1st order DQ )
                %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$%
                %x'=Ax+Bu 
                %state_space eq of system 
                %y=Cx+Du  
                            A13 = -M*g/(J/(R^2)+M);
                            A=[0 1 0 0;0 0  A13  0;0 0 0 1;0 0 0 0];
                            B = [0 0 0 1]';
                            C = [1 0 0 0];
                            D = 0;
                            ball_ss = ss(A,B,C,D);%converted system to state_space model
                             while true 
                                              prompt = 'What do I show you ?';
                                                Stage0 = menu(prompt,'show you range of  poles for system ',' then write over defult  poles ','Next stage');       
                               switch Stage0
                                   case 1            %%know your range to choose your poles%%%%
                                                 V0=tf([1   0.01],[1  5]);
                                                 rlocus(V0*Gp_ball)
                                                 sgrid(0.7, 1.9)
                                                 axis([-5 5 -2 2])
                                                 [U0,poles0] = rlocfind(V0*Gp_ball);
                                                    text(0.5,0.5,num2str(poles0));
                                                 
                                   case 2              
                                %%%%%%%%%%%%%state_space controller%%%%%%%%%%%%%
                                        %take selected poles and entered in poles
                                        prompt = {'Enter poles n1 ?','Enter poles n2?','Enter poles n3?','Enter poles n4? ' };
                                        dlgtitle = 'Input';
                                        dims = [1 35];
                                        definput = {'-2+4i','-2-4i','-20','-40'};
                                        answer = inputdlg(prompt,dlgtitle,dims,definput);
                                        p1 = str2double(answer{1});
                                        p2 = str2double(answer{2});
                                        p3 = str2double(answer{3});%this poles we put it far left to not affect on the stability of my systems 
                                        p4 = str2double(answer{4});
                                        K = place(A,B,[p1,p2,p3,p4]);
                                        
                                   case 3
                             
                      while true 
                               prompt = 'What do I show you ?';
                               Stage_1 = menu(prompt,'closed loop responce befor modefing s.s error ','closed loop after modifing s.s error','Exit');
                          switch Stage_1
                     case  1
                
                        pole_textB=0:0.01:5;
                        uB = 0.25*ones(size(pole_textB));
                        sys_closedB = ss(A-B*K,B,C,D);
                        [yB,pole_textB,xB]=lsim(sys_closed,uB,pole_textB);
                        
                        figure('Name','closed loop responce befor modefing s.s error ','NumberTitle','off')
                        plot(pole_textB,yB);
                        hold on
                        xlabel('time(second)');
                        ylabel('steady_state_error');
                    case 2             
                        Nbar=rscale(ball_ss,K);
                        pole_textA=0:0.01:5;
                        uA = 0.25*ones(size(pole_textA));
                        sys_closed = ss(A-B*K,B,C,D);
                        [yA,pole_textA,xA]=lsim(Nbar*sys_closed,uA,pole_textA);
                         figure('Name','closed loop after modifing s.s error ','NumberTitle','off')
                        plot(pole_textA,yA);
                        xlabel('time(second)');
                        ylabel('steady_state_error');
                    case 3
                                break;
                          end
                      end
                                   break;
                                  
                               end
                             end
                             
       case 3  
           
      %%%%%%%%pid_controller%%%%%%%%%%%%%
             prompt = {'enter gain term of system closed loop?','Enter derivative term to the controller?','enter dambing ratio?','Enter natuer frequency ' };
            dlgtitle = 'Input';
            dims = [1 35];
            definput = {'1','1','0.7','1.9'};
            answer = inputdlg(prompt,dlgtitle,dims,definput);
            Kp=str2double(answer{1});% gain proppotional with closed loop transfer function
            Kd=str2double(answer{2}); %add a derivative term to the controller
            F = pid(Kp,0,Kd);
            sys_closed=feedback(F*Gp_ball,1); %closed loop 
            T=0:0.01:5;
            figure('Name','step responce of closed loop system ','NumberTitle','off')
            step(0.25*sys_closed)%step responce of sys_closed
            xlabel('time(sec)')
            ylabel('amplituide')
           
       case 4
                           %%%%%%%%%root locus%%%%%%%%%%%%%%%%%%%%%%

                %%%The main idea of the root locus design is 
                %%%to estimate the closed-loop response from the open-loop root locus plot
                %%%By adding zeroes and/or poles to the original system (adding a compensator)
                prompt = {'Enter zero ?','Enter pole no1?','enter dambing ratio?','Enter natuer frequency ' };
                dlgtitle = 'Input';
                dims = [1 35];
                definput = {'0.01','5','0.7','1.9'};
                answer = inputdlg(prompt,dlgtitle,dims,definput);
                R_damped=str2double(answer{3});
                N_freq=str2double(answer{4});
                z0 = str2double(answer{1});
                p1 = str2double(answer{2});
                V=tf([1   z0],[1  p1]);

                rlocus(V*Gp_ball)
                sgrid(R_damped, N_freq)
                axis([-5 5 -2 2])
                [U,poles]=rlocfind(V*Gp_ball) ;% we may select a gain that will satisfy our design requirements
            case 5 
                   c_text=text(0.1,0.8,'Thanks for using the program, and I hope the code is clear ');
                   c_text.FontSize=100;
                   b_text=text(0.2,0.7,' "I hope that you observe our efforts" ');
                   b_text.FontSize=100;
              break;
   end
end