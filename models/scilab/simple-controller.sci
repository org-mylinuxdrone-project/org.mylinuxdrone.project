A=[-1,-1,1,1;1,-1,1,-1;1,-1,-1,1;1,1,1,1];
RM=[60/100,0,0,0;0,80/100,0,0;0,0,75/100,0;0,0,0,68/100];
Omega=[10,10, 10, 10, 10, 10;
        0, 0,  0,  0,  0,  0;
        0, 0,  0,  0,  0,  0;
       20,20, 20, 20, 20, 20;
];

function out = genOmega(num,rip)
    for j=[1:1:num]
         for i=[1:1:rip]
             m(1:4,i+(j-1)*rip)=[30*sin(j-1);0;0;20];
             //m(1:4,i+(j-1)*rip)=[10;0;0;20];
         end
    end
    out = m;
endfunction

function out = calcIn(Omega)
         m(1:4,1)=Omega(1:4,1);
         for i=[2:1:size(Omega,2)]
             m(1:4,i)=Omega(1:4,i)-Omega(1:4,i-1);
         end
         out = m;
endfunction

function out = calcF(In)
         for i=[1:1:size(In,2)]
             m(1:4,i)=A'/4*In(1:4,i);
         end
         out = m;
endfunction

function [Imu, Err, ErrI, ErrD, Out, Motors, Resp] = calcStepsSimple(F,kp,ki,kd, Rnd)
         Imu(1:4,1)=[0;0;0;0];
         Err(1:4,1)=[0;0;0;0];
         ErrI(1:4,1)=[0;0;0;0];
         ErrD(1:4,1)=[0;0;0;0];
         Resp(1:4,1)=[1;1;1;1];
         Out(1:4,1)=F(1:4,1)+A'/4*Err(1:4,1);
         Motors(1:4,1)=[1000;1000;1000;1000] + 1000*Out(1:4,1)/100;
         for i=[2:1:size(F,2)]
           Imu(1:4,i)=A*((RM.*cos(%pi/4*(i/size(F,2))))*(Motors(1:4,i-1)-1000))/1000*100 + Rnd(i);
           Err(1:4,i)=A*Out(1:4,i-1)-(Imu(1:4,i)-Imu(1:4,i-1));
           ErrI(1:4,i)=Err(1:4,i)+ErrI(1:4,i-1);
           ErrD(1:4,i)=Err(1:4,i)-Err(1:4,i-1);
           //Resp(1:4,i)=0.6*(1000*(A'/4*Imu(1:4,i))/100)./(Motors(1:4,i-1)-1000)+0.4*Resp(1:4,i-1);
           Resp(1:4,i)=(Motors(1:4,i-1)-1000)./(1000*(A'/4*Imu(1:4,i))/100);
           Out(1:4,i)=F(1:4,i)+A'/4*(kp*Err(1:4,i)+kd*ErrD(1:4,i)+ki*ErrI(1:4,i));
           Motors(1:4,i)=Motors(1:4,i-1) + 1000*Out(1:4,i)/100;
         end
endfunction

function [Imu, Err, ErrI, ErrD, Out, Motors, Resp] = calcStepsResp(F,kp,ki,kd, Rnd)
         Imu(1:4,1)=[0;0;0;0];
         Err(1:4,1)=[0;0;0;0];
         ErrI(1:4,1)=[0;0;0;0];
         ErrD(1:4,1)=[0;0;0;0];
         Resp(1:4,1)=[1;1;1;1];
         Out(1:4,1)=F(1:4,1)+A'/4*Err(1:4,1);
         Motors(1:4,1)=[1000;1000;1000;1000] + 1000*Out(1:4,1)/100;
         for i=[2:1:size(F,2)]
           Imu(1:4,i)=A*((RM.*cos(%pi/4*(i/size(F,2))))*(Motors(1:4,i-1)-1000))/1000*100 + Rnd(i);
           Err(1:4,i)=A*(Out(1:4,i-1)./Resp(1:4,i-1))-(Imu(1:4,i)-Imu(1:4,i-1));
           ErrI(1:4,i)=Err(1:4,i)+ErrI(1:4,i-1);
           ErrD(1:4,i)=Err(1:4,i)-Err(1:4,i-1);

           // FIXME: Quando la Resp tende a zero, Out tende a infinito.
           Resp(1:4,i)=0.1*(Motors(1:4,i-1)-1000)./(1000*(A'/4*Imu(1:4,i))/100)+0.9*Resp(1:4,i-1);
           Out(1:4,i)=(F(1:4,i)+A'/4*(kp*Err(1:4,i)+kd*ErrD(1:4,i)+ki*ErrI(1:4,i))).*Resp(1:4,i);
           Motors(1:4,i)=Motors(1:4,i-1) + 1000*Out(1:4,i)/100;
         end
endfunction

rand('normal');
F=calcF(calcIn(genOmega(10,10)));
Rnd=rand([1:1:size(F,2)])';
[myImu, myErr, myErrI, myErrD, myOut, myMotors, myResp]=calcStepsSimple(F, 1,0,0, Rnd);
[myImu1, myErr1, myErrI1, myErrD1, myOut1, myMotors1, myResp1]=calcStepsResp(F, 1.0,0.000,0.0,Rnd);
[myImu2, myErr2, myErrI2, myErrD2, myOut2, myMotors2, myResp2]=calcStepsResp(F, 1.0,0.00,0.00,Rnd);

T=[1:1:size(myImu,2)]
scf();
plot2d(T',[myImu(1,1:size(myImu,2))',myImu1(1,1:size(myImu1,2))',myImu2(1,1:size(myImu2,2))'],[3,4,5],leg="myImu@myImu1@myImu2");
scf();
plot2d(T',[myOut(1,1:size(myOut,2))',myOut1(1,1:size(myOut1,2))',myOut2(1,1:size(myOut2,2))'],[3,4,5],leg="myOut@myOut1@myOut2");
scf();
plot2d(T',[myErr(1,1:size(myErr,2))',myErr1(1,1:size(myErr1,2))',myErr2(1,1:size(myErr2,2))'],[3,4,5],leg="myErr@myErr1@myErr2");

