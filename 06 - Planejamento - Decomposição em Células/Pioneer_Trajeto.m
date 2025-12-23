% Programa de teste do robô Manta modelo Ackerman

clc;
close all;
clear all;

disp('Início do Programa');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);% inicia a comunicação com o servidor



if clientID > -1
    disp('Conectou ao Servidor');
    % Colocar o se código aqui

    % Ativa modo síncrono
    sim.simxSynchronous(clientID,true);
    
    % Obtendo a etiqueta (handle) do robô
    [returnCode, robotHandle] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking); % modo bloqueante simx_opmode_oneshot_wait=imx_opmode_blocking
  
    % Obtendo as etiquetas dos motores
    [returnCode, l_motor] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking); 
    [returnCode, r_motor] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking);


   
    w0 = 5; %velocidade de operação

  
    [returnCode, robotPos] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait);
    disp([returnCode, robotPos]);



    %Dados do trajeto
    x_trajeto=[-4.25 -4.25 4 -7];
    y_trajeto=[-4.25 1.75 3 1];
    %x_trajeto=[-7 4];
    %y_trajeto=[1 1];
    x0=x_trajeto(1);
    y0=y_trajeto(1);
    x1=x_trajeto(2);
    y1=y_trajeto(2);

    N_pontos=length(x_trajeto);
    Trajeto_atual=1;
    Desligar =0; % Flag para desligar o carrinho
    
    %Dados do controlador
    erro=0; % no simulador dá de ver que precisa filtrar o erro
    erro=0;
    erro_an=0;
    erro_an2=0;
    integrador=0;
    Kp=2.0;     % 0.9
    Kd=1.2;     % 1.2
    Ki=0.0;     % Trabalhar junto com o erro de alinhamento das rodas 
    delta_v_max=3; %variação máxima das velocidades do motor da esquerda e da direita
    delta=0; %inicialmente sem varição de velocidade nas rodas
    %erro_alinhamento_beta=8*pi/180; %supondo que existe um erro de alinhamento de 8º entre o volante e as rodasdianteiras
    erro_alinhamento_beta=0*pi/180; %supondo que existe um erro de alinhamento de 8º entre o volante e as rodasdianteiras
    FxyT=-1; % supoe-se que inicialmente o carro está antes do objetivo

    % Inicialização de variáveis
    t = 0;         % tempo inicial
    Ts=50e-3; % Periodo de amostragem adotado. Obtido do simulador CoppeliaSim

    k=1;


    while Desligar == 0

        if FxyT>=0 %Se positivo é porque passou a barreira do reta perpendicular objetivo então pega o próximo ponto
            if Trajeto_atual < N_pontos % Se alcanço o ponto objetivo, muda para a próxima reta de referência
                %disp('alcançou o objetivo parcial');
                x0=x_trajeto(Trajeto_atual);
                y0=y_trajeto(Trajeto_atual);
                x1=x_trajeto(Trajeto_atual+1);
                y1=y_trajeto(Trajeto_atual+1);
                Trajeto_atual=Trajeto_atual+1;
            else
                Desligar=1;
                disp('alcançou o objetivo final');
            end
        end

        %pegar as coordenadas do robô para controlar a sua posição
        [returnCode, robotPos] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_streaming + 10);

        x=robotPos(1);
        y=robotPos(2);
        %f(x,y)=ax+by+c reta de referência
        a=y1-y0;
        b=x0-x1;
        c=x1*y0-x0*y1;
        Fxy=a*x+b*y+c;

        %reta ortogonal passando por (x1,y1)
        c2=b*x1-a*y1;
        FxyT=-b*x+a*y+c2;


        %controlador de faixa
        erro_an2=erro_an;
        erro_an=erro;

        erro=Fxy/sqrt(a^2+b^2);

        %implementação da ação de controle
        %Será necessário um integrador devido à folga do volante
        % Avaliar qual dos controladores é melhor

        integrador = integrador+erro*Ts;
        Prop=Kp*erro;
        Deriv=Kd*(erro-erro_an)/Ts;
        Integ=Ki*integrador;

        delta= Prop+ Deriv+Integ; %

        if delta > delta_v_max  %saturação da ação de controle
            delta= delta_v_max;
            integrador = integrador-erro*Ts; % Desacumula o integrador
        elseif delta < -delta_v_max
            delta=-delta_v_max;
            integrador = integrador-erro*Ts; % Desacumula o integrador
        end

        %atuando nas rodas do robô

        wD=w0+delta;
        wE=w0-delta;
        % Enviando dados para as rodas do robô
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,r_motor,wD,sim.simx_opmode_blocking);
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,l_motor,wE,sim.simx_opmode_blocking);

        %armazenando dados para graficar
        dt_v(k)=Ts;
        t_v(k)=t;

        erro_v(k)=erro;
        errof_v(k)=erro;

        Prop_v(k)=Prop;
        Deriv_v(k)=Deriv;
        delta_v(k)=delta;


        k=k+1;
        sim.simxSynchronousTrigger(clientID); % dispara a sincronização
        t=t+Ts;

    end

    figure;
    plot(t_v,dt_v);
    title('diferencial de tempo');
    %ylim([0 0.03]);

    figure;
    plot(t_v,delta_v,t_v,Prop_v,t_v,Deriv_v, t_v, erro_v,t_v, errof_v);
    title('ação de controle');
    legend('Delta','Proporcional','Derivativo','Erro','Erro Filtrado');
    grid;

    
    %Parando o robô
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,r_motor,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,l_motor,0,sim.simx_opmode_blocking);

    % Parando a simulação
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    
    % Fechando a conexão com o CoppeliaSim
    sim.simxFinish(clientID)
else
    disp('Falha na conexão no servidor remoto');
end
sim.delete(); % call the destructor!

disp('Fim do programa');
