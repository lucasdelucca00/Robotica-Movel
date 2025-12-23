% Primeiro teste do CoppeliaSim
% Este código  conecta ao CoppeliaSim, para isso o Coppelia precisa
% estar rodando (play) e no script da cena principal acrescentar o código a
% seguir: simRemoteApi.start(19999)
% Na sequência são testados os motores e sensores ultrassônicos

clc;
close all;
clear all;

disp('Início do Programa');
sim=remApi('remoteApi'); % cria uma instância de simulação
sim.simxFinish(-1); % por precaução, feche todas as conexões abertas
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);% inicia a comunicação com o servidor


if clientID > -1
    disp('Conectou');
   
    %etiquetando o robo
    [returnCode, robotHandle] = sim.simxGetObjectHandle(clientID, '/Manta', sim.simx_opmode_blocking);
   
    % Obtendo as etiquetas dos motores
    [returnCode, Motor] = sim.simxGetObjectHandle(clientID, '/Manta/motor_joint', sim.simx_opmode_blocking);
    [returnCode, Volante] = sim.simxGetObjectHandle(clientID, '/Manta/steer_joint', sim.simx_opmode_blocking);

    %referencia final
    q_U_ref=[1.9; 3.975];

    % Posiciona a referência na cena
    [returnCode, goalFrame] = sim.simxGetObjectHandle(clientID, '/Objetivo', sim.simx_opmode_oneshot_wait);
    returnCode = sim.simxSetObjectPosition(clientID, goalFrame, -1, [q_U_ref(1), q_U_ref(2), 0], sim.simx_opmode_oneshot_wait);

    %ganho empirico
    K_rho = 2.5; %ganho em relação a velocidade
    K_alpha = 1.2; %ganho em relação ao quanto vira o volante


    while(1)
        
        %posição do robo em x e y
        [returnCode, pos] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait);
        [returnCode, ori] = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait);
        q_U = [pos(1); pos(2)];
        teta = ori; %orientação do robo
    
        %referencial final menos a posição atual
        delta_X_Y = q_U_ref - q_U; 

        %distância do robo ate posição final
        p = norm(delta_X_Y(1:2));

        %ângulo da posição final em relaçao ao eixo xy
        phi = atan2(delta_X_Y(1), delta_X_Y(2));

        %ângulo do robo em relacao ao objetivo
        alpha = NormalizaAngulo(-ori(3) + atan2(delta_X_Y(2), delta_X_Y(1)));
        Vrob = K_rho*p; %controle de velocidade vruuuUUUUUUUM

        %controle do volante(quanto vira pra direita ou esquerda)
        ang_volante = K_alpha*alpha;

        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Motor,Vrob,sim.simx_opmode_blocking); % roda 1
        [returnCode]=sim.simxSetJointTargetPosition(clientID,Volante,ang_volante,sim.simx_opmode_blocking); % roda 2
        
        disp(delta_X_Y);
        
        if norm(delta_X_Y(1:2)) < 0.5
        break;
        end
    end

    % Parando a simulação
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    % Fecha a conexão com o CoppeliaSim
    sim.simxFinish(clientID);
else
    disp('Falha na conexão no servidor remoto');
end
sim.delete(); % call the destructor!

disp('Fim do programa');

%garantir que o ângulo fique entre -180 e 180
function Angulo = NormalizaAngulo(Angulo)
    Angulo = mod(Angulo + pi, 2*pi) - pi;
end