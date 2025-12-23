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
    %Colocar o código aqui

    %Obtendo a etiqueta do Robô
    % No referencia original o eixo x não aponta para frete do robô, por
    % isso foi adicionado um referencial denotado de /youBot_ref no qual o
    % eixo x aponta para a frente do robô
    [returnCode, KUKA] = sim.simxGetObjectHandle(clientID, '/youBot/youBot_ref', sim.simx_opmode_oneshot_wait);
   
    % Obtendo as etiquetas dos motores
    [returnCode, omega_1] = sim.simxGetObjectHandle(clientID, '/youBot/rollingJoint_rr', sim.simx_opmode_blocking); % roda 1
    [returnCode, omega_2] = sim.simxGetObjectHandle(clientID, '/youBot/rollingJoint_rl', sim.simx_opmode_blocking); % roda 2
    [returnCode, omega_3] = sim.simxGetObjectHandle(clientID, '/youBot/rollingJoint_fl', sim.simx_opmode_blocking); % roda 3
    [returnCode, omega_4] = sim.simxGetObjectHandle(clientID, '/youBot/rollingJoint_fr', sim.simx_opmode_blocking); % roda 4    
    
    %parametros do robo p/ cinematica inversa
    L = 0.47/2
    W = 0.3/2
    r = 0.0475

    %ponto de referencia
    q_U_ref=[1.725; 1.7; deg2rad(180)];

    %pesos de ganho
    Kx = 0.5;
    Ky = 0.5;
    Kteta = 0.5;

    matriz_ganho=[Kx, 0, 0;
                  0, Ky, 0;
                  0,  0, Kteta]; 

    %posicionando/etiquetando referencial final
    [returnCode, goalFrame] = sim.simxGetObjectHandle(clientID, '/Objetivo', sim.simx_opmode_oneshot_wait);
    returnCode = sim.simxSetObjectPosition(clientID, goalFrame, -1, [q_U_ref(1), q_U_ref(2), 0.039], sim.simx_opmode_oneshot_wait);
    returnCode = sim.simxSetObjectOrientation(clientID, goalFrame, -1, [0, 0, q_U_ref(3)], sim.simx_opmode_oneshot_wait);
    
    while (1)
        
        [returnCode, pos] = sim.simxGetObjectPosition(clientID, KUKA, -1, sim.simx_opmode_oneshot_wait);
        [returnCode, ori] = sim.simxGetObjectOrientation(clientID, KUKA, -1, sim.simx_opmode_oneshot_wait);
        q_U = [pos(1); pos(2); ori(3)]; % Posição do robo ao referencial global
    
        %referencial final menos a posição atual
        erro = q_U_ref - q_U;

        %orientação do robo
        theta=q_U(3);
        rot_U_R=[cos(theta), sin(theta), 0;
                 -sin(theta), cos(theta), 0;
                 0 0 1];
    
        %referencial velocidade global
        q_dot_U = matriz_ganho*erro;
        %referencial velocidade no robo
        q_dot_R = rot_U_R*q_dot_U;
    
        % Cinemática Inversa
        M_inv = 1/r*[1, -1, -(L+W); 
                1, 1, (L+W); 
                1, -1, (L+W);
                1, 1, -(L+W)]; 

        %velocidade
        Vrob = M_inv*q_dot_R;

        [returnCode]=sim.simxSetJointTargetVelocity(clientID,omega_1,Vrob(1),sim.simx_opmode_blocking); % roda 1
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,omega_2,Vrob(2),sim.simx_opmode_blocking); % roda 2
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,omega_3,Vrob(3),sim.simx_opmode_blocking); % roda 3
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,omega_4,Vrob(4),sim.simx_opmode_blocking); % roda 4

        if norm(erro(1:2)) < 0.02
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