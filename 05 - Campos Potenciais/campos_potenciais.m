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

% Definição de parâmetros do Robô
L = 0.331;
r = 0.09751;
maxv = 1.0;
maxw = deg2rad(45);

lastTime=0;

if clientID > -1
    disp('Conectou');
    sim.simxSynchronous(clientID,true);

    %Colocar o código aqui
   
    % Obtendo a etiqueta (handle) do robô
    [returnCode, robotHandle] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking); % modo bloqueante simx_opmode_oneshot_wait=imx_opmode_blocking
  
    % Obtendo as etiquetas dos motores
    [returnCode, l_motor] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking); 
    [returnCode, r_motor] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking);

     % Obtendo dados do laser
    laser_range_data = 'hokuyo_range_data';
    laser_angle_data = 'hokuyo_angle_data';

     % Aguardando a primeira leitura válida (As primeiras leituras preisam ser no modo Streaming)
    returnCode1 = 1;
    returnCode2 = 1;
    while (returnCode1 ~= 0 && returnCode2 ~= 0)
        [returnCode1, string_range_data] = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_streaming + 10); % 10 representa 10ms de espera entre 2 comandos consecutivos no lado do servidor
        [returnCode2, string_angle_data] = sim.simxGetStringSignal(clientID, laser_angle_data, sim.simx_opmode_streaming + 10); % 10 representa 10ms de espera entre 2 comandos consecutivos no lado do servidor
    end

    N_leituras = 0;

    % Verificar se os dados do laser foram obtidos corretamente
    if ~isempty(string_range_data) && ~isempty(string_angle_data)
        % Descompactar os dados de intervalo e ângulo
        raw_range_data = sim.simxUnpackFloats(string_range_data);
        raw_angle_data = sim.simxUnpackFloats(string_angle_data);
    else
        raw_range_data = [];
        raw_angle_data = [];
    end

    N_leituras=length(raw_range_data);
    % Inicialização de variáveis
    P_Uref=[3; 2];     % Configuração de refência onde quero chegar
    kr = 0.4;     % velocidade 
    ka = 0.6;     % angulo do robo

    katr = 0.4;   % força atrativa
    krep = 0.01;    % força de desvio
    t = 0;         % tempo inicial

    % Posiciona a referência onde encontra o objetivo
    [returnCode, goalFrame] = sim.simxGetObjectHandle(clientID, '/Objetivo', sim.simx_opmode_oneshot_wait);
    returnCode = sim.simxSetObjectPosition(clientID, goalFrame, -1, [P_Uref(1), P_Uref(2), 0.039], sim.simx_opmode_oneshot_wait);
    %returnCode = sim.simxSetObjectOrientation(clientID, goalFrame, -1, [0, 0, 0], sim.simx_opmode_oneshot_wait);

    % Critério de parada: Se chegar a 10cm do objetivo termina.
    rho = inf;
    while rho > 0.1
         %As demais leituras podem ser no modo buffer
        [~, string_range_data] = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_buffer);
        [~, string_angle_data] = sim.simxGetStringSignal(clientID, laser_angle_data, sim.simx_opmode_buffer);
    
        % Verificar se os dados foram obtidos corretamente
        if (~isempty(string_range_data) && ~isempty(string_angle_data))
            % Descompactar os dados de intervalo e ângulo
            raw_range_data = sim.simxUnpackFloats(string_range_data);
            raw_angle_data = sim.simxUnpackFloats(string_angle_data);
        else
            raw_range_data = [];
            raw_angle_data = [];
        end

        N_leituras=length(raw_range_data);

        [returnCode, pos] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait);   
        pos_xy = [pos(1), pos(2)];
        [returnCode, ori] = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait);
        
        theta_r = ori(3);  % orientação do robô (em rad)
        matriz_r = [cos(theta_r), -sin(theta_r);
                    sin(theta_r),  cos(theta_r)];
        
        Ftotal = [0; 0];
        Fatr = [0; 0];
        Frep = [0; 0];
        
        for q = 1:N_leituras
            if raw_range_data(q) < 0.99
                % Posição do obstáculo relativa ao robô (em cartesiano)
                [x, y] = pol2cart(raw_angle_data(q), raw_range_data(q));
                q_rel = [x; y];
            
                % Transformação para o sistema global
                q_i = matriz_r * q_rel + pos_xy(:);  % posição global do obstáculo
  
                % Vetor entre robô e obstáculo (robô - obstáculo)
                difv = pos_xy(:) - q_i(:); 

                %formula da força de repulsão
                if raw_range_data(q) > 0.05
                    gamma = 2;
                    Frep = Frep + ((krep / raw_range_data(q)^3) * ((1/raw_range_data(q) - 1/0.95)^(gamma - 1)) * difv);
                end
            end
        end

        %controle da força de repulsão
        Frep_max = 5;
        if norm(Frep) > Frep_max
            Frep = (Frep / norm(Frep)) * Frep_max;
        end

        Fatr = katr * (P_Uref - pos_xy(:)); % força de atração até a meta

        Ftotal = Fatr + Frep

        Delta_x = Ftotal(1); 
        Delta_y = Ftotal(2);

        %distancia do robo em relação a meta
        rho = sqrt(Delta_x^2 + Delta_y^2);
        alpha = NormalizaAngulo(-ori(3) + atan2(Delta_y, Delta_x));

        if(alpha>pi/2 || alpha < -pi/2) % Se o alvo estiver para trás do robô
            v_R=0; % Ao invés disso pode ser programado para o robô andar de Re.
        else          
            v_R = kr*rho*cos(alpha);
        end

        w_R = ka*alpha;

        % Saturação das velocidades
        v_R = max(min(v_R, maxv), -maxv);
        w_R = max(min(w_R, maxw), -maxw);

        % Cinemática Inversa
        wD = ((2.0*v_R) + (w_R*L))/(2.0*r);
        wE = ((2.0*v_R) - (w_R*L))/(2.0*r);


        % Enviando dados para as rodas do robô
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,r_motor,wD,sim.simx_opmode_blocking);
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,l_motor,wE,sim.simx_opmode_blocking);

    end

    % Parar os motores
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,r_motor,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,l_motor,0,sim.simx_opmode_blocking);

    % Parando a simulação
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    % Fecha a conexão com o CoppeliaSim
    sim.simxFinish(clientID);
else
    disp('Falha na conexão no servidor remoto');
end
sim.delete(); % call the destructor!

disp('Fim do programa');


%Função para normalizar o ângulo entre (-pi e pi]
function Angulo = NormalizaAngulo(Angulo)
    Angulo = mod(Angulo + pi, 2*pi) - pi;
end