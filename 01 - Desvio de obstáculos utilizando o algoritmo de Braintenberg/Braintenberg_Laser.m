clc;
close all;
clear all;

disp('Início do Programa');
sim=remApi('remoteApi');
sim.simxFinish(-1); 
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if clientID > -1
    disp('Conectou ao Servidor');

    % Obtendo a etiqueta (handle) do robô
    [returnCode, robotHandle] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking); % modo bloqueante simx_opmode_oneshot_wait=imx_opmode_blocking
  
    % Obtendo as etiquetas dos motores
    [returnCode, l_motor] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking); 
    [returnCode, r_motor] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking);

    % Dados do Hokuyo
    laser_range_data = 'hokuyo_range_data';
    laser_angle_data = 'hokuyo_angle_data';

    % Aguardando a primeira leitura válida (As primeiras leituras preisam ser no modo Streaming)
    returnCode1 = 1;
    returnCode2 = 1;
    while returnCode1 ~= 0 && returnCode2 ~= 0
        [returnCode1, string_range_data] = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_streaming + 10); % 10 representa 10ms de espera entre 2 comandos consecutivos no lado do servidor
        [returnCode2, string_angle_data] = sim.simxGetStringSignal(clientID, laser_angle_data, sim.simx_opmode_streaming + 10); % 10 representa 10ms de espera entre 2 comandos consecutivos no lado do servidor
    end

    N_leituras=0;

    % Verificar se os dados foram obtidos corretamente
    if ~isempty(string_range_data) && ~isempty(string_angle_data)
        % Descompactar os dados de intervalo e ângulo
        raw_range_data = sim.simxUnpackFloats(string_range_data);
        raw_angle_data = sim.simxUnpackFloats(string_angle_data);
    else
        raw_range_data = [];
        raw_angle_data = [];
    end
    
            %As próximas Leituras podem ser no modo buffer
        [~, string_range_data] = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_buffer);
        [~, string_angle_data] = sim.simxGetStringSignal(clientID, laser_angle_data, sim.simx_opmode_buffer);
    
        % Verificar se os dados foram obtidos corretamente
        if ~isempty(string_range_data) && ~isempty(string_angle_data)
            % Descompactar os dados de intervalo e ângulo
            raw_range_data = sim.simxUnpackFloats(string_range_data);
            raw_angle_data = sim.simxUnpackFloats(string_angle_data);
    
        else
            raw_range_data = [];
            raw_angle_data = [];
        end
    
    N_leituras = length(raw_angle_data);
    N_dados = 5000;
    kk=1:N_dados;

    Vrob=8; 
    Dist_sem_objeto=0.99;   % 99 cm
    Dist_zona_morta=0.2;   % 20 cm
    ganho = -0.0480; % intervalo dos pesos dividido pelo numero de sensores
    pesos_dir = linspace(0.2, 1.6, N_leituras) * ganho;
    pesos_esq = fliplr(pesos_dir);

    for i=1:N_dados
        %As próximas Leituras podem ser no modo buffer
        [~, string_range_data] = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_buffer);
        [~, string_angle_data] = sim.simxGetStringSignal(clientID, laser_angle_data, sim.simx_opmode_buffer);
    
        % Verificar se os dados foram obtidos corretamente
        if ~isempty(string_range_data) && ~isempty(string_angle_data)
            % Descompactar os dados de intervalo e ângulo
            raw_range_data = sim.simxUnpackFloats(string_range_data);
            raw_angle_data = sim.simxUnpackFloats(string_angle_data);
    
        else
            raw_range_data = [];
            raw_angle_data = [];
        end

        % Vetores dos sensores
        detect= zeros(1,N_leituras); 
        
        % Fazer a leitura e verificar se há algo na frente
        for j=1:N_leituras  % J até leituraas

            if(raw_range_data(j) < Dist_sem_objeto)
                if(raw_range_data(j) < Dist_zona_morta) %Satura em 20cm na Zona morta
                    detect(j) = Dist_zona_morta;
                end
                    detect(j) = (Dist_sem_objeto - raw_range_data(j))/(Dist_sem_objeto - Dist_zona_morta); %Equação detect
            else
                detect(j) = 0;
            end

        end

         v_esq = Vrob;
         v_dir = Vrob;

         %Diminuir recursivamente a velocidade dos motores
         for k=1:N_leituras
             v_esq = v_esq + pesos_esq(k)*detect(k);
             v_dir = v_dir + pesos_dir(k)*detect(k);
         end

        % Atualizar velocidades

        sim.simxSetJointTargetVelocity(clientID, r_motor, v_dir, sim.simx_opmode_oneshot);
        sim.simxSetJointTargetVelocity(clientID, l_motor, v_esq, sim.simx_opmode_oneshot);
        pause(0.01);

    end

        % Plotanto em coordenadas cartesianas 
        [x,y]=pol2cart(raw_angle_data,raw_range_data); 
        figure; 
        plot(x,y,'o'); 
        title('Teste do sensor Laser Hokuyo Cartesiano'); 
        grid; 
        axis equal;
       
        % Parando a simulação
        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);    
        sim.simxFinish(clientID);
        sim.delete();
        disp('Conexão finalizada');
end

