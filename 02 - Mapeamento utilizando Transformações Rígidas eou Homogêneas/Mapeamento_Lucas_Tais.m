% estar rodando (play) e no script da cena principal acrescentar o código a
% seguir: simRemoteApi.start(19999) 

clc;
close all;
clear all;

disp('Início do Programa');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if clientID > -1
    disp('Conectou ao Servidor');
    
    % Obtendo a etiqueta (handle) do robô
    [returnCode, robotHandle] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking); % modo bloqueante simx_opmode_oneshot_wait=imx_opmode_blocking
  
    % Obtendo as etiquetas dos motores
    [returnCode, l_motor] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking); 
    [returnCode, r_motor] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking);

    % Obtendo as etiquetas dos sensores da esquerda pra direita
    [returnCode, sensor_zero] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor[0]', sim.simx_opmode_blocking);
    [returnCode, sensor_um] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor[1]', sim.simx_opmode_blocking);
    [returnCode, sensor_dois] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor[2]', sim.simx_opmode_blocking);
    [returnCode, sensor_tres] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor[3]', sim.simx_opmode_blocking);
    [returnCode, sensor_quatro] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor[4]', sim.simx_opmode_blocking);
    [returnCode, sensor_cinco] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor[5]', sim.simx_opmode_blocking);
    [returnCode, sensor_seis] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor[6]', sim.simx_opmode_blocking);
    [returnCode, sensor_sete] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor[7]', sim.simx_opmode_blocking);

    % Obtendo dados do laser
    laser_range_data = 'hokuyo_range_data';
    laser_angle_data = 'hokuyo_angle_data';

    %realizando a primeira leitura no modo streaming conforme manual do
    %CoppeliaSim. Se naõ fizer isso não funciona
    % leitura de dados stream(buffer)
    [returnCode,detectionState,detectedPoint0,~,~]=sim.simxReadProximitySensor(clientID,sensor_zero,sim.simx_opmode_streaming); 
    [returnCode,detectionState,detectedPoint1,~,~]=sim.simxReadProximitySensor(clientID,sensor_um,sim.simx_opmode_streaming);
    [returnCode,detectionState,detectedPoint2,~,~]=sim.simxReadProximitySensor(clientID,sensor_dois,sim.simx_opmode_streaming);
    [returnCode,detectionState,detectedPoint3,~,~]=sim.simxReadProximitySensor(clientID,sensor_tres,sim.simx_opmode_streaming);
    [returnCode,detectionState,detectedPoint4,~,~]=sim.simxReadProximitySensor(clientID,sensor_quatro,sim.simx_opmode_streaming);
    [returnCode,detectionState,detectedPoint5,~,~]=sim.simxReadProximitySensor(clientID,sensor_cinco,sim.simx_opmode_streaming);
    [returnCode,detectionState,detectedPoint6,~,~]=sim.simxReadProximitySensor(clientID,sensor_seis,sim.simx_opmode_streaming);
    [returnCode,detectionState,detectedPoint7,~,~]=sim.simxReadProximitySensor(clientID,sensor_sete,sim.simx_opmode_streaming);

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
    
    % Tempo de simulação
    N_sensores = 8;
    N_leituras=length(raw_range_data);
    N_dados = 700;
    dist_zero=zeros(1,N_dados); %% Vetores dos sensores
    dist_um=zeros(1,N_dados);
    dist_dois=zeros(1,N_dados);
    dist_tres=zeros(1,N_dados);
    dist_quatro=zeros(1,N_dados);
    dist_cinco=zeros(1,N_dados);
    dist_seis=zeros(1,N_dados);
    dist_sete=zeros(1,N_dados);

    kk = 1:N_dados;

    % Dados do Pioneer
    V_rob = 4; %Robot Velocity
    Dist_sem_objeto=0.8;   % Distancia sem objeto na frente = 80cm
    Dist_zona_morta=0.2;   % Zona morta de deteção, menor a isso não detecta = 20cm
    converte = [];
    cont = 1;    

    for i=1:N_dados
        
       %pegando a leitura de cada sensor que foram para o stream(buffer)
       [returnCode,detectionState_zero,detectedPoint0,~,~]=sim.simxReadProximitySensor(clientID,sensor_zero,sim.simx_opmode_buffer);
       [returnCode,detectionState_um,detectedPoint1,~,~]=sim.simxReadProximitySensor(clientID,sensor_um,sim.simx_opmode_buffer);
       [returnCode,detectionState_dois,detectedPoint2,~,~]=sim.simxReadProximitySensor(clientID,sensor_dois,sim.simx_opmode_buffer);
       [returnCode,detectionState_tres,detectedPoint3,~,~]=sim.simxReadProximitySensor(clientID,sensor_tres,sim.simx_opmode_buffer);
       [returnCode,detectionState_quatro,detectedPoint4,~,~]=sim.simxReadProximitySensor(clientID,sensor_quatro,sim.simx_opmode_buffer);
       [returnCode,detectionState_cinco,detectedPoint5,~,~]=sim.simxReadProximitySensor(clientID,sensor_cinco,sim.simx_opmode_buffer);
       [returnCode,detectionState_seis,detectedPoint6,~,~]=sim.simxReadProximitySensor(clientID,sensor_seis,sim.simx_opmode_buffer);
       [returnCode,detectionState_sete,detectedPoint7,~,~]=sim.simxReadProximitySensor(clientID,sensor_sete,sim.simx_opmode_buffer);

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

        % Obter posição e orientação do robô
        [returnCode, pos] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait);
        pos_xy = [pos(1), pos(2)]; % X e Y no plano
        [returnCode, ori] = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait);
        theta_r = ori(3); % Yaw (rotação em torno do eixo Z)

        % Calculo das matrizes de rotação
        % Pos_mundo = Rot(θr)*Pos_robo + Pos_xy
        matriz_r = [cos(theta_r) , -sin(theta_r); %projeção em 2D
                    sin(theta_r) , cos(theta_r)];
        
        for j=1:N_leituras
            if(raw_range_data(j) <= 4.9 )
                [x, y] = pol2cart(raw_angle_data(j), raw_range_data(j));
                conversao_cartesiano = [x; y];
                converte(cont, :) = (matriz_r * conversao_cartesiano + pos_xy(:));
                cont = cont+1;
            end
        end
       
       %vetor de distancia euclidiana do sensor até o ponto detectado (em metros)
       dist_zero(i) = norm(detectedPoint0);
       dist_um(i) = norm(detectedPoint1);
       dist_dois(i) = norm(detectedPoint2);
       dist_tres(i) = norm(detectedPoint3);
       dist_quatro(i) = norm(detectedPoint4);
       dist_cinco(i) = norm(detectedPoint5);
       dist_seis(i) = norm(detectedPoint6);
       dist_sete(i) = norm(detectedPoint7);

        %Saturação e padronização dos sensores de 0 a 1
        %sensor 0
        if detectionState_zero>0 && dist_zero(i)<Dist_sem_objeto % se achou e está menos de 0.6m
            if (dist_zero(i)<Dist_zona_morta) % se a distancia menor a 0.2, satura em 0.2m
                dist_zero(i)=Dist_zona_morta;
            end
            %padronizando a detecção de 0 a 1. Em que 0 (sem obstáculo) e 1 (obstáculo).
            detect_zero=(Dist_sem_objeto-dist_zero(i))/(Dist_sem_objeto-Dist_zona_morta);
        else
            detect_zero=0;
        end

        %sensor 1
        if detectionState_um>0 && dist_um(i)<Dist_sem_objeto
            if (dist_um(i)<Dist_zona_morta)
                dist_um(i)=Dist_zona_morta;
            end
            %padronizando a detecção de 0 a 1. Em que 0 (sem obstáculo) e 1 (obstáculo).
            detect_um=(Dist_sem_objeto-dist_um(i))/(Dist_sem_objeto-Dist_zona_morta);
        else
            detect_um=0;
        end

        %sensor 2
         if detectionState_dois>0 && dist_dois(i)<Dist_sem_objeto
            if (dist_dois(i)<Dist_zona_morta) 
                dist_dois(i)=Dist_zona_morta;
            end
            %padronizando a detecção de 0 a 1. Em que 0 (sem obstáculo) e 1 (obstáculo ).
            detect_dois=(Dist_sem_objeto-dist_dois(i))/(Dist_sem_objeto-Dist_zona_morta);
        else
            detect_dois=0;
         end

         %sensor 3
          if detectionState_tres>0 && dist_tres(i)<Dist_sem_objeto % 
            if (dist_tres(i)<Dist_zona_morta) 
                dist_tres(i)=Dist_zona_morta;
            end
            %padronizando a detecção de 0 a 1. Em que 0 (sem obstáculo) e 1 (obstáculo).  
            detect_tres=(Dist_sem_objeto-dist_tres(i))/(Dist_sem_objeto-Dist_zona_morta);
            else
            detect_tres=0;
          end

           %sensor 4
           if detectionState_quatro>0 && dist_quatro(i)<Dist_sem_objeto
            if (dist_quatro(i)<Dist_zona_morta) 
                dist_quatro(i)=Dist_zona_morta;
            end
            %padronizando a detecção de 0 a 1. Em que 0 (sem obstáculo) e 1 (obstáculo). 
            detect_quatro=(Dist_sem_objeto-dist_quatro(i))/(Dist_sem_objeto-Dist_zona_morta);
            else
            detect_quatro=0;
           end

         %sensor 5
         if detectionState_cinco>0 && dist_cinco(i)<Dist_sem_objeto 
            if (dist_cinco(i)<Dist_zona_morta) 
                dist_cinco(i)=Dist_zona_morta;
            end
            %padronizando a detecção de 0 a 1. Em que 0 (sem obstáculo) e 1 (obstáculo).    
            detect_cinco=(Dist_sem_objeto-dist_cinco(i))/(Dist_sem_objeto-Dist_zona_morta);
        else
            detect_cinco=0;
         end

        %sensor 6
        if detectionState_seis>0 && dist_seis(i)<Dist_sem_objeto 
            if (dist_seis(i)<Dist_zona_morta) 
                dist_seis(i)=Dist_zona_morta;
            end
            %padronizando a detecção de 0 a 1. Em que 0 (sem obstáculo) e 1 (obstáculo).     
            detect_seis=(Dist_sem_objeto-dist_seis(i))/(Dist_sem_objeto-Dist_zona_morta);
        else
            detect_seis=0;
        end
       
        %sensor 7
        if detectionState_sete>0 && dist_sete(i)<Dist_sem_objeto 
            if (dist_sete(i)<Dist_zona_morta) 
                dist_sete(i)=Dist_zona_morta;
            end
            %padronizando a detecção de 0 a 1. Em que 0 (sem obstáculo) e 1 (obstáculo).         
            detect_sete=(Dist_sem_objeto-dist_sete(i))/(Dist_sem_objeto-Dist_zona_morta);
        else
            detect_sete=0;
        end

         %vetores de Braintenberg (calcula velocidades das rodas)
         detect = [detect_zero, detect_um, detect_dois, detect_tres, detect_quatro, detect_cinco, detect_seis, detect_sete];
         pesos_esq = [-0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6];
         pesos_dir = [-1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2];
         v_esq = V_rob;
         v_dir = V_rob;

         %Diminuir recursivamente a velocidade dos motores
         for k=1:N_sensores
             v_esq = v_esq + pesos_esq(k)*detect(k);
             v_dir = v_dir + pesos_dir(k)*detect(k);
         end

            [returnCode]=sim.simxSetJointTargetVelocity(clientID,l_motor,v_esq,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,r_motor,v_dir,sim.simx_opmode_blocking);
    end

    converte = round(converte, 5);
    converte(converte > 5) = 5;
    converte(converte < -5) = -5;

    coords_unicos = unique(converte, 'rows');

    figure; 
    plot(coords_unicos(:,1), coords_unicos(:,2), 'o'); 
    grid on;

    % Parando a simulação
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);    
    sim.simxFinish(clientID);
    sim.delete();
    disp('Conexão finalizada');
end