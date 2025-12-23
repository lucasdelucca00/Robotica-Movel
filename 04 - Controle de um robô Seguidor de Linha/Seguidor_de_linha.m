% Implementação do algoritmo de seguidor de linha LineTracer
% Na cena principal acrescentar o código a seguir: simRemoteApi.start(19999)

clc;
close all;
clear all;

disp('Início do Programa');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if clientID > -1
    disp('Conectou ao Servidor');
    % Colocar o se código aqui

    % Ativa modo síncrono
    %sim.simxSynchronous(clientID,true);

    %Obtendo a etiqueta do robô
    [returnCode, LineTracer] = sim.simxGetObjectHandle(clientID, '/LineTracer', sim.simx_opmode_blocking); 
    
    % Obtendo as etiquetas dos motores do seguidor de linha
    [returnCode, l_motor_seg] = sim.simxGetObjectHandle(clientID, '/LineTracer/DynamicLeftJoint', sim.simx_opmode_blocking); 
    [returnCode, r_motor_seg] = sim.simxGetObjectHandle(clientID, '/LineTracer/DynamicRightJoint', sim.simx_opmode_blocking);

    %obtendo as etiquetas dos sensores do seguidor de linha
    [returnCode, faixa_esq] = sim.simxGetObjectHandle(clientID, '/LineTracer/LeftFaixa', sim.simx_opmode_blocking);
    [returnCode, sensor_Se3] = sim.simxGetObjectHandle(clientID, '/LineTracer/Sensor_Se3', sim.simx_opmode_blocking);
    [returnCode, sensor_Se2] = sim.simxGetObjectHandle(clientID, '/LineTracer/Sensor_Se2', sim.simx_opmode_blocking);
    [returnCode, sensor_Se1] = sim.simxGetObjectHandle(clientID, '/LineTracer/Sensor_Se1', sim.simx_opmode_blocking);
    [returnCode, sensor_S0] = sim.simxGetObjectHandle(clientID, '/LineTracer/Sensor_S0', sim.simx_opmode_blocking);
    [returnCode, sensor_Sd1] = sim.simxGetObjectHandle(clientID, '/LineTracer/Sensor_Sd1', sim.simx_opmode_blocking);
    [returnCode, sensor_Sd2] = sim.simxGetObjectHandle(clientID, '/LineTracer/Sensor_Sd2', sim.simx_opmode_blocking);
    [returnCode, sensor_Sd3] = sim.simxGetObjectHandle(clientID, '/LineTracer/Sensor_Sd3', sim.simx_opmode_blocking);
    [returnCode, faixa_dir] = sim.simxGetObjectHandle(clientID, '/LineTracer/RightFaixa', sim.simx_opmode_blocking);

    %realizando a primeira leitura no modo streaming conforme manual do
    %CoppeliaSim. Se naõ fizer isso não funciona
    [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,faixa_esq,sim.simx_opmode_streaming);
    [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,sensor_Se3,sim.simx_opmode_streaming);
    [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,sensor_Se2,sim.simx_opmode_streaming);
    [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,sensor_Se1,sim.simx_opmode_streaming);
    [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,sensor_S0,sim.simx_opmode_streaming);
    [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,sensor_Sd1,sim.simx_opmode_streaming);
    [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,sensor_Sd2,sim.simx_opmode_streaming);
    [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,sensor_Sd3,sim.simx_opmode_streaming);
    [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,faixa_dir,sim.simx_opmode_streaming);

    %Inicializando a leitura dos sensores analógicos
    sensor_an_Se3=0.1;
    sensor_an_Se2=0.1;
    sensor_an_Se1=0.1;
    sensor_an_S0=0.1;
    sensor_an_Sd1=0.1;
    sensor_an_Sd2=0.1;
    sensor_an_Sd3=0.1;
    Leitura_Faixa_esq = 1;

    cont_faixa_esq = 0;
    cont_faixa_dir = 0;
    linha_preta_esq = 0;
    linha_preta_dir = 0;

    %Info do robo
    V_rob = 20;

    while (cont_faixa_dir < 5) % implementar um critério de parada

        %Tratamento dos sensores analógicos
        [returnCode,detectionState,cor,~]=sim.simxReadVisionSensor(clientID,sensor_Se3,sim.simx_opmode_buffer);

        %Leitura_Sensor_esq=detectionState;
        if length(cor)>11
            sensor_an_Se3=cor(12); % me devolve a intensidade da cor entre 0 e 1 (claro 1, preto 0)
        end
        [returnCode,detectionState,cor,~]=sim.simxReadVisionSensor(clientID,sensor_Se2,sim.simx_opmode_buffer);
        %Leitura_Sensor_esq=detectionState;
        if length(cor)>11
            sensor_an_Se2=cor(12); % me devolve a intensidade da cor entre 0 e 1
        end
        [returnCode,detectionState,cor,~]=sim.simxReadVisionSensor(clientID,sensor_Se1,sim.simx_opmode_buffer);
        %Leitura_Sensor_esq=detectionState;
        if length(cor)>11
            sensor_an_Se1=cor(12); % me devolve a intensidade da cor entre 0 e 1
        end        
        [returnCode,detectionState,cor,~]=sim.simxReadVisionSensor(clientID,sensor_S0,sim.simx_opmode_buffer);
        %Leitura_Sensor_frente=detectionState;
        if length(cor)>11
            sensor_an_S0=cor(12); % me devolve a intensidade da cor entre 0 e 1
        end
        [returnCode,detectionState,cor,~]=sim.simxReadVisionSensor(clientID,sensor_Sd1,sim.simx_opmode_buffer);
        %Leitura_Sensor_dir=detectionState;
        if length(cor)>11
            sensor_an_Sd1=cor(12); % me devolve a intensidade da cor entre 0 e 1
        end
        [returnCode,detectionState,cor,~]=sim.simxReadVisionSensor(clientID,sensor_Sd2,sim.simx_opmode_buffer);
        %Leitura_Sensor_dir=detectionState;
        if length(cor)>11
            sensor_an_Sd2=cor(12); % me devolve a intensidade da cor entre 0 e 1
        end
        [returnCode,detectionState,cor,~]=sim.simxReadVisionSensor(clientID,sensor_Sd3,sim.simx_opmode_buffer);
        %Leitura_Sensor_dir=detectionState;
        if length(cor)>11
            sensor_an_Sd3=cor(12); % me devolve a intensidade da cor entre 0 e 1
        end 
     
        % Calcular posição do sensor de faixa
        V_posicao = [sensor_an_Se3; sensor_an_Se2; sensor_an_Se1; sensor_an_S0; sensor_an_Sd1; sensor_an_Sd2; sensor_an_Sd3];
        V_peso = [-7; -6; -5; 0; 5; 6; 7];
        Soma = 0;
        Posicao = 0;

        for i = 1:7
            Soma = V_posicao(i) + Soma;
            V_posicao(i) = V_posicao(i) * V_peso(i); 
        end

        for i = 1:7
            Posicao = Posicao + (V_posicao(i)/Soma);
        end

        V_ori = Posicao;

         % Implementar o controlador PID
        V_esq = V_rob - V_ori;
        V_dir = V_rob + V_ori;

        switch cont_faixa_esq

            case 2
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,V_esq-6,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,V_dir-6,sim.simx_opmode_blocking);
            case 3
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,V_esq-17,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,V_dir-17,sim.simx_opmode_blocking);
            case 4
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,V_esq-3,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,V_dir-3,sim.simx_opmode_blocking);
            case 6
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,V_esq-17,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,V_dir-17,sim.simx_opmode_blocking);
            case 7
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,V_esq-6,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,V_dir-6,sim.simx_opmode_blocking);
            case 8
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,20,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,20,sim.simx_opmode_blocking);
            case 10
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,V_esq-17,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,V_dir-17,sim.simx_opmode_blocking);   
            case 11
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,V_esq-6,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,V_dir-6,sim.simx_opmode_blocking); 
            case 12
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,V_esq-17,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,V_dir-17,sim.simx_opmode_blocking);   
            case 13
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,V_esq+5,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,V_dir+5,sim.simx_opmode_blocking);   
            otherwise
                sim.simxSetJointTargetVelocity(clientID,l_motor_seg,V_esq,sim.simx_opmode_blocking);    
                sim.simxSetJointTargetVelocity(clientID,r_motor_seg,V_dir,sim.simx_opmode_blocking);
             
        end



        % Leitura digital dos sensores de faixa: usar essas leituras para
        % implementar o critério de parada da simulação
        [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,faixa_esq,sim.simx_opmode_buffer);
        Leitura_Faixa_esq=detectionState;

        if(Leitura_Faixa_esq == 1)
           linha_preta_esq = 0;
        end

        if(linha_preta_esq == 0 && Leitura_Faixa_esq == 0)
           cont_faixa_esq = cont_faixa_esq + 1;
           linha_preta_esq = 1;
        end
  

        [returnCode,detectionState,~,~]=sim.simxReadVisionSensor(clientID,faixa_dir,sim.simx_opmode_buffer);
        Leitura_Faixa_dir=detectionState;

        if(Leitura_Faixa_dir == 1)
           linha_preta_dir = 1;
        end

        if(linha_preta_dir == 1 && Leitura_Faixa_dir == 0)
           cont_faixa_dir = cont_faixa_dir + 1
           linha_preta_dir = 0;
        end

    end
    
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxFinish(clientID);

else
    disp('Falha na conexão no servidor remoto');
end
sim.delete(); % call the destructor!

disp('Fim do programa');