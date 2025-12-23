% estar rodando (play) e no script da cena principal acrescentar o código a
% seguir: simRemoteApi.start(19999) 

clc;
close all;
clear all;

disp('Início do Programa');
sim=remApi('remoteApi'); % cria uma instância de simulação
sim.simxFinish(-1); % por precaução, feche todas as conexões abertas
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); % inicia a comunicação com o servidor

% Definição de parâmetros do Robô
L = 0.331;
r = 0.09751;
maxv = 1.5;
maxw = deg2rad(90);

if clientID > -1
    disp('Conectou');
    %Colocar o código aqui
    %% Programa para criar o mapa em escala de cinza                  
    figure('Position', [100, 100, 800, 800]); %  usado para definir o tamanho da figura
                       %[ posX , posY , largura , altura ]
    % 100: Posição X da borda inferior esquerda da janela (distância do canto esquerdo da tela).
    % 100: Posição Y da borda inferior esquerda da janela (distância do canto inferior da tela).
    % 800: Largura da janela da figura (em pixels).
    % 800: Altura da janela da figura (em pixels).

    map_size = [16, 16];
    cell_size = 0.1;

    rows = ceil(map_size(1) / cell_size); % número de linhas
    cols = ceil(map_size(2) / cell_size); % número de colunas

    m1 = ones(rows, cols); %matriz de uns
    m1=0.5*m1;             % atribuição de probabilidade de ocupação 0,5 em cada célula.

    %alocar os limites da escala de cinza
    m1(1, 1) = 1;         %branco
    m1(rows, cols) = 0;   %preto

    log_odds= zeros(rows, cols); %inicialização do Log_Odds

    %% Programa do robô
    % Obtendo a etiqueta (handle) do robô
    [returnCode, robotHandle] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking); % modo bloqueante simx_opmode_oneshot_wait=imx_opmode_blocking

    % Obtendo as etiquetas dos motores
    [returnCode, l_motor] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking);
    [returnCode, r_motor] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking);

    % Obtendo etiqueta da referencia
    [returnCode, referencia] = sim.simxGetObjectHandle(clientID, '/ReferenceFrame', sim.simx_opmode_blocking);

    % Dados do sensor Hokuyo para mapeamento
    % Obtendo dados do laser
    laser_range_data = 'hokuyo_range_data';
    laser_angle_data = 'hokuyo_angle_data';

    % Aguardando a primeira leitura válida (As primeiras leituras preisam ser no modo Streaming)
    returnCode1 = 1;
    returnCode2 = 1;
    while returnCode1 ~= 0 && returnCode2 ~= 0
        [returnCode1, string_range_data] = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_streaming + 10); % 10 representa 10ms de espera entre 2 comandos consecutivos no lado do servidor
        [returnCode2, string_angle_data] = sim.simxGetStringSignal(clientID, laser_angle_data, sim.simx_opmode_streaming + 10); % 10 representa 10ms de espera entre 2 comandos consecutivos no lado do servidor
    end

    % Inicialização de variáveis
    kr = 1.0;    % 0.4 ganho do conrolador Rho
    ka = 2.5;    % 0.8 ganho do controlador alpha

    for i=1:300
        %pega os dados do robô e da referência móvel
        [returnCode, pos] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait); 
        pos_xy=[pos(1);pos(2)];
        [returnCode, ori] = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait);
        [returnCode, P_Uref] = sim.simxGetObjectPosition(clientID, referencia, -1, sim.simx_opmode_oneshot_wait);       
       

        Delta_x = P_Uref(1)-pos(1);
        Delta_y = P_Uref(2)-pos(2);

        rho = sqrt(Delta_x^2 + Delta_y^2);
        ori_z=ori(3);
        alpha = NormalizaAngulo(-ori_z + atan2(Delta_y, Delta_x));

        if(alpha>pi/2 || alpha < -pi/2) % Se o alvo estiver para trás do robô
            v_R=0; % Ao invés disso pode ser programado para o robô andar de Re.
        else          
            v_R = kr*rho*cos(alpha);
        end

        %v_R = kr*rho*cos(alpha);
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

        %% Pega os dados do sensor hokuyo para desenhar o mapeamento
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

        % definição do ponto inicial e final em metros de cada feixe do
        % laser
        %coordenadas do robô em metros
        x = pos(1);
        y = pos(2);
        % deslocando o centro do sistema de referência
        xd=x+map_size(2)/2;
        yd=map_size(1)-(y+map_size(1)/2);
        % mudando de escala para pixels
        xd=round(xd/cell_size);
        yd=round(yd/cell_size);


        % Matriz de rotação 2D de R->U
        R=[cos(ori_z), -sin(ori_z); sin(ori_z) cos(ori_z)];

        log_odds_anterior = log_odds;
        pa_oc = 0.6; %probabilidade de ocupado
        pa_free = 0.45; %probabilidade de livre
        log_odds_oc = log(pa_oc/(1-pa_oc));
        log_odds_free = log(pa_free/(1-pa_free));


        for i = 1:length(raw_range_data)
            ang = raw_angle_data(i);
            dist = raw_range_data(i);

            [xR,yR]=pol2cart(ang,dist);
            R_P=[xR;yR];    %Ponto no referencial do robô
            U_P=R*R_P;      %Ponto rotacionado no referencial Universal (Global)
            U_P=U_P+pos_xy; %Deslocando a origem
            xo = U_P(1); % coordenadas do obstáculo em metros
            yo = U_P(2);

            % deslocando para o sistema de referencia
            xod=xo+map_size(2)/2;
            yod=map_size(1)-(yo+map_size(1)/2);

            %mudando a escala para pixels
            xod=floor(xod/cell_size); %coordenadas do obstáculo em pixels
            yod=floor(yod/cell_size);


            % Gerar a linha de Bresenham
            pontos=bresenham(yd, xd, yod, xod);
            rr=pontos(:,1);
            cc=pontos(:,2);

            % Quando o feixe não acerta nada, retorna o valor máximo (definido na simulação)
            % Logo, usar um pequeno limiar do máximo para considerar a leitura
            if dist < 4.99 % pinta de preto as celulas alcançadas e de branco as anteriores
                for k = 1:length(rr) % faz para todos os pontos da linha de Bresenham
                    if rr(k) >= 1 && rr(k) <= rows && cc(k) >= 1 && cc(k) <= cols % se a linha de Bresenham está dentro do mapa, faz
                        if k==length(rr) %aqui está sendo pego apenas a última célula. Dependendo do modelo inverso do sensor pode ser que pegue mais de uma célula como ocupada.
                            %implementar Log_odds
                            log_odds(rr(k), cc(k)) = log_odds_anterior(rr(k), cc(k)) + log_odds_oc; % o obstáculo pinta de preto
                        else
                            %implementar Log_odds Pfree
                            if m1(rr(k), cc(k))~=0 % se a celula não é preta (pode ser cinza) então pinta de branco
                                log_odds(rr(k), cc(k)) = log_odds_anterior(rr(k), cc(k)) + log_odds_free; % a distância até o obstáculo pinta de branco
                            end
                        end
                    end
                end

            else % não tem obstáculo, então pinta todas as células de branco
                for k = 1:length(rr)
                    if rr(k) >= 1 && rr(k) <= rows && cc(k) >= 1 && cc(k) <= cols
                        %implementar Log_odds Pfree
                        if m1(rr(k), cc(k))~=0 % se a celula não é preta (pode ser cinza) então pinta de branco
                            log_odds(rr(k), cc(k)) = log_odds_anterior(rr(k), cc(k)) + log_odds_free; % a distância até o obstáculo pinta de branco
                            m1(rr(k), cc(k)) = 1; % a distância até o obstáculo pinta de branco
                        end
                    end
                end
            end
        end

    end

    m1 = 1 ./ (1 + exp(-log_odds));

    figure('Position', [100, 100, 800, 800]);
    % Mostrar a matriz original
    imagesc([1 cols], [1 rows], 1-m1);
    colormap('gray');
    colorbar;

    % Ajustes dos ticks dos eixos
    % set(gca, 'YDir', 'normal');
    % set(gca, 'XTick', 1:cell_size:cols);
    % set(gca, 'YTick', 1:cell_size:rows);
    set (gca,'Ydir','reverse');
    axis equal;


    % Parando a simulação
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    % Fecha a conexão com o CoppeliaSim
    sim.simxFinish(clientID);
else
    disp('Falha na conexão no servidor remoto');
end
sim.delete(); % call the destructor!

disp('Fim do programa');
