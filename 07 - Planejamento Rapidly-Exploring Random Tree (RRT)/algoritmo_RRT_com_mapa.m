clc;
clear all;
close all;

% Leitura da imagem do mapa
%img = imread('imagens/imagem_processada_50x50.png');
img = imread('Img_processada_700_x_700.png');
%img = imread('imagens/maze.png');

img=rgb2gray(img); %Converter para gray
[n_linhas_img, n_colunas_img]=size(img);
figure;
image(img); % mostra a imagem em escala de cinza
set(gcf,'colormap',gray);
axis equal;

% Definição das coordenadas do ponto inicial e final do robô
start_point = [-4, -4];
goal_point = [2.8, 2.76];

%dimensões do mapa (considera-se que o sistema de ref está no centro do mapa)
size_x_mapa=10;
size_y_mapa=10;

%escala entre o mapa real e a imagem em pixels
escala_colunas=n_colunas_img/size_x_mapa; %pixels/metro
escala_linhas=n_linhas_img/size_y_mapa; %pixels/metro

% Parâmetros do algoritmo RRT
max_iter = 1000; % número máximo de iterações
step_size = 1;

% Inicialização da árvore
tree = [start_point, 0]; % A árvore é uma matriz onde cada linha é um nó: [x, y, índice do pai]

for iter = 1:max_iter
    % Gerar um ponto aleatório no espaço de configuração
    if (mod(iter,10)==0) % se multiplo de 5 gera um ponto em direção da meta
        random_point = goal_point;
    else
        random_point = [rand()*size_x_mapa-size_x_mapa/2, rand()*size_y_mapa-size_y_mapa/2]; % Substitua 10 pelos limites do seu ambiente
    end
    
    % Buscar o nó mais próximo na árvore
    distances = sqrt(sum((tree(:, 1:2) - random_point).^2, 2));
    [~, nearest_idx] = min(distances);
    nearest_node = tree(nearest_idx, 1:2);
    
    % Gerar um novo ponto na direção do ponto aleatório a uma distância
    % step_size
    direction = (random_point - nearest_node) / norm(random_point - nearest_node);
    new_point = nearest_node + step_size * direction;

    %possível nó a ser adicionado
    new_node = [new_point, nearest_idx];

    %segemento entre o nó pai se o novo ponto (apenas para graficar)
    segmento=[nearest_node; new_node(1:2)];

    %segmento em termos de pixels para graficar junto com o mapa
    segmento_pixels=[(segmento(:,1)+size_x_mapa/2)*escala_colunas,n_linhas_img-(segmento(:,2)+size_y_mapa/2)*escala_linhas];
    segmento_pixels=round(segmento_pixels);    
    
    % Verificar se o novo ponto colide com o ambiente (pode variar de acordo com o ambiente)
    if collision_check(segmento_pixels,img)
        continue; % Ignorar este ponto se houver colisão
    end
    
    % Adicionar o novo ponto à árvore
    tree = [tree; new_node];
    
    % Verificar se o objetivo foi alcançado
    if norm(new_point - goal_point) < step_size
        % Se sim, adicionar o ponto final à árvore e sair do loop
        final_node = [goal_point, size(tree, 1)];
        tree = [tree; final_node];
        break;
    end
    


    %gráfico o novo segmento
    if(iter==1)
        %figure;
        hold on;
        % image(img); % mostra a imagem em escala de cinza
        % set(gcf,'colormap',gray);
        % axis equal;
        % Definir limites do gráfico
        %esca de metros
        % xlim([-size_x_mapa/2, size_x_mapa/2]); % Ajustar conforme necessário
        % ylim([-size_y_mapa/2, size_y_mapa/2]); % Ajustar conforme necessário
        %escala de pixels
        xlim([0, n_colunas_img]); % Ajustar conforme necessário
        ylim([0, n_linhas_img]); % Ajustar conforme necessário        
    end
    %plot(segmento(:,1),segmento(:,2),'b');
    plot(segmento_pixels(:,1),segmento_pixels(:,2),'b');
    set (gca,'Ydir','reverse');
    pause(0.01);
end
%hold off;

% Plotar a árvore
%plot(tree(:,1), tree(:,2), '-o'); % Plotar os nós
%hold on;

% Plotar a linha do caminho final
if norm(tree(end, 1:2) - goal_point) < step_size
    path = backtrack(tree, size(tree, 1)); %caminho em metros
    path_pixels_x=(path(:,1)+size_x_mapa/2)*escala_colunas;
    path_pixels_y=n_linhas_img-(path(:,2)+size_y_mapa/2)*escala_linhas;
    path_pixels_x=round(path_pixels_x);
    path_pixels_y=round(path_pixels_y);

    path
    
    %plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2); % Caminho final em vermelho
    plot(path_pixels_x, path_pixels_y, 'r-', 'LineWidth', 2); % Caminho final em vermelho
    disp('Caminho encontrado:');
else
    disp('Nenhum caminho encontrado.');
end

% Adicionar marcadores para o ponto inicial e final
start_point_pixels=[(start_point(1)+size_x_mapa/2)*escala_colunas,n_linhas_img-(start_point(2)+size_y_mapa/2)*escala_linhas];
goal_point_pixels=[(goal_point(1)+size_x_mapa/2)*escala_colunas,n_linhas_img-(goal_point(2)+size_y_mapa/2)*escala_linhas];
start_point_pixels=round(start_point_pixels);
goal_point_pixels=round(goal_point_pixels);

% plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Ponto inicial em verde
% plot(goal_point(1), goal_point(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Ponto final em vermelho

plot(start_point_pixels(1), start_point_pixels(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Ponto inicial em verde
plot(goal_point_pixels(1), goal_point_pixels(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Ponto final em vermelho


hold off;

% Adicionar rótulos e título
xlabel('X');
ylabel('Y');
title('RRT para Planejamento de Trajetória');

% figure;
% hold on
% for kl= 1:n_linhas_img
%     for kc=1:n_colunas_img
%         if img(kl,kc) <127
%             plot(kc,kl,'xr');
%         else
%             plot(kc,kl,'bo');
%         end
% 
%     end
% end
% set (gca,'Ydir','reverse');
% hold off

% Função para verificar colisões (substitua pela sua implementação)
function collision = collision_check(segmento_pixels,img)
    % Implemente a lógica para verificar se há colisão entre os pontos
    collision=true;
    y1=segmento_pixels(1,1);
    x1=segmento_pixels(1,2);
    y2=segmento_pixels(2,1);
    x2=segmento_pixels(2,2);

    [n_linhas_img, n_colunas_img]=size(img);
    if (x1<1 || y1<1 || x2<1 || y2<1)
        return;
    end
    if (x1>n_colunas_img || y1>n_linhas_img || x2>n_colunas_img || y2>n_linhas_img)
        return
    end
    % Inicializa as coordenadas iniciais
    x = round(x1);
    y = round(y1);
    
    % Calcula as diferenças
    dx = round(x2) - round(x1);
    dy = round(y2) - round(y1);

    if dx == 0 && dy == 0
        if img(round(y1),round(x1))>127
            collision = false; % Por enquanto, assumimos que não há colisão
            
        else
            collision=true;
        end
        return;
    end

    % Determina o número de passos
    if abs(dx) > abs(dy)
        steps = abs(dx);
    else
        steps = abs(dy);
    end
    

    % Calcula as incrementações
    x_increment = dx / steps;
    y_increment = dy / steps;
    

    for i = 1:steps+1
        if img(round(x),round(y))>127
            collision = false; % Por enquanto, assumimos que não há colisão

        else
            collision=true;
        end
        x = x + x_increment;
        y = y + y_increment;
        
    end    
    %collision = false; % Por enquanto, assumimos que não há colisão
end

% Função para realizar backtracking e recuperar o caminho
function path = backtrack(tree, end_idx)
    path = [];
    current_idx = end_idx;
    while current_idx ~= 1
        path = [tree(current_idx, 1:2); path];
        current_idx = tree(current_idx, 3);
    end
    path = [tree(1, 1:2); path];
end