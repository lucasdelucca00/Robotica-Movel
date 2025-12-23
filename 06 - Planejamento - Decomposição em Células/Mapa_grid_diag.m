clc;
clear all;
close all;

%img = imread('imagens/mapa1.bmp');
%img = imread('imagens/mapa2.jpg');
%img = imread('imagens/mapa3.png');
img = imread('mapa.png');
img=rgb2gray(img); %Converter para gray
[n_linhas_img, n_colunas_img]=size(img);
image(img); % mostra a imagem em escala de cinza
set(gcf,'colormap',gray);
axis equal;

%Tamanho do mapa em metros
size_x_mapa=10;
size_y_mapa=10;


%intervalo do grid em metros
size_grid=0.4;
etiqueta_x=-size_x_mapa/2:size_grid:size_x_mapa/2;
etiqueta_y=size_y_mapa/2:-size_grid:-size_y_mapa/2;

%convertendos as etiquetas int em array para etiquetar os eixos do mapa
etiqueta_x_string = arrayfun(@num2str, etiqueta_x, 'UniformOutput', false);
etiqueta_y_string = arrayfun(@num2str, etiqueta_y, 'UniformOutput', false);

escala_colunas=n_colunas_img/size_x_mapa; %pixels/metro
escala_linhas=n_linhas_img/size_y_mapa; %pixels/metro

ticks_x=0:size_grid*escala_colunas:n_colunas_img;
ticks_y=0:size_grid*escala_linhas:n_linhas_img;


%Apenas para garantir que só teremos esses dois valores
threshold = 127;
img(img > threshold) = 255;
img(img <= threshold) = 0;


% Preenchendo o GRID, onde tiver algum pixel preto, preenche toda a celula
% Cada célula recebe o somatório dos valores dos Pixels
rows=round(n_linhas_img/(size_grid*escala_linhas)); %celulas em pixels linhas (obs! o tamanho vertical está dividido em linhas)
cols=round(n_colunas_img/(size_grid*escala_colunas)); %celulas em pixels linhas
Grade = zeros(rows, cols);
for r = 1:rows
    for c = 1:cols
        
        colunas_i = round((c-1) * size_grid*escala_colunas) + 1;
        colunas_f = round(colunas_i + size_grid*escala_colunas) - 1;
        
        linhas_i = round((r-1) * size_grid*escala_linhas) + 1;
        linhas_f = round(linhas_i + size_grid*escala_linhas) - 1;
       
        if (colunas_f <= n_colunas_img) &&(linhas_f < n_linhas_img)
            matriz_aux=img(linhas_i:linhas_f, colunas_i:colunas_f);
        end
        %se a matriz auxiliar tiver algum zero, zerar tudo
        aux1=min(matriz_aux);
        aux2=min(aux1);
        if aux2~=0
            aux2=255;
        end
        Grade(r, c) = aux2;
    end
end
% para cada bloco foi asocciado um pixel, então a imagem diminui de tamanho

%Zoom do gráfico para plotar junto com o original
Grade_Completa=zeros(n_linhas_img, n_colunas_img);
escalax=n_colunas_img/cols;
escalay=n_linhas_img/rows;
for kl=1:n_linhas_img
    klg=ceil(kl/escalay);
    for kc=1:n_colunas_img
        kcg=ceil(kc/escalax);
        Grade_Completa(kl,kc)=Grade(klg,kcg);
    end
end
%Grade_Completa = imresize(Grade,[size_y_img size_y_img]); 

%Figura na escala do mapa em metros sem o grafo
figure
image(img);
hold on;
image(Grade_Completa, 'AlphaData', 0.4);
set(gcf,'colormap',gray);
xticks(ticks_x);
xticklabels(etiqueta_x_string);
yticks(ticks_y);
yticklabels(etiqueta_y_string);
grid;
hold off;
%axis equal;

%Figura na escala do mapa em metros adicionando o grafo
figure
image(img);
hold on;
image(Grade_Completa, 'AlphaData', 0.4);
set(gcf,'colormap',gray);
xticks(ticks_x);
xticklabels(etiqueta_x_string);
yticks(ticks_y);
yticklabels(etiqueta_y_string);
grid;
%hold off;

% Criando nós no grafo
% se o elemento Grade(x,y) for igual a zero, então é obstáculo.


no=0; %número inicial de nós do grafo
for r = 1:rows
    for c = 1:cols  
        if Grade(r, c)==255 %criar nó
            no=no+1; 
            y_no(no)=r; %posição do nó em y em pixels na escala Grade(x,y)
            x_no(no)=c; %posição do nó em x em pixels na escala Grade(x,y)
        end
    end
end

%Criar a matriz de adjacencia
Adj=zeros(no,no);
for k=1:no
    %pego a coordenada do nó k
    cord_x=x_no(k);
    cord_y=y_no(k);
    for kk=1:no % testo o nó k com todos os outros para ver se são vizinhos
        if k~=kk
            cord_xv=x_no(kk);
            cord_yv=y_no(kk);

            if (cord_x+1)== cord_xv && cord_y == cord_yv% verifica se o vizinho está na direita
                %adicionar aresta
                Adj(k,kk)=1;
            end
            if (cord_x-1)== cord_xv && cord_y == cord_yv% verifica se o vizinho está à esquerda
                %adicionar aresta
                Adj(k,kk)=1;
            end
            if cord_x== cord_xv && (cord_y+1) == cord_yv% verifica se o vizinho está na acima
                %adicionar aresta
                Adj(k,kk)=1;
            end
            if cord_x== cord_xv && (cord_y-1) == cord_yv% verifica se o vizinho está à abaixo
                %adicionar aresta
                Adj(k,kk)=1;
            end
            %adicionando as diagonais (o peso das diagonais é sqrt(2))
            if (cord_x+1)== cord_xv && (cord_y+1) == cord_yv% verifica se o vizinho está na direita superior
                %adicionar aresta
                Adj(k,kk)=sqrt(2);
            end
            if (cord_x+1)== cord_xv && (cord_y-1) == cord_yv% verifica se o vizinho está na direita superior
                %adicionar aresta
                Adj(k,kk)=sqrt(2);
            end
            if (cord_x-1)== cord_xv && (cord_y+1) == cord_yv% verifica se o vizinho está à esquerda superior
                %adicionar aresta
                Adj(k,kk)=sqrt(2);
            end
            if (cord_x-1)== cord_xv && (cord_y-1) == cord_yv% verifica se o vizinho está à esquerda superior
                %adicionar aresta
                Adj(k,kk)=sqrt(2);
            end            
        end
    end
end

Grafo=graph(Adj);

%sabe-se que x_no e y_no está na escala do grid, ou seja,  x_no*Size_grid => escala em
%metros
%posição do nós em pixels
x_no_disp=round(x_no*n_colunas_img/kcg - escala_colunas*size_grid/2);
y_no_disp=round(y_no*n_linhas_img/klg - escala_linhas*size_grid/2);

%posição inicial e final do robô (Lembrar o o referencial está no meio do mapa)
posicao_ini_x_robo_mt=-4.5; %em metros
posicao_ini_y_robo_mt=-4.5; %em metros
posicao_fim_x_robo_mt=3.5; %em metros
posicao_fim_y_robo_mt=3.4; %em metros

%posição inicial do robô em pixels
posicao_ini_x_robo_pix=round((posicao_ini_x_robo_mt+size_x_mapa/2)*escala_colunas); %em pixels
posicao_ini_y_robo_pix=round((-posicao_ini_y_robo_mt+size_y_mapa/2)*escala_linhas); %em pixels
posicao_fim_x_robo_pix=round((posicao_fim_x_robo_mt+size_x_mapa/2)*escala_colunas); %em pixels
posicao_fim_y_robo_pix=round((-posicao_fim_y_robo_mt+size_y_mapa/2)*escala_linhas); %em pixels

vertice_inicial=Vertice_mais_prox(posicao_ini_x_robo_pix,posicao_ini_y_robo_pix,x_no_disp,y_no_disp);
vertice_final=Vertice_mais_prox(posicao_fim_x_robo_pix,posicao_fim_y_robo_pix,x_no_disp,y_no_disp);

% vertice_inicial=1; %vertice inicial do grafo
% vertice_final=4;   %vertice final do grafo
[caminho, menor_distancia]=shortestpath(Grafo,vertice_inicial,vertice_final);


p=plot(Grafo,'XData',x_no_disp,'YData',y_no_disp);
set (gca,'Ydir','reverse');
highlight(p,caminho,'EdgeColor','r','LineWidth',2);
plot(posicao_ini_x_robo_pix,posicao_ini_y_robo_pix,'or', posicao_fim_x_robo_pix,posicao_fim_y_robo_pix,'xr');
hold off;

%Convertendo as coordenadas dos nós de pixels para metros
x_no_metros=(x_no_disp-n_colunas_img/2)/escala_colunas;
y_no_metros=(n_linhas_img/2-y_no_disp)/escala_linhas;

%Mostrando as coordenadas do caminho em metros
for k=1:length(caminho)
    xx=x_no_metros(caminho(k))
    yy=y_no_metros(caminho(k))
end

