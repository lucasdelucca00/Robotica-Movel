clc;
clear all;
close all;


img = imread('mapa.png');

img=rgb2gray(img); %Converter para gray
[size_x_img, size_y_img]=size(img);
image(img);
set(gcf,'colormap',gray);



%Apenas para garantir que só teremos esses dois valores
threshold = 127;
img(img > threshold) = 255;
img(img <= threshold) = 0;

%imagem invertida para colocar no copeliaSim
aux=img;
NImg = 255 - aux;

NImg_100_x_100 = imresize(NImg,[100 100]);

imwrite(NImg_100_x_100,'imagens\Imagen_Negativa_100_x_100.png');

[B,L] = bwboundaries(NImg, 'noholes');
stats = regionprops(L, 'Area');

figure
imshow(img);
figure
imshow(NImg);

%Criação do robô para aplicar soma de Minkowski
Escala=700/10; % 700 pixels são 10 metros
Raio_Pioneer=(455e-3)*1.5; %será adotado como envolucro circular (O lado mais comprido é 455mm)
Raio_Pioneer_Pixels=Raio_Pioneer*Escala;
robo = cria_circulo([0, 0], Raio_Pioneer_Pixels, 50); %coordenadas do centro, raio, e Nºpontos (colocar o raio na escala de metros)
robo_poly=polyshape(robo);
robo_poly_refletido=polyshape(-robo);

%plot(robo_poly,'FaceColor','red');

figure;
hold on;
for k = 1:length(B)
    area = stats(k).Area;

    if area > 50 %somente plota aquele que tiver área maior a 50
        boundary = B{k}; %contorno do objeto que foi encontrado

        %O primeiro objeto é o contorno. Para esse não será realizado
        %minkowski
        if(k==1)
            x_contorno=[0 size_x_img size_x_img 0 0];
            y_contorno=[0 0 size_y_img size_y_img 0];
            plot(x_contorno,y_contorno,'k','LineWidth',1);
        else
            obs_poly=polyshape(boundary(:,2), boundary(:,1));
            Mink = minkowskiSum(robo_poly_refletido,obs_poly);
            %figure;
            plot(obs_poly,'FaceColor','red');
            plot(Mink,'FaceColor','blue','FaceAlpha',0.7);
            %plot(Mink,'FaceColor','red','LineStyle','--','FaceAlpha',0.2);
            set(gca,'Ydir','reverse');
        end

        % figure;
        % plot(boundary(:,2), boundary(:,1), 'black', 'LineWidth', 2);
        % text(boundary(1,2), boundary(1,1), sprintf('%.0f',area),...
        %     'Color', 'white',...
        %     'FontSize', 12,...
        %     'FontWeight', 'bold',...
        %     'BackgroundColor', 'black');
        % axis equal;
    end
end
hold off;
axis equal;
xticks([]);
yticks([]);
xlim([0 size_x_img]);
ylim([0 size_y_img]);
axis off;                     %  If you have axis titles, labels, etc.
set (gca,'Position',[0 0 1 1]);  
%exportgraphics(gca,'imagens\Im_pro_res_150.png','Resolution',150)
exportgraphics(gca,'imagens\Im_pro_res_160.png','Resolution',160)
img = imread('imagens/Im_pro_res_160.png');
Img_700_x_t00 = imresize(img,[700 700]);
imwrite(Img_700_x_t00,'imagens\Img_processada_700_x_700.png');

