%função para calcular o vértice mais próximo de um grafo, dadas as
%coordenadas do mapa
function [Vertice] = Vertice_mais_prox(x_pix,y_pix,x_no_disp,y_no_disp)

N_vertices=length(x_no_disp);
Melhor_dist=inf;
pos=[x_pix,y_pix]

for k=1:N_vertices
    delta_d=pos-[x_no_disp(k),y_no_disp(k)];
    dist_atual=norm(delta_d);
    if(dist_atual<Melhor_dist)
        Vertice=k;
        Melhor_dist=dist_atual;
    end

end

end